#include "RPLidarS2.h"

// Protocol (Slamtec SDK sdk/include/sl_lidar_cmd.h)
static const uint8_t CMD_SYNC         = 0xA5;
static const uint8_t CMD_STOP         = 0x25;
static const uint8_t CMD_RESET        = 0x40;
static const uint8_t CMD_SCAN         = 0x20;
static const uint8_t CMD_EXPRESS_SCAN = 0x82;
static const uint8_t CMD_INFO         = 0x50;
static const uint8_t CMD_HEALTH       = 0x52;
static const uint8_t CMD_GET_CONF     = 0x84;
static const uint8_t CMD_MOTOR_SPEED  = 0xA8;   // payload: uint16 rpm

static const uint8_t ANS_SYNC1 = 0xA5;
static const uint8_t ANS_SYNC2 = 0x5A;
static const uint8_t ANS_FLAG_LOOP = 0x1;       // mode bits of the answer header

static const int32_t FULL_Q6 = 360 * 64;

static uint16_t u16le(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint32_t u32le(const uint8_t *p) { return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

// ------------------------------------------------------------------ commands (queue and return)

void RPLidarS2::send(uint8_t cmd)
{
    uint8_t buf[2] = {CMD_SYNC, cmd};
    port_.write(buf, 2);
}

// Command with payload: A5 cmd size payload... checksum (XOR of all previous bytes)
void RPLidarS2::send(uint8_t cmd, const uint8_t *payload, uint8_t size)
{
    uint8_t buf[3 + 16 + 1];
    if (size > 16) return;
    buf[0] = CMD_SYNC;
    buf[1] = cmd;
    buf[2] = size;
    memcpy(buf + 3, payload, size);
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 3 + size; i++) sum ^= buf[i];
    buf[3 + size] = sum;
    port_.write(buf, 4 + size);
}

void RPLidarS2::sendStop()      { send(CMD_STOP); }
void RPLidarS2::sendReset()     { send(CMD_RESET); }
void RPLidarS2::requestInfo()   { send(CMD_INFO); }
void RPLidarS2::requestHealth() { send(CMD_HEALTH); }

void RPLidarS2::requestConf(uint32_t type, int32_t mode)
{
    uint8_t p[6] = {(uint8_t)type, (uint8_t)(type >> 8), (uint8_t)(type >> 16), (uint8_t)(type >> 24),
                    (uint8_t)mode, (uint8_t)(mode >> 8)};
    send(CMD_GET_CONF, p, mode >= 0 ? 6 : 4);
}

void RPLidarS2::sendMotorRpm(uint16_t rpm)
{
    uint8_t p[2] = {(uint8_t)(rpm & 0xFF), (uint8_t)(rpm >> 8)};
    send(CMD_MOTOR_SPEED, p, 2);
}

void RPLidarS2::sendScan(const ScanMode &mode)
{
    setUsPerSample(mode.usPerSample);
    if (mode.ansType == ANS_NORMAL) {
        send(CMD_SCAN);
    } else {
        uint8_t p[5] = {(uint8_t)mode.id, 0, 0, 0, 0};  // working_mode, working_flags, param
        send(CMD_EXPRESS_SCAN, p, 5);
    }
}

// Largest start angle step between two capsules that is still real rotation (SDK check)
void RPLidarS2::setUsPerSample(float us)
{
    uint32_t sps = us > 0 ? (uint32_t)(1e6f / us) : 0;
    maxDiffQ8_ = sps ? (int32_t)((360UL * 100 * DENSE_SAMPLES / sps) << 8) : (360 << 8);
}

// ------------------------------------------------------------------ answer parsers

bool RPLidarS2::parseInfo(const uint8_t *d, uint16_t n, Info &info)
{
    if (n < 20) return false;
    info.model    = d[0];
    info.fwMinor  = d[1];
    info.fwMajor  = d[2];
    info.hardware = d[3];
    memcpy(info.serial, d + 4, 16);
    return true;
}

bool RPLidarS2::parseHealth(const uint8_t *d, uint16_t n, Health &health)
{
    if (n < 3) return false;
    health.status    = d[0];
    health.errorCode = u16le(d + 1);
    return true;
}

const uint8_t *RPLidarS2::confData(const uint8_t *d, uint16_t n, uint32_t type, uint16_t &dataLen)
{
    if (n < 4 || u32le(d) != type) return nullptr;
    dataLen = n - 4;
    return d + 4;
}

// ------------------------------------------------------------------ receive: codec (sl_lidarprotocol_codec.cpp)

void RPLidarS2::discardUntilIdle(uint32_t idleUs)
{
    discardInputFor(0);
    discardIdleUs_ = idleUs;
    lastByteUs_ = micros();
}

void RPLidarS2::discardInputFor(uint32_t ms)
{
    discardUntil_ = millis() + ms;
    discardIdleUs_ = 0;
    state_ = WAIT_SYNC1;
    ansType_ = 0;
    bufLen_ = 0;
    respReady_ = false;
}

bool RPLidarS2::response(uint8_t &type)
{
    if (!respReady_) return false;
    respReady_ = false;
    type = respType_;
    return true;
}

void RPLidarS2::poll()
{
    if (discardIdleUs_ && micros() - lastByteUs_ >= discardIdleUs_) discardIdleUs_ = 0;   // line quiet
    // Only what has arrived so far: bytes landing meanwhile wait for the next loop(), so poll() is bounded
    for (int n = port_.available(); n > 0; n--) {
        uint8_t b = port_.read();
        if (discardIdleUs_) {               // still discarding: this byte restarts the quiet time
            lastByteUs_ = micros();
            continue;
        }
        if ((int32_t)(millis() - discardUntil_) < 0) continue;
        if (state_ == STREAM) {
            if (ansType_ == ANS_DENSE) decodeDense(b);
            else                       decodeNormal(b);
        } else {
            codecByte(b);
        }
    }
}

void RPLidarS2::codecByte(uint8_t b)
{
    switch (state_) {
    case WAIT_SYNC1:
        if (b == ANS_SYNC1) state_ = WAIT_SYNC2;
        break;
    case WAIT_SYNC2:
        if (b == ANS_SYNC2)      { state_ = WAIT_LEN; hdrPos_ = 0; }
        else if (b != ANS_SYNC1) state_ = WAIT_SYNC1;
        break;
    case WAIT_LEN:
        hdr_[hdrPos_++] = b;
        if (hdrPos_ == 4) {
            uint32_t v = u32le(hdr_);
            len_  = v & 0x3FFFFFFF;
            loop_ = ((v >> 30) & ANS_FLAG_LOOP) != 0;
            state_ = WAIT_TYPE;
        }
        break;
    case WAIT_TYPE:
        type_ = b;
        if (loop_ && (b == ANS_DENSE || b == ANS_NORMAL)) {   // scan answer: stream from here on
            respType_ = b;
            respLen_ = 0;
            respReady_ = true;
            startStream(b);
        } else if (len_ == 0) {
            respType_ = b;
            respLen_ = 0;
            respReady_ = true;
            state_ = WAIT_SYNC1;
        } else {
            payloadPos_ = 0;
            state_ = WAIT_PAYLOAD;
        }
        break;
    case WAIT_PAYLOAD:
        if (payloadPos_ < RESP_MAX) resp_[payloadPos_] = b;
        if (++payloadPos_ == len_) {
            respType_ = type_;
            respLen_ = min<uint32_t>(len_, RESP_MAX);
            respReady_ = true;
            state_ = WAIT_SYNC1;
        }
        break;
    case STREAM:
        break;
    }
}

void RPLidarS2::startStream(uint8_t type)
{
    state_    = STREAM;
    ansType_  = type;
    bufLen_   = 0;
    havePrev_ = false;
    lastSync_ = false;
    good_ = bad_ = 0;
    lastPacketMs_ = millis();
}

// ------------------------------------------------------------------ receive: scan samples

// Standard sample, 5 bytes:
//   b0: quality[7:2] | !S[1] | S[0]
//   b1: angle_q6[6:0] | C[0]=1
//   b2: angle_q6[14:7]
//   b3..b4: distance_q2 (mm * 4)
void RPLidarS2::decodeNormal(uint8_t b)
{
    buf_[bufLen_++] = b;
    if (bufLen_ < 5) return;

    bool s    = buf_[0] & 0x01;
    bool notS = (buf_[0] >> 1) & 0x01;
    bool c    = buf_[1] & 0x01;
    uint16_t angleQ6 = (buf_[1] >> 1) | ((uint16_t)buf_[2] << 7);

    if (s == notS || !c || angleQ6 >= FULL_Q6) {   // out of sync, slide by one byte
        memmove(buf_, buf_ + 1, 4);
        bufLen_ = 4;
        bad_++;
        return;
    }
    bufLen_ = 0;
    good_++;
    lastPacketMs_ = millis();

    if (handler_) handler_(angleQ6, u16le(buf_ + 3) / 4, buf_[0] >> 2, s);
}

// Dense capsule, 84 bytes (handler_capsules.cpp, UnpackerHandler_DenseCapsuleNode):
//   b0: 0xA<<4 | checksum[3:0]    b1: 0x5<<4 | checksum[7:4]
//   b2..b3: start_angle_q6 (bit 15 = first capsule of a new measurement)
//   b4..b83: 40 x uint16 distance in mm
void RPLidarS2::decodeDense(uint8_t b)
{
    buf_[bufLen_++] = b;

    // Drop bytes until the buffer starts with a plausible header
    while (bufLen_ > 0 && ((buf_[0] >> 4) != 0xA || (bufLen_ > 1 && (buf_[1] >> 4) != 0x5))) {
        memmove(buf_, buf_ + 1, --bufLen_);
        havePrev_ = false;
    }
    if (bufLen_ < DENSE_LEN) return;

    uint8_t sum = 0;
    for (uint8_t i = 2; i < DENSE_LEN; i++) sum ^= buf_[i];
    if (sum != ((buf_[0] & 0x0F) | ((buf_[1] & 0x0F) << 4))) {
        // False header or damaged capsule: slide by one and resync on the next header
        bad_++;
        havePrev_ = false;
        memmove(buf_, buf_ + 1, --bufLen_);
        while (bufLen_ > 0 && ((buf_[0] >> 4) != 0xA || (bufLen_ > 1 && (buf_[1] >> 4) != 0x5)))
            memmove(buf_, buf_ + 1, --bufLen_);
        return;
    }
    good_++;
    lastPacketMs_ = millis();
    denseCapsule();
    bufLen_ = 0;
}

// Samples of the previous capsule are spread between its start angle and this one's
void RPLidarS2::denseCapsule()
{
    uint16_t startRaw = u16le(buf_ + 2);
    if (startRaw & 0x8000) havePrev_ = false;

    if (havePrev_ && handler_) {
        int32_t curQ8  = (int32_t)(startRaw & 0x7FFF) << 2;
        int32_t prevQ8 = (int32_t)(u16le(prevCap_ + 2) & 0x7FFF) << 2;
        int32_t diffQ8 = curQ8 - prevQ8;
        if (prevQ8 > curQ8) diffQ8 += 360 << 8;

        if (diffQ8 <= maxDiffQ8_) {
            int32_t incQ16   = (diffQ8 << 8) / DENSE_SAMPLES;
            int32_t angleQ16 = prevQ8 << 8;
            for (uint8_t k = 0; k < DENSE_SAMPLES; k++) {
                bool sync = ((angleQ16 + incQ16) % (360L << 16)) < (incQ16 << 1);
                bool newTurn = sync && !lastSync_;
                lastSync_ = sync;

                int32_t angleQ6 = (angleQ16 >> 10) % FULL_Q6;
                angleQ16 += incQ16;

                uint16_t dist = u16le(prevCap_ + 4 + 2 * k);
                handler_((uint16_t)angleQ6, dist, dist ? 0x2F : 0, newTurn);
            }
        }
    }
    memcpy(prevCap_, buf_, DENSE_LEN);
    havePrev_ = true;
}
