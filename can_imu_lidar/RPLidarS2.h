#pragma once

// Non-blocking RPLIDAR S2 driver for STM32duino, structured like the Slamtec SDK used by rplidar_ros:
//   send*/request*  only queue a command frame and return (no flush, no delay)
//   poll()          runs the protocol codec on whatever bytes arrived:
//                     A5 5A | len (30 bits) + mode (2 bits) | type | payload
//                   a normal answer becomes available through response(); an answer in LOOP mode
//                   (SCAN / EXPRESS_SCAN) switches to streaming and every sample goes to the SampleHandler.
// Nothing in here waits. The caller sequences commands and keeps its own timers (see the sketch).

#include <Arduino.h>

class RPLidarS2 {
public:
    struct Info {
        uint8_t model;
        uint8_t fwMinor;
        uint8_t fwMajor;
        uint8_t hardware;
        uint8_t serial[16];
    };

    struct Health {
        uint8_t  status;                    // 0 Good, 1 Warning, 2 Error
        uint16_t errorCode;
    };

    struct ScanMode {
        uint16_t id;
        float    usPerSample;
        uint32_t maxDistanceM;
        uint8_t  ansType;                   // ANS_NORMAL or ANS_DENSE
        char     name[32];
    };

    // angleQ6: angle in 1/64 degree, 0..23039 clockwise (integer: the F207 has no FPU)
    // distMm: 0 = no return, newTurn: first sample of a revolution
    typedef void (*SampleHandler)(uint16_t angleQ6, uint16_t distMm, uint8_t quality, bool newTurn);

    // Answer types
    static const uint8_t ANS_INFO   = 0x04;
    static const uint8_t ANS_HEALTH = 0x06;
    static const uint8_t ANS_CONF   = 0x20;
    static const uint8_t ANS_NORMAL = 0x81;
    static const uint8_t ANS_DENSE  = 0x85;

    // GET_LIDAR_CONF types
    static const uint32_t CONF_DESIRED_ROT_FREQ        = 0x01;
    static const uint32_t CONF_SCAN_MODE_US_PER_SAMPLE = 0x71;
    static const uint32_t CONF_SCAN_MODE_MAX_DISTANCE  = 0x74;
    static const uint32_t CONF_SCAN_MODE_ANS_TYPE      = 0x75;
    static const uint32_t CONF_SCAN_MODE_TYPICAL       = 0x7C;
    static const uint32_t CONF_SCAN_MODE_NAME          = 0x7F;

    explicit RPLidarS2(HardwareSerial &port) : port_(port) {}

    void begin(uint32_t baud = 1000000) { port_.begin(baud); }
    void onSample(SampleHandler h) { handler_ = h; }

    // Commands: queue the frame and return
    void sendStop();
    void sendReset();
    void requestInfo();
    void requestHealth();
    void requestConf(uint32_t type, int32_t mode = -1);    // mode >= 0 adds the scan mode id
    void sendMotorRpm(uint16_t rpm);
    void sendScan(const ScanMode &mode);                   // SCAN or EXPRESS_SCAN, per mode.ansType

    // Receive side
    void poll();                            // decode everything that has arrived
    void discardInputFor(uint32_t ms);      // drop bytes for a while, resets the codec
    void discardUntilIdle(uint32_t idleUs); // drop bytes until the line has been quiet for idleUs, resets the codec
    bool discarding() const { return discardIdleUs_ != 0; }
    bool response(uint8_t &type);           // true once per complete answer (for a scan: its descriptor)
    const uint8_t *responseData() const { return resp_; }
    uint16_t responseLen() const { return respLen_; }
    bool streaming() const { return ansType_ != 0; }

    // Answer parsers
    static bool parseInfo(const uint8_t *d, uint16_t n, Info &info);
    static bool parseHealth(const uint8_t *d, uint16_t n, Health &health);
    // GET_LIDAR_CONF answer: uint32 type echo + data. Returns the data or nullptr if it is for another type.
    static const uint8_t *confData(const uint8_t *d, uint16_t n, uint32_t type, uint16_t &dataLen);

    uint32_t goodPackets() const { return good_; }
    uint32_t badPackets() const { return bad_; }
    uint32_t lastPacketMs() const { return lastPacketMs_; }

private:
    static const uint8_t DENSE_LEN     = 84;
    static const uint8_t DENSE_SAMPLES = 40;
    static const uint16_t RESP_MAX     = 96;

    enum CodecState : uint8_t { WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN, WAIT_TYPE, WAIT_PAYLOAD, STREAM };

    void send(uint8_t cmd);
    void send(uint8_t cmd, const uint8_t *payload, uint8_t size);
    void codecByte(uint8_t b);
    void startStream(uint8_t type);
    void decodeNormal(uint8_t b);
    void decodeDense(uint8_t b);
    void denseCapsule();

    HardwareSerial &port_;
    SampleHandler handler_ = nullptr;

    // Codec
    CodecState state_ = WAIT_SYNC1;
    uint8_t  hdr_[4];
    uint8_t  hdrPos_ = 0;
    uint32_t len_ = 0;
    bool     loop_ = false;
    uint8_t  type_ = 0;
    uint8_t  resp_[RESP_MAX];
    uint16_t respLen_ = 0;
    uint16_t payloadPos_ = 0;
    bool     respReady_ = false;
    uint8_t  respType_ = 0;
    uint32_t discardUntil_ = 0;
    uint32_t discardIdleUs_ = 0;             // != 0: discarding until quiet this long
    uint32_t lastByteUs_ = 0;

    // Scan stream
    uint8_t  ansType_ = 0;
    uint8_t  buf_[DENSE_LEN];
    uint8_t  bufLen_ = 0;
    uint8_t  prevCap_[DENSE_LEN];
    bool     havePrev_ = false;
    bool     lastSync_ = false;
    int32_t  maxDiffQ8_ = 360 << 8;

    uint32_t good_ = 0;
    uint32_t bad_ = 0;
    uint32_t lastPacketMs_ = 0;

public:
    void setUsPerSample(float us);          // for the capsule angle check, from the scan mode
};
