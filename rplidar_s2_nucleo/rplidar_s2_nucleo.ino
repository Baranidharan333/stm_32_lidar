#include <Arduino.h>
#include "RPLidarS2.h"
#include "DmaSoftUart.h"

// RPLIDAR S2 (S2M1) -> NUCLEO-F207ZG, prints scan data on the USB serial (ST-Link VCP)
// for ros2_ws/lidar_reader.py (prints the scans, --ros publishes sensor_msgs/LaserScan on /scan).
//
// Wiring (LiDAR XH2.54-5P connector):
//   Red    VCC     -> external 5V supply (needs up to 1.5A at start, don't use the Nucleo 5V pin)
//   Black  GND     -> GND (common with Nucleo GND)
//   LiDAR TX (pin 2, output) -> D1 / PG14    LiDAR RX (pin 3, input) -> D0 / PG9
//   Blue   MOTOCTL -> PA3 (driven HIGH below) or tie to 3.3V
//
// That is the reverse of USART6 (RX PG9, TX PG14), which can't swap its pins, so the LiDAR link is
// DmaSoftUart: timer + DMA for both directions. Set LIDAR_ON_USART6 to 1 if the LiDAR TX is moved to D0
// and its RX to D1, to use the hardware UART instead.
#define LIDAR_ON_USART6 0
//
// Don't use PA1 (UART4 RX) on this board: SB13 connects it to the Ethernet PHY's 50 MHz RMII
// reference clock, which drowns the LiDAR's TX so no command ever gets an answer.
//
// No delay() and no waiting anywhere: loop() never blocks, status text is queued (LogQueue). The
// parameters are set and the scan is started once at power-up by a state machine: each step sends one
// command and moves on the moment the answer arrives. After STOP/RESET it goes on as soon as the line
// has been quiet for QUIET_US (and after RESET, as soon as HEALTH answers), not after fixed pauses.
// After that the board only receives. If the LiDAR stops sending, the board says so once and keeps
// listening; nothing is reset or restarted automatically.
// PC commands (send one character): r = start again (with RESET if needed), s = STOP.
//
// Output at 10 Hz (motor at 600 rpm = 10 revolutions/s), one block per 100 ms of data:
//   # SCAN <n> points=<valid samples> hz=<head speed in rev/s over the block>
//   <angle_deg>,<distance_mm>,<quality>      one line per degree that has a hit (nearest hit wins)
//   # END

// Setup steps (declared before any function: the Arduino IDE puts its generated prototypes there)
enum Step : uint8_t {
    ST_STOP, ST_INFO, ST_HEALTH, ST_RESET, ST_TYPICAL, ST_US, ST_ANS, ST_NAME, ST_RPM,
    ST_MOTOR, ST_SCAN, ST_STREAM, ST_IDLE
};

#if LIDAR_ON_USART6
Uart LidarSerial(PG9, PG14);                // (rx, tx) USART6, header pins D0/D1
#else
DmaSoftUart LidarSerial(14, 9);             // (rx PG14 = D1, tx PG9 = D0)
#endif
RPLidarS2 lidar(LidarSerial);

const uint32_t LIDAR_BAUD   = 1000000;      // S2: 1M, 8N1
const uint32_t PC_BAUD      = 921600;       // one 360-line block is ~5 KB, 10 blocks/s fit
const uint32_t MOTOCTL_PIN  = PA3;
const uint32_t PERIOD_MS    = 100;          // output rate: 10 Hz
const uint32_t STOPPED_MS   = 500;          // no packet for this long = stream stopped
const uint32_t ANSWER_MS    = 100;          // setup step timeout (the S2 answers within a few ms)
const uint32_t QUIET_US     = 3000;         // after STOP/RESET: go on once the line is quiet this long
                                            // (a DenseBoost stream never pauses more than ~0.4 ms)
const uint32_t REBOOT_MS    = 3000;         // after RESET: poll HEALTH until it answers, at most this long
const uint32_t REBOOT_POLL  = 20;           // HEALTH poll interval while the LiDAR reboots
const uint32_t RPM_PAUSE    = 1;            // after the motor speed command (SDK uses 10 ms)
const uint32_t SPINUP_MS    = 5000;         // first capsule can take ~2.3 s while the motor spins up

// ------------------------------------------------------------------ status text, never blocks

// Status messages go into this queue; pumpOutput() sends them between value blocks as the USB serial
// TX buffer has room, so a print never stalls loop(). If the queue is full the text is dropped.
class LogQueue : public Print {
public:
    size_t write(uint8_t c) override {
        uint16_t next = (head + 1) % sizeof(buf);
        if (next == tail) return 0;
        buf[head] = c;
        head = next;
        return 1;
    }
    using Print::write;
    bool empty() const { return head == tail; }
    // Move as much as the serial TX buffer takes right now
    void drainTo(HardwareSerial &out) {
        while (!empty()) {
            int room = out.availableForWrite();
            if (room <= 0) return;
            uint16_t chunk = (head > tail ? head : sizeof(buf)) - tail;
            if (chunk > (uint16_t)room) chunk = room;
            out.write(buf + tail, chunk);
            tail = (tail + chunk) % sizeof(buf);
        }
    }
private:
    uint8_t  buf[2048];
    uint16_t head = 0, tail = 0;
};
LogQueue Log;

// ------------------------------------------------------------------ values, one bin per degree

uint16_t binDist[360];
uint8_t  binQual[360];
uint16_t binPoints = 0;
uint32_t binTravel = 0;                     // degrees the head turned while this block was collected

uint16_t outDist[360];                      // block being printed, as the TX buffer has room
uint8_t  outQual[360];
int      outIndex = -1;                     // -1 = nothing to print, -2 = header pending
uint32_t outBlockNo = 0;
uint16_t outPoints = 0;
uint32_t outHz10 = 0;                       // head speed in 0.1 rev/s (integer: no FPU)
uint32_t blockCount = 0;
uint32_t nextOutputMs = 0;

int16_t  prevDeg = -1;
uint32_t runSamples = 0;
uint32_t runTravel = 0;
uint32_t usPerSampleQ8 = 31 * 256;          // of the running scan mode, 1/256 us

void clearBins()
{
    memset(binDist, 0, sizeof(binDist));
    memset(binQual, 0, sizeof(binQual));
    binPoints = 0;
    binTravel = 0;
}

void onSample(uint16_t angleQ6, uint16_t distMm, uint8_t quality, bool newTurn)
{
    (void)newTurn;
    runSamples++;
    int16_t d = angleQ6 >> 6;               // whole degrees, integer only (no FPU on the F207)
    if (prevDeg >= 0) {
        int16_t step = d - prevDeg;
        if (step < 0) step += 360;
        if (step < 90) { binTravel += step; runTravel += step; }   // bigger jumps are glitches
    }
    prevDeg = d;
    if (distMm == 0) return;                // no return for this sample

    uint16_t deg = d < 360 ? d : 0;
    if (binDist[deg] == 0 || distMm < binDist[deg]) {
        binDist[deg] = distMm;
        binQual[deg] = quality;
    }
    binPoints++;
}

// Hand the bins to the printer. If the previous block is still going out, keep collecting and retry.
void emitBins(uint32_t periodMs)
{
    if (outIndex != -1 || binPoints == 0) return;
    memcpy(outDist, binDist, sizeof(binDist));
    memcpy(outQual, binQual, sizeof(binQual));
    outBlockNo = ++blockCount;
    outPoints = binPoints;
    outHz10 = periodMs ? binTravel * 10000UL / (360UL * periodMs) : 0;
    outIndex = -2;
    clearBins();
}

// Unsigned to decimal, returns the end (snprintf is slow on this CPU)
static char *putU(char *p, uint32_t v)
{
    char tmp[10];
    int n = 0;
    do { tmp[n++] = '0' + v % 10; v /= 10; } while (v);
    while (n) *p++ = tmp[--n];
    return p;
}

// Emit lines while the TX buffer has room, never waits. At most LINES_PER_PASS per call so one
// loop() pass stays short; the rest goes out on the next passes.
const int LINES_PER_PASS = 4;
void pumpOutput()
{
    if (outIndex == -1 || outIndex == -2) {     // status text only between blocks, never inside one
        Log.drainTo(Serial);
        if (!Log.empty()) return;               // finish the text first, then start the next block
    }
    char line[48];
    for (int lines = 0; lines < LINES_PER_PASS && outIndex != -1 &&
                        Serial.availableForWrite() >= (int)sizeof(line); lines++) {
        if (outIndex == -2) {
            char *p = line;
            memcpy(p, "# SCAN ", 7);          p = putU(p + 7, outBlockNo);
            memcpy(p, " points=", 8);         p = putU(p + 8, outPoints);
            memcpy(p, " hz=", 4);             p = putU(p + 4, outHz10 / 10);
            *p++ = '.';                       p = putU(p, outHz10 % 10);
            *p++ = '\r';
            *p++ = '\n';
            Serial.write((const uint8_t *)line, p - line);
            outIndex = 0;
            continue;
        }
        while (outIndex < 360 && outDist[outIndex] == 0) outIndex++;
        if (outIndex >= 360) {
            Serial.write("# END\r\n");
            outIndex = -1;
            return;
        }
        char *p = putU(line, outIndex);
        *p++ = ',';
        p = putU(p, outDist[outIndex]);
        *p++ = ',';
        p = putU(p, outQual[outIndex]);
        *p++ = '\r';
        *p++ = '\n';
        Serial.write((const uint8_t *)line, p - line);
        outIndex++;
    }
}

// ------------------------------------------------------------------ one-time setup, as a state machine

Step     step = ST_IDLE;
uint32_t stepMs = 0;
uint8_t  resets = 0;
uint32_t rebootUntil = 0;                   // != 0: LiDAR rebooting after RESET, poll HEALTH                        // RESETs sent in this start
RPLidarS2::ScanMode mode;
uint16_t rpm = 600;
bool     streaming = false;                 // packets arriving
bool     stopReported = false;
uint32_t lastStatusMs = 0;
uint32_t loopMaxUs = 0;                     // longest loop() pass since the scan started

void enterStep(Step s)
{
    step = s;
    stepMs = millis();
    switch (s) {
    case ST_STOP:    lidar.sendStop(); lidar.discardUntilIdle(QUIET_US); break;
    case ST_INFO:    lidar.requestInfo(); break;
    case ST_HEALTH:  lidar.requestHealth(); break;
    case ST_RESET:   lidar.sendReset(); lidar.discardUntilIdle(QUIET_US); resets++;
                     rebootUntil = millis() + REBOOT_MS; break;
    case ST_TYPICAL: lidar.requestConf(RPLidarS2::CONF_SCAN_MODE_TYPICAL); break;
    case ST_US:      lidar.requestConf(RPLidarS2::CONF_SCAN_MODE_US_PER_SAMPLE, mode.id); break;
    case ST_ANS:     lidar.requestConf(RPLidarS2::CONF_SCAN_MODE_ANS_TYPE, mode.id); break;
    case ST_NAME:    lidar.requestConf(RPLidarS2::CONF_SCAN_MODE_NAME, mode.id); break;
    case ST_RPM:     lidar.requestConf(RPLidarS2::CONF_DESIRED_ROT_FREQ); break;
    case ST_MOTOR:   lidar.sendMotorRpm(rpm); break;
    case ST_SCAN:    lidar.sendScan(mode); break;
    case ST_STREAM:
        Log.println("Scanning... (receiving only, values at 10 Hz)");
        clearBins();
        prevDeg = -1;
        runSamples = runTravel = 0;
        streaming = stopReported = false;
        nextOutputMs = millis() + PERIOD_MS;
        loopMaxUs = 0;
        break;
    case ST_IDLE: break;
    }
}

void startLidar()
{
    resets = 0;
    mode = {0, 0, 0, RPLidarS2::ANS_NORMAL, "Standard"};
    enterStep(ST_STOP);
}

void printHealth(const RPLidarS2::Health &h)
{
    const char *names[] = {"Good", "Warning", "Error"};
    Log.print("Health: "); Log.print(h.status <= 2 ? names[h.status] : "?");
    Log.print("  code: 0x"); Log.print(h.errorCode, HEX);
    // Slamtec RPLIDAR FAQ Q7: "S2 Lidar jumps health error code 4" -> "Insufficient Lidar voltage."
    if (h.status == 2 && h.errorCode == 4) Log.print("  (Slamtec FAQ: insufficient LiDAR voltage)");
    Log.println();
}

// Answer of a GET_LIDAR_CONF step, or nullptr if it hasn't arrived (yet)
const uint8_t *confAnswer(bool got, uint8_t type, uint32_t conf, uint16_t &n)
{
    if (!got || type != RPLidarS2::ANS_CONF) return nullptr;
    return RPLidarS2::confData(lidar.responseData(), lidar.responseLen(), conf, n);
}

void runSetup()
{
    uint32_t elapsed = millis() - stepMs;
    uint8_t type = 0;
    bool got = lidar.response(type);
    const uint8_t *d;
    uint16_t n;

    switch (step) {
    case ST_STOP:
        if (!lidar.discarding()) enterStep(ST_INFO);        // stream stopped: line quiet
        break;

    case ST_INFO:
        if (got && type == RPLidarS2::ANS_INFO) {
            RPLidarS2::Info info;
            if (RPLidarS2::parseInfo(lidar.responseData(), lidar.responseLen(), info)) {
                char sn[33], line[96];
                for (int i = 0; i < 16; i++) snprintf(sn + 2 * i, 3, "%02X", info.serial[i]);
                snprintf(line, sizeof(line), "Model: 0x%02X  FW: %u.%02u  HW: %u  SN: %s",
                         info.model, info.fwMajor, info.fwMinor, info.hardware, sn);
                Log.println(line);
            }
            enterStep(ST_HEALTH);
        } else if (elapsed > ANSWER_MS) {
            Log.println("INFO: no response (check wiring / 5V supply), trying again");
            enterStep(ST_STOP);
        }
        break;

    case ST_HEALTH:
        if (got && type == RPLidarS2::ANS_HEALTH) {
            rebootUntil = 0;
            RPLidarS2::Health h = {0, 0};
            if (RPLidarS2::parseHealth(lidar.responseData(), lidar.responseLen(), h)) printHealth(h);
            if (h.status != 2) {
                enterStep(ST_TYPICAL);
            } else if (resets < 2) {        // Error: the LiDAR refuses to scan until it is reset
                Log.println("LiDAR in error state (protection stop: scan speed unstable/too slow), sending RESET");
                enterStep(ST_RESET);
            } else {
                Log.println("Still Error after RESET: power-cycle the LiDAR, then send 'r'");
                enterStep(ST_IDLE);
            }
        } else if (rebootUntil && (int32_t)(millis() - rebootUntil) < 0) {
            if (elapsed >= REBOOT_POLL) enterStep(ST_HEALTH);  // still rebooting: ask again
        } else if (elapsed > ANSWER_MS) {
            rebootUntil = 0;
            Log.println("HEALTH: no response, trying again");
            enterStep(ST_STOP);
        }
        break;

    case ST_RESET:
        if (!lidar.discarding()) enterStep(ST_HEALTH);      // then poll HEALTH until it has rebooted
        break;

    case ST_TYPICAL:
        if ((d = confAnswer(got, type, RPLidarS2::CONF_SCAN_MODE_TYPICAL, n)) && n >= 2) {
            mode.id = d[0] | (d[1] << 8);
            enterStep(ST_US);
        } else if (elapsed > ANSWER_MS) {
            Log.println("Scan mode query unanswered, using Standard");
            enterStep(ST_RPM);
        }
        break;

    case ST_US:
        if ((d = confAnswer(got, type, RPLidarS2::CONF_SCAN_MODE_US_PER_SAMPLE, n)) && n >= 4) {
            mode.usPerSample = (d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24)) / 256.0f;
            enterStep(ST_ANS);
        } else if (elapsed > ANSWER_MS) enterStep(ST_STOP);
        break;

    case ST_ANS:
        if ((d = confAnswer(got, type, RPLidarS2::CONF_SCAN_MODE_ANS_TYPE, n)) && n >= 1) {
            mode.ansType = d[0];
            enterStep(ST_NAME);
        } else if (elapsed > ANSWER_MS) enterStep(ST_STOP);
        break;

    case ST_NAME:
        if ((d = confAnswer(got, type, RPLidarS2::CONF_SCAN_MODE_NAME, n))) {
            n = min<uint16_t>(n, sizeof(mode.name) - 1);
            memcpy(mode.name, d, n);
            mode.name[n] = 0;
            enterStep(ST_RPM);
        } else if (elapsed > ANSWER_MS) enterStep(ST_RPM);
        break;

    case ST_RPM:
        if ((d = confAnswer(got, type, RPLidarS2::CONF_DESIRED_ROT_FREQ, n)) && n >= 2) {
            rpm = d[0] | (d[1] << 8);
        } else if (elapsed <= ANSWER_MS) {
            break;
        }
        Log.print("Mode: "); Log.print(mode.name);
        Log.print(" (id "); Log.print(mode.id);
        Log.print(", "); Log.print(mode.usPerSample, 1); Log.print(" us/sample)  Motor: ");
        Log.print(rpm); Log.println(" rpm");
        if (mode.usPerSample > 0) usPerSampleQ8 = (uint32_t)(mode.usPerSample * 256);   // once, at setup
        enterStep(ST_MOTOR);
        break;

    case ST_MOTOR:
        if (elapsed >= RPM_PAUSE) enterStep(ST_SCAN);
        break;

    case ST_SCAN:
        if (got && type == mode.ansType) {
            enterStep(ST_STREAM);
        } else if (elapsed > ANSWER_MS) {
            Log.println("SCAN: no descriptor, trying again");
            enterStep(ST_STOP);
        }
        break;

    case ST_STREAM:
    case ST_IDLE:
        break;
    }
}

// ------------------------------------------------------------------ receiving

void runStream()
{
    uint32_t now = millis();
    bool fresh = lidar.goodPackets() > 0 && now - lidar.lastPacketMs() < STOPPED_MS;

    if ((int32_t)(now - nextOutputMs) >= 0) {           // 10 Hz: send whatever arrived in the last 100 ms
        emitBins(PERIOD_MS + (now - nextOutputMs));
        nextOutputMs += PERIOD_MS;
        if ((int32_t)(now - nextOutputMs) >= 0) nextOutputMs = now + PERIOD_MS;   // fell behind
    }

    if (fresh) {
        streaming = true;
        stopReported = false;
    } else if (!stopReported && (streaming || now - stepMs > SPINUP_MS)) {
        emitBins(0);
        // rev/s * 100 = degrees / 360 / (samples * us per sample), all integer
        uint64_t dataUsQ8 = (uint64_t)runSamples * usPerSampleQ8;
        uint32_t centiRev = dataUsQ8 ? (uint32_t)((uint64_t)runTravel * 100ULL * 1000000ULL * 256ULL / (360ULL * dataUsQ8)) : 0;
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "# LiDAR stopped sending (%lu packets, %lu samples, head %lu.%02lu rev/s, target 10; "
                 "longest loop %lu us). Still listening; send 'r' to start again.\r\n",
                 (unsigned long)lidar.goodPackets(), (unsigned long)runSamples,
                 (unsigned long)(centiRev / 100), (unsigned long)(centiRev % 100), (unsigned long)loopMaxUs);
        Log.print(msg);
        streaming = false;
        stopReported = true;
        lastStatusMs = now;
    } else if (stopReported && now - lastStatusMs >= 1000) {
        // Once a second while nothing arrives, so the PC side sees why it gets no values
        lastStatusMs = now;
        Log.print("# waiting: no data from the LiDAR for ");
        Log.print((now - lidar.lastPacketMs()) / 1000);
        Log.println(" s (it stopped itself). Send 'r' to start again.");
    }
}

void handlePcCommand()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') {
            Log.println("# start requested");
            startLidar();
        } else if (c == 's' || c == 'S') {
            lidar.sendStop();
            lidar.discardUntilIdle(QUIET_US);
            enterStep(ST_IDLE);
            Log.println("# stopped, send 'r' to start again");
        }
    }
}

void setup()
{
    Serial.begin(PC_BAUD);
    pinMode(MOTOCTL_PIN, OUTPUT);
    digitalWrite(MOTOCTL_PIN, HIGH);

    Log.println("\n=== RPLIDAR S2 on NUCLEO-F207ZG @ 1M ===");
    lidar.onSample(onSample);
    lidar.begin(LIDAR_BAUD);
    startLidar();
}

void loop()
{
    uint32_t t0 = micros();
    lidar.poll();
    if (step == ST_STREAM) runStream();
    else runSetup();
    pumpOutput();
    handlePcCommand();
    uint32_t dt = micros() - t0;
    if (dt > loopMaxUs) loopMaxUs = dt;
}
