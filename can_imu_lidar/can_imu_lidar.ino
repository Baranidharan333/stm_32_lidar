#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <LwIP.h>                           // before STM32Ethernet, so the builder finds STM32duino_LwIP
#include <STM32Ethernet.h>
#include "RPLidarS2.h"
#include "DmaSoftUart.h"
#include "twai.h"

// RPLIDAR S2 + BNO085 IMU + CAN bus on a NUCLEO-F207ZG, all over the on-board Ethernet port as UDP
// for can_imu_lidar_node.py (/scan, /imu/data, CAN <-> virtual can0).
// LiDAR + IMU part = lidar_imu_nucleo_udp, CAN part = cansend/can_udp (TWAI driver on bxCAN).
//
// Wiring:
//   LiDAR (XH2.54-5P): Red VCC -> external 5V (up to 1.5A at start), Black GND -> GND,
//                      LiDAR TX -> D1 / PG14, LiDAR RX -> D0 / PG9 (the S2 starts its motor on SCAN)
//   BNO085 (I2C 0x4A): SDA -> D14 / PB9, SCL -> D15 / PB8, INT -> D7 / PF13, 3.3V, GND
//   CAN transceiver:   PD1 (CAN1_TX, CN9 pin 27) -> TXD, PD0 (CAN1_RX, CN9 pin 25) <- RXD,
//                      CANH / CANL -> bus (120 ohm at each end), GND common with the motors
//   RJ45 of the Nucleo -> same switch/router as the PC
#define LIDAR_ON_USART6 0
//
// Network: static address, no DHCP (a DHCP request would block setup for seconds).
//   Board: BOARD_IP, listens for commands on BOARD_PORT.
//   Data goes to the PC that last sent the board a packet (the ROS node sends 'h' every second),
//   to the address/port it sent from. With no PC heard for PEER_TIMEOUT_MS it is broadcast on the
//   subnet to PC_PORT.
//
// UDP datagrams from the board:
//   scan   1096 bytes, little-endian:
//            0  char[4]  "RPS2"
//            4  uint8    version (1)
//            5  uint8    reserved
//            6  uint16   points      valid samples in the block
//            8  uint32   block       running block number
//           12  uint16   hz10        head speed in 0.1 rev/s
//           14  uint16   hits        degrees with a hit
//           16  uint16[360] distance in mm per degree (0 = no hit), clockwise as the LiDAR reports
//          736  uint8[360]  quality per degree
//   imu    56 bytes at 100 Hz, the stm_imu.ino packet (Python "<2sBBII10fI"):
//            0xAA 0x55, version 3, reserved, uint32 sequence, uint32 device time us,
//            float qw qx qy qz (game rotation vector), gx gy gz (rad/s), ax ay az (m/s^2),
//            uint32 CRC32 of the first 52 bytes (= zlib.crc32)
//   can    'C' 'B' <type> <count> + payload:
//            type 1: count x 16-byte SocketCAN struct can_frame received from the bus
//                    { uint32 can_id (EFF 0x80000000 / RTR 0x40000000 flags), uint8 len,
//                      uint8 pad[3], uint8 data[8] }
//            type 2: CAN status text (CANSTAT once a second, CANALERT, ERROR)
//   text   one status line (board startup, health, "LiDAR stopped sending", ...)
// From the PC:
//   'C' 'B' 1 <count> + count x can_frame  -> put on the CAN bus
//   anything else (the hello)               -> only tells the board where to send
// Every datagram from the PC registers it as the receiver. The LiDAR is set up and started once at
// power-up; to start it again, reset the board. CAN bus-off is recovered automatically.
// The USB serial (19200 baud) prints the startup lines and then only faults: when a fault appears,
// every 10 s while it stays, and when it clears (NET / LIDAR / IMU / CAN / LOOP FAULT lines).
//
// No delay() in loop(). Timing-critical parts do not depend on how long a loop() pass takes:
//   - lwIP runs in the library's 1 ms TIM14 interrupt; each datagram from the PC is taken there
//     (onUdpArrival), so CAN frames go into the TWAI TX queue at once and are never overwritten.
//   - LiDAR samples (TIM8 + DMA2) are decoded in the DMA half/complete interrupt every ~4 ms.
//   - CAN RX is interrupt-driven into a 128-frame queue, forwarded to the PC twice per pass.
// The IMU is read at most IMU_EVENTS_PER_PASS events (~1 ms of I2C each) per pass.
// begin_I2C() holds ~300 ms, so the IMU is only started in setup().
//
// LiDAR output at 10 Hz, one datagram per 100 ms of data.

// Setup steps (declared before any function: the Arduino IDE puts its generated prototypes there)
enum Step : uint8_t {
    ST_STOP, ST_INFO, ST_HEALTH, ST_RESET, ST_TYPICAL, ST_US, ST_ANS, ST_NAME, ST_SCAN, ST_STREAM, ST_IDLE
};

struct __attribute__((packed)) ScanPacket {
    char     magic[4];
    uint8_t  version;
    uint8_t  reserved;
    uint16_t points;
    uint32_t block;
    uint16_t hz10;
    uint16_t hits;
    uint16_t dist[360];
    uint8_t  qual[360];
};
static_assert(sizeof(ScanPacket) == 1096, "ScanPacket layout");

struct __attribute__((packed)) IMUPacket {
    uint8_t  sync1, sync2;                  // 0xAA 0x55
    uint8_t  version;                       // 3
    uint8_t  reserved;
    uint32_t sequence;
    uint32_t device_time_us;
    float    qw, qx, qy, qz;                // game rotation vector
    float    gx, gy, gz;                    // calibrated gyroscope, rad/s
    float    ax, ay, az;                    // accelerometer, m/s^2
    uint32_t crc32;                         // of everything before it
};
static_assert(sizeof(IMUPacket) == 56, "IMUPacket layout");

#if LIDAR_ON_USART6
Uart LidarSerial(PG9, PG14);                // (rx, tx) USART6, header pins D0/D1
#else
DmaSoftUart LidarSerial(14, 9);             // (rx PG14 = D1, tx PG9 = D0)
#endif
RPLidarS2 lidar(LidarSerial);

const uint32_t LIDAR_BAUD   = 1000000;      // S2: 1M, 8N1
const uint32_t PC_BAUD      = 19200;        // USB serial: status text and network debug only
const uint32_t PERIOD_MS    = 100;          // output rate: 10 Hz
const uint32_t STOPPED_MS   = 500;          // no packet for this long = stream stopped
const uint32_t ANSWER_MS    = 100;          // setup step timeout (the S2 answers within a few ms)
const uint32_t QUIET_US     = 3000;         // after STOP/RESET: go on once the line is quiet this long
const uint32_t REBOOT_MS    = 3000;         // after RESET: poll HEALTH until it answers, at most this long
const uint32_t REBOOT_POLL  = 20;           // HEALTH poll interval while the LiDAR reboots
const uint32_t SPINUP_MS    = 5000;         // first capsule can take ~2.3 s while the motor spins up

// IMU (BNO085)
const uint32_t BNO_SDA      = D14;
const uint32_t BNO_SCL      = D15;
const uint32_t BNO_INT_PIN  = D7;
const uint8_t  BNO_ADDR     = 0x4A;
// BNO085 RST pin. -1 = not wired: after a board reset the BNO085 can stay stuck until it is power-cycled.
// Wire its RST to a free pin (e.g. D6 / PE9) and put that pin here: the board then hard-resets it at start.
const int      BNO_RST_PIN  = -1;
const uint32_t IMU_REPORT_US = 10000;       // 100 Hz reports
const uint32_t IMU_SEND_MS  = 10;           // 100 Hz packets
const uint8_t  IMU_EVENTS_PER_PASS = 2;
const uint32_t IMU_FROZEN_MS = 1000;        // no event this long -> enable the reports again

// Network (PC is on 192.168.200.0/24; change all four if the board goes on another network)
IPAddress BOARD_IP(192, 168, 200, 177);
IPAddress NETMASK(255, 255, 255, 0);
IPAddress GATEWAY(192, 168, 200, 1);
IPAddress BROADCAST_IP(192, 168, 200, 255);
const uint16_t BOARD_PORT      = 5600;      // board listens here
const uint16_t PC_PORT         = 5601;      // broadcast destination while no PC is known
const uint32_t PEER_TIMEOUT_MS = 5000;      // PC silent this long -> back to broadcast
const uint32_t FAULT_CHECK_MS  = 1000;      // fault check; the USB serial only prints faults
const uint32_t FAULT_REPEAT_MS = 10000;     // a fault that stays is printed again this often
const uint32_t LOOP_SLOW_US    = 20000;     // a loop() pass longer than this is reported

// CAN (DaMiao motors default to 1 Mbit/s)
const uint32_t CAN_TX_PIN      = PD1;
const uint32_t CAN_RX_PIN      = PD0;
const uint32_t CAN_STATUS_MS   = 1000;      // CANSTAT heartbeat to the PC

EthernetUDP Udp;
bool      netReady = false;

// The PC to send to. Written by onUdpArrival() in the lwIP (TIM14) interrupt, read in loop().
volatile bool     havePeer = false;
volatile uint32_t peerAddr = 0;
volatile uint16_t peerPort = PC_PORT;
volatile uint32_t peerSeenMs = 0;
volatile bool     peerIsNew = false;        // loop() prints "# PC ... connected"

// Counters for the fault check
uint32_t udpSent = 0, udpFailed = 0, udpNoLink = 0;
volatile uint32_t udpReceived = 0;

void udpSend(const uint8_t *data, size_t len)
{
    if (!netReady) return;
    if (Ethernet.linkStatus() != LinkON) { udpNoLink++; return; }
    noInterrupts();
    bool unicast = havePeer && millis() - peerSeenMs < PEER_TIMEOUT_MS;
    IPAddress ip(peerAddr);
    uint16_t port = peerPort;
    interrupts();
    bool ok = Udp.beginPacket(unicast ? ip : BROADCAST_IP, unicast ? port : PC_PORT);
    if (ok) ok = Udp.write(data, len) == len;
    if (ok) ok = Udp.endPacket();           // hands the frame to the MAC DMA, doesn't wait
    if (ok) udpSent++; else udpFailed++;
}

// ------------------------------------------------------------------ status text, never blocks

// Status messages go into this queue; pumpOutput() sends them to the USB serial as its TX buffer has
// room, so a print never stalls loop(). Each complete line is also sent as one UDP datagram.
class LogQueue : public Print {
public:
    size_t write(uint8_t c) override {
        if (c == '\n') {
            if (toUdp) udpSend(line, lineLen);
            lineLen = 0;
        } else if (c != '\r' && lineLen < sizeof(line)) {
            line[lineLen++] = c;
        }
        uint16_t next = (head + 1) % sizeof(buf);
        if (next == tail) return 0;
        buf[head] = c;
        head = next;
        return 1;
    }
    using Print::write;
    bool toUdp = true;                      // false: USB serial only (network debug lines)
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
    uint8_t  line[200];
    uint16_t lineLen = 0;
};
LogQueue Log;

// ------------------------------------------------------------------ network debug (USB serial only)

const uint8_t PHY_ADDR = 0;                 // LAN8742A on the Nucleo-144

// One MDIO read of a PHY register, or -1 if the MAC never finishes it (ETH clock off / PHY dead).
// Interrupts are off for the ~30 us it takes, so lwIP's own PHY polling (TIM14 IRQ) can't cut in.
int32_t phyRead(uint8_t reg)
{
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_ETHMACEN)) return -1;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t v = -1;
    if (!(ETH->MACMIIAR & ETH_MACMIIAR_MB)) {
        uint32_t cr = ETH->MACMIIAR & ETH_MACMIIAR_CR;
        ETH->MACMIIAR = ((uint32_t)PHY_ADDR << 11) | ((uint32_t)reg << 6) | cr | ETH_MACMIIAR_MB;
        for (uint32_t i = 0; i < 50000 && (ETH->MACMIIAR & ETH_MACMIIAR_MB); i++) {}
        if (!(ETH->MACMIIAR & ETH_MACMIIAR_MB)) v = ETH->MACMIIDR & 0xFFFF;
    }
    if (!primask) __enable_irq();
    return v;
}

void logHex(int32_t v)
{
    if (v < 0) { Log.print("none"); return; }
    Log.print("0x");
    Log.print((uint32_t)v, HEX);
}

// Prints a fault when it appears, again every FAULT_REPEAT_MS while it stays, and once when it clears.
// Faults also go to the PC as text (ROS log).
struct Fault {
    bool active = false;
    uint32_t lastMs = 0;
    void update(bool now, const char *text, const char *cleared) {
        uint32_t t = millis();
        if (now && (!active || t - lastMs >= FAULT_REPEAT_MS)) {
            Log.println(text);
            lastMs = t;
        } else if (!now && active && cleared) {
            Log.println(cleared);
        }
        active = now;
    }
};

// Once after Ethernet.begin(): RMII pins and PHY identity, printed only if something is wrong
void netSetupReport()
{
    // RMII pins of the NUCLEO-F207ZG, all must be alternate function 11 (ETH)
    struct EthPin { GPIO_TypeDef *port; char name; uint8_t pin; const char *sig; };
    const EthPin pins[] = {
        {GPIOA, 'A', 1, "REF_CLK"}, {GPIOA, 'A', 2, "MDIO"}, {GPIOC, 'C', 1, "MDC"},
        {GPIOA, 'A', 7, "CRS_DV"}, {GPIOC, 'C', 4, "RXD0"}, {GPIOC, 'C', 5, "RXD1"},
        {GPIOG, 'G', 11, "TX_EN"}, {GPIOG, 'G', 13, "TXD0"}, {GPIOB, 'B', 13, "TXD1"},
    };
    for (const EthPin &p : pins) {
        uint32_t moder = (p.port->MODER >> (2 * p.pin)) & 3;
        uint32_t af = (p.port->AFR[p.pin / 8] >> (4 * (p.pin % 8))) & 0xF;
        if (moder != 2 || af != 11) {
            Log.print("NET FAULT: pin P"); Log.print(p.name); Log.print(p.pin); Log.print(" ("); Log.print(p.sig);
            Log.print(") not set to ETH: mode "); Log.print(moder); Log.print(" AF "); Log.println(af);
        }
    }

    int32_t id1 = phyRead(2), id2 = phyRead(3);
    if (!(id1 == 0x0007 && (id2 & 0xFFF0) == 0xC130)) {
        Log.print("NET FAULT: PHY id "); logHex(id1); Log.print(" "); logHex(id2);
        Log.print(", expected 0x7 0xC13x: PHY not answering (ETH clock ");
        Log.print(RCC->AHB1ENR & RCC_AHB1ENR_ETHMACEN ? "on" : "OFF");
        Log.println(", jumpers SB13 etc.)");
    }
}

Fault faultPhy, faultLink, faultCrc, faultUdp;
uint32_t prevCrcErrors = 0, prevUdpFailed = 0;

// Once a second: PHY, link, CRC errors, failed sends. Prints nothing while all is well.
void netFaults()
{
    char s[112];
    int32_t bsr = phyRead(1);
    faultPhy.update(bsr < 0, "NET FAULT: PHY does not answer on MDIO (ETH clock / jumper SB13)",
                    "NET: PHY answers again");
    faultLink.update(bsr >= 0 && !(bsr & 0x0004),
                     "NET FAULT: no Ethernet link - cable unplugged, or switch/PC port off",
                     "NET: Ethernet link up again");

    uint32_t crc = ETH->MMCRFCECR;
    snprintf(s, sizeof(s), "NET FAULT: %lu CRC errors so far (RMII clock/signal, PA1 50 MHz line?)",
             (unsigned long)crc);
    faultCrc.update(crc != prevCrcErrors, s, nullptr);
    prevCrcErrors = crc;

    snprintf(s, sizeof(s), "NET FAULT: %lu UDP sends failed so far (lwIP out of buffers?)",
             (unsigned long)udpFailed);
    faultUdp.update(udpFailed != prevUdpFailed, s, nullptr);
    prevUdpFailed = udpFailed;
}

// ------------------------------------------------------------------ values, one bin per degree

ScanPacket pkt;                             // bins being collected, sent as they are
uint32_t binTravel = 0;                     // degrees the head turned while this block was collected
uint32_t blockCount = 0;
uint32_t nextOutputMs = 0;

int16_t  prevDeg = -1;
uint32_t runSamples = 0;
uint32_t runTravel = 0;
uint32_t usPerSampleQ8 = 31 * 256;          // of the running scan mode, 1/256 us

void clearBins()
{
    memset(pkt.dist, 0, sizeof(pkt.dist));
    memset(pkt.qual, 0, sizeof(pkt.qual));
    pkt.points = 0;
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
    if (pkt.dist[deg] == 0 || distMm < pkt.dist[deg]) {
        pkt.dist[deg] = distMm;
        pkt.qual[deg] = quality;
    }
    pkt.points++;
}

// Send the collected block as one datagram and start the next one
void emitBins(uint32_t periodMs)
{
    if (pkt.points == 0) return;
    memcpy(pkt.magic, "RPS2", 4);
    pkt.version = 1;
    pkt.reserved = 0;
    pkt.block = ++blockCount;
    pkt.hz10 = periodMs ? binTravel * 10000UL / (360UL * periodMs) : 0;
    pkt.hits = 0;
    for (int i = 0; i < 360; i++) if (pkt.dist[i]) pkt.hits++;
    udpSend((const uint8_t *)&pkt, sizeof(pkt));
    clearBins();
}

void pumpOutput()
{
    Log.drainTo(Serial);
}

// ------------------------------------------------------------------ one-time setup, as a state machine

Step     step = ST_IDLE;
uint32_t stepMs = 0;
uint8_t  resets = 0;
uint32_t rebootUntil = 0;                   // != 0: LiDAR rebooting after RESET, poll HEALTH
RPLidarS2::ScanMode mode;
bool     streaming = false;                 // packets arriving
bool     stopReported = false;
uint32_t lastStatusMs = 0;
uint32_t loopMaxUs = 0;                     // longest loop() pass in the last second

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
    case ST_SCAN:    lidar.sendScan(mode); break;
    case ST_STREAM:
        Log.println("Scanning... (receiving only, UDP at 10 Hz)");
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

// Scan mode known: send SCAN (the S2 spins its motor up by itself, at its configured speed)
void startScan()
{
    Log.print("Mode: "); Log.print(mode.name);
    Log.print(" (id "); Log.print(mode.id);
    Log.print(", "); Log.print(mode.usPerSample, 1); Log.println(" us/sample)");
    if (mode.usPerSample > 0) usPerSampleQ8 = (uint32_t)(mode.usPerSample * 256);   // once, at setup
    enterStep(ST_SCAN);
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
                Log.println("Still Error after RESET: power-cycle the LiDAR, then reset the board");
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
            startScan();
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
            startScan();
        } else if (elapsed > ANSWER_MS) startScan();
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
                 "longest loop %lu us). Still listening; reset the board to start again.\r\n",
                 (unsigned long)lidar.goodPackets(), (unsigned long)runSamples,
                 (unsigned long)(centiRev / 100), (unsigned long)(centiRev % 100), (unsigned long)loopMaxUs);
        Log.print(msg);
        streaming = false;
        stopReported = true;
        lastStatusMs = now;
    } else if (stopReported && now - lastStatusMs >= FAULT_REPEAT_MS) {
        // Repeated while nothing arrives, so the PC side sees why it gets no values
        lastStatusMs = now;
        Log.print("# waiting: no data from the LiDAR for ");
        Log.print((now - lidar.lastPacketMs()) / 1000);
        Log.println(" s (it stopped itself). Reset the board to start again.");
    }
}

Fault faultLidarBad, faultLidarUart, faultLidarLoop;
uint32_t prevBadPackets = 0, prevFramingErrors = 0, prevOverruns = 0;

// Once a second: corrupted LiDAR packets, UART errors, loop too slow. Prints nothing while all is well.
// (The LiDAR stopping is reported by runStream().)
void lidarFaults()
{
    char s[128];
    uint32_t bad = lidar.badPackets();
    snprintf(s, sizeof(s), "LIDAR FAULT: %lu bad packets so far (checksum / sync)", (unsigned long)bad);
    faultLidarBad.update(bad != prevBadPackets, s, nullptr);
    prevBadPackets = bad;

#if !LIDAR_ON_USART6
    uint32_t framing = LidarSerial.framingErrors(), overruns = LidarSerial.overruns();
    snprintf(s, sizeof(s), "LIDAR FAULT: UART framing errors %lu, overruns %lu so far (wiring / noise / loop too slow)",
             (unsigned long)framing, (unsigned long)overruns);
    faultLidarUart.update(framing != prevFramingErrors || overruns != prevOverruns, s, nullptr);
    prevFramingErrors = framing;
    prevOverruns = overruns;
#endif

    // LiDAR samples are decoded in the DMA interrupt, so a slow pass only delays CAN replies to the PC
    snprintf(s, sizeof(s), "LOOP FAULT: loop() took %lu us (> %lu), CAN replies to the PC delayed",
             (unsigned long)loopMaxUs, (unsigned long)LOOP_SLOW_US);
    faultLidarLoop.update(loopMaxUs > LOOP_SLOW_US, s, nullptr);
    loopMaxUs = 0;
}

// ------------------------------------------------------------------ IMU (BNO085), from stm_imu.ino

Adafruit_BNO08x bno08x(BNO_RST_PIN);        // hard reset in begin_I2C() when the pin is wired
sh2_SensorValue_t sensorValue;
bool     imuFound = false;
float    qw = 1, qx = 0, qy = 0, qz = 0;    // latest values, sent at 100 Hz
float    gx = 0, gy = 0, gz = 0;
float    ax = 0, ay = 0, az = 0;
uint32_t accelEvents = 0, gyroEvents = 0, quatEvents = 0, totalEvents = 0;
uint32_t imuReads = 0;                      // getSensorEvent() calls (INT was low)
uint32_t imuSequence = 0, imuReEnables = 0;
uint32_t imuLastEventMs = 0, imuPrevEvents = 0, imuNextSendMs = 0;

// CRC32 as Python zlib.crc32(data) & 0xFFFFFFFF
uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
    return ~crc;
}

bool enableImuReports()
{
    bool ok = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_US);
    ok &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_US);
    ok &= bno08x.enableReport(SH2_ACCELEROMETER, IMU_REPORT_US);
    return ok;
}

// A board reset in the middle of an I2C read leaves the BNO085 holding SDA low (it still waits for
// clocks), and then it is "not found" until a power-cycle. Clock SCL until it lets go, then send a STOP.
void i2cBusRecover()
{
    pinMode(BNO_SDA, INPUT_PULLUP);
    pinMode(BNO_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(BNO_SCL, HIGH);
    delayMicroseconds(5);
    for (int i = 0; i < 18 && digitalRead(BNO_SDA) == LOW; i++) {
        digitalWrite(BNO_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(BNO_SCL, HIGH);
        delayMicroseconds(5);
    }
    // STOP: SDA low -> high while SCL is high
    pinMode(BNO_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(BNO_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(BNO_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(BNO_SDA, HIGH);
    delayMicroseconds(5);
    pinMode(BNO_SDA, INPUT);
    pinMode(BNO_SCL, INPUT);
}

void setupImu()
{
    pinMode(BNO_INT_PIN, INPUT);

    for (int attempt = 1; attempt <= 3 && !imuFound; attempt++) {
        if (attempt > 1) {
            Wire.end();
            delay(100);
        }
        i2cBusRecover();
        Wire.setSDA(BNO_SDA);
        Wire.setSCL(BNO_SCL);
        Wire.begin();
        Wire.setClock(400000);
        Wire.setTimeout(10);
        imuFound = bno08x.begin_I2C(BNO_ADDR, &Wire);
    }
    if (!imuFound) {
        Log.println("IMU FAULT: BNO085 NOT FOUND at 0x4A (SDA D14, SCL D15) after 3 tries, no IMU packets. "
                    "Power-cycle the board.");
        return;
    }
    Log.println(enableImuReports() ? "IMU: BNO085 found, game rotation vector + gyro + accel at 100 Hz"
                                   : "IMU: BNO085 found, but enabling a report failed");
    imuLastEventMs = millis();
    imuNextSendMs = millis() + IMU_SEND_MS;
}

// Read at most IMU_EVENTS_PER_PASS events while INT is low, keep the LiDAR serviced in between
void pollImu()
{
    if (!imuFound) return;
    if (bno08x.wasReset()) {
        Log.println("IMU: BNO085 reset detected, enabling reports again");
        accelEvents = gyroEvents = quatEvents = 0;
        enableImuReports();
    }
    for (uint8_t n = 0; n < IMU_EVENTS_PER_PASS && digitalRead(BNO_INT_PIN) == LOW; n++) {
        imuReads++;
        bool got = bno08x.getSensorEvent(&sensorValue);
        lidar.poll();
        if (!got) continue;                 // non-sensor SHTP packet, handled inside the SH2 library
        totalEvents++;
        switch (sensorValue.sensorId) {
        case SH2_GAME_ROTATION_VECTOR:
            qw = sensorValue.un.gameRotationVector.real;
            qx = sensorValue.un.gameRotationVector.i;
            qy = sensorValue.un.gameRotationVector.j;
            qz = sensorValue.un.gameRotationVector.k;
            quatEvents++;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            gx = sensorValue.un.gyroscope.x;
            gy = sensorValue.un.gyroscope.y;
            gz = sensorValue.un.gyroscope.z;
            gyroEvents++;
            break;
        case SH2_ACCELEROMETER:
            ax = sensorValue.un.accelerometer.x;
            ay = sensorValue.un.accelerometer.y;
            az = sensorValue.un.accelerometer.z;
            accelEvents++;
            break;
        }
    }

    uint32_t now = millis();
    // Frozen watchdog: no event for a second -> enable the reports again
    if (totalEvents != imuPrevEvents) {
        imuPrevEvents = totalEvents;
        imuLastEventMs = now;
    } else if (now - imuLastEventMs >= IMU_FROZEN_MS) {
        imuLastEventMs = now;
        imuReEnables++;                     // reported by imuFaults(), not every second
        enableImuReports();
    }

    // Latest values at 100 Hz
    if ((int32_t)(now - imuNextSendMs) >= 0) {
        imuNextSendMs += IMU_SEND_MS;
        if ((int32_t)(now - imuNextSendMs) >= 0) imuNextSendMs = now + IMU_SEND_MS;   // fell behind
        IMUPacket p;
        p.sync1 = 0xAA;
        p.sync2 = 0x55;
        p.version = 3;
        p.reserved = 0;
        p.sequence = imuSequence++;
        p.device_time_us = micros();
        p.qw = qw; p.qx = qx; p.qy = qy; p.qz = qz;
        p.gx = gx; p.gy = gy; p.gz = gz;
        p.ax = ax; p.ay = ay; p.az = az;
        p.crc32 = crc32((const uint8_t *)&p, sizeof(p) - sizeof(p.crc32));
        udpSend((const uint8_t *)&p, sizeof(p));
    }
}

Fault faultImuMissing, faultImuReport;

// Once a second: IMU missing, or one of its three reports not arriving. Prints nothing while all is well.
// (Frozen IMU and BNO085 resets are reported by pollImu().)
void imuFaults()
{
    static uint32_t prevAcc = 0, prevGyro = 0, prevQuat = 0, prevReads = 0;
    faultImuMissing.update(!imuFound,
                           "IMU FAULT: BNO085 not found at 0x4A (SDA D14, SCL D15), no IMU data. "
                           "Power-cycle the board.", nullptr);
    if (imuFound) {
        uint32_t acc = accelEvents - prevAcc, gyro = gyroEvents - prevGyro, quat = quatEvents - prevQuat;
        char s[144];
        snprintf(s, sizeof(s), "IMU FAULT: report missing, events/s acc %lu gyro %lu quat %lu, "
                 "reads/s %lu, INT %s, I2C %s",
                 (unsigned long)acc, (unsigned long)gyro, (unsigned long)quat,
                 (unsigned long)(imuReads - prevReads), digitalRead(BNO_INT_PIN) == LOW ? "LOW" : "HIGH",
                 digitalRead(BNO_SDA) == LOW ? "SDA stuck LOW" : "SDA high");
        faultImuReport.update(acc == 0 || gyro == 0 || quat == 0, s, "IMU: all reports arriving again");
    }
    prevAcc = accelEvents; prevGyro = gyroEvents; prevQuat = quatEvents; prevReads = imuReads;
}

// ------------------------------------------------------------------ CAN (TWAI driver on bxCAN CAN1)

#define CAN_PKT_MAGIC0   'C'
#define CAN_PKT_MAGIC1   'B'
#define CAN_PKT_FRAMES   1
#define CAN_PKT_TEXT     2
#define CAN_PKT_HEADER   4
#define CAN_FRAME_SIZE   16                 // SocketCAN struct can_frame
#define CAN_MAX_FRAMES   32                 // per datagram
#define CAN_MAX_PKT      (CAN_PKT_HEADER + CAN_FRAME_SIZE * CAN_MAX_FRAMES)

#define CAN_EFF_FLAG 0x80000000UL
#define CAN_RTR_FLAG 0x40000000UL
#define CAN_ERR_FLAG 0x20000000UL
#define CAN_SFF_MASK 0x000007FFUL
#define CAN_EFF_MASK 0x1FFFFFFFUL

bool     canReady = false;
uint32_t canRxForwarded = 0;                // frames from the bus sent to the PC
uint32_t canNextStatusMs = 0;

// Written by canFromPc() in the lwIP interrupt, reported by loop()
volatile uint32_t canTxDropped = 0;         // frames from the PC that did not fit in the TX queue
volatile uint32_t canTxDroppedReported = 0;
volatile esp_err_t canLastTxErr = ESP_OK;

// CAN status line: to the PC as a 'CB' type-2 datagram, and to the USB serial if toSerial
void canText(const char *text, bool toSerial = true)
{
    if (toSerial) {
        Log.toUdp = false;
        Log.println(text);
        Log.toUdp = true;
    }

    uint8_t p[CAN_PKT_HEADER + 160];
    size_t len = strlen(text);
    if (len > sizeof(p) - CAN_PKT_HEADER) len = sizeof(p) - CAN_PKT_HEADER;
    p[0] = CAN_PKT_MAGIC0;
    p[1] = CAN_PKT_MAGIC1;
    p[2] = CAN_PKT_TEXT;
    p[3] = 0;
    memcpy(p + CAN_PKT_HEADER, text, len);
    udpSend(p, CAN_PKT_HEADER + len);
}

void setupCan()
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g.tx_queue_len = 32;
    g.rx_queue_len = 128;
    g.alerts_enabled = TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE |
                       TWAI_ALERT_BUS_OFF | TWAI_ALERT_RECOVERY_IN_PROGRESS |
                       TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_QUEUE_FULL |
                       TWAI_ALERT_RX_FIFO_OVERRUN;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) {
        Log.print("CAN FAULT: TWAI driver install failed: ");
        Log.println(esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err == ESP_ERR_TIMEOUT) {
        Log.println("CAN FAULT: start timed out - RX line stuck dominant? Check transceiver power and PD0 <- RXD.");
        return;
    }
    if (err != ESP_OK) {
        Log.print("CAN FAULT: TWAI start failed: ");
        Log.println(esp_err_to_name(err));
        return;
    }
    canReady = true;
    canNextStatusMs = millis() + CAN_STATUS_MS;
    Log.println("CAN: TWAI started at 1 Mbit/s on PD1 (TX) / PD0 (RX)");
}

// PC -> bus: 'CB' type-1 datagram with count x can_frame.
// Runs in the lwIP interrupt (onUdpArrival), so it only queues frames and counts; no printing.
void canFromPc(const uint8_t *p, int n)
{
    uint8_t count = p[3];

    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *f = p + CAN_PKT_HEADER + i * CAN_FRAME_SIZE;
        if (f + CAN_FRAME_SIZE > p + n) break;

        uint32_t canId;
        memcpy(&canId, f, 4);
        if (canId & CAN_ERR_FLAG) continue;

        twai_message_t msg = {};
        msg.extd = (canId & CAN_EFF_FLAG) ? 1 : 0;
        msg.rtr  = (canId & CAN_RTR_FLAG) ? 1 : 0;
        msg.identifier = canId & (msg.extd ? CAN_EFF_MASK : CAN_SFF_MASK);
        msg.data_length_code = f[4] > 8 ? 8 : f[4];
        memcpy(msg.data, f + 8, 8);

        esp_err_t err = canReady ? twai_transmit(&msg, 0) : ESP_ERR_INVALID_STATE;
        if (err != ESP_OK) {
            canTxDropped++;
            canLastTxErr = err;
        }
    }
}

// Bus -> PC: everything received so far, up to 32 frames per datagram.
// Called several times per loop() pass so replies do not wait for the IMU / LiDAR work.
void canToPc()
{
    if (!canReady) return;

    uint8_t p[CAN_MAX_PKT];
    twai_message_t msg;

    for (;;) {
        uint8_t count = 0;
        while (count < CAN_MAX_FRAMES && twai_receive(&msg, 0) == ESP_OK) {
            uint8_t *f = p + CAN_PKT_HEADER + count * CAN_FRAME_SIZE;
            uint32_t canId = msg.extd ? (msg.identifier & CAN_EFF_MASK) | CAN_EFF_FLAG
                                      : (msg.identifier & CAN_SFF_MASK);
            if (msg.rtr) canId |= CAN_RTR_FLAG;
            memset(f, 0, CAN_FRAME_SIZE);
            memcpy(f, &canId, 4);
            f[4] = msg.data_length_code > 8 ? 8 : msg.data_length_code;
            memcpy(f + 8, msg.data, 8);
            count++;
        }
        if (count == 0) return;

        p[0] = CAN_PKT_MAGIC0;
        p[1] = CAN_PKT_MAGIC1;
        p[2] = CAN_PKT_FRAMES;
        p[3] = count;
        udpSend(p, CAN_PKT_HEADER + count * CAN_FRAME_SIZE);
        canRxForwarded += count;
        if (count < CAN_MAX_FRAMES) return;
    }
}

// Alerts, with bus-off recovery: BUS_OFF -> initiate recovery, BUS_RECOVERED -> start again.
// All of them are faults (or the end of one), so they also go to the USB serial.
void canAlerts()
{
    uint32_t alerts = 0;
    if (twai_read_alerts(&alerts, 0) != ESP_OK) return;

    if (alerts & TWAI_ALERT_TX_FAILED)       canText("CANALERT TX failed (aborted)");
    if (alerts & TWAI_ALERT_ERR_PASS)        canText("CANALERT error passive");
    if (alerts & TWAI_ALERT_ERR_ACTIVE)      canText("CANALERT error active");
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL)   canText("CANALERT RX queue full");
    if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) canText("CANALERT RX FIFO overrun");
    if (alerts & TWAI_ALERT_BUS_OFF) {
        canText("CANALERT bus-off, starting recovery");
        twai_initiate_recovery();
    }
    if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
        canText("CANALERT recovery in progress (waiting for 128x11 recessive bits)");
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        canText("CANALERT bus recovered, restarting");
        esp_err_t err = twai_start();
        if (err != ESP_OK) {
            char text[64];
            snprintf(text, sizeof(text), "ERROR: CAN restart failed: %s", esp_err_to_name(err));
            canText(text);
        }
    }

    // Frames the interrupt could not queue
    uint32_t dropped = canTxDropped;
    if (dropped != canTxDroppedReported) {
        char text[112];
        esp_err_t err = canLastTxErr;
        snprintf(text, sizeof(text), "ERROR: %lu CAN frame(s) not sent: %s%s",
                 (unsigned long)(dropped - canTxDroppedReported), esp_err_to_name(err),
                 err == ESP_ERR_TIMEOUT ? " (TX queue full - frames not being ACKed?)" : "");
        canTxDroppedReported = dropped;
        canText(text);
    }
}

// CANSTAT once a second to the PC (the node logs it only when a fault field changes).
// The USB serial gets it only while something is wrong.
void canStatus()
{
    if (!canReady) {
        canText("CANSTAT STATE=NOT_STARTED (see USB serial for the error)", false);
        return;
    }
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) return;

    static uint32_t prevTxFailed = 0, prevRxMissed = 0, prevOverrun = 0, prevDropped = 0;
    uint32_t dropped = canTxDropped;
    bool fault = s.state != TWAI_STATE_RUNNING || s.tx_error_counter > 0 || s.rx_error_counter > 0 ||
                 s.tx_failed_count != prevTxFailed || s.rx_missed_count != prevRxMissed ||
                 s.rx_overrun_count != prevOverrun || dropped != prevDropped;
    prevTxFailed = s.tx_failed_count;
    prevRxMissed = s.rx_missed_count;
    prevOverrun = s.rx_overrun_count;
    prevDropped = dropped;

    static const char *stateNames[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
    char text[160];
    snprintf(text, sizeof(text),
             "CANSTAT STATE=%s TEC=%lu REC=%lu BUSERR=%lu TXOK=%lu TXFAIL=%lu TXPEND=%lu TXDROP=%lu RX=%lu",
             stateNames[s.state],
             (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
             (unsigned long)s.bus_error_count, (unsigned long)s.tx_success_count,
             (unsigned long)s.tx_failed_count, (unsigned long)s.msgs_to_tx,
             (unsigned long)dropped, (unsigned long)canRxForwarded);
    canText(text, fault);
}

void pollCan()
{
    if (canReady) {
        canToPc();
        canAlerts();
    }
    uint32_t now = millis();
    if ((int32_t)(now - canNextStatusMs) >= 0) {
        canNextStatusMs = now + CAN_STATUS_MS;
        canStatus();
    }
}

// ------------------------------------------------------------------ UDP from the PC

// Called by the Ethernet library in the lwIP (TIM14) interrupt the moment a datagram arrives.
// The library keeps only one received datagram and frees it when the next one comes in, so reading
// it here, instead of once per loop() pass, means no CAN datagram (or hello) is ever overwritten
// while loop() is busy with the LiDAR and the IMU. CAN frames go straight into the TWAI TX queue.
void onUdpArrival()
{
    int size = Udp.parsePacket();
    if (size <= 0) return;
    udpReceived++;

    uint32_t ip = Udp.remoteIP();
    uint16_t port = Udp.remotePort();
    uint32_t now = millis();
    if (!havePeer || ip != peerAddr || port != peerPort || now - peerSeenMs >= PEER_TIMEOUT_MS) {
        peerIsNew = true;
    }
    peerAddr = ip;
    peerPort = port;
    peerSeenMs = now;
    havePeer = true;

    uint8_t p[CAN_MAX_PKT];
    int n = Udp.read(p, sizeof(p));
    while (Udp.available() > 0) Udp.read();     // drop anything longer (frees the buffer)

    if (n >= CAN_PKT_HEADER && p[0] == CAN_PKT_MAGIC0 && p[1] == CAN_PKT_MAGIC1 && p[2] == CAN_PKT_FRAMES) {
        canFromPc(p, n);
    }
}

// Every datagram registers its sender as the PC to send to; announce a new one
void handleUdp()
{
    if (!peerIsNew) return;
    peerIsNew = false;
    Log.print("# PC ");
    Log.print(IPAddress(peerAddr));
    Log.print(":");
    Log.print(peerPort);
    Log.println(" connected, sending scan/IMU/CAN data to it");
}

void setup()
{
    Serial.begin(PC_BAUD);
    Log.println("\n=== RPLIDAR S2 + BNO085 + CAN on NUCLEO-F207ZG, UDP ===");
    setupImu();
    // Once at power-up. With no cable the PHY auto-negotiation can hold this for a few seconds.
    Ethernet.begin(BOARD_IP, NETMASK, GATEWAY);
    Udp.begin(BOARD_PORT);
    Udp.onDataArrival(onUdpArrival);
    netReady = true;
    Log.print("Board IP ");
    Log.print(Ethernet.localIP());
    Log.print("  UDP port ");
    Log.print(BOARD_PORT);
    Log.println(Ethernet.linkStatus() == LinkON ? "  link up" : "  link DOWN (cable?)");
    netSetupReport();
    setupCan();

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
    canToPc();
    pumpOutput();
    handleUdp();
    pollCan();
    lidar.poll();
    pollImu();
    canToPc();
    static uint32_t faultCheckMs = 0;
    if (millis() - faultCheckMs >= FAULT_CHECK_MS) {
        faultCheckMs = millis();
        netFaults();
        lidarFaults();
        imuFaults();
    }
    uint32_t dt = micros() - t0;
    if (dt > loopMaxUs) loopMaxUs = dt;
}
