#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <LwIP.h>                           // before STM32Ethernet, so the builder finds STM32duino_LwIP
#include <STM32Ethernet.h>
#include "RPLidarS2.h"
#include "DmaSoftUart.h"

// RPLIDAR S2 + BNO085 IMU on a NUCLEO-F207ZG, both over the on-board Ethernet port as UDP
// for ros2_ws/lidar_imu_udp_node.py (/scan, /imu/data).
// LiDAR part = rplidar_s2_nucleo_udp, IMU part = imu/stm_imu.ino.
//
// Wiring:
//   LiDAR (XH2.54-5P): Red VCC -> external 5V (up to 1.5A at start), Black GND -> GND,
//                      LiDAR TX -> D1 / PG14, LiDAR RX -> D0 / PG9 (the S2 starts its motor on SCAN)
//   BNO085 (I2C 0x4A): SDA -> D14 / PB9, SCL -> D15 / PB8, INT -> D7 / PF13, 3.3V, GND
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
//   text   one status line (board startup, health, "LiDAR stopped sending", ...)
// From the PC the board only takes the hello datagram; its content is ignored, it just tells the
// board where to send. The LiDAR is set up and started once at power-up; to start it again, reset
// the board. The USB serial (19200 baud) prints the status text and, once a second, NET / LIDAR / IMU
// debug lines.
//
// No delay() in loop(). lwIP runs from the library's 1 ms TIM14 interrupt; the LiDAR link uses
// TIM1/TIM8 + DMA2 and must be serviced at least every ~8 ms, so the IMU is read at most
// IMU_EVENTS_PER_PASS events (~1 ms of I2C each) per loop() pass with lidar.poll() in between.
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
const uint32_t NET_DEBUG_MS    = 1000;      // "NET" status line on the USB serial

EthernetUDP Udp;
bool      netReady = false;
bool      havePeer = false;
IPAddress peerIp;
uint16_t  peerPort = PC_PORT;
uint32_t  peerSeenMs = 0;

// Counters for the NET debug line
uint32_t udpSent = 0, udpFailed = 0, udpNoLink = 0, udpReceived = 0;

void udpSend(const uint8_t *data, size_t len)
{
    if (!netReady) return;
    if (Ethernet.linkStatus() != LinkON) { udpNoLink++; return; }
    bool unicast = havePeer && millis() - peerSeenMs < PEER_TIMEOUT_MS;
    bool ok = Udp.beginPacket(unicast ? peerIp : BROADCAST_IP, unicast ? peerPort : PC_PORT);
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

// Once after Ethernet.begin(): MAC, pins, PHY identity
void netSetupReport()
{
    Log.toUdp = false;
    uint8_t *mac = Ethernet.MACAddress();
    char s[40];
    snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Log.print("NET MAC "); Log.print(s);
    Log.print("  IP "); Log.print(Ethernet.localIP());
    Log.print("  mask "); Log.print(Ethernet.subnetMask());
    Log.print("  gw "); Log.println(Ethernet.gatewayIP());

    // RMII pins of the NUCLEO-F207ZG, all must be alternate function 11 (ETH)
    struct EthPin { GPIO_TypeDef *port; char name; uint8_t pin; const char *sig; };
    const EthPin pins[] = {
        {GPIOA, 'A', 1, "REF_CLK"}, {GPIOA, 'A', 2, "MDIO"}, {GPIOC, 'C', 1, "MDC"},
        {GPIOA, 'A', 7, "CRS_DV"}, {GPIOC, 'C', 4, "RXD0"}, {GPIOC, 'C', 5, "RXD1"},
        {GPIOG, 'G', 11, "TX_EN"}, {GPIOG, 'G', 13, "TXD0"}, {GPIOB, 'B', 13, "TXD1"},
    };
    int bad = 0;
    for (const EthPin &p : pins) {
        uint32_t moder = (p.port->MODER >> (2 * p.pin)) & 3;
        uint32_t af = (p.port->AFR[p.pin / 8] >> (4 * (p.pin % 8))) & 0xF;
        if (moder != 2 || af != 11) {
            Log.print("NET pin P"); Log.print(p.name); Log.print(p.pin); Log.print(" ("); Log.print(p.sig);
            Log.print(") not set to ETH: mode "); Log.print(moder); Log.print(" AF "); Log.println(af);
            bad++;
        }
    }
    if (!bad) Log.println("NET RMII pins OK (all 9 on AF11)");

    Log.print("NET ETH clock "); Log.print(RCC->AHB1ENR & RCC_AHB1ENR_ETHMACEN ? "on" : "OFF");
    int32_t id1 = phyRead(2), id2 = phyRead(3);
    Log.print("  PHY id "); logHex(id1); Log.print(" "); logHex(id2);
    if (id1 == 0x0007 && (id2 & 0xFFF0) == 0xC130) Log.println("  = LAN8742A, MDIO OK");
    else Log.println("  expected 0x7 0xC13x: PHY not answering (ETH clock / jumpers SB13 etc.)");
    Log.toUdp = true;
}

// Once a second: PHY link, MAC speed, frame counters, UDP counters, where data is going
void netDebug()
{
    Log.toUdp = false;
    int32_t bsr = phyRead(1), scsr = phyRead(31);
    bool phyLink = bsr >= 0 && (bsr & 0x0004);
    bool aneg = bsr >= 0 && (bsr & 0x0020);
    static const char *speeds[8] = {"?", "10HD", "100HD", "?", "?", "10FD", "100FD", "?"};

    Log.print("NET lwip:"); Log.print(Ethernet.linkStatus() == LinkON ? "UP" : "DOWN");
    Log.print(" phy:"); Log.print(bsr < 0 ? "no answer" : phyLink ? "link" : "NO LINK");
    if (phyLink) { Log.print(aneg ? " aneg " : " aneg-pending "); Log.print(scsr >= 0 ? speeds[(scsr >> 2) & 7] : "?"); }
    Log.print(" mac:"); Log.print(ETH->MACCR & ETH_MACCR_FES ? "100" : "10");
    Log.print(ETH->MACCR & ETH_MACCR_DM ? "FD" : "HD");
    Log.print(" | frames tx "); Log.print(ETH->MMCTGFCR);
    Log.print(" rx-unicast "); Log.print(ETH->MMCRGUFCR);
    Log.print(" rx-crc-err "); Log.print(ETH->MMCRFCECR);
    Log.print(" rx-missed "); Log.print(ETH->DMAMFBOCR & 0xFFFF);
    Log.print(" | udp sent "); Log.print(udpSent);
    Log.print(" fail "); Log.print(udpFailed);
    Log.print(" no-link "); Log.print(udpNoLink);
    Log.print(" rx "); Log.print(udpReceived);
    bool unicast = havePeer && millis() - peerSeenMs < PEER_TIMEOUT_MS;
    Log.print(" | to ");
    Log.print(unicast ? peerIp : BROADCAST_IP);
    Log.print(":"); Log.print(unicast ? peerPort : PC_PORT);
    Log.println(unicast ? "" : " (broadcast, no PC heard)");

    if (bsr < 0) Log.println("NET hint: PHY does not answer on MDIO");
    else if (!phyLink) Log.println("NET hint: no Ethernet link - cable unplugged, or switch/PC port off");
    else if (ETH->MMCRFCECR) Log.println("NET hint: CRC errors - RMII clock/signal problem (PA1 50 MHz clock line?)");
    else if (!udpReceived) { Log.print("NET hint: link up but nothing from the PC yet - same subnet? ping "); Log.println(BOARD_IP); }
    Log.toUdp = true;
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
    } else if (stopReported && now - lastStatusMs >= 1000) {
        // Once a second while nothing arrives, so the PC side sees why it gets no values
        lastStatusMs = now;
        Log.print("# waiting: no data from the LiDAR for ");
        Log.print((now - lidar.lastPacketMs()) / 1000);
        Log.println(" s (it stopped itself). Reset the board to start again.");
    }
}

// Once a second, after the NET line: where the LiDAR side is (USB serial only)
void lidarDebug()
{
    static const char *steps[] = {"STOP", "INFO", "HEALTH", "RESET", "TYPICAL", "US", "ANS", "NAME",
                                  "SCAN", "STREAM", "IDLE"};
    Log.toUdp = false;
    Log.print("LIDAR step "); Log.print(steps[step]);
    Log.print(lidar.discarding() ? " (discarding until quiet)" : "");
    Log.print(" | packets good "); Log.print(lidar.goodPackets());
    Log.print(" bad "); Log.print(lidar.badPackets());
    Log.print(" last "); Log.print(lidar.goodPackets() ? (millis() - lidar.lastPacketMs()) : 0); Log.print(" ms ago");
#if !LIDAR_ON_USART6
    Log.print(" | uart bytes "); Log.print(LidarSerial.bytesReceived());
    Log.print(" framing-err "); Log.print(LidarSerial.framingErrors());
    Log.print(" overruns "); Log.print(LidarSerial.overruns());
#endif
    Log.print(" | samples "); Log.print(runSamples);
    Log.print(" block-points "); Log.print(pkt.points);
    Log.print(" scans sent "); Log.print(blockCount);
    Log.print(" | loop max "); Log.print(loopMaxUs); Log.println(" us");
    Log.toUdp = true;
}

// ------------------------------------------------------------------ IMU (BNO085), from stm_imu.ino

Adafruit_BNO08x bno08x(-1);                 // no reset pin
sh2_SensorValue_t sensorValue;
bool     imuFound = false;
float    qw = 1, qx = 0, qy = 0, qz = 0;    // latest values, sent at 100 Hz
float    gx = 0, gy = 0, gz = 0;
float    ax = 0, ay = 0, az = 0;
uint32_t accelEvents = 0, gyroEvents = 0, quatEvents = 0, totalEvents = 0;
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

void setupImu()
{
    Wire.setSDA(BNO_SDA);
    Wire.setSCL(BNO_SCL);
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(10);
    pinMode(BNO_INT_PIN, INPUT);

    imuFound = bno08x.begin_I2C(BNO_ADDR, &Wire);
    if (!imuFound) {
        Log.println("IMU: BNO085 NOT FOUND at 0x4A (SDA D14, SCL D15), no IMU packets. Fix and reset the board.");
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
        imuReEnables++;
        Log.println("IMU: no events for 1 s, enabling reports again");
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

void imuDebug()
{
    static uint32_t prevAcc = 0, prevGyro = 0, prevQuat = 0;
    Log.toUdp = false;
    Log.print("IMU ");
    if (!imuFound) {
        Log.println("not found");
    } else {
        Log.print("events/s acc "); Log.print(accelEvents - prevAcc);
        Log.print(" gyro "); Log.print(gyroEvents - prevGyro);
        Log.print(" quat "); Log.print(quatEvents - prevQuat);
        Log.print(" | INT "); Log.print(digitalRead(BNO_INT_PIN) == LOW ? "LOW" : "HIGH");
        Log.print(" re-enables "); Log.print(imuReEnables);
        Log.print(" | q "); Log.print(qw, 3); Log.print(" "); Log.print(qx, 3); Log.print(" ");
        Log.print(qy, 3); Log.print(" "); Log.print(qz, 3);
        Log.print(" | acc "); Log.print(ax, 2); Log.print(" "); Log.print(ay, 2); Log.print(" "); Log.print(az, 2);
        Log.print(" | packets "); Log.println(imuSequence);
    }
    prevAcc = accelEvents; prevGyro = gyroEvents; prevQuat = quatEvents;
    Log.toUdp = true;
}

// Every datagram registers its sender as the PC to send to; its content is ignored
void handleUdp()
{
    if (!netReady || Udp.parsePacket() <= 0) return;
    udpReceived++;
    IPAddress ip = Udp.remoteIP();
    uint16_t port = Udp.remotePort();
    bool isNew = !havePeer || ip != peerIp || port != peerPort || millis() - peerSeenMs >= PEER_TIMEOUT_MS;
    havePeer = true;
    peerIp = ip;
    peerPort = port;
    peerSeenMs = millis();
    if (isNew) {
        Log.print("# PC ");
        Log.print(ip);
        Log.print(":");
        Log.print(port);
        Log.println(" connected, sending scan/IMU data to it");
    }
    char buf[32];
    while (Udp.read((unsigned char *)buf, sizeof(buf)) > 0) {}
}

void setup()
{
    Serial.begin(PC_BAUD);
    Log.println("\n=== RPLIDAR S2 + BNO085 on NUCLEO-F207ZG, UDP ===");
    setupImu();
    // Once at power-up. With no cable the PHY auto-negotiation can hold this for a few seconds.
    Ethernet.begin(BOARD_IP, NETMASK, GATEWAY);
    Udp.begin(BOARD_PORT);
    netReady = true;
    Log.print("Board IP ");
    Log.print(Ethernet.localIP());
    Log.print("  UDP port ");
    Log.print(BOARD_PORT);
    Log.println(Ethernet.linkStatus() == LinkON ? "  link up" : "  link DOWN (cable?)");
    netSetupReport();

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
    handleUdp();
    pollImu();
    static uint32_t netDebugMs = 0;
    if (millis() - netDebugMs >= NET_DEBUG_MS) {
        netDebugMs = millis();
        netDebug();
        lidarDebug();
        imuDebug();
    }
    uint32_t dt = micros() - t0;
    if (dt > loopMaxUs) loopMaxUs = dt;
}
