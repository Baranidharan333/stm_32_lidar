/**
 * can_udp.ino  –  NUCLEO-F207ZG Ethernet (UDP) <-> CAN bridge (TWAI driver on bxCAN)
 *
 * PC side: udp_can_bridge.py (creates the virtual can0 and forwards frames).
 *
 *   PC can0  <-->  udp_can_bridge.py  <-- UDP / Ethernet -->  STM32  <-->  CAN bus
 *
 * Network:
 *   Board 192.168.200.177, listens on UDP 5700.
 *   Replies go to the PC that last sent a datagram; while no PC has been
 *   heard for 5 s they are broadcast to 192.168.200.255:5701.
 *
 * Datagram (little-endian):
 *   byte 0..1  'C' 'B'
 *   byte 2     type: 1 = CAN frames, 2 = status text, 3 = hello (PC keep-alive)
 *   byte 3     count (frames) / 0
 *   then, type 1: count x 16-byte SocketCAN struct can_frame
 *                 { u32 can_id (EFF/RTR flags), u8 len, u8 pad[3], u8 data[8] }
 *         type 2: ASCII text (CANSTAT / CANALERT / ERROR lines)
 *   PC -> board type 1: frames to put on the bus.
 *   board -> PC type 1: frames received from the bus.
 *
 * CAN wiring:
 *   PD1 (CAN1_TX, CN9 pin 27) → transceiver TXD
 *   PD0 (CAN1_RX, CN9 pin 25) → transceiver RXD
 *   Transceiver CANH / CANL → bus, GND common with the motors.
 *
 * Bus-off is recovered automatically: BUS_OFF → twai_initiate_recovery(),
 * BUS_RECOVERED → twai_start().
 */

#include <Arduino.h>
#include <LwIP.h>          // before STM32Ethernet, so the builder finds STM32duino_LwIP
#include <STM32Ethernet.h>
#include <string.h>
#include "twai.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define SERIAL_BAUDRATE 115200

#define CAN_TX_PIN PD1
#define CAN_RX_PIN PD0

IPAddress BOARD_IP(192, 168, 200, 177);
IPAddress NETMASK(255, 255, 255, 0);
IPAddress GATEWAY(192, 168, 200, 1);
IPAddress BROADCAST_IP(192, 168, 200, 255);
const uint16_t BOARD_PORT = 5700;        // board listens here
const uint16_t PC_PORT    = 5701;        // broadcast destination while no PC is known

const uint32_t PEER_TIMEOUT_MS = 5000;
const uint32_t STATUS_MS       = 1000;   // CANSTAT heartbeat

// =====================================================
// PACKET FORMAT
// =====================================================

#define PKT_MAGIC0  'C'
#define PKT_MAGIC1  'B'
#define PKT_FRAMES  1
#define PKT_TEXT    2
#define PKT_HELLO   3

#define PKT_HEADER_SIZE     4
#define FRAME_SIZE          16
#define MAX_FRAMES_PER_PKT  32
#define MAX_PKT_SIZE        (PKT_HEADER_SIZE + FRAME_SIZE * MAX_FRAMES_PER_PKT)

// SocketCAN can_id flags
#define CAN_EFF_FLAG 0x80000000UL
#define CAN_RTR_FLAG 0x40000000UL
#define CAN_ERR_FLAG 0x20000000UL
#define CAN_SFF_MASK 0x000007FFUL
#define CAN_EFF_MASK 0x1FFFFFFFUL

EthernetUDP Udp;

bool      havePeer = false;
IPAddress peerIp;
uint16_t  peerPort = PC_PORT;
uint32_t  peerSeenMs = 0;

bool canReady = false;
uint32_t txDropped = 0;     // frames from the PC that did not fit in the TX queue
uint32_t rxForwarded = 0;   // frames from the bus sent to the PC

// =====================================================
// UDP OUTPUT
// =====================================================

void udpSend(const uint8_t *data, size_t len) {
  if (Ethernet.linkStatus() != LinkON) {
    return;
  }
  bool unicast = havePeer && millis() - peerSeenMs < PEER_TIMEOUT_MS;
  if (Udp.beginPacket(unicast ? peerIp : BROADCAST_IP, unicast ? peerPort : PC_PORT)) {
    Udp.write(data, len);
    Udp.endPacket();
  }
}

// Status text goes to the PC and to the USB serial log.
void sendText(const char *text) {
  Serial.println(text);

  uint8_t pkt[PKT_HEADER_SIZE + 160];
  size_t len = strlen(text);
  if (len > sizeof(pkt) - PKT_HEADER_SIZE) {
    len = sizeof(pkt) - PKT_HEADER_SIZE;
  }
  pkt[0] = PKT_MAGIC0;
  pkt[1] = PKT_MAGIC1;
  pkt[2] = PKT_TEXT;
  pkt[3] = 0;
  memcpy(pkt + PKT_HEADER_SIZE, text, len);
  udpSend(pkt, PKT_HEADER_SIZE + len);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();
  Serial.println("================================");
  Serial.println(" STM32 NUCLEO F207ZG CAN <-> UDP");
  Serial.println("================================");

  // With no cable the PHY auto-negotiation can hold this for a few seconds.
  Ethernet.begin(BOARD_IP, NETMASK, GATEWAY);
  Udp.begin(BOARD_PORT);
  Serial.print("Board IP ");
  Serial.print(Ethernet.localIP());
  Serial.print("  UDP port ");
  Serial.print(BOARD_PORT);
  Serial.println(Ethernet.linkStatus() == LinkON ? "  link up" : "  link DOWN (cable?)");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 16;
  g_config.rx_queue_len = 64;
  g_config.alerts_enabled = TWAI_ALERT_TX_FAILED |
                            TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE |
                            TWAI_ALERT_BUS_OFF | TWAI_ALERT_RECOVERY_IN_PROGRESS |
                            TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_RX_QUEUE_FULL |
                            TWAI_ALERT_RX_FIFO_OVERRUN;

  // DaMiao motors default to 1 Mbit/s
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.print("ERROR: TWAI driver install failed: ");
    Serial.println(esp_err_to_name(err));
    return;
  }

  err = twai_start();
  if (err == ESP_ERR_TIMEOUT) {
    Serial.println("ERROR: TWAI start timed out - RX line stuck dominant?");
    Serial.println("       Check transceiver power and PD0 <- RXD wiring.");
    return;
  }
  if (err != ESP_OK) {
    Serial.print("ERROR: TWAI start failed: ");
    Serial.println(esp_err_to_name(err));
    return;
  }

  canReady = true;
  Serial.println("TWAI started at 1 Mbit/s.");
  Serial.println("================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  handleUdp();
  if (canReady) {
    readCANMessages();
    handleAlerts();
  }
  reportCANStatus();
}

// =====================================================
// UDP INPUT: PC -> CAN bus
// =====================================================

void handleUdp() {
  int size = Udp.parsePacket();
  if (size <= 0) {
    return;
  }

  // Every datagram registers its sender as the PC to reply to.
  IPAddress ip = Udp.remoteIP();
  uint16_t port = Udp.remotePort();
  bool isNew = !havePeer || ip != peerIp || port != peerPort ||
               millis() - peerSeenMs >= PEER_TIMEOUT_MS;
  havePeer = true;
  peerIp = ip;
  peerPort = port;
  peerSeenMs = millis();

  uint8_t pkt[MAX_PKT_SIZE];
  int n = Udp.read(pkt, sizeof(pkt));
  while (Udp.available() > 0) {
    Udp.read();  // drop anything longer than MAX_PKT_SIZE
  }

  if (isNew) {
    char text[64];
    snprintf(text, sizeof(text), "# PC %d.%d.%d.%d:%u connected",
             ip[0], ip[1], ip[2], ip[3], port);
    sendText(text);
  }

  if (n < PKT_HEADER_SIZE || pkt[0] != PKT_MAGIC0 || pkt[1] != PKT_MAGIC1) {
    return;
  }
  if (pkt[2] != PKT_FRAMES) {
    return;  // hello: only registers the peer
  }

  uint8_t count = pkt[3];
  uint32_t dropped = 0;
  esp_err_t lastErr = ESP_OK;

  for (uint8_t i = 0; i < count; i++) {
    const uint8_t *f = pkt + PKT_HEADER_SIZE + i * FRAME_SIZE;
    if (f + FRAME_SIZE > pkt + n) {
      break;
    }

    uint32_t canId;
    memcpy(&canId, f, 4);
    if (canId & CAN_ERR_FLAG) {
      continue;
    }

    twai_message_t msg = {};
    msg.extd = (canId & CAN_EFF_FLAG) ? 1 : 0;
    msg.rtr  = (canId & CAN_RTR_FLAG) ? 1 : 0;
    msg.identifier = canId & (msg.extd ? CAN_EFF_MASK : CAN_SFF_MASK);
    msg.data_length_code = f[4] > 8 ? 8 : f[4];
    memcpy(msg.data, f + 8, 8);

    esp_err_t err = canReady ? twai_transmit(&msg, 0) : ESP_ERR_INVALID_STATE;
    if (err != ESP_OK) {
      dropped++;
      lastErr = err;
    }
  }

  if (dropped) {
    txDropped += dropped;
    char text[96];
    snprintf(text, sizeof(text), "ERROR: %lu frame(s) not sent: %s%s",
             (unsigned long)dropped, esp_err_to_name(lastErr),
             lastErr == ESP_ERR_TIMEOUT ? " (TX queue full - frames not being ACKed?)" : "");
    sendText(text);
  }
}

// =====================================================
// CAN RX: bus -> PC (batched, up to 32 frames per datagram)
// =====================================================

void readCANMessages() {
  uint8_t pkt[MAX_PKT_SIZE];
  uint8_t count = 0;
  twai_message_t msg;

  while (count < MAX_FRAMES_PER_PKT && twai_receive(&msg, 0) == ESP_OK) {
    uint8_t *f = pkt + PKT_HEADER_SIZE + count * FRAME_SIZE;

    uint32_t canId = msg.extd ? (msg.identifier & CAN_EFF_MASK) | CAN_EFF_FLAG
                              : (msg.identifier & CAN_SFF_MASK);
    if (msg.rtr) {
      canId |= CAN_RTR_FLAG;
    }

    memset(f, 0, FRAME_SIZE);
    memcpy(f, &canId, 4);
    f[4] = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    memcpy(f + 8, msg.data, 8);
    count++;
  }

  if (count == 0) {
    return;
  }

  pkt[0] = PKT_MAGIC0;
  pkt[1] = PKT_MAGIC1;
  pkt[2] = PKT_FRAMES;
  pkt[3] = count;
  udpSend(pkt, PKT_HEADER_SIZE + count * FRAME_SIZE);
  rxForwarded += count;
}

// =====================================================
// ALERTS + BUS-OFF RECOVERY
// =====================================================

void handleAlerts() {
  uint32_t alerts = 0;

  if (twai_read_alerts(&alerts, 0) != ESP_OK) {
    return;
  }

  if (alerts & TWAI_ALERT_TX_FAILED) {
    sendText("CANALERT TX failed (aborted)");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    sendText("CANALERT error passive");
  }
  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    sendText("CANALERT error active");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    sendText("CANALERT RX queue full");
  }
  if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
    sendText("CANALERT RX FIFO overrun");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    sendText("CANALERT bus-off, starting recovery");
    twai_initiate_recovery();
  }
  if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    sendText("CANALERT recovery in progress (waiting for 128x11 recessive bits)");
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    sendText("CANALERT bus recovered, restarting");
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
      char text[64];
      snprintf(text, sizeof(text), "ERROR: TWAI restart failed: %s", esp_err_to_name(err));
      sendText(text);
    }
  }
}

// =====================================================
// CAN STATUS HEARTBEAT (every STATUS_MS)
// =====================================================

void reportCANStatus() {
  static uint32_t lastMs = 0;

  if (millis() - lastMs < STATUS_MS) {
    return;
  }
  lastMs = millis();

  if (!canReady) {
    sendText("CANSTAT STATE=NOT_STARTED (see USB serial for the error)");
    return;
  }

  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) {
    return;
  }

  static const char *stateNames[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
  char text[160];
  snprintf(text, sizeof(text),
           "CANSTAT STATE=%s TEC=%lu REC=%lu BUSERR=%lu TXOK=%lu TXFAIL=%lu "
           "TXPEND=%lu TXDROP=%lu RX=%lu",
           stateNames[s.state],
           (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
           (unsigned long)s.bus_error_count, (unsigned long)s.tx_success_count,
           (unsigned long)s.tx_failed_count, (unsigned long)s.msgs_to_tx,
           (unsigned long)txDropped, (unsigned long)rxForwarded);
  sendText(text);
}
