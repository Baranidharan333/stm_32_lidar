/**
 * can.ino  –  NUCLEO-F207ZG serial <-> CAN bridge (TWAI driver on bxCAN)
 *
 * Serial protocol (used by serial_can_bridge.py):
 *   in : "ID#DATA\n"                  e.g. 001#FFFFFFFFFFFFFFFC
 *   out: "TX ID=.. DLC=.. DATA=.. TX ok (ACKed)" / "... TX pending (no ACK yet)"
 *        "ERROR ..."
 *        "RX ID=0x.. DLC=.. DATA=.. STD"
 *        "CANSTAT ...", "CANALERT ..."
 *
 * Wiring:
 *   PD1 (CAN1_TX) → CAN transceiver TXD  (e.g. SN65HVD230)
 *   PD0 (CAN1_RX) → CAN transceiver RXD
 *   Transceiver CANH / CANL → CAN bus (120 Ω at each end)
 *   Transceiver VCC → 3.3 V,  GND → GND (common with the motors)
 *
 * Bus-off is recovered automatically: BUS_OFF → twai_initiate_recovery(),
 * BUS_RECOVERED → twai_start().
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "twai.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define SERIAL_BAUDRATE 115200
#define MAX_LINE_LENGTH 80

#define CAN_TX_PIN PD1
#define CAN_RX_PIN PD0

// How long to wait for the frame to be ACKed before replying
#define TX_ACK_TIMEOUT_MS 20

char serialBuffer[MAX_LINE_LENGTH];
uint8_t serialIndex = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(SERIAL_BAUDRATE);

  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {
    // Wait briefly for Serial Monitor
  }

  Serial.println();
  Serial.println("================================");
  Serial.println(" STM32 NUCLEO F207ZG CAN BRIDGE");
  Serial.println("================================");
  Serial.println("Serial baud: 115200");
  Serial.println("CAN baud:    1000000");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 5;
  g_config.rx_queue_len = 32;
  g_config.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
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

  Serial.println("TWAI started.");
  Serial.println();
  Serial.println("Command format: ID#DATA");
  Serial.println("Example: 123#11223344");
  Serial.println("Example: 001#FFFFFFFFFFFFFFFC");
  Serial.println();
  Serial.println("Use Newline as Serial Monitor line ending.");
  Serial.println("================================");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  readSerialCommands();
  readCANMessages();
  handleAlerts(0);
  reportCANStatus();
}

// =====================================================
// ALERTS + BUS-OFF RECOVERY
// Returns the alerts that were read. timeoutMs = 0 just polls.
// =====================================================

uint32_t handleAlerts(uint32_t timeoutMs) {
  uint32_t alerts = 0;

  if (twai_read_alerts(&alerts, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) {
    return 0;
  }

  if (alerts & TWAI_ALERT_ERR_PASS) {
    Serial.println("CANALERT error passive");
  }
  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    Serial.println("CANALERT error active");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("CANALERT RX queue full");
  }
  if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
    Serial.println("CANALERT RX FIFO overrun");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.println("CANALERT bus-off, starting recovery");
    twai_initiate_recovery();
  }
  if (alerts & TWAI_ALERT_RECOVERY_IN_PROGRESS) {
    Serial.println("CANALERT recovery in progress (waiting for 128x11 recessive bits)");
  }
  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    Serial.println("CANALERT bus recovered, restarting");
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
      Serial.print("ERROR: TWAI restart failed: ");
      Serial.println(esp_err_to_name(err));
    }
  }

  return alerts;
}

// =====================================================
// CAN ERROR STATUS
// Prints state and error counters whenever the state or the
// error level (active / warning / passive) changes, at most
// every 200 ms.
// =====================================================

void reportCANStatus() {
  static int lastKey = -1;
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 200) {
    return;
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    return;
  }

  uint32_t worst = max(status.tx_error_counter, status.rx_error_counter);
  int level = worst >= 128 ? 2 : (worst >= 96 ? 1 : 0);
  int key = status.state * 4 + level;

  if (key == lastKey) {
    return;
  }
  lastKey = key;
  lastPrint = millis();

  Serial.print("CANSTAT STATE=");
  switch (status.state) {
    case TWAI_STATE_STOPPED:    Serial.print("STOPPED"); break;
    case TWAI_STATE_RUNNING:    Serial.print("RUNNING"); break;
    case TWAI_STATE_BUS_OFF:    Serial.print("BUS_OFF"); break;
    case TWAI_STATE_RECOVERING: Serial.print("RECOVERING"); break;
  }
  Serial.print(" TEC=");
  Serial.print(status.tx_error_counter);
  Serial.print(" REC=");
  Serial.print(status.rx_error_counter);
  Serial.print(" BUSERR=");
  Serial.print(status.bus_error_count);
  Serial.print(" TXFAIL=");
  Serial.print(status.tx_failed_count);
  Serial.print(" TXPEND=");
  Serial.println(status.msgs_to_tx);
}

// =====================================================
// READ SERIAL INPUT
// =====================================================

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuffer[serialIndex] = '\0';

      if (serialIndex > 0) {
        processCommand(serialBuffer);
      }

      serialIndex = 0;
      continue;
    }

    if (serialIndex < MAX_LINE_LENGTH - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      Serial.println("ERROR: Command too long");
      serialIndex = 0;
    }
  }
}

// =====================================================
// HEX CONVERSION
// =====================================================

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// =====================================================
// PROCESS COMMAND: ID#DATA
// Example: 123#11223344
// =====================================================

void processCommand(char *command) {
  char *separator = strchr(command, '#');

  if (separator == NULL) {
    Serial.println("ERROR: Missing # separator");
    return;
  }

  *separator = '\0';

  char *idString = command;
  char *dataString = separator + 1;

  if (strlen(idString) == 0) {
    Serial.println("ERROR: Missing CAN ID");
    return;
  }

  if (strlen(dataString) % 2 != 0) {
    Serial.println("ERROR: Data needs pairs of hex digits");
    return;
  }

  // Parse standard 11-bit CAN ID
  char *endPtr;
  unsigned long canId = strtoul(idString, &endPtr, 16);

  if (*endPtr != '\0' || canId > 0x7FF) {
    Serial.println("ERROR: Invalid CAN ID (000-7FF)");
    return;
  }

  size_t dataLength = strlen(dataString) / 2;

  if (dataLength > 8) {
    Serial.println("ERROR: Maximum CAN payload is 8 bytes");
    return;
  }

  twai_message_t txMsg = {};
  txMsg.identifier = canId;
  txMsg.data_length_code = dataLength;
  txMsg.extd = 0;
  txMsg.rtr = 0;

  for (size_t i = 0; i < dataLength; i++) {
    int high = hexValue(dataString[i * 2]);
    int low  = hexValue(dataString[i * 2 + 1]);

    if (high < 0 || low < 0) {
      Serial.println("ERROR: Invalid hexadecimal data");
      return;
    }

    txMsg.data[i] = (high << 4) | low;
  }

  esp_err_t err = twai_transmit(&txMsg, 0);

  if (err == ESP_ERR_TIMEOUT) {
    printTxLine(txMsg);
    Serial.println(" ERROR: TX queue full (frames not being ACKed?)");
    return;
  }
  if (err != ESP_OK) {
    printTxLine(txMsg);
    Serial.print(" ERROR: CAN transmit failed: ");
    Serial.println(esp_err_to_name(err));
    return;
  }

  // Wait for the controller to report whether another node ACKed it.
  // Alerts seen meanwhile are printed first, so the TX line stays whole.
  const char *result = " TX pending (no ACK yet)";
  unsigned long start = millis();
  while (millis() - start < TX_ACK_TIMEOUT_MS) {
    uint32_t alerts = handleAlerts(TX_ACK_TIMEOUT_MS - (millis() - start));

    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      result = " TX ok (ACKed)";
      break;
    }
    if (alerts & TWAI_ALERT_TX_FAILED) {
      result = " ERROR: TX failed";
      break;
    }
  }

  printTxLine(txMsg);
  Serial.println(result);
}

void printTxLine(twai_message_t &msg) {
  Serial.print("TX ID=0x");
  Serial.print(msg.identifier, HEX);
  Serial.print(" DLC=");
  Serial.print(msg.data_length_code);
  Serial.print(" DATA=");
  printData(msg.data, msg.data_length_code);
}

// =====================================================
// READ CAN RX
// =====================================================

void readCANMessages() {
  twai_message_t rxMsg;

  while (twai_receive(&rxMsg, 0) == ESP_OK) {
    Serial.print("RX ID=0x");
    Serial.print(rxMsg.identifier, HEX);

    Serial.print(" DLC=");
    Serial.print(rxMsg.data_length_code);

    Serial.print(" DATA=");
    if (!rxMsg.rtr) {
      printData(rxMsg.data, min((int)rxMsg.data_length_code, TWAI_FRAME_MAX_DLC));
    }

    if (rxMsg.extd) {
      Serial.print(" EXT");
    } else {
      Serial.print(" STD");
    }

    if (rxMsg.rtr) {
      Serial.print(" RTR");
    }

    Serial.println();
  }
}

// =====================================================
// PRINT DATA BYTES
// =====================================================

void printData(uint8_t *data, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }

    Serial.print(data[i], HEX);

    if (i < length - 1) {
      Serial.print(' ');
    }
  }
}
