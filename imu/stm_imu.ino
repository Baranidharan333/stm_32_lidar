#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include <LwIP.h>
#include <STM32Ethernet.h>
#include <EthernetUdp.h>

// ============================================================
// BNO085 CONFIGURATION
// ============================================================

#define BNO08X_RESET   -1
#define BNO_INT_PIN    D7

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ============================================================
// I2C PINS
// ============================================================

#define BNO_SDA D14
#define BNO_SCL D15

// ============================================================
// ETHERNET CONFIGURATION
// ============================================================

byte mac[] = {
  0x02,
  0x12,
  0x34,
  0x56,
  0x78,
  0x20
};

IPAddress localIP(192, 168, 10, 20);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 10, 1);

IPAddress laptopIP(192, 168, 10, 10);

EthernetUDP udp;

const uint16_t UDP_PORT = 5005;

// ============================================================
// RELAY & PUSH BUTTON CONFIGURATION
// ============================================================

#define BUTTON_PIN      D2    // Push button input pin (Internal INPUT_PULLUP)
#define BUTTON_LED_PIN  D3    // Button LED indicator output pin
#define RELAY_1_PIN     D4    // Relay Channel 1 output pin
#define RELAY_2_PIN     D5    // Relay Channel 2 output pin

// Active LOW for standard relay modules (LOW = Relay ON, HIGH = Relay OFF)
#define RELAY_ACTIVE    LOW
#define RELAY_INACTIVE  HIGH

#define BUTTON_PRESSED  LOW

// Relay states
bool relay1State = false; // false = OFF, true = ON
bool relay2State = false; // false = OFF, true = ON

// Button state machine
bool lastButtonRawState = HIGH;
uint32_t buttonPressStartTime = 0;
bool isHoldingButton = false;

// ============================================================
// IMU DATA
// ============================================================

// Quaternion
float qw = 1.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;

// Gyroscope [rad/s]
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

// Accelerometer [m/s^2]
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

// ============================================================
// SENSOR EVENT COUNTERS
// ============================================================

uint32_t accelEvents = 0;
uint32_t gyroEvents = 0;
uint32_t quatEvents = 0;

uint32_t totalEvents = 0;

// ============================================================
// PACKET COUNTERS
// ============================================================

uint32_t sequence = 0;
uint32_t packetsSent = 0;

// ============================================================
// 55-BYTE PACKET
//
// Python:
//
// <2sBII10fI
//
// 2 bytes  sync
// 1 byte   version
// 4 bytes  sequence
// 4 bytes  device time
// 40 bytes 10 floats
// 4 bytes  CRC32
//
// TOTAL = 55 bytes
// ============================================================

struct __attribute__((packed)) IMUPacket
{
  uint8_t sync1;
  uint8_t sync2;

  uint8_t version;
  uint8_t reserved; // 1-byte alignment padding for ARM Cortex-M 4-byte boundary

  uint32_t sequence;
  uint32_t device_time_us;

  // Quaternion
  float qw;
  float qx;
  float qy;
  float qz;

  // Gyroscope
  float gx;
  float gy;
  float gz;

  // Accelerometer
  float ax;
  float ay;
  float az;

  uint32_t crc32;
};

static_assert(
  sizeof(IMUPacket) == 56,
  "ERROR: IMUPacket is not 56 bytes!"
);

// ============================================================
// CRC32
// Compatible with Python:
//
// zlib.crc32(data) & 0xFFFFFFFF
// ============================================================

uint32_t crc32(
  const uint8_t *data,
  size_t length
)
{
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 1)
      {
        crc =
          (crc >> 1) ^
          0xEDB88320;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

// ============================================================
// ENABLE BNO085 REPORTS
// ============================================================

void enableReports()
{
  Serial.println();
  Serial.println("Enabling BNO085 reports...");

  // ----------------------------------------------------------
  // Game Rotation Vector
  // 50 Hz
  // ----------------------------------------------------------

  if (
    !bno08x.enableReport(
      SH2_GAME_ROTATION_VECTOR,
      20000
    )
  )
  {
    Serial.println(
      "ERROR: Game Rotation Vector failed"
    );
  }
  else
  {
    Serial.println(
      "Game Rotation Vector: 50 Hz"
    );
  }

  // ----------------------------------------------------------
  // Calibrated Gyroscope
  // 50 Hz
  // ----------------------------------------------------------

  if (
    !bno08x.enableReport(
      SH2_GYROSCOPE_CALIBRATED,
      20000
    )
  )
  {
    Serial.println(
      "ERROR: Gyroscope failed"
    );
  }
  else
  {
    Serial.println(
      "Calibrated Gyroscope: 50 Hz"
    );
  }

  // ----------------------------------------------------------
  // Accelerometer
  // 50 Hz
  // ----------------------------------------------------------

  if (
    !bno08x.enableReport(
      SH2_ACCELEROMETER,
      20000
    )
  )
  {
    Serial.println(
      "ERROR: Accelerometer failed"
    );
  }
  else
  {
    Serial.println(
      "Accelerometer: 50 Hz"
    );
  }

  Serial.println(
    "BNO085 reports enabled"
  );
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "=========================================="
  );
  Serial.println(
    "STM32F207ZG + BNO085 + Ethernet"
  );
  Serial.println(
    "=========================================="
  );

  // ==========================================================
  // I2C
  // ==========================================================

  Serial.println();
  Serial.println("Starting I2C...");

  Wire.setSDA(BNO_SDA);
  Wire.setSCL(BNO_SCL);

  Wire.begin();

  Wire.setClock(400000);
  Wire.setTimeout(10);

  pinMode(
    BNO_INT_PIN,
    INPUT
  );

  Serial.println(
    "I2C initialized"
  );

  Serial.print(
    "SDA: "
  );
  Serial.println(BNO_SDA);

  Serial.print(
    "SCL: "
  );
  Serial.println(BNO_SCL);

  Serial.print(
    "INT: "
  );
  Serial.println(BNO_INT_PIN);

  // ==========================================================
  // BNO085
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting BNO085..."
  );

  if (
    !bno08x.begin_I2C(
      0x4A,
      &Wire
    )
  )
  {
    Serial.println();
    Serial.println(
      "ERROR: BNO085 NOT FOUND!"
    );

    while (1)
    {
      delay(1000);
    }
  }

  Serial.println(
    "BNO085 detected"
  );

  delay(500);

  enableReports();

  // ==========================================================
  // ETHERNET
  // ==========================================================

  Serial.println();
  Serial.println(
    "Starting Ethernet..."
  );

  Ethernet.begin(
    localIP,
    subnet,
    gateway
  );

  delay(500);

  Serial.print(
    "STM32 IP: "
  );
  Serial.println(
    Ethernet.localIP()
  );

  Serial.print(
    "Subnet: "
  );
  Serial.println(
    Ethernet.subnetMask()
  );

  Serial.print(
    "Gateway: "
  );
  Serial.println(
    Ethernet.gatewayIP()
  );

  // ==========================================================
  // UDP
  // ==========================================================

  udp.begin(
    UDP_PORT
  );

  Serial.println();
  Serial.println(
    "UDP started"
  );

  Serial.print(
    "Destination: "
  );
  Serial.print(
    laptopIP
  );
  Serial.print(":");
  Serial.println(
    UDP_PORT
  );

  setupRelayAndButton();

  Serial.println();
  Serial.println(
    "Streaming started..."
  );
}
// ============================================================
// SETUP RELAY AND BUTTON
// ============================================================

void setupRelayAndButton()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);

  // Set initial relay states (OFF)
  digitalWrite(RELAY_1_PIN, RELAY_INACTIVE);
  digitalWrite(RELAY_2_PIN, RELAY_INACTIVE);

  // LED initial state (OFF)
  digitalWrite(BUTTON_LED_PIN, LOW);

  Serial.println();
  Serial.println("Relay and Button Subsystem Initialized:");
  Serial.print("  Button Pin  : "); Serial.println(BUTTON_PIN);
  Serial.print("  LED Pin     : "); Serial.println(BUTTON_LED_PIN);
  Serial.print("  Relay 1 Pin : "); Serial.println(RELAY_1_PIN);
  Serial.print("  Relay 2 Pin : "); Serial.println(RELAY_2_PIN);
}

// ============================================================
// UPDATE RELAY AND BUTTON (Debounced & Robust)
// ============================================================

void updateRelayAndButton()
{
  uint32_t now = millis();
  bool rawReading = digitalRead(BUTTON_PIN);

  static bool debouncedState = HIGH;
  static bool lastRawReading = HIGH;
  static uint32_t lastDebounceTime = 0;

  // ----------------------------------------------------------
  // Debounce Filtering (Ignore noise pulses < 40ms)
  // ----------------------------------------------------------
  if (rawReading != lastRawReading)
  {
    lastDebounceTime = now;
    lastRawReading = rawReading;
  }

  if ((now - lastDebounceTime) > 40)
  {
    if (rawReading != debouncedState)
    {
      debouncedState = rawReading;

      // Detect Press (HIGH -> LOW)
      if (debouncedState == BUTTON_PRESSED)
      {
        buttonPressStartTime = now;
        isHoldingButton = true;
        Serial.println();
        Serial.println(">>> Button Pressed! <<<");
      }
      // Detect Release (LOW -> HIGH)
      else if (debouncedState != BUTTON_PRESSED && isHoldingButton)
      {
        uint32_t totalHoldTime = now - buttonPressStartTime;
        isHoldingButton = false;

        Serial.println();
        Serial.print(">>> Button Released after ");
        Serial.print(totalHoldTime);
        Serial.println(" ms <<<");

        if (totalHoldTime >= 1000)
        {
          // Long Press (>= 1 second) -> Toggle Relay 2
          relay2State = !relay2State;
          digitalWrite(RELAY_2_PIN, relay2State ? RELAY_ACTIVE : RELAY_INACTIVE);

          Serial.println("==========================================");
          Serial.print("ACTION: RELAY 2 TOGGLED -> ");
          Serial.println(relay2State ? "ON [ACTIVE]" : "OFF [INACTIVE]");
          Serial.println("==========================================");
        }
        else
        {
          // Short Click (< 1 second) -> Toggle Relay 1
          relay1State = !relay1State;
          digitalWrite(RELAY_1_PIN, relay1State ? RELAY_ACTIVE : RELAY_INACTIVE);

          Serial.println("==========================================");
          Serial.print("ACTION: RELAY 1 TOGGLED -> ");
          Serial.println(relay1State ? "ON [ACTIVE]" : "OFF [INACTIVE]");
          Serial.println("==========================================");
        }
      }
    }
  }

  // ----------------------------------------------------------
  // Visual LED Feedback
  // ----------------------------------------------------------
  if (debouncedState == BUTTON_PRESSED && isHoldingButton)
  {
    uint32_t holdDuration = now - buttonPressStartTime;

    if (holdDuration < 1000)
    {
      // 0..1s: Solid ON (Relay 1 selected)
      digitalWrite(BUTTON_LED_PIN, HIGH);
    }
    else
    {
      // >= 1s: Fast Blink (Relay 2 selected)
      digitalWrite(BUTTON_LED_PIN, (now / 60) % 2 ? HIGH : LOW);
    }
  }
  else if (!isHoldingButton)
  {
    // Idle LED state: Solid ON if any relay is active, else OFF
    if (relay1State || relay2State)
    {
      digitalWrite(BUTTON_LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(BUTTON_LED_PIN, LOW);
    }
  }
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // Process Relay and Button State Machine
  updateRelayAndButton();
  // ==========================================================
  // BNO085 RESET DETECTION
  // ==========================================================

  if (bno08x.wasReset())
  {
    Serial.println();
    Serial.println(
      "!!! BNO085 RESET DETECTED !!!"
    );

    // Reset local event counters
    accelEvents = 0;
    gyroEvents = 0;
    quatEvents = 0;

    // Re-enable reports
    enableReports();

    Serial.println(
      "BNO085 reports re-enabled"
    );
  }



  if (
    digitalRead(BNO_INT_PIN) == LOW
  )
  {
    uint8_t eventCount = 0;

    while (
      digitalRead(BNO_INT_PIN) == LOW &&
      eventCount < 20
    )
    {
      // ------------------------------------------------------
      // Read one event
      // ------------------------------------------------------

      if (
        !bno08x.getSensorEvent(
          &sensorValue
        )
      )
      {
        // Non-sensor SHTP packet processed internally by SH2 library.
        // Keep draining buffer if INT pin is still LOW!
        continue;
      }

      eventCount++;
      totalEvents++;

      // ======================================================
      // GAME ROTATION VECTOR
      // ======================================================

      if (
        sensorValue.sensorId ==
        SH2_GAME_ROTATION_VECTOR
      )
      {
        qw =
          sensorValue
            .un
            .gameRotationVector
            .real;

        qx =
          sensorValue
            .un
            .gameRotationVector
            .i;

        qy =
          sensorValue
            .un
            .gameRotationVector
            .j;

        qz =
          sensorValue
            .un
            .gameRotationVector
            .k;

        quatEvents++;
      }

      // ======================================================
      // CALIBRATED GYROSCOPE
      // ======================================================

      else if (
        sensorValue.sensorId ==
        SH2_GYROSCOPE_CALIBRATED
      )
      {
        gx =
          sensorValue
            .un
            .gyroscope
            .x;

        gy =
          sensorValue
            .un
            .gyroscope
            .y;

        gz =
          sensorValue
            .un
            .gyroscope
            .z;

        gyroEvents++;
      }

      // ======================================================
      // ACCELEROMETER
      // ======================================================

      else if (
        sensorValue.sensorId ==
        SH2_ACCELEROMETER
      )
      {
        ax =
          sensorValue
            .un
            .accelerometer
            .x;

        ay =
          sensorValue
            .un
            .accelerometer
            .y;

        az =
          sensorValue
            .un
            .accelerometer
            .z;

        accelEvents++;
      }
    }
  }

  // ==========================================================
  // SENSOR FROZEN WATCHDOG
  //
  // If no new sensor events received for 1000 ms, auto-re-enable reports!
  // ==========================================================

  static uint32_t lastEventTime = 0;
  static uint32_t prevEventsCount = 0;

  if (totalEvents != prevEventsCount)
  {
    prevEventsCount = totalEvents;
    lastEventTime = millis();
  }
  else if (millis() - lastEventTime >= 1000)
  {
    lastEventTime = millis();
    Serial.println();
    Serial.println("!!! WARNING: BNO085 FROZEN (No events for 1s) !!!");
    Serial.println("Auto re-enabling BNO085 reports...");
    enableReports();
  }

  // ==========================================================
  // UDP PACKET
  //
  // Send latest sensor data every 20 ms = 50 Hz
  // ==========================================================

  static uint32_t lastSend = 0;

  uint32_t nowMillis = millis();

  if (
    nowMillis - lastSend >= 20
  )
  {
    lastSend = nowMillis;

    IMUPacket packet;

    // --------------------------------------------------------
    // Header
    // --------------------------------------------------------

    packet.sync1 = 0xAA;
    packet.sync2 = 0x55;

    packet.version = 3;
    packet.reserved = 0;

    // --------------------------------------------------------
    // Sequence
    // --------------------------------------------------------

    packet.sequence =
      sequence++;

    // --------------------------------------------------------
    // STM32 device time
    // --------------------------------------------------------

    packet.device_time_us =
      micros();

    // --------------------------------------------------------
    // Quaternion
    // --------------------------------------------------------

    packet.qw = qw;
    packet.qx = qx;
    packet.qy = qy;
    packet.qz = qz;

    // --------------------------------------------------------
    // Gyroscope
    // --------------------------------------------------------

    packet.gx = gx;
    packet.gy = gy;
    packet.gz = gz;

    // --------------------------------------------------------
    // Accelerometer
    // --------------------------------------------------------

    packet.ax = ax;
    packet.ay = ay;
    packet.az = az;

    // --------------------------------------------------------
    // CRC32
    //
    // CRC covers first 51 bytes.
    // --------------------------------------------------------

    packet.crc32 =
      crc32(
        reinterpret_cast<uint8_t *>(&packet),
        sizeof(IMUPacket) -
        sizeof(uint32_t)
      );

    // --------------------------------------------------------
    // UDP SEND
    // --------------------------------------------------------

    udp.beginPacket(
      laptopIP,
      UDP_PORT
    );

    udp.write(
      reinterpret_cast<uint8_t *>(&packet),
      sizeof(IMUPacket)
    );

    udp.endPacket();

    packetsSent++;
  }

  // ==========================================================
  // LwIP SERVICE
  // ==========================================================

  Ethernet.schedule();

  // ==========================================================
  // DEBUG STATUS
  //
  // Print every 2 seconds.
  // ==========================================================

  static uint32_t lastStatus = 0;

  if (
    millis() - lastStatus >= 2000
  )
  {
    lastStatus = millis();

    Serial.println();
    Serial.println(
      "--------------- STATUS ---------------"
    );

    Serial.print(
      "Packets sent      : "
    );
    Serial.println(
      packetsSent
    );

    Serial.print(
      "Total BNO events  : "
    );
    Serial.println(
      totalEvents
    );

    Serial.print(
      "Accel events      : "
    );
    Serial.println(
      accelEvents
    );

    Serial.print(
      "Gyro events       : "
    );
    Serial.println(
      gyroEvents
    );

    Serial.print(
      "Quaternion events : "
    );
    Serial.println(
      quatEvents
    );

    Serial.print(
      "INT state         : "
    );
    Serial.println(
      digitalRead(BNO_INT_PIN)
        == LOW
        ? "LOW"
        : "HIGH"
    );

    Serial.print(
      "Relay 1 State     : "
    );
    Serial.println(
      relay1State ? "ON [ACTIVE]" : "OFF [INACTIVE]"
    );

    Serial.print(
      "Relay 2 State     : "
    );
    Serial.println(
      relay2State ? "ON [ACTIVE]" : "OFF [INACTIVE]"
    );

    Serial.println();

    Serial.println(
      "Quaternion:"
    );

    Serial.print(
      "  qw = "
    );
    Serial.println(
      qw,
      6
    );

    Serial.print(
      "  qx = "
    );
    Serial.println(
      qx,
      6
    );

    Serial.print(
      "  qy = "
    );
    Serial.println(
      qy,
      6
    );

    Serial.print(
      "  qz = "
    );
    Serial.println(
      qz,
      6
    );

    Serial.println();

    Serial.println(
      "Gyroscope [rad/s]:"
    );

    Serial.print(
      "  X = "
    );
    Serial.println(
      gx,
      6
    );

    Serial.print(
      "  Y = "
    );
    Serial.println(
      gy,
      6
    );

    Serial.print(
      "  Z = "
    );
    Serial.println(
      gz,
      6
    );

    Serial.println();
// ============================================================
//
// RPLidar S2 Driver for STM32F207ZG (NUCLEO-144)
// Arduino IDE - Self-Contained Implementation
//
// Protocol Reference:
//   SLAMTEC RPLidar Communication Protocol
//   Based on rplidar_sdk and ESP-rplidarS2 library analysis
//
// Hardware:
//   Board:  NUCLEO-F207ZG (STM32F207ZG)
//   LiDAR:  SLAMTEC RPLidar S2
//   IDE:    Arduino IDE with STM32duino core
//
// Wiring:
//   RPLidar S2        NUCLEO-F207ZG
//   ────────────────────────
─────────
//   GND (Black)   →   GND
//   VCC (Red)     →   5V (or external 5V supply)
//   TX  (Yellow)  →   D0 (PG9,  USART6_RX) ← CN10 Pin 1  *** MCU receives from LiDAR ***
//   RX  (Green)   →   D1 (PG14, USART6_TX) ← CN10 Pin 2  *** MCU sends to LiDAR ***
//
// UART Mapping:
//   Serial  → USART3 (PD8/PD9) → ST-LINK USB → Debug output
//   USART6  → PG9(RX=D0) / PG14(TX=D1) → RPLidar S2
//
// NOTE:
//   The RPLidar S2 uses 1,000,000 baud (1M bps), NOT 115200.
//   The S2 has an integrated motor controller (no external PWM needed).
//   The motor starts when a SCAN command is issued and stops on STOP.
//
// ============================================================

// ============================================================
// RPLIDAR SERIAL PORT
// ============================================================

// USART6: PG9 (D0 = USART6_RX) / PG14 (D1 = USART6_TX)
// Instantiate Uart directly to bypass precompiled core limitations
#include "Serial.h"
Uart LidarSerial(PG_9, PG_14);

#define RPLIDAR_SERIAL   LidarSerial
#define RPLIDAR_BAUDRATE 1000000

#define DEBUG_SERIAL     Serial
#define DEBUG_BAUDRATE   115200

// ============================================================
// RPLIDAR PROTOCOL CONSTANTS
// ============================================================

// Sync bytes
#define RPLIDAR_CMD_SYNC_BYTE     0xA5
#define RPLIDAR_ANS_SYNC_BYTE1    0xA5
#define RPLIDAR_ANS_SYNC_BYTE2    0x5A

// Command flag
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80

// Command codes (Host → LiDAR)
#define RPLIDAR_CMD_STOP              0x25
#define RPLIDAR_CMD_RESET             0x40
#define RPLIDAR_CMD_SCAN              0x20
#define RPLIDAR_CMD_FORCE_SCAN        0x21
#define RPLIDAR_CMD_GET_DEVICE_INFO   0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH 0x52

// Response type codes
#define RPLIDAR_ANS_TYPE_MEASUREMENT  0x81
#define RPLIDAR_ANS_TYPE_DEVINFO      0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH    0x06

// Measurement data bit masks
#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        0x01
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       0x01
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

// Health status codes
#define RPLIDAR_STATUS_OK       0
#define RPLIDAR_STATUS_WARNING  1
#define RPLIDAR_STATUS_ERROR    2

// Timeouts (milliseconds)
#define RPLIDAR_TIMEOUT_DEFAULT  1000
#define RPLIDAR_TIMEOUT_SCAN     2000

// ============================================================
// DATA STRUCTURES (packed for binary protocol parsing)
// ============================================================


// Response descriptor (7 bytes from LiDAR)
struct __attribute__((packed)) RplidarResponseDescriptor
{
  uint8_t  syncByte1;   // Must be 0xA5
  uint8_t  syncByte2;   // Must be 0x5A
  uint32_t sizeAndMode; // [29:0] = size, [31:30] = send mode
  uint8_t  dataType;    // Response type code
};

// Standard scan measurement node (5 bytes)
struct __attribute__((packed)) RplidarMeasurementNode
{
  uint8_t  sync_quality;      // [0] sync bit, [1] !sync, [7:2] quality
  uint16_t angle_q6_checkbit; // [0] check bit (=1), [15:1] angle in Q6
  uint16_t distance_q2;       // Distance in Q2 format (mm * 4)
};

// Device info response (20 bytes)
struct __attribute__((packed)) RplidarDeviceInfo
{
  uint8_t  model;
  uint16_t firmware_version;
  uint8_t  hardware_version;
  uint8_t  serialnum[16];
};

// Device health response (3 bytes)
struct __attribute__((packed)) RplidarDeviceHealth
{
  uint8_t  status;
  uint16_t error_code;
};

// Parsed scan point (for application use)
struct LidarPoint
{
  float   angle;     // Degrees [0.0 - 360.0)
  float   distance;  // Millimeters (0 = invalid)
  uint8_t quality;   // Signal quality [0-63]
  bool    startBit;  // true = first point of new 360° scan
};

// ============================================================
// GLOBAL STATE
// ============================================================

bool     lidarConnected = false;
bool     scanActive     = false;
uint32_t totalPoints    = 0;
uint32_t validPoints    = 0;
uint32_t scanCount      = 0;

// Timing for scan rate measurement
uint32_t lastScanStartTime = 0;
float    measuredScanHz    = 0.0f;

// Point counter within current scan
uint32_t pointsInCurrentScan = 0;

// ============================================================
// RPLIDAR PROTOCOL FUNCTIONS
// ============================================================

// ----------------------------------------------------------
// Send a command without payload
// ----------------------------------------------------------

void rplidarSendCommand(uint8_t cmd)
{
  uint8_t pkt[2];

  pkt[0] = RPLIDAR_CMD_SYNC_BYTE;
  pkt[1] = cmd;

  RPLIDAR_SERIAL.write(pkt, 2);
}

// ----------------------------------------------------------
// Send a command with payload
// ----------------------------------------------------------

void rplidarSendCommandWithPayload(
  uint8_t cmd,
  const uint8_t *payload,
  uint8_t payloadSize
)
{
  uint8_t cmdWithFlag = cmd | RPLIDAR_CMDFLAG_HAS_PAYLOAD;

  // Calculate checksum (XOR of all bytes)
  uint8_t checksum = 0;
  checksum ^= RPLIDAR_CMD_SYNC_BYTE;
  checksum ^= cmdWithFlag;
  checksum ^= payloadSize;

  for (uint8_t i = 0; i < payloadSize; i++)
  {
    checksum ^= payload[i];
  }

  // Send: sync + cmd + size + payload + checksum
  RPLIDAR_SERIAL.write(RPLIDAR_CMD_SYNC_BYTE);
  RPLIDAR_SERIAL.write(cmdWithFlag);
  RPLIDAR_SERIAL.write(payloadSize);
  RPLIDAR_SERIAL.write(payload, payloadSize);
  RPLIDAR_SERIAL.write(checksum);
}

// ----------------------------------------------------------
// Clear the serial RX buffer
// ----------------------------------------------------------

void rplidarClearBuffer()
{
  while (RPLIDAR_SERIAL.available())
  {
    RPLIDAR_SERIAL.read();
  }
}

// ----------------------------------------------------------
// Wait for response descriptor (7 bytes)
//
// Returns true if valid descriptor received
// ----------------------------------------------------------

bool rplidarWaitDescriptor(
  RplidarResponseDescriptor *desc,
  uint32_t timeoutMs
)
{
  uint8_t *buf = (uint8_t *)desc;
  uint8_t recvPos = 0;
  uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs)
  {
    if (RPLIDAR_SERIAL.available() <= 0)
    {
      continue;
    }

    int b = RPLIDAR_SERIAL.read();

    if (b < 0)
    {
      continue;
    }

    switch (recvPos)
    {
      case 0:
        if ((uint8_t)b != RPLIDAR_ANS_SYNC_BYTE1)
        {
          continue;
        }
        break;

      case 1:
        if ((uint8_t)b != RPLIDAR_ANS_SYNC_BYTE2)
        {
          recvPos = 0;
          continue;
        }
        break;
    }

    buf[recvPos++] = (uint8_t)b;

    if (recvPos == sizeof(RplidarResponseDescriptor))
    {
      return true;
    }
  }

  return false; // Timeout
}

// ----------------------------------------------------------
// Read N bytes with timeout
// ----------------------------------------------------------

bool rplidarReadBytes(
  uint8_t *buffer,
  size_t count,
  uint32_t timeoutMs
)
{
  size_t received = 0;
  uint32_t startMs = millis();

  while (received < count)
  {
    if ((millis() - startMs) >= timeoutMs)
    {
      return false; // Timeout
    }

    if (RPLIDAR_SERIAL.available() > 0)
    {
      int b = RPLIDAR_SERIAL.read();

      if (b >= 0)
      {
        buffer[received++] = (uint8_t)b;
      }
    }
  }

  return true;
}

// ============================================================
// STOP COMMAND
// ============================================================

void rplidarStop()
{
  rplidarSendCommand(RPLIDAR_CMD_STOP);
  delay(10); // Allow device to process
  rplidarClearBuffer();
  scanActive = false;
}

// ============================================================
// RESET COMMAND
// ============================================================

void rplidarReset()
{
  rplidarSendCommand(RPLIDAR_CMD_RESET);
  delay(800); // Wait for device reboot
  rplidarClearBuffer();
  scanActive = false;
}

// ============================================================
// GET DEVICE INFO
// ============================================================

bool rplidarGetDeviceInfo(RplidarDeviceInfo *info)
{
  rplidarClearBuffer();

  rplidarSendCommand(RPLIDAR_CMD_GET_DEVICE_INFO);

  // Wait for response descriptor
  RplidarResponseDescriptor desc;

  if (!rplidarWaitDescriptor(&desc, RPLIDAR_TIMEOUT_DEFAULT))
  {
    DEBUG_SERIAL.println(
      "ERROR: Device info descriptor timeout"
    );
    return false;
  }

  // Verify response type
  if (desc.dataType != RPLIDAR_ANS_TYPE_DEVINFO)
  {
    DEBUG_SERIAL.print(
      "ERROR: Wrong response type: 0x"
    );
    DEBUG_SERIAL.println(desc.dataType, HEX);
    return false;
  }

  // Read device info payload (20 bytes)
  if (!rplidarReadBytes(
    (uint8_t *)info,
    sizeof(RplidarDeviceInfo),
    RPLIDAR_TIMEOUT_DEFAULT
  ))
  {
    DEBUG_SERIAL.println(
      "ERROR: Device info payload timeout"
    );
    return false;
  }

  return true;
}

// ============================================================
// GET DEVICE HEALTH
// ============================================================

bool rplidarGetDeviceHealth(RplidarDeviceHealth *health)
{
  rplidarClearBuffer();

  rplidarSendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH);

  // Wait for response descriptor
  RplidarResponseDescriptor desc;

  if (!rplidarWaitDescriptor(&desc, RPLIDAR_TIMEOUT_DEFAULT))
  {
    DEBUG_SERIAL.println(
      "ERROR: Health descriptor timeout"
    );
    return false;
  }

  // Verify response type
  if (desc.dataType != RPLIDAR_ANS_TYPE_DEVHEALTH)
  {
    DEBUG_SERIAL.print(
      "ERROR: Wrong response type: 0x"
    );
    DEBUG_SERIAL.println(desc.dataType, HEX);
    return false;
  }

  // Read health payload (3 bytes)
  if (!rplidarReadBytes(
    (uint8_t *)health,
    sizeof(RplidarDeviceHealth),
    RPLIDAR_TIMEOUT_DEFAULT
  ))
  {
    DEBUG_SERIAL.println(
      "ERROR: Health payload timeout"
    );
    return false;
  }

  return true;
}

// ============================================================
// START STANDARD SCAN
//
// The LiDAR will continuously stream 5-byte measurement
// nodes after the response descriptor is received.
// ============================================================

bool rplidarStartScan()
{
  rplidarStop(); // Stop any ongoing scan first

  delay(50);
  rplidarClearBuffer();

  rplidarSendCommand(RPLIDAR_CMD_SCAN);

  // Wait for response descriptor
  RplidarResponseDescriptor desc;

  if (!rplidarWaitDescriptor(&desc, RPLIDAR_TIMEOUT_SCAN))
  {
    DEBUG_SERIAL.println(
      "ERROR: Scan descriptor timeout"
    );
    return false;
  }

  // Verify measurement response type
  if (desc.dataType != RPLIDAR_ANS_TYPE_MEASUREMENT)
  {
    DEBUG_SERIAL.print(
      "ERROR: Wrong scan response type: 0x"
    );
    DEBUG_SERIAL.println(desc.dataType, HEX);
    return false;
  }

  scanActive = true;
  lastScanStartTime = millis();
  pointsInCurrentScan = 0;
  scanCount = 0;

  DEBUG_SERIAL.println("Scan started successfully");
  return true;
}

// ============================================================
// WAIT FOR SINGLE SCAN POINT
//
// Reads one 5-byte measurement node from the stream.
// Implements sync validation (byte 0 sync bits + byte 1
// check bit) to maintain frame alignment.
//
// Returns true if valid point received.
// ============================================================

bool rplidarWaitPoint(
  LidarPoint *point,
  uint32_t timeoutMs
)
{
  RplidarMeasurementNode node;
  uint8_t *nodebuf = (uint8_t *)&node;
  uint8_t recvPos = 0;
  uint32_t startMs = millis();

  while ((millis() - startMs) < timeoutMs)
  {
    int currentByte = RPLIDAR_SERIAL.read();

    if (currentByte < 0)
    {
      continue;
    }

    switch (recvPos)
    {
      case 0:
      {
        // Byte 0: sync_quality
        // Bit 0 (S) and Bit 1 (!S) must be inverse of each other
        uint8_t tmp = ((uint8_t)currentByte >> 1);

        if (!((tmp ^ (uint8_t)currentByte) & 0x01))
        {
          // Sync validation failed — skip this byte
          continue;
        }
        break;
      }

      case 1:
      {
        // Byte 1: low byte of angle_q6_checkbit
        // Bit 0 (check bit) must be 1
        if (!((uint8_t)currentByte &
              RPLIDAR_RESP_MEASUREMENT_CHECKBIT))
        {
          // Check bit failed — restart from byte 0
          recvPos = 0;
          continue;
        }
        break;
      }
    }

    nodebuf[recvPos++] = (uint8_t)currentByte;

    if (recvPos == sizeof(RplidarMeasurementNode))
    {
      // Successfully received 5-byte measurement node
      // Parse the fields

      point->distance =
        node.distance_q2 / 4.0f;

      point->angle =
        (node.angle_q6_checkbit >>
         RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

      point->quality =
        (node.sync_quality >>
         RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

      point->startBit =
        (node.sync_quality &
         RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? true : false;

      return true;
    }
  }

  return false; // Timeout
}

// ============================================================
// PRINT DEVICE INFO
// ============================================================

void printDeviceInfo(const RplidarDeviceInfo *info)
{
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "── RPLidar Device Info ──────────────"
  );

  DEBUG_SERIAL.print("  Model:     ");
  DEBUG_SERIAL.println(info->model);

  DEBUG_SERIAL.print("  Firmware:  ");
  DEBUG_SERIAL.print(
    (info->firmware_version >> 8) & 0xFF
  );
  DEBUG_SERIAL.print(".");
  DEBUG_SERIAL.println(
    info->firmware_version & 0xFF
  );

  DEBUG_SERIAL.print("  Hardware:  ");
  DEBUG_SERIAL.println(info->hardware_version);

  DEBUG_SERIAL.print("  Serial:    ");
  for (int i = 0; i < 16; i++)
  {
    if (info->serialnum[i] < 0x10)
    {
      DEBUG_SERIAL.print("0");
    }
    DEBUG_SERIAL.print(info->serialnum[i], HEX);
  }
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println(
    "─────────────────────────────────────"
  );
}

// ============================================================
// PRINT DEVICE HEALTH
// ============================================================

void printDeviceHealth(const RplidarDeviceHealth *health)
{
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "── RPLidar Device Health ────────────"
  );

  DEBUG_SERIAL.print("  Status:     ");
  switch (health->status)
  {
    case RPLIDAR_STATUS_OK:
      DEBUG_SERIAL.println("OK");
      break;

    case RPLIDAR_STATUS_WARNING:
      DEBUG_SERIAL.println("WARNING");
      break;

    case RPLIDAR_STATUS_ERROR:
      DEBUG_SERIAL.println("ERROR");
      break;

    default:
      DEBUG_SERIAL.print("UNKNOWN (");
      DEBUG_SERIAL.print(health->status);
      DEBUG_SERIAL.println(")");
      break;
  }

  DEBUG_SERIAL.print("  Error Code: 0x");
  if (health->error_code < 0x1000)
    DEBUG_SERIAL.print("0");
  if (health->error_code < 0x100)
    DEBUG_SERIAL.print("0");
  if (health->error_code < 0x10)
    DEBUG_SERIAL.print("0");
  DEBUG_SERIAL.println(health->error_code, HEX);

  DEBUG_SERIAL.println(
    "─────────────────────────────────────"
  );
}

// ============================================================
// INITIALIZE RPLIDAR
//
// Full initialization sequence:
//   1. Reset device
//   2. Get device info
//   3. Check health
//   4. Start scan
// ============================================================

bool initRplidar()
{
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "Initializing RPLidar S2..."
  );

  // ----------------------------------------------------------
  // Step 1: Reset the device
  // ----------------------------------------------------------

  DEBUG_SERIAL.println("  Resetting device...");
  rplidarReset();

  DEBUG_SERIAL.println("  Reset complete");

  // ----------------------------------------------------------
  // Step 2: Get Device Info
  // ----------------------------------------------------------

  DEBUG_SERIAL.println("  Querying device info...");

  RplidarDeviceInfo info;

  if (!rplidarGetDeviceInfo(&info))
  {
    DEBUG_SERIAL.println(
      "  FAILED: Cannot get device info"
    );
    DEBUG_SERIAL.println(
      "  Check wiring: TX(LiDAR)→PA10, RX(LiDAR)→PA9"
    );
    return false;
  }

  printDeviceInfo(&info);

  // ----------------------------------------------------------
  // Step 3: Check Health
  // ----------------------------------------------------------

  DEBUG_SERIAL.println("  Checking device health...");

  RplidarDeviceHealth health;

  if (!rplidarGetDeviceHealth(&health))
  {
    DEBUG_SERIAL.println(
      "  FAILED: Cannot get device health"
    );
    return false;
  }

  printDeviceHealth(&health);

  if (health.status == RPLIDAR_STATUS_ERROR)
  {
    DEBUG_SERIAL.println(
      "  Device reports ERROR status!"
    );
    DEBUG_SERIAL.println(
      "  Attempting reset..."
    );

    rplidarReset();

    // Re-check health
    if (!rplidarGetDeviceHealth(&health))
    {
      return false;
    }

    if (health.status == RPLIDAR_STATUS_ERROR)
    {
      DEBUG_SERIAL.println(
        "  Device still in ERROR after reset!"
      );
      return false;
    }
  }

  // ----------------------------------------------------------
  // Step 4: Start Standard Scan
  // ----------------------------------------------------------

  DEBUG_SERIAL.println("  Starting scan...");

  if (!rplidarStartScan())
  {
    DEBUG_SERIAL.println(
      "  FAILED: Cannot start scan"
    );
    return false;
  }

  lidarConnected = true;

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "RPLidar S2 initialized successfully!"
  );

  return true;
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ----------------------------------------------------------
  // Debug Serial (ST-LINK Virtual COM Port)
  // ----------------------------------------------------------

  DEBUG_SERIAL.begin(DEBUG_BAUDRATE);

  delay(1000); // Wait for serial monitor connection

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "=========================================="
  );
  DEBUG_SERIAL.println(
    " STM32F207ZG + RPLidar S2"
  );
  DEBUG_SERIAL.println(
    "=========================================="
  );

  DEBUG_SERIAL.print("Debug Serial: ");
  DEBUG_SERIAL.print(DEBUG_BAUDRATE);
  DEBUG_SERIAL.println(" baud");

  // ----------------------------------------------------------
  // RPLidar Serial (USART1: PA9/PA10)
  // ----------------------------------------------------------

  DEBUG_SERIAL.print("LiDAR Serial: ");
  DEBUG_SERIAL.print(RPLIDAR_BAUDRATE);
  DEBUG_SERIAL.println(" baud (Serial1, PA9/PA10)");

  RPLIDAR_SERIAL.begin(RPLIDAR_BAUDRATE);

  delay(100);

  // ----------------------------------------------------------
  // Initialize RPLidar
  // ----------------------------------------------------------

  if (!initRplidar())
  {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(
      "!!! RPLIDAR INIT FAILED !!!"
    );
    DEBUG_SERIAL.println(
      "Check wiring and power supply."
    );
    DEBUG_SERIAL.println(
      "Will retry every 3 seconds..."
    );
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "=========================================="
  );
  DEBUG_SERIAL.println(
    " Streaming LiDAR Data"
  );
  DEBUG_SERIAL.println(
    "=========================================="
  );
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(
    "Format: Angle(deg) | Distance(mm) | Quality"
  );
  DEBUG_SERIAL.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Reconnect if not connected
  // ----------------------------------------------------------

  if (!lidarConnected || !scanActive)
  {
    DEBUG_SERIAL.println(
      "Attempting to reconnect RPLidar..."
    );

    RPLIDAR_SERIAL.end();
    delay(500);
    RPLIDAR_SERIAL.begin(RPLIDAR_BAUDRATE);
    delay(100);

    if (initRplidar())
    {
      DEBUG_SERIAL.println(
        "Reconnected successfully!"
      );
    }
    else
    {
      delay(3000); // Wait before retry
      return;
    }
  }

  // ----------------------------------------------------------
  // Read scan data points
  // ----------------------------------------------------------

  LidarPoint point;

  if (rplidarWaitPoint(&point, RPLIDAR_TIMEOUT_DEFAULT))
  {
    totalPoints++;

    // --------------------------------------------------------
    // Detect new 360° scan (start bit = 1)
    // --------------------------------------------------------

    if (point.startBit)
    {
      // Calculate scan rate
      uint32_t now = millis();

      if (lastScanStartTime > 0)
      {
        uint32_t elapsed = now - lastScanStartTime;

        if (elapsed > 0)
        {
          measuredScanHz = 1000.0f / (float)elapsed;
        }

        // Print scan summary every scan
        scanCount++;

        if ((scanCount % 10) == 0)
        {
          DEBUG_SERIAL.println();
          DEBUG_SERIAL.println(
            "────── Scan Statistics ──────"
          );

          DEBUG_SERIAL.print("  Scan #");
          DEBUG_SERIAL.println(scanCount);

          DEBUG_SERIAL.print("  Points/Scan: ");
          DEBUG_SERIAL.println(pointsInCurrentScan);

          DEBUG_SERIAL.print("  Scan Rate:   ");
          DEBUG_SERIAL.print(measuredScanHz, 1);
          DEBUG_SERIAL.println(" Hz");

          DEBUG_SERIAL.print("  Total Points: ");
          DEBUG_SERIAL.println(totalPoints);

          DEBUG_SERIAL.print("  Valid Points: ");
          DEBUG_SERIAL.println(validPoints);

          DEBUG_SERIAL.println(
            "─────────────────────────────"
          );
          DEBUG_SERIAL.println();
        }
      }

      lastScanStartTime = now;
      pointsInCurrentScan = 0;
    }

    pointsInCurrentScan++;

    // --------------------------------------------------------
    // Process valid points (distance > 0)
    // --------------------------------------------------------

    if (point.distance > 0.0f)
    {
      validPoints++;

      // Print a sample of points to avoid overflowing the 115200 baud serial output
      // (Full 32k sample stream exceeds 115200 baud capacity)
      if ((validPoints % 20) == 0)
      {
        DEBUG_SERIAL.print(point.angle, 2);
        DEBUG_SERIAL.print(" deg\t| ");
        DEBUG_SERIAL.print(point.distance, 1);
        DEBUG_SERIAL.print(" mm\t| Q:");
        DEBUG_SERIAL.println(point.quality);
      }
    }
  }
  else
  {
    // --------------------------------------------------------
    // Timeout: no data received
    // --------------------------------------------------------

    static uint32_t lastTimeoutReport = 0;

    if ((millis() - lastTimeoutReport) > 5000)
    {
      lastTimeoutReport = millis();

      DEBUG_SERIAL.println();
      DEBUG_SERIAL.println(
        "WARNING: No data from RPLidar (timeout)"
      );
      DEBUG_SERIAL.println(
        "Attempting restart..."
      );

      scanActive = false; // Trigger reconnect
    }
  }
}

    Serial.println(
      "Acceleration [m/s2]:"
    );

    Serial.print(
      "  X = "
    );
    Serial.println(
      ax,
      6
    );

    Serial.print(
      "  Y = "
    );
    Serial.println(
      ay,
      6
    );

    Serial.print(
      "  Z = "
    );
    Serial.println(
      az,
      6
    );

    Serial.println(
      "---------------------------------------"
    );
  }
}