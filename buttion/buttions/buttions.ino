// ============================================================
// STM32 NUCLEO-F207ZG (STM32duino core)
// BUTTON + 2 RELAY CONTROL, STATUS OVER ETHERNET (UDP)
// ============================================================
//
// ROS 2 side: button_relay_node.py (same folder)
//
// Pins are on the Arduino (Zio) header, CN10 / CN7:
//
// Button:
//   D2 (PF15) -> Push button -> GND
//
// LED:
//   D3 (PE13) -> Button LED (via resistor) -> GND
//
// Relay:
//   D4 (PF14) -> Relay 1 IN
//   D5 (PE11) -> Relay 2 IN
//
// None of these pins clash with Ethernet (RMII), the LiDAR
// USART6 (PG9/PG14) or CAN (PD0/PD1).
//
// GPIO is 3.3 V. Use a relay module that triggers from 3.3 V.
//
// Serial = ST-LINK virtual COM port (USB), 115200 baud.
// RJ45 of the Nucleo -> same switch/router as the PC.
//
// Operation:
//   Short press (< 1 second) -> Toggle Relay 1
//   Long press  (>= 1 second) -> Toggle Relay 2
//
// Relay logic:
//   LOW  = ON
//   HIGH = OFF
//
// Network: static address, no DHCP.
//   Board: BOARD_IP, listens on BOARD_PORT.
//   Status goes to the PC that last sent the board a packet
//   (the ROS node sends 'h' every second). With no PC heard
//   for PEER_TIMEOUT_MS it is broadcast to PC_PORT.
//   Without an Ethernet link, Ethernet.begin() holds setup()
//   for ~10 s; the relays are already OFF during that time.
//
// UDP datagrams from the board:
//   status  20 bytes, little-endian (Python "<2sBBIIBBBBI"),
//           every STATUS_MS and at once on every change:
//      0  char[2]  "BR"
//      2  uint8    version (1)
//      3  uint8    type (1 = status)
//      4  uint32   sequence
//      8  uint32   uptime ms
//     12  uint8    flags: bit0 relay 1 ON, bit1 relay 2 ON,
//                         bit2 button pressed,
//                         bit3 held >= 1 s (long press armed)
//     13  uint8    event: 0 none (periodic), 1 boot,
//                         2 button pressed, 3 short press,
//                         4 long press, 5 reboot requested
//     14  uint8    reset cause of the last start:
//                  0 unknown, 1 power-on, 2 reset pin,
//                  3 software (reboot), 4 independent watchdog,
//                  5 window watchdog, 6 brown-out, 7 low-power
//     15  uint8    reserved
//     16  uint32   last hold time ms (of the last release)
//   text    one status line (startup, reboot)
//
// From the PC:
//   "REBOOT"   -> answer with a status (event 5), then soft
//                 reboot (NVIC_SystemReset) REBOOT_DELAY_MS later
//   anything   -> only tells the board where to send (hello)
// ============================================================

#include <Arduino.h>
#include <LwIP.h>                 // before STM32Ethernet, so the builder finds STM32duino_LwIP
#include <STM32Ethernet.h>


// ============================================================
// PIN CONFIGURATION
// ============================================================

#define BUTTON_PIN      PF15   // D2
#define BUTTON_LED_PIN  PE13   // D3

#define RELAY_1_PIN     PF14   // D4
#define RELAY_2_PIN     PE11   // D5


// ============================================================
// LOGIC
// ============================================================

#define RELAY_ACTIVE    LOW
#define RELAY_INACTIVE  HIGH

#define BUTTON_PRESSED  LOW

const uint32_t LONG_PRESS_MS = 1000;
const uint32_t DEBOUNCE_MS   = 40;


// ============================================================
// NETWORK
// ============================================================

// PC is on 192.168.200.0/24; change all four for another network
IPAddress BOARD_IP(192, 168, 200, 177);
IPAddress NETMASK(255, 255, 255, 0);
IPAddress GATEWAY(192, 168, 200, 1);
IPAddress BROADCAST_IP(192, 168, 200, 255);

const uint16_t BOARD_PORT      = 5600;   // board listens here
const uint16_t PC_PORT         = 5601;   // broadcast destination while no PC is known
const uint32_t PEER_TIMEOUT_MS = 5000;   // PC silent this long -> back to broadcast
const uint32_t STATUS_MS       = 200;    // periodic status, 5 Hz
const uint32_t REBOOT_DELAY_MS = 100;    // lets the reboot answer leave before the reset

EthernetUDP Udp;

bool      havePeer   = false;
IPAddress peerIp;
uint16_t  peerPort   = PC_PORT;
uint32_t  peerSeenMs = 0;

uint32_t  rebootAtMs = 0;                // != 0: soft reboot pending


// ============================================================
// STATUS PACKET
// ============================================================

enum StatusEvent : uint8_t
{
  EV_NONE        = 0,
  EV_BOOT        = 1,
  EV_PRESSED     = 2,
  EV_SHORT_PRESS = 3,
  EV_LONG_PRESS  = 4,
  EV_REBOOT      = 5
};

struct __attribute__((packed)) StatusPacket
{
  char     magic[2];      // "BR"
  uint8_t  version;       // 1
  uint8_t  type;          // 1 = status
  uint32_t seq;
  uint32_t uptimeMs;
  uint8_t  flags;
  uint8_t  event;
  uint8_t  resetCause;
  uint8_t  reserved;
  uint32_t lastHoldMs;
};

static_assert(sizeof(StatusPacket) == 20, "status packet must be 20 bytes");

uint32_t statusSeq    = 0;
uint32_t lastStatusMs = 0;
uint8_t  resetCause   = 0;
uint32_t lastHoldMs   = 0;


// ============================================================
// RELAY STATES
// ============================================================

bool relay1State = false;
bool relay2State = false;


// ============================================================
// BUTTON STATE
// ============================================================

uint32_t buttonPressStartTime = 0;

bool isHoldingButton = false;

bool debouncedState = HIGH;


// ============================================================
// RESET CAUSE (read once at start, then the flags are cleared)
// ============================================================

uint8_t readResetCause()
{
  uint32_t csr = RCC->CSR;

  RCC->CSR |= RCC_CSR_RMVF;

  // A software reset also drives NRST, so PINRSTF is set too:
  // check the specific causes first.
  if (csr & RCC_CSR_SFTRSTF)  return 3;
  if (csr & RCC_CSR_IWDGRSTF) return 4;
  if (csr & RCC_CSR_WWDGRSTF) return 5;
  if (csr & RCC_CSR_LPWRRSTF) return 7;
  if (csr & RCC_CSR_PORRSTF)  return 1;
  if (csr & RCC_CSR_BORRSTF)  return 6;
  if (csr & RCC_CSR_PINRSTF)  return 2;

  return 0;
}

const char *resetCauseName(uint8_t cause)
{
  switch (cause)
  {
    case 1:  return "power-on";
    case 2:  return "reset pin";
    case 3:  return "software (reboot)";
    case 4:  return "independent watchdog";
    case 5:  return "window watchdog";
    case 6:  return "brown-out";
    case 7:  return "low-power";
    default: return "unknown";
  }
}


// ============================================================
// UDP SEND
// ============================================================

void udpSend(const uint8_t *data, size_t len)
{
  if (Ethernet.linkStatus() != LinkON)
  {
    return;
  }

  bool unicast =
    havePeer &&
    millis() - peerSeenMs < PEER_TIMEOUT_MS;

  if (Udp.beginPacket(
        unicast ? peerIp : BROADCAST_IP,
        unicast ? peerPort : PC_PORT))
  {
    Udp.write(data, len);
    Udp.endPacket();
  }
}

// Text line to the USB serial and to the PC
void logLine(const char *text)
{
  Serial.println(text);

  udpSend((const uint8_t *)text, strlen(text));
}

void sendStatus(uint8_t event)
{
  StatusPacket p;

  p.magic[0]   = 'B';
  p.magic[1]   = 'R';
  p.version    = 1;
  p.type       = 1;
  p.seq        = statusSeq++;
  p.uptimeMs   = millis();

  p.flags = 0;

  if (relay1State) p.flags |= 0x01;
  if (relay2State) p.flags |= 0x02;

  if (debouncedState == BUTTON_PRESSED)
  {
    p.flags |= 0x04;
  }

  if (isHoldingButton &&
      millis() - buttonPressStartTime >= LONG_PRESS_MS)
  {
    p.flags |= 0x08;
  }

  p.event      = event;
  p.resetCause = resetCause;
  p.reserved   = 0;
  p.lastHoldMs = lastHoldMs;

  udpSend((const uint8_t *)&p, sizeof(p));

  lastStatusMs = millis();
}


// ============================================================
// UDP RECEIVE (hello / REBOOT)
// ============================================================

void pollUdp()
{
  int size = Udp.parsePacket();

  if (size <= 0)
  {
    return;
  }

  char buf[16];

  int n = Udp.read(buf, sizeof(buf) - 1);

  if (n < 0)
  {
    n = 0;
  }

  buf[n] = '\0';

  // Every datagram registers the PC as the receiver
  if (!havePeer ||
      peerIp != Udp.remoteIP() ||
      peerPort != Udp.remotePort())
  {
    peerIp   = Udp.remoteIP();
    peerPort = Udp.remotePort();
    havePeer = true;

    Serial.print("PC ");
    Serial.print(peerIp);
    Serial.print(":");
    Serial.print(peerPort);
    Serial.println(" connected");

    sendStatus(EV_NONE);
  }

  peerSeenMs = millis();

  if (size == 6 && strcmp(buf, "REBOOT") == 0 && rebootAtMs == 0)
  {
    sendStatus(EV_REBOOT);

    logLine("Soft reboot requested by the PC");

    rebootAtMs = millis() + REBOOT_DELAY_MS;
    if (rebootAtMs == 0)
    {
      rebootAtMs = 1;
    }
  }
}


// ============================================================
// RELAYS
// ============================================================

void writeRelays()
{
  digitalWrite(
    RELAY_1_PIN,
    relay1State ? RELAY_ACTIVE : RELAY_INACTIVE
  );

  digitalWrite(
    RELAY_2_PIN,
    relay2State ? RELAY_ACTIVE : RELAY_INACTIVE
  );
}


// ============================================================
// SETUP RELAY AND BUTTON
// ============================================================

void setupRelayAndButton()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(BUTTON_LED_PIN, OUTPUT);


  // ----------------------------------------------------------
  // Initial relay state = OFF
  // Write the level before enabling the output so the
  // active-LOW relays don't click on at boot.
  // ----------------------------------------------------------

  relay1State = false;
  relay2State = false;

  writeRelays();

  pinMode(RELAY_1_PIN, OUTPUT);

  pinMode(RELAY_2_PIN, OUTPUT);


  // ----------------------------------------------------------
  // LED OFF
  // ----------------------------------------------------------

  digitalWrite(BUTTON_LED_PIN, LOW);
}

void printPins()
{
  Serial.println();
  Serial.println("Relay and Button Initialized");

  Serial.println("Button Pin  : D2 (PF15)");

  Serial.println("LED Pin     : D3 (PE13)");

  Serial.println("Relay 1 Pin : D4 (PF14)");

  Serial.println("Relay 2 Pin : D5 (PE11)");
}


// ============================================================
// UPDATE BUTTON AND RELAYS
// ============================================================

void updateRelayAndButton()
{
  uint32_t now = millis();

  bool rawReading = digitalRead(BUTTON_PIN);


  // ----------------------------------------------------------
  // Debounce variables
  // ----------------------------------------------------------

  static bool lastRawReading = HIGH;
  static uint32_t lastDebounceTime = 0;
  static bool longPressAnnounced = false;


  // ----------------------------------------------------------
  // Detect raw button change
  // ----------------------------------------------------------

  if (rawReading != lastRawReading)
  {
    lastDebounceTime = now;

    lastRawReading = rawReading;
  }


  // ----------------------------------------------------------
  // 40 ms debounce
  // ----------------------------------------------------------

  if ((now - lastDebounceTime) > DEBOUNCE_MS)
  {
    if (rawReading != debouncedState)
    {
      debouncedState = rawReading;


      // ======================================================
      // BUTTON PRESSED
      // ======================================================

      if (debouncedState == BUTTON_PRESSED)
      {
        buttonPressStartTime = now;

        isHoldingButton = true;

        longPressAnnounced = false;

        Serial.println();
        Serial.println(">>> Button Pressed <<<");

        sendStatus(EV_PRESSED);
      }


      // ======================================================
      // BUTTON RELEASED
      // ======================================================

      else if (isHoldingButton)
      {
        uint32_t totalHoldTime =
          now - buttonPressStartTime;

        isHoldingButton = false;

        lastHoldMs = totalHoldTime;


        Serial.print("Button Released after ");
        Serial.print(totalHoldTime);
        Serial.println(" ms");


        // ====================================================
        // LONG PRESS
        // ====================================================

        if (totalHoldTime >= LONG_PRESS_MS)
        {
          relay2State = !relay2State;

          writeRelays();

          Serial.print("RELAY 2 -> ");
          Serial.println(relay2State ? "ON" : "OFF");

          sendStatus(EV_LONG_PRESS);
        }


        // ====================================================
        // SHORT PRESS
        // ====================================================

        else
        {
          relay1State = !relay1State;

          writeRelays();

          Serial.print("RELAY 1 -> ");
          Serial.println(relay1State ? "ON" : "OFF");

          sendStatus(EV_SHORT_PRESS);
        }
      }
    }
  }


  // ==========================================================
  // BUTTON LED FEEDBACK
  // ==========================================================

  if (
    debouncedState == BUTTON_PRESSED &&
    isHoldingButton
  )
  {
    uint32_t holdDuration =
      now - buttonPressStartTime;


    // --------------------------------------------------------
    // Holding < 1 second
    // --------------------------------------------------------

    if (holdDuration < LONG_PRESS_MS)
    {
      digitalWrite(BUTTON_LED_PIN, HIGH);
    }


    // --------------------------------------------------------
    // Holding >= 1 second
    // --------------------------------------------------------

    else
    {
      digitalWrite(
        BUTTON_LED_PIN,
        ((now / 60) % 2) ? HIGH : LOW
      );

      // Tell the PC once that the long press is armed
      if (!longPressAnnounced)
      {
        longPressAnnounced = true;

        sendStatus(EV_NONE);
      }
    }
  }


  // ==========================================================
  // IDLE LED
  // ==========================================================

  else if (!isHoldingButton)
  {
    digitalWrite(
      BUTTON_LED_PIN,
      (relay1State || relay2State) ? HIGH : LOW
    );
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  resetCause = readResetCause();

  // Relays OFF first, before anything that can take time
  setupRelayAndButton();

  Serial.begin(115200);

  delay(1000);


  Serial.println();
  Serial.println("==========================================");
  Serial.println("NUCLEO-F207ZG Button + Relay Control (UDP)");
  Serial.println("==========================================");

  Serial.print("Reset cause : ");
  Serial.println(resetCauseName(resetCause));

  printPins();


  // ----------------------------------------------------------
  // Ethernet
  // ----------------------------------------------------------

  Ethernet.begin(BOARD_IP, NETMASK, GATEWAY);

  Udp.begin(BOARD_PORT);

  Serial.print("Board IP ");
  Serial.print(Ethernet.localIP());
  Serial.print(", UDP port ");
  Serial.println(BOARD_PORT);

  if (Ethernet.linkStatus() != LinkON)
  {
    Serial.println("No Ethernet link yet (cable / switch?)");
  }


  Serial.println();
  Serial.println("System Ready!");
  Serial.println("Short press -> Relay 1");
  Serial.println("Long press  -> Relay 2");
  Serial.println();

  char line[64];

  snprintf(
    line, sizeof(line),
    "Button/relay board started, reset cause: %s",
    resetCauseName(resetCause)
  );

  udpSend((const uint8_t *)line, strlen(line));

  sendStatus(EV_BOOT);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  pollUdp();

  updateRelayAndButton();

  if (millis() - lastStatusMs >= STATUS_MS)
  {
    sendStatus(EV_NONE);
  }


  // ----------------------------------------------------------
  // Soft reboot requested by the PC
  // ----------------------------------------------------------

  if (rebootAtMs != 0 &&
      (int32_t)(millis() - rebootAtMs) >= 0)
  {
    Serial.println("Rebooting...");
    Serial.flush();

    relay1State = false;
    relay2State = false;

    writeRelays();

    NVIC_SystemReset();
  }
}

