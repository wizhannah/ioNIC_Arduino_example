#include <SPI.h>
#include "pico/stdlib.h"

// -------------------------------------------------------------------------
// Minimal W55RP20-S2E SPI AT command example (send one line, read response)
// -------------------------------------------------------------------------

// Pin map (Pico -> W55RP20-S2E)
static const uint8_t PIN_SCK  = 2;
static const uint8_t PIN_MISO = 4;
static const uint8_t PIN_MOSI = 3;
static const uint8_t PIN_CSN  = 5;
static const uint8_t PIN_INT  = 26;
static const uint8_t PIN_SEL  = 13; // UART/SPI select (High=SPI)

// SPI config
static const uint32_t SPI_BAUD = 100000; // 1 MHz for stability
static SPISettings S2E_SPI_SETTINGS(SPI_BAUD, MSBFIRST, SPI_MODE0);

// Timing
static const uint32_t ACK_TIMEOUT_MS = 2000;
static const uint32_t INT_TIMEOUT_MS = 200;
static const uint32_t INT_CS_DELAY_US = 500;
static const uint32_t CS_HOLD_US = 2;
static const uint32_t CS_GAP_US = 20;

// Protocol constants
static const uint8_t DUMMY = 0xFF;
static const uint8_t ACK   = 0x0A;
static const uint8_t NACK  = 0x0B;
static const uint8_t CMD_B0 = 0xB0;
static const uint8_t RSP_B1 = 0xB1;

static void printHelp() {
  Serial.println("=== W55RP20-S2E AT Help ===");
  Serial.println("Save settings: SV  | Reboot: RT  | Factory reset: FR");
  Serial.println("");
  Serial.println("[Device Info] (RO)");
  Serial.println("MC  -> MAC address (ex: MC00:08:DC:00:00:01)");
  Serial.println("VR  -> Firmware version (ex: VR1.0.0)");
  Serial.println("MN  -> Product name (ex: MNWIZ5XXRSR-RP)");
  Serial.println("ST  -> Status (BOOT/OPEN/CONNECT/UPGRADE/ATMODE)");
  Serial.println("UN  -> UART interface str (ex: UNRS-232/TTL)");
  Serial.println("UI  -> UART interface code (ex: UI0)");
  Serial.println("");
  Serial.println("[Network] (RW)");
  Serial.println("OPx -> Mode: 0 TCP client, 1 TCP server, 2 mixed, 3 UDP, 4 SSL, 5 MQTT, 6 MQTTS");
  Serial.println("IMx -> IP alloc: 0 static, 1 DHCP");
  Serial.println("LIa.b.c.d -> Local IP (ex: LI192.168.11.2)");
  Serial.println("SMa.b.c.d -> Subnet (ex: SM255.255.255.0)");
  Serial.println("GWa.b.c.d -> Gateway (ex: GW192.168.11.1)");
  Serial.println("DSa.b.c.d -> DNS (ex: DS8.8.8.8)");
  Serial.println("LPn -> Local port (ex: LP5000)");
  Serial.println("RHa.b.c.d / domain -> Remote host (ex: RH192.168.11.3)");
  Serial.println("RPn -> Remote port (ex: RP5000)");
  Serial.println("");
  Serial.println("[UART] (RW)");
  Serial.println("BRx -> Baud (12=115200, 13=230400)");
  Serial.println("DBx -> Data bits (0=7bit, 1=8bit)");
  Serial.println("PRx -> Parity (0=None, 1=Odd, 2=Even)");
  Serial.println("SBx -> Stop bits (0=1bit, 1=2bit)");
  Serial.println("FLx -> Flow (0=None, 1=XON/XOFF, 2=RTS/CTS)");
  Serial.println("ECx -> Echo (0=Off, 1=On)");
  Serial.println("");
  Serial.println("[Packing] (RW)");
  Serial.println("PTn -> Time delimiter ms (ex: PT1000)");
  Serial.println("PSn -> Size delimiter bytes (ex: PS64)");
  Serial.println("PDxx -> Char delimiter hex (ex: PD0D)");
  Serial.println("");
  Serial.println("[Options] (RW)");
  Serial.println("ITn -> Inactivity sec (ex: IT30)");
  Serial.println("RIn -> Reconnect interval ms (ex: RI3000)");
  Serial.println("CPx -> Conn password enable (0/1)");
  Serial.println("NPxxxx -> Conn password (max 8 chars)");
  Serial.println("SPxxxx -> Search ID (max 8 chars)");
  Serial.println("DGx -> Debug msg (0/1)");
  Serial.println("KAx -> Keep-alive (0/1)");
  Serial.println("KIn -> KA initial interval ms (ex: KI7000)");
  Serial.println("KEn -> KA retry interval ms (ex: KE5000)");
  Serial.println("SOn -> SSL recv timeout ms (ex: SO2000)");
  Serial.println("");
  Serial.println("[MQTT] (RW)");
  Serial.println("QUuser QPpass QCid QK60 PUtopic");
  Serial.println("U0sub U1sub U2sub QO0");
  Serial.println("");
  Serial.println("Type HELP or ? to show this list again.");
}

static inline void cs_low()  { digitalWrite(PIN_CSN, LOW); }
static inline void cs_high() { digitalWrite(PIN_CSN, HIGH); }

static inline uint8_t spi_xfer(uint8_t v) {
  SPI.beginTransaction(S2E_SPI_SETTINGS);
  cs_low();
  if (CS_HOLD_US) {
    delayMicroseconds(CS_HOLD_US);
  }
  uint8_t r = SPI.transfer(v);
  cs_high();
  SPI.endTransaction();
  if (CS_GAP_US) {
    delayMicroseconds(CS_GAP_US);
  }
  return r;
}

static bool wait_int_low(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (digitalRead(PIN_INT) == LOW) {
      return true;
    }
    delay(1);
  }
  return false;
}

static int wait_ack(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t b = spi_xfer(DUMMY);
    if (b == ACK) {
      // consume remaining 3 bytes of ACK frame
      spi_xfer(DUMMY); spi_xfer(DUMMY); spi_xfer(DUMMY);
      return 0;
    }
    if (b == NACK) {
      return -1;
    }
  }
  return -2;
}

// Send AT SET frame (e.g., "LI\r\n")
static int at_set(const char *cmdLine) {
  if (!cmdLine || cmdLine[0] == '\0') {
    return -10;
  }

  String s = String(cmdLine);
  if (!s.endsWith("\r\n")) {
    s += "\r\n";
  }
  if (s.length() < 2) {
    return -11;
  }

  uint16_t totalLen = (uint16_t)s.length();
  uint16_t dataLen = (uint16_t)(totalLen - 2);


  // Header: first 2 bytes, then length (LE)
  spi_xfer((uint8_t)s[0]);
  spi_xfer((uint8_t)s[1]);
  spi_xfer((uint8_t)(dataLen & 0xFF));
  spi_xfer((uint8_t)((dataLen >> 8) & 0xFF));

  int r = wait_ack(ACK_TIMEOUT_MS);
  if (r < 0) {
    return r;
  }

  // Payload: remaining bytes
  for (uint16_t i = 2; i < totalLen; ++i) {
    spi_xfer((uint8_t)s[i]);
  }

  r = wait_ack(ACK_TIMEOUT_MS);
  return r;
}

// Send AT GET request: 2-char command + CRLF (4 bytes total)
static int at_get_request(const char *cmd2) {
  if (!cmd2 || strlen(cmd2) != 2) {
    return -12;
  }
  spi_xfer((uint8_t)cmd2[0]);
  spi_xfer((uint8_t)cmd2[1]);
  spi_xfer('\r');
  spi_xfer('\n');
  return 0;
}

// Simple GET/SET decision:
// - 2-byte command with no param -> GET
// - command with param -> SET
static int send_cmd(const String &rawLine, uint8_t *rxBuf, uint16_t rxCap, int &outLen) {
  outLen = 0;
  String line = rawLine;
  line.trim();
  if (line.length() == 0) {
    return -100;
  }

  String upperLine = line;
  upperLine.toUpperCase();
  if (upperLine == "HELP" || upperLine == "?") {
    printHelp();
    return 0;
  }

  String cmd = line;
  String param = "";
  if (line.length() > 2) {
    cmd = line.substring(0, 2);
    param = line.substring(2);
  }

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();
  const bool isSetNoParam = (cmdUpper == "SV" || cmdUpper == "RT" || cmdUpper == "FR");

  if (!isSetNoParam && param.length() == 0 && cmd.length() == 2) {
    // GET path
    Serial.print("[GET] ");
    Serial.println(cmd);
    int rq = at_get_request(cmd.c_str());
    if (rq < 0) {
      return rq;
    }
    int n = at_get_response(rxBuf, rxCap, INT_TIMEOUT_MS);
    if (n < 0) {
      return n;
    }
    outLen = n;
    return 0;
  }

  // SET path
  Serial.print("[SET] ");
  Serial.println(line);
  int r = at_set(line.c_str());
  if (r < 0) {
    return r;
  }
  int n = at_get_response(rxBuf, rxCap, INT_TIMEOUT_MS);
  if (n > 0) {
    outLen = n;
  }
  return 0;
}

// Read AT GET response (wait for INT low, then B1 + len + data)
static int at_get_response(uint8_t *out, uint16_t outCap, uint32_t intTimeoutMs) {
  if (!wait_int_low(intTimeoutMs)) {
    return -1;
  }
  if (INT_CS_DELAY_US) {
    delayMicroseconds(INT_CS_DELAY_US);
  }

  // Wait for B1
  uint32_t start = millis();
  uint8_t b = 0;
  while (millis() - start < ACK_TIMEOUT_MS) {
    b = spi_xfer(DUMMY);
    if (b == RSP_B1) {
      break;
    }
    if (b == NACK) {
      return -2;
    }
  }
  if (b != RSP_B1) {
    return -3;
  }

  // Length (LE) + dummy
  uint8_t lenL = spi_xfer(DUMMY);
  uint8_t lenH = spi_xfer(DUMMY);
  spi_xfer(DUMMY);
  uint16_t len = (uint16_t)lenL | ((uint16_t)lenH << 8);

  if (len == 0) {
    return 0;
  }

  uint16_t toRead = len;
  if (toRead > outCap) {
    toRead = outCap;
  }
  for (uint16_t i = 0; i < toRead; ++i) {
    out[i] = spi_xfer(DUMMY);
  }
  for (uint16_t i = toRead; i < len; ++i) {
    spi_xfer(DUMMY);
  }

  return (int)toRead;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Select SPI mode on the S2E side.
  pinMode(PIN_SEL, OUTPUT);
  digitalWrite(PIN_SEL, HIGH);

  // INT pin
  pinMode(PIN_INT, INPUT_PULLUP);

  // --- CS ---
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);

  // SPI pin mapping
  SPI.setRX(PIN_MISO);
  SPI.setTX(PIN_MOSI);
  SPI.setSCK(PIN_SCK);

  SPI.begin();

  delay(50);

  Serial.println("Ready. Type AT command (e.g., VR, LI, MN).");
  Serial.println("Type 'HELP' or '?' for command guide.");
  Serial.println("");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      String cmd = line;
      line = "";
      cmd.trim();
      if (cmd.length() == 0) {
        return;
      }

      static uint8_t rxBuf[2048];
      int outLen = 0;
      int rc = send_cmd(cmd, rxBuf, sizeof(rxBuf), outLen);
      if (rc < 0) {
        return;
      }
      if (outLen == 0) {
        return;
      }
      for (int i = 0; i < outLen; ++i) {
        Serial.write((char)rxBuf[i]);
      }
      Serial.println();
    } else {
      line += c;
    }
  }
}
