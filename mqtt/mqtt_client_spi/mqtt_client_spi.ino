#include <SPI.h>
#include "pico/stdlib.h"

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
static const uint8_t CMD_DATA_TX = 0xA0;
static const uint8_t CMD_DATA_RX = 0xB0;
static const uint8_t RSP_DATA    = 0xB1;

static const uint16_t MAX_DATA_LEN = 2048;

// MQTT broker config
static const char *BROKER_HOST = "192.168.11.100";
static const char *BROKER_PORT = "1883";
static const char *MQTT_USER = "user";
static const char *MQTT_PASS = "";
static const char *MQTT_CLIENT_ID = "pico-spi";
static const char *MQTT_KEEPALIVE = "60";
static const char *MQTT_PUB_TOPIC = "/w55rp20/pub";
static const char *MQTT_SUB_TOPIC = "/w55rp20/sub";
static const char *MQTT_PAYLOAD = "Hello ioNIC MQTT Test\r\n";

static volatile bool spiRxPending = false;

static void onIntFall() {
  spiRxPending = true;
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
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      return 0;
    }
    if (b == NACK) {
      return -1;
    }
  }
  return -2;
}

static int wait_ack_frame(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t b = spi_xfer(DUMMY);
    if (b == ACK) {
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      return 0;
    }
    if (b == NACK) {
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      spi_xfer(DUMMY);
      return -1;
    }
  }
  return -2;
}

// Send AT SET frame (e.g., "LI192.168.11.2\r\n")
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

  String s_no_crlf = s;
  if (s_no_crlf.endsWith("\r\n")) {
    s_no_crlf.remove(s_no_crlf.length() - 2);
  }
  Serial.print("AT Set > ");
  Serial.println(s_no_crlf);

  uint16_t totalLen = (uint16_t)s.length();
  uint16_t dataLen = (uint16_t)(totalLen - 2);

  spi_xfer((uint8_t)s[0]);
  spi_xfer((uint8_t)s[1]);
  spi_xfer((uint8_t)(dataLen & 0xFF));
  spi_xfer((uint8_t)((dataLen >> 8) & 0xFF));

  int r = wait_ack(ACK_TIMEOUT_MS);
  if (r < 0) {
    return r;
  }

  for (uint16_t i = 2; i < totalLen; ++i) {
    spi_xfer((uint8_t)s[i]);
  }

  r = wait_ack(ACK_TIMEOUT_MS);
  return r;
}

// Read data frame (MQTT payload / status)
static int data_rx(uint8_t *out, uint16_t outCap) {
  if (!wait_int_low(INT_TIMEOUT_MS)) {
    return -1;
  }
  if (INT_CS_DELAY_US) {
    delayMicroseconds(INT_CS_DELAY_US);
  }

  spi_xfer(CMD_DATA_RX);
  spi_xfer(DUMMY);
  spi_xfer(DUMMY);
  spi_xfer(DUMMY);

  uint32_t start = millis();
  uint8_t b = 0;
  while (millis() - start < ACK_TIMEOUT_MS) {
    b = spi_xfer(DUMMY);
    if (b == RSP_DATA) {
      break;
    }
    if (b == NACK) {
      return -2;
    }
  }
  if (b != RSP_DATA) {
    return -3;
  }

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

// Send data frame (MQTT publish payload)
static int data_tx(const uint8_t *data, uint16_t len) {
  if (len == 0) {
    return 0;
  }
  if (len > MAX_DATA_LEN) {
    return -10;
  }

  spi_xfer(CMD_DATA_TX);
  spi_xfer((uint8_t)(len & 0xFF));
  spi_xfer((uint8_t)((len >> 8) & 0xFF));
  spi_xfer(DUMMY);

  int r = wait_ack_frame(ACK_TIMEOUT_MS);
  if (r < 0) {
    return r;
  }

  for (uint16_t i = 0; i < len; ++i) {
    spi_xfer(data[i]);
  }

  r = wait_ack_frame(ACK_TIMEOUT_MS);
  return r;
}

static void sendSerialChunk() {
  static uint8_t txBuf[256];
  while (Serial.available() > 0) {
    uint16_t n = 0;
    while (Serial.available() > 0 && n < sizeof(txBuf)) {
      txBuf[n++] = (uint8_t)Serial.read();
    }
    if (n > 0) {
      data_tx(txBuf, n);
    }
  }
}

static void sendMqttPayload(const char *payload) {
  if (!payload || payload[0] == '\0') {
    return;
  }
  data_tx((const uint8_t *)payload, (uint16_t)strlen(payload));
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
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onIntFall, FALLING);

  // CS pin
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);

  // SPI pin mapping
  SPI.setRX(PIN_MISO);
  SPI.setTX(PIN_MOSI);
  SPI.setSCK(PIN_SCK);
  SPI.begin();

  delay(50);

  Serial.println("=== Config W55RP20 with AT command (SPI) ===");
  Serial.println("AT config is applied automatically at boot.");
  Serial.println("W55RP20 is Rebooting...");

  at_set("FR");
  delay(3000);

  at_set("OP5");            // MQTT mode
  at_set("LI192.168.11.2"); // Local IP
  at_set("SM255.255.255.0");
  at_set("GW192.168.11.1");
  at_set("DS8.8.8.8");
  at_set((String("RH") + BROKER_HOST).c_str());
  at_set((String("RP") + BROKER_PORT).c_str());
  at_set((String("QU") + MQTT_USER).c_str());
  at_set((String("QP") + MQTT_PASS).c_str());
  at_set((String("QC") + MQTT_CLIENT_ID).c_str());
  at_set((String("QK") + MQTT_KEEPALIVE).c_str());
  at_set((String("PU") + MQTT_PUB_TOPIC).c_str());
  at_set((String("U0") + MQTT_SUB_TOPIC).c_str());
  at_set("QO0");
  at_set("PT10");
  at_set("SV");

  Serial.println("Auto-configuration done. Module will reboot.");
  at_set("RT");
  delay(1000);

  Serial.println("=== SPI MQTT Client ===");
  Serial.println("Waiting for SUB topic data...");
}

void loop() {
  sendSerialChunk();

  if (spiRxPending || digitalRead(PIN_INT) == LOW) {
    static uint8_t dataBuf[MAX_DATA_LEN];
    bool received = false;
    while (digitalRead(PIN_INT) == LOW) {
      int n = data_rx(dataBuf, sizeof(dataBuf));
      if (n <= 0) {
        break;
      }
      Serial.write(dataBuf, n);
      received = true;
    }
    if (received) {
      sendMqttPayload(MQTT_PAYLOAD);
    }
    spiRxPending = false;
  }
}
