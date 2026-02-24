#include <stdio.h>
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

static uart_inst_t *const UART_ID = uart1;
static const uint8_t S2E_TX_PIN = 4; // Pico TX -> S2E RX
static const uint8_t S2E_RX_PIN = 5; // Pico RX <- S2E TX
static const uint32_t S2E_BAUD = 115200;

// UART RX ring buffer (IRQ fills, loop drains).
static const uint16_t RX_BUF_SIZE = 1024;
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;
static volatile uint8_t rxBuf[RX_BUF_SIZE];

static inline bool rxBufferEmpty() {
  return rxHead == rxTail;
}

static inline bool rxBufferFull() {
  return ((rxHead + 1) % RX_BUF_SIZE) == rxTail;
}

static inline void rxBufferPush(uint8_t c) {
  if (rxBufferFull()) {
    return;
  }
  rxBuf[rxHead] = c;
  rxHead = (rxHead + 1) % RX_BUF_SIZE;
}

static inline bool rxBufferPop(uint8_t &c) {
  if (rxBufferEmpty()) {
    return false;
  }
  c = rxBuf[rxTail];
  rxTail = (rxTail + 1) % RX_BUF_SIZE;
  return true;
}

static void onUartRx() {
  while (uart_is_readable(UART_ID)) {
    uint8_t c = (uint8_t)uart_getc(UART_ID);
    rxBufferPush(c);
  }
}

static void dumpFromS2E(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t c;
    while (rxBufferPop(c)) {
      Serial.write((char)c);
    }
  }
}

static void sendAtLine(const char *cmd, uint32_t waitMs) {
  uart_puts(UART_ID, cmd);
  uart_puts(UART_ID, "\r\n");
  dumpFromS2E(waitMs);
}

static void at_set(const char *key, const char *value) {
  if (key == nullptr) {
    return;
  }

  if (value == nullptr) {
    Serial.print("AT Set > ");
    Serial.println(key);
    delay(10);
    sendAtLine(key, 1000);
    return;
  }

  String line = String(key) + String(value);
  Serial.print("AT Set > ");
  Serial.println(line);
  delay(10);
  sendAtLine(line.c_str(), 1000);
}

static void factory_reset() {
  sendAtLine("FR", 2000);
  delay(1500);
}

static void device_reset() {
  sendAtLine("RT", 1000);
  delay(1500);
}

static void enterCommandMode() {
  // Guard time before "+++"
  delay(600);
  uart_puts(UART_ID, "+++"); // no CR/LF
  // Guard time after "+++"
  delay(600);
  dumpFromS2E(3000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  uart_init(UART_ID, S2E_BAUD);
  gpio_set_function(S2E_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(S2E_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_fifo_enabled(UART_ID, true);
  irq_set_exclusive_handler(UART1_IRQ, onUartRx);
  irq_set_enabled(UART1_IRQ, true);
  uart_set_irq_enables(UART_ID, true, false);

  Serial.println("AT config is applied automatically at boot.");
  Serial.println("W55RP20 is Rebooting...");

  // Auto configuration on boot
  enterCommandMode();              // Send +++ command
  factory_reset();                 // Send Factory Reset command
  enterCommandMode();              // Send +++ command
  at_set("OP", "1");               // TCP client mode
  at_set("LI", "192.168.11.2");    // Local IP
  at_set("SM", "255.255.255.0");   // Subnet mask
  at_set("GW", "192.168.11.1");    // Gateway
  at_set("DS", "8.8.8.8");         // DNS
  at_set("LP", "5000");            // Local port
  at_set("SV", nullptr);           // Save
  delay(100);
  Serial.println("Auto-configuration done. Module will reboot.");
  device_reset();                  // Reset device

  Serial.println("=== UART Loopback Server ===");
  Serial.println("Pins: TX=GPIO4, RX=GPIO5, Baud=115200");
  Serial.println("");
}

void loop() {
  uint8_t c;
  while (rxBufferPop(c)) {
    uart_putc_raw(UART_ID, c); // Echo back to UART sender.
    Serial.write((char)c);     // Also print loopback data to USB.
  }
}
