
static uart_inst_t *const UART_ID = uart1;
static const uint8_t S2E_TX_PIN = 4; // Pico TX -> S2E RX
static const uint8_t S2E_RX_PIN = 5; // Pico RX <- S2E TX
static const uint32_t S2E_BAUD = 115200;

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

static bool commandMode = false;
static String usbLine;

static void printHelp() {
  Serial.println("=== W55RP20-S2E AT Help ===");
  Serial.println("Enter command mode: +++ (guard time >= 500ms before/after)");
  Serial.println("Exit command mode: EX");
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

// UART RX ring buffer (IRQ fills, loop drains).
static const uint16_t RX_BUF_SIZE = 512;
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

static void handleUsbChar(char c) {
  if (c == '\r') {
    return;
  }
  if (c == '\n') {
    String line = usbLine;
    usbLine = "";
    line.trim();
    if (line.length() == 0) {
      return;
    }
    if (!commandMode && line == "+++") {
      enterCommandMode();
      return;
    }
    if (line == "HELP" || line == "?") {
      printHelp();
      return;
    }
    if (commandMode && line == "EX") {
      exitCommandMode();
      return;
    }
    uart_puts(UART_ID, line.c_str());
    uart_puts(UART_ID, "\r\n");
  } else {
    usbLine += c;
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

static void enterCommandMode() {
  // Guard time before "+++"
  delay(600);
  uart_puts(UART_ID, "+++"); // no CR/LF
  // Guard time after "+++"
  delay(600);
  commandMode = true;
  dumpFromS2E(3000);
}

static void exitCommandMode() {
  uart_puts(UART_ID, "EX\r\n");
  commandMode = false;
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

  Serial.println("Ready.");
  Serial.println("Type '+++' then Enter to switch to command mode.");
  Serial.println("In command mode, type W55RP20 commands (e.g., VR, MN, MC, LI).");
  Serial.println("Type 'EX' then Enter to exit command mode.");
  Serial.println("Type 'HELP' or '?' for command guide.");
  Serial.println("");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    handleUsbChar(c);
  }
  // Drain UART RX ring buffer to USB.
  uint8_t c;
  while (rxBufferPop(c)) {
    Serial.write((char)c);
  }
}
