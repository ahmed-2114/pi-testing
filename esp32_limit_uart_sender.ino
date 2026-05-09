/*
  ESP32 UART limit switch sender for Raspberry Pi.

  Wiring:
  - Limit switch between LIMIT_PIN and GND
  - ESP32 TX pin -> Raspberry Pi RX (GPIO15, physical pin 10)
  - ESP32 RX pin -> Raspberry Pi TX (GPIO14, physical pin 8)
  - ESP32 GND -> Raspberry Pi GND

  Important:
  - This is 3.3V TTL UART only.
  - Do not connect to RS-232 voltage levels.
*/

#if !defined(ARDUINO_ARCH_ESP32)
#error This sketch targets ESP32 boards.
#endif

static const int LIMIT_PIN = 23;
static const int PI_UART_RX_PIN = 3;
static const int PI_UART_TX_PIN = 1;
static const int TEST_OUTPUT_PIN = 2;
static const unsigned long UART_BAUD = 115200;
static const unsigned long DEBOUNCE_MS = 25;
static const bool LIMIT_USE_PULLUP = false;
static const bool LIMIT_ACTIVE_LOW = true;

HardwareSerial PiUart(2);

bool stablePressed = false;
bool lastRawPressed = false;
unsigned long lastChangeMs = 0;
String rxLine;

bool readLimitPressed() {
  int level = digitalRead(LIMIT_PIN);
  return LIMIT_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void sendLimitState(bool pressed) {
  PiUart.println(pressed ? "LIMIT:PRESSED" : "LIMIT:RELEASED");
  Serial.println(pressed ? "TX LIMIT:PRESSED" : "TX LIMIT:RELEASED");
}

void sendTestOutputState(bool high) {
  PiUart.println(high ? "TEST:HIGH" : "TEST:LOW");
  Serial.println(high ? "TX TEST:HIGH" : "TX TEST:LOW");
}

void handlePiCommand(const String& rawLine) {
  String line = rawLine;
  line.trim();
  line.toUpperCase();

  if (line.length() == 0) {
    return;
  }

  if (line == "PING") {
    PiUart.println("PONG");
    Serial.println("RX PING -> TX PONG");
    return;
  }

  if (line == "STATUS?") {
    sendLimitState(stablePressed);
    sendTestOutputState(digitalRead(TEST_OUTPUT_PIN) == HIGH);
    return;
  }

  if (line == "TEST HIGH") {
    digitalWrite(TEST_OUTPUT_PIN, HIGH);
    sendTestOutputState(true);
    return;
  }

  if (line == "TEST LOW") {
    digitalWrite(TEST_OUTPUT_PIN, LOW);
    sendTestOutputState(false);
    return;
  }

  if (line.startsWith("ACK:")) {
    Serial.print("RX ");
    Serial.println(line);
    return;
  }

  Serial.print("RX UNKNOWN ");
  Serial.println(line);
}

void readPiUart() {
  while (PiUart.available() > 0) {
    char c = static_cast<char>(PiUart.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      handlePiCommand(rxLine);
      rxLine = "";
      continue;
    }

    if (rxLine.length() < 80) {
      rxLine += c;
    }
  }
}

void updateLimitState() {
  bool rawPressed = readLimitPressed();
  unsigned long now = millis();

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastChangeMs = now;
  }

  if ((now - lastChangeMs) >= DEBOUNCE_MS && rawPressed != stablePressed) {
    stablePressed = rawPressed;
    sendLimitState(stablePressed);
  }
}

void setup() {
  pinMode(LIMIT_PIN, LIMIT_USE_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(TEST_OUTPUT_PIN, OUTPUT);
  digitalWrite(TEST_OUTPUT_PIN, LOW);

  Serial.begin(115200);
  PiUart.begin(UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  delay(200);

  stablePressed = readLimitPressed();
  lastRawPressed = stablePressed;
  lastChangeMs = millis();

  Serial.println("ESP32 limit UART sender started");
  Serial.println(LIMIT_ACTIVE_LOW ? "Limit pressed logic: LOW" : "Limit pressed logic: HIGH");
  PiUart.println("BOOT:ESP32_LIMIT_UART_READY");
  sendLimitState(stablePressed);
  sendTestOutputState(false);
}

void loop() {
  updateLimitState();
  readPiUart();
}
