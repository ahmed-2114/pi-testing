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
  - This sketch also starts a small Wi-Fi access point and status page:
    SSID: ESP32-UART-Debug
    Password: esp32debug
    Page: http://192.168.4.1
*/

#if !defined(ARDUINO_ARCH_ESP32)
#error This sketch targets ESP32 boards.
#endif

#include <WiFi.h>
#include <WebServer.h>

static const int LIMIT_PIN = 23;
static const int PI_UART_RX_PIN = 3;
static const int PI_UART_TX_PIN = 1;
static const int TEST_OUTPUT_PIN = 2;
static const unsigned long UART_BAUD = 115200;
static const unsigned long DEBOUNCE_MS = 25;
static const bool LIMIT_USE_PULLUP = false;
static const bool LIMIT_ACTIVE_LOW = true;
static const char AP_SSID[] = "ESP32-UART-Debug";
static const char AP_PASSWORD[] = "esp32debug";

WebServer server(80);

bool stablePressed = false;
bool lastRawPressed = false;
unsigned long lastChangeMs = 0;
unsigned long lastLimitEventMs = 0;
unsigned long lastPiCommandMs = 0;
unsigned long sentMessageCount = 0;
unsigned long receivedCommandCount = 0;
String rxLine;
String lastPiCommand = "NONE";
String lastSentMessage = "NONE";
String apIpAddress = "192.168.4.1";

void sendToPi(const String& message) {
  lastSentMessage = message;
  sentMessageCount += 1;
  Serial.println(message);
}

int piSerialAvailable() {
  return Serial.available();
}

int piSerialRead() {
  return Serial.read();
}

String htmlEscape(const String& text) {
  String escaped;
  escaped.reserve(text.length() + 16);

  for (size_t i = 0; i < text.length(); ++i) {
    char c = text[i];
    if (c == '&') {
      escaped += "&amp;";
    } else if (c == '<') {
      escaped += "&lt;";
    } else if (c == '>') {
      escaped += "&gt;";
    } else if (c == '"') {
      escaped += "&quot;";
    } else {
      escaped += c;
    }
  }

  return escaped;
}

String millisAgo(unsigned long eventMs) {
  if (eventMs == 0) {
    return "never";
  }

  return String(millis() - eventMs) + " ms ago";
}

String buildStatusPage() {
  String html;
  html.reserve(5000);

  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='1'>";
  html += "<title>ESP32 UART Debug</title>";
  html += "<style>";
  html += "body{margin:0;font-family:Segoe UI,Tahoma,sans-serif;background:linear-gradient(135deg,#0d1b2a,#1b263b);color:#f5f7fb;}";
  html += ".wrap{max-width:980px;margin:0 auto;padding:24px;}";
  html += ".hero,.card{background:rgba(255,255,255,0.08);border:1px solid rgba(255,255,255,0.12);border-radius:18px;box-shadow:0 18px 40px rgba(0,0,0,0.22);}";
  html += ".hero{padding:22px 24px;margin-bottom:18px;}";
  html += "h1{margin:0 0 10px;font-size:36px;}p{margin:0;color:#d7dfeb;}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:16px;}";
  html += ".card{padding:18px;}";
  html += ".label{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:#9fb1c8;}";
  html += ".value{margin-top:8px;font-size:28px;font-weight:700;}";
  html += ".good{color:#94f7c5;}.warn{color:#ffd08a;}.bad{color:#ff9696;}";
  html += ".mono{font-family:Consolas,monospace;font-size:15px;line-height:1.45;word-break:break-word;}";
  html += ".footer{margin-top:18px;color:#c7d4e6;font-size:14px;}";
  html += "</style></head><body><div class='wrap'>";
  html += "<section class='hero'><h1>ESP32 UART Debug</h1>";
  html += "<p>This page refreshes every second so you can confirm whether the ESP32 is receiving Pi commands and sending UART messages back.</p>";
  html += "</section><section class='grid'>";

  html += "<div class='card'><div class='label'>Limit State</div><div class='value ";
  html += stablePressed ? "good" : "warn";
  html += "'>";
  html += stablePressed ? "PRESSED" : "RELEASED";
  html += "</div><div class='footer'>Last change: ";
  html += millisAgo(lastLimitEventMs);
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Last Pi Command</div><div class='value mono'>";
  html += htmlEscape(lastPiCommand);
  html += "</div><div class='footer'>Received: ";
  html += millisAgo(lastPiCommandMs);
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Last Sent To Pi</div><div class='value mono'>";
  html += htmlEscape(lastSentMessage);
  html += "</div><div class='footer'>Messages sent: ";
  html += String(sentMessageCount);
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Pi Command Count</div><div class='value'>";
  html += String(receivedCommandCount);
  html += "</div><div class='footer'>Wi-Fi AP: ";
  html += htmlEscape(AP_SSID);
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Access Page</div><div class='value mono'>";
  html += htmlEscape(apIpAddress);
  html += "</div><div class='footer'>Password: ";
  html += htmlEscape(AP_PASSWORD);
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Test Output Pin</div><div class='value ";
  html += digitalRead(TEST_OUTPUT_PIN) == HIGH ? "good" : "warn";
  html += "'>";
  html += digitalRead(TEST_OUTPUT_PIN) == HIGH ? "HIGH" : "LOW";
  html += "</div><div class='footer'>GPIO";
  html += String(TEST_OUTPUT_PIN);
  html += "</div></div>";

  html += "</section></div></body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", buildStatusPage());
}

void startWifiPage() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apIpAddress = WiFi.softAPIP().toString();
  server.on("/", handleRoot);
  server.begin();
}

bool readLimitPressed() {
  int level = digitalRead(LIMIT_PIN);
  return LIMIT_ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

void sendLimitState(bool pressed) {
  sendToPi(pressed ? "LIMIT:PRESSED" : "LIMIT:RELEASED");
}

void sendTestOutputState(bool high) {
  sendToPi(high ? "TEST:HIGH" : "TEST:LOW");
}

void handlePiCommand(const String& rawLine) {
  String line = rawLine;
  line.trim();
  line.toUpperCase();

  if (line.length() == 0) {
    return;
  }

  lastPiCommand = line;
  lastPiCommandMs = millis();
  receivedCommandCount += 1;

  if (line == "PING") {
    sendToPi("PONG");
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
    return;
  }
}

void readPiSerial() {
  while (piSerialAvailable() > 0) {
    char c = static_cast<char>(piSerialRead());
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
    lastLimitEventMs = now;
    sendLimitState(stablePressed);
  }
}

void setup() {
  pinMode(LIMIT_PIN, LIMIT_USE_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(TEST_OUTPUT_PIN, OUTPUT);
  digitalWrite(TEST_OUTPUT_PIN, LOW);

  // Use ESP32 UART0 on GPIO3 (RX0) and GPIO1 (TX0) for Pi communication.
  Serial.begin(UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

  startWifiPage();

  delay(200);

  stablePressed = readLimitPressed();
  lastRawPressed = stablePressed;
  lastChangeMs = millis();
  lastLimitEventMs = lastChangeMs;

  sendToPi("BOOT:ESP32_LIMIT_UART_READY");
  sendLimitState(stablePressed);
  sendTestOutputState(false);
}

void loop() {
  updateLimitState();
  readPiSerial();
  server.handleClient();
}
