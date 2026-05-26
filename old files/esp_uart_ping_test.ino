#include <Arduino.h>

const uint32_t UART_BAUD = 115200;

void setup() {
  Serial.begin(UART_BAUD);
  delay(300);
  Serial.println("{\"type\":\"ready\",\"name\":\"esp_uart_ping_test\"}");
}

void loop() {
  if (!Serial.available()) {
    delay(5);
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("PING")) {
    Serial.println("{\"type\":\"pong\",\"ok\":true,\"echo\":\"PING\"}");
  } else if (line.startsWith("HELLO")) {
    Serial.println("{\"type\":\"hello\",\"ok\":true,\"message\":\"uart_link_alive\"}");
  } else {
    Serial.println("{\"type\":\"error\",\"ok\":false,\"message\":\"unknown_command\"}");
  }
}