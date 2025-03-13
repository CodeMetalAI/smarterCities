#include <WiFi.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// OLED Display Configuration
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define OLED_ADDR 0x3c

SSD1306Wire display(OLED_ADDR, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

void setup() {
  Serial.begin(115200);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  
  // Initialize display
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Starting up...");
  display.display();

  // Create Wi-Fi STA interface
  create_sta();
}

void loop() {
  network_finder();
  delay(300);  // Scan every 300 milliseconds
}

void create_sta() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Activating Wi-Fi STA interface...");
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    delay(1000);  // Give some time for activation
    Serial.print("STA Active: ");
    Serial.println(WiFi.status() == WL_CONNECTED);
  } else {
    Serial.println("STA interface already active.");
  }
}

String decode_security(int code) {
  switch (code) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2-PSK";
    default: return "UNKNOWN";
  }
}

void display_networks(int networksFound) {
  display.clear();
  int y = 0;
  for (int i = 0; i < networksFound && i < 3; i++) {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    String security = decode_security(WiFi.encryptionType(i));

    String str1 = String(i + 1) + ".)" + ssid;
    String str2 = "S:" + security + "|P:" + String(abs(rssi));

    Serial.println(str1);
    Serial.println(str2);

    display.drawString(0, y, str1);
    display.drawString(0, y + 8, str2);
    y += 16;
  }
  display.display();
}

void network_finder() {
  int networksFound = WiFi.scanNetworks();
  Serial.printf("\nFound %d networks.\n", networksFound);
  display_networks(networksFound);
}
