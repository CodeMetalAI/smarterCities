
// Device 2 (Receiver/RX) Code - Save as separate file
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
WiFiUDP udp;
const uint16_t UDP_PORT = 12345;
String partnerIP = "";

void VextON() {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

void setupDisplay() {
    VextON();
    delay(100);
    display.init();
    display.setFont(ArialMT_Plain_10);
}

void updateDisplay() {
    display.clear();
    display.drawString(0, 0, "RX Device");
    display.drawString(0, 16, "My IP: " + WiFi.localIP().toString());
    if(partnerIP != "") {
        display.drawString(0, 32, "TX IP: " + partnerIP);
        display.drawString(0, 48, "Responding to pings...");
    } else {
        display.drawString(0, 32, "Waiting for TX...");
    }
    display.display();
}

void setup() {
    Serial.begin(115200);
    setupDisplay();
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    display.clear();
    display.drawString(0, 0, "Connecting to WiFi...");
    display.display();
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    
    udp.begin(UDP_PORT);
    
    display.clear();
    display.drawString(0, 0, "RX Device");
    display.drawString(0, 16, "Waiting for TX...");
    display.drawString(0, 32, "IP: " + WiFi.localIP().toString());
    display.display();
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
            String packet = String(incomingPacket);
            
            // Check if it's a ping and not from ourselves
            if (packet.startsWith("PING:")) {
                String sourceIP = packet.substring(5);
                if (sourceIP != WiFi.localIP().toString()) {  // Ignore self
                    partnerIP = sourceIP;
                    // Send response back to sender
                    udp.beginPacket(udp.remoteIP(), udp.remotePort());
                    String response = "PONG:" + WiFi.localIP().toString();
                    udp.print(response);
                    udp.endPacket();
                    updateDisplay();
                }
            }
        }
    }
    delay(1);
}