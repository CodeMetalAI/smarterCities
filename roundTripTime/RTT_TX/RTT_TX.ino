// Device 1 (Sender/TX) Code
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "time.h"
#include "config.h"

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
WiFiUDP udp;
const uint16_t UDP_PORT = 12345;

// NTP Config
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
unsigned long lastNTPSync = 0;
time_t ntpTime;
unsigned long long NTPoffset = 0;
const unsigned long NTP_SYNC_INTERVAL = 1000*10;

// RTT measurement variables
const int PING_INTERVAL = 500;  // Increased to 500ms for stability
unsigned long lastPingSent = 0;
unsigned long sendTime = 0;
float rttHistory[10] = {0};
int rttIndex = 0;
String partnerIP = "";  // Store the IP of the other device

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

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    display.clear();
    display.drawString(0, 0, "Connecting to WiFi...");
    display.display();
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    
    udp.begin(UDP_PORT);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void syncNTP() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }
    time(&ntpTime);
    NTPoffset = ((unsigned long long)ntpTime * 1000) - millis();
    lastNTPSync = millis();
}

void updateDisplay(float rtt) {
    display.clear();
    
    float avgRtt = 0;
    int validMeasurements = 0;
    for(int i = 0; i < 10; i++) {
        if(rttHistory[i] > 0) {
            avgRtt += rttHistory[i];
            validMeasurements++;
        }
    }
    if(validMeasurements > 0) {
        avgRtt /= validMeasurements;
    }
    
    display.drawString(0, 0, "TX Device");
    display.drawString(0, 16, "Last RTT: " + String(rtt, 2) + " ms");
    display.drawString(0, 32, "Avg RTT: " + String(avgRtt, 2) + " ms");
    display.drawString(0, 48, "My IP: " + WiFi.localIP().toString());
    if(partnerIP != "") {
        display.drawString(0, 56, "RX IP: " + partnerIP);
    } else {
        display.drawString(0, 56, "Waiting for RX...");
    }
    display.display();
}

void setup() {
    Serial.begin(115200);
    setupDisplay();
    setupWiFi();
    syncNTP();
    
    // Initial display
    display.clear();
    display.drawString(0, 0, "TX Device");
    display.drawString(0, 16, "Waiting for RX...");
    display.drawString(0, 32, "IP: " + WiFi.localIP().toString());
    display.display();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();
    }
    if (millis() - lastNTPSync >= NTP_SYNC_INTERVAL) {
        syncNTP();
    }
    
    unsigned long currentTime = millis();
    
    // Send ping
    if (currentTime - lastPingSent >= PING_INTERVAL) {
        udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
        String msg = "PING:" + WiFi.localIP().toString();
        udp.print(msg);
        udp.endPacket();
        sendTime = currentTime;
        lastPingSent = currentTime;
    }
    
    // Check for responses
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
            String packet = String(incomingPacket);
            
            // Parse response
            if (packet.startsWith("PONG:")) {
                String sourceIP = packet.substring(5);
                if (sourceIP != WiFi.localIP().toString()) {  // Ignore self
                    partnerIP = sourceIP;
                    float rtt = (millis() - sendTime);
                    rttHistory[rttIndex] = rtt;
                    rttIndex = (rttIndex + 1) % 10;
                    updateDisplay(rtt);
                }
            }
        }
    }
    
    delay(1);
}