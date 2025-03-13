#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include "config.h"

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

WiFiUDP udp;
BLEScan* pBLEScan;
const uint16_t UDP_PORT = 12345;  // Fixed port

const int SCAN_TIME = 1;
const int DISPLAY_INTERVAL = 100;
unsigned long lastDisplayUpdate = 0;

struct Detection {
    unsigned long timestamp;
    String macAddress;
    int rssi;
    String deviceName;
};

const int BUFFER_SIZE = 100;
Detection detectionBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

void VextON() {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    display.clear();
    display.drawString(0, 0, "trying to connect to wifi");
    display.display();
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    udp.begin(UDP_PORT);
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Detection detection;
        detection.timestamp = millis();
        detection.macAddress = String(advertisedDevice.getAddress().toString().c_str());
        detection.rssi = advertisedDevice.getRSSI();
        detection.deviceName = advertisedDevice.haveName() ? 
            String(advertisedDevice.getName().c_str()) : "Unknown";
        
        detectionBuffer[bufferIndex] = detection;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        if (bufferCount < BUFFER_SIZE) bufferCount++;
        
        StaticJsonDocument<256> doc;
        doc["mac"] = detection.macAddress;
        doc["rssi"] = detection.rssi;
        doc["timestamp"] = detection.timestamp;
        doc["name"] = detection.deviceName;
        doc["scanner_ip"] = WiFi.localIP().toString();
        
        String jsonString;
        serializeJson(doc, jsonString);
        udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT);
        udp.print(jsonString);
        udp.endPacket();
    }
};

void setupBluetooth() {
    BLEDevice::init("BTScanner");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
}

void setupDisplay() {
    VextON();
    delay(100);
    display.init();
    display.setFont(ArialMT_Plain_10);
}

void updateDisplay() {
    display.clear();
    
    if (bufferCount == 0) {
        display.drawString(0, 0, "Waiting for devices...");
    } else {
        int lastIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        Detection latest = detectionBuffer[lastIndex];
        
        display.drawString(0, 0, "WiFi IP: " + WiFi.localIP().toString());
        display.drawString(0, 12, "Last detection:");
        display.drawString(0, 24, "MAC: " + latest.macAddress);
        display.drawString(0, 36, "RSSI: " + String(latest.rssi) + " dBm");
        display.drawString(0, 48, latest.deviceName);
    }
    
    display.display();
}

void setup() {
    Serial.begin(115200);
    setupDisplay();
    setupWiFi();
    setupBluetooth();
    
    display.clear();
    // display.drawString(0, 0, "BT Scanner");
    display.drawString(0, 0, WiFi.localIP().toString());
    display.display();
    delay(2000);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();
    }
    
    pBLEScan->start(SCAN_TIME, false);
    pBLEScan->clearResults();
    
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = currentTime;
    }
    
    delay(10);
}