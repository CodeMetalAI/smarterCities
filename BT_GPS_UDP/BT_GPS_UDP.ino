


#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include "HT_TinyGPS++.h"
#include <HardwareSerial.h>
#include "config.h"

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

WiFiUDP udp;
BLEScan* pBLEScan;
const uint16_t UDP_PORT = 12345;

// GPS setup
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Use UART1

const int SCAN_TIME = 1;
const int DISPLAY_INTERVAL = 100;
unsigned long lastDisplayUpdate = 0;

struct Detection {
    uint32_t timestamp;  // Changed to uint32_t for GPS time
    String macAddress;
    int rssi;
    String deviceName;
    double latitude;     // Added for GPS
    double longitude;    // Added for GPS
    double altitude;     // Added for GPS
    // double hdop;      // Accuracy fields in GPS module? These weren't correct 
    // double vdop;
    // double pdop;
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
    timeClient.begin();
    timeClient.setUpdateInterval(30000); // Update NTP every 30 seconds
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Detection detection;
        detection.timestamp = gps.time.isValid() ? gps.time.value() : millis();  // Use GPS time if available
        detection.macAddress = String(advertisedDevice.getAddress().toString().c_str());
        detection.rssi = advertisedDevice.getRSSI();
        detection.deviceName = advertisedDevice.haveName() ? 
            String(advertisedDevice.getName().c_str()) : "Unknown";
            
        // Add GPS data if available
        detection.latitude = gps.location.isValid() ? gps.location.lat() : 0;
        detection.longitude = gps.location.isValid() ? gps.location.lng() : 0;
        detection.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0;
        // detection.hdop = gps.hdop.isValid() ? gps.hdop.value() : 0;  // Horizontal dilution of precision
        // detection.vdop = gps.vdop.isValid() ? gps.vdop.value() : 0;  // Vertical dilution of precision
        // detection.pdop = gps.pdop.isValid() ? gps.pdop.value() : 0;  // Position (3D) dilution of precision
        
        detectionBuffer[bufferIndex] = detection;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        if (bufferCount < BUFFER_SIZE) bufferCount++;
        // x
        StaticJsonDocument<384> doc;  // Increased size for GPS data
        doc["mac"] = detection.macAddress;
        doc["rssi"] = detection.rssi;
        doc["timestamp"] = detection.timestamp;
        doc["name"] = detection.deviceName;
        doc["scanner_ip"] = WiFi.localIP().toString();
        doc["lat"] = detection.latitude;
        doc["lon"] = detection.longitude;
        doc["alt"] = detection.altitude;
        // doc["hdop"] = detection.hdop;
        // doc["vdop"] = detection.vdop;
        // doc["pdop"] = detection.pdop;
        
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

void setupGPS() {
    GPSSerial.begin(9600, SERIAL_8N1, 5, 6);  // Using pins 5 & 6 for GPS
}

void setupDisplay() {
    VextON();
    delay(100);
    display.init();
    display.setFont(ArialMT_Plain_10);
}

void updateDisplay() {
    display.clear();
    
    // Show GPS status
    if (gps.location.isValid()) {
        display.drawString(0, 0, "GPS: " + String(gps.location.lat(), 6));
        display.drawString(0, 12, "    " + String(gps.location.lng(), 6));
    } else {
        display.drawString(0, 0, "GPS: No Fix");
    }
    
    // Show BLE status
    if (bufferCount == 0) {
        display.drawString(0, 24, "Waiting for devices...");
    } else {
        int lastIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        Detection latest = detectionBuffer[lastIndex];
        
        display.drawString(0, 24, "Last detection:");
        display.drawString(0, 36, "MAC: " + latest.macAddress);
        display.drawString(0, 48, "RSSI: " + String(latest.rssi) + " dBm");
    }
    
    display.display();
}

void setup() {
    Serial.begin(115200);
    setupDisplay();
    setupWiFi();
    setupBluetooth();
    setupGPS();
    
    display.clear();
    display.drawString(0, 0, WiFi.localIP().toString());
    display.display();
    delay(2000);
}

void loop() {
    // Update GPS
    while (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }
    
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