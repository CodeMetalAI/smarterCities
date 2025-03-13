#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ArduinoJson.h>
#include "time.h"
#include "config.h"


static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

WiFiUDP udp;
BLEScan* pBLEScan;
const uint16_t UDP_PORT = 12345;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

const int SCAN_TIME = 1;
const int DISPLAY_INTERVAL = 100;
unsigned long lastDisplayUpdate = 0;
unsigned long lastNTPSync = 0;
unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_CHECK_INTERVAL = 1000; // Check battery every 30 seconds
time_t ntpTime;
unsigned long long NTPoffset = 0;
uint32_t raw = 0;
// unsigned long raw = 0;
const unsigned long NTP_SYNC_INTERVAL = 1000*10; // (ms) reocurrence of NTP sync 

// Battery monitoring
#define VBAT_PIN 1            // Battery voltage pin
#define ADC_CTRL_PIN 37       // Control pin for ADC
#define BATTERY_SAMPLES 20    // Number of samples to average
float batteryVoltage = 0.0;
int batteryPercentage = 0;

struct Detection {
    uint64_t timestamp;  // Unix timestamp in milliseconds
    String macAddress;
    int rssi;
    String deviceName;
    uint32_t sequence;   // Sequence number for ordering
};

const int BUFFER_SIZE = 100;
Detection detectionBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;
uint32_t sequenceNumber = 0;

void VextON() {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    display.clear();
    display.drawString(0, 0, "Connecting to WiFi...");
    display.display();
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    udp.begin(UDP_PORT);
    
    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    Serial.println("\nWiFi connected");
    Serial.println("IP address: " + WiFi.localIP().toString());
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

void VBAT_Init() {
    pinMode(VBAT_PIN, INPUT);
    pinMode(ADC_CTRL_PIN, OUTPUT);
}

float readBattVoltage() {
    digitalWrite(ADC_CTRL_PIN, LOW);
    
    raw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        raw += analogRead(VBAT_PIN);
    }
    raw = raw / BATTERY_SAMPLES;
    
    digitalWrite(ADC_CTRL_PIN, HIGH);
    return .049* raw; // 5.42 * (3.3 / 1024.0)*2.8 
}

void checkBatteryStatus() {
    // Read battery voltage using the improved method
    batteryVoltage = readBattVoltage(); // lol 
    
    // Calculate battery percentage (assuming LiPo battery: 3.0V = 0%, 4.2V = 100%)
    batteryPercentage = map(int(batteryVoltage * 100), 300, 420, 0, 100);
    
    // Constrain to 0-100% range
    if (batteryPercentage < 0) batteryPercentage = 0;
    if (batteryPercentage > 100) batteryPercentage = 100;
    // Serial.print("Battery raw: ");
    // Serial.print(batteryVoltage);
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print("V, Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Detection detection;
        detection.timestamp = NTPoffset + millis();
        detection.macAddress = String(advertisedDevice.getAddress().toString().c_str());
        detection.rssi = advertisedDevice.getRSSI();
        detection.deviceName = advertisedDevice.haveName() ? 
            String(advertisedDevice.getName().c_str()) : "Unknown";
        detection.sequence = sequenceNumber++;
        
        // Add to buffer
        detectionBuffer[bufferIndex] = detection;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        if (bufferCount < BUFFER_SIZE) bufferCount++;
        
        // Send detection
        StaticJsonDocument<384> doc;
        doc["mac"] = detection.macAddress;
        doc["rssi"] = detection.rssi;
        doc["timestamp"] = detection.timestamp;
        doc["name"] = detection.deviceName;
        doc["seq"] = detection.sequence;
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

// Draw battery icon based on percentage
void drawBatteryIcon(int x, int y, int percentage) {
    // Battery outline
    display.drawRect(x, y, 20, 10);
    display.fillRect(x + 20, y + 2, 2, 6);
    
    // Battery fill level
    int fillWidth = map(percentage, 0, 100, 0, 18);
    display.fillRect(x + 1, y + 1, fillWidth, 8);
    
    // Battery percentage text
    display.drawString(x + 24, y, String(percentage) + "%");
}

void updateDisplay() {
    display.clear();
    
    // Show battery status
    // drawBatteryIcon(80, 0, batteryPercentage);
    
    // Show time status
    struct tm timeinfo;
    if(getLocalTime(&timeinfo)){
        char timeString[20];
        strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
        display.drawString(0, 0, "Time: " + String(timeString));
    } else {
        display.drawString(0, 0, "Time not set");
    }
    
    // Show WiFi status
    display.drawString(0, 12, "IP: " + WiFi.localIP().toString());
    
    // Show BLE status
    if (bufferCount == 0) {
        display.drawString(0, 24, "Waiting for devices...");
    } else {
        int lastIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        Detection latest = detectionBuffer[lastIndex];
        
        display.drawString(0, 24, "batteryRaw: " + String(raw));
        display.drawString(0, 36, latest.deviceName);
        
        display.drawString(0, 48, "RSSI: " + String(latest.rssi) + " dBm");
    }
    
    display.display();
}

void setup() {
    Serial.begin(115200);
    
    // Initialize battery monitoring
    VBAT_Init();
    
    setupDisplay();
    setupWiFi();
    setupBluetooth();
    syncNTP();
    checkBatteryStatus();  // Initial battery check
    
    display.clear();
    display.drawString(0, 0, WiFi.localIP().toString());
    display.drawString(0, 12, "Battery: " + String(batteryVoltage) + "V");
    display.display();
    delay(2000);
}

void loop() {
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();
    }
    
    // Sync NTP periodically
    if (millis() - lastNTPSync >= NTP_SYNC_INTERVAL) {
        syncNTP();
    }
    
    // Check battery periodically
    if (millis() - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
        checkBatteryStatus();
        lastBatteryCheck = millis();
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



