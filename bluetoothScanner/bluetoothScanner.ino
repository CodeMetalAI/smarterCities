#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <time.h>

// Bluetooth scan settings
const int SCAN_TIME = 1;  // seconds
BLEScan* pBLEScan;

// Display update rate
const int DISPLAY_INTERVAL = 100;  // 10Hz = 100ms
unsigned long lastDisplayUpdate = 0;

// Structure for storing detection information
struct Detection {
    unsigned long timestamp;  // microseconds since boot
    String macAddress;
    int rssi;
    String deviceName;
};

// Circular buffer implementation (since we removed external library dependency)
const int BUFFER_SIZE = 100;
Detection detectionBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Detection detection;
        detection.timestamp = micros(); // microsecond precision timer
        detection.macAddress = String(advertisedDevice.getAddress().toString().c_str());
        detection.rssi = advertisedDevice.getRSSI();
        
        if (advertisedDevice.haveName()) {
            detection.deviceName = String(advertisedDevice.getName().c_str());
        } else {
            detection.deviceName = "Unknown";
        }
        
        // Add to circular buffer
        detectionBuffer[bufferIndex] = detection;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        if (bufferCount < BUFFER_SIZE) bufferCount++;
    }
};

void setup() {
    heltec_setup();
    
    // Initialize Bluetooth
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    display.clear();
    display.println("BT Scanner");
    display.println("Starting...");
    display.display();
    delay(1000);
}

void formatMicrosToStr(unsigned long micros, char* timeStr) {
    unsigned long seconds = micros / 1000000;
    unsigned long subsec = (micros % 1000000) / 1000; // milliseconds
    
    unsigned long hours = seconds / 3600;
    unsigned long minutes = (seconds % 3600) / 60;
    unsigned long secs = seconds % 60;
    
    sprintf(timeStr, "%02lu:%02lu:%02lu.%03lu", hours, minutes, secs, subsec);
}

void updateDisplay() {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    
    if (bufferCount == 0) {
        display.println("Waiting for devices...");
    } else {
        // Get most recent detection
        int lastIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        Detection latest = detectionBuffer[lastIndex];
        
        // Format timestamp
        char timeStr[16];
        formatMicrosToStr(latest.timestamp, timeStr);
        
        display.printf("Time: %s\n", timeStr);
        
        // Show last 6 chars of MAC for brevity
        String shortMAC = latest.macAddress.substring(latest.macAddress.length() - 6);
        display.printf("MAC: ...%s\n", shortMAC.c_str());
        display.printf("RSSI: %ddBm\n", latest.rssi);
        
        // Truncate name if too long
        String name = latest.deviceName;
        if (name.length() > 16) {
            name = name.substring(0, 13) + "...";
        }
        display.printf("Name: %s\n", name.c_str());
        
        // Show buffer stats
        display.printf("Buffer: %d/%d", bufferCount, BUFFER_SIZE);
    }
    
    display.display();
}

void loop() {
    // Continuous Bluetooth scanning
    pBLEScan->start(SCAN_TIME, false);
    pBLEScan->clearResults();
    
    // Update display at 10Hz
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        updateDisplay();
        lastDisplayUpdate = currentTime;
        
        // Blink LED to show activity
        heltec_led(100);
        delay(10);
        heltec_led(0);
    }
    
    // Handle button press - clear buffer
    if (button.isSingleClick()) {
        bufferIndex = 0;
        bufferCount = 0;
        display.clear();
        display.println("Buffer cleared!");
        display.display();
        delay(500);
    }
    
    heltec_loop();
}