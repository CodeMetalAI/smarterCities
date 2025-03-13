#include "bluetooth.h"

class BluetoothManager::DeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    private:
        BluetoothManager* manager;
    
    public:
        DeviceCallbacks(BluetoothManager* mgr) : manager(mgr) {}
        
        void onResult(BLEAdvertisedDevice advertisedDevice) {
            Detection detection;
            detection.timestamp = micros();
            detection.macAddress = String(advertisedDevice.getAddress().toString().c_str());
            detection.rssi = advertisedDevice.getRSSI();
            detection.deviceName = advertisedDevice.haveName() ? 
                String(advertisedDevice.getName().c_str()) : "Unknown";
                
            manager->transmitDetection(detection);
        }
};

BluetoothManager::BluetoothManager() : bufferIndex(0), bufferCount(0) {}

void BluetoothManager::begin() {
    BLEDevice::init("HeltecScannerTransmitter");
    
    // Setup scanning
    pBLEScan = BLEDevice::getScan();
    callbacks = new DeviceCallbacks(this);
    pBLEScan->setAdvertisedDeviceCallbacks(callbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    // Setup server
    pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
}

void BluetoothManager::startScanning() {
    pBLEScan->start(SCAN_TIME, false);
    pBLEScan->clearResults();
}

void BluetoothManager::transmitDetection(const Detection& detection) {
    // Store in buffer
    detectionBuffer[bufferIndex] = detection;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) bufferCount++;
    
    // Format and transmit
    char txString[128];
    sprintf(txString, "%lu,%s,%d,%s", 
        detection.timestamp,
        detection.macAddress.c_str(),
        detection.rssi,
        detection.deviceName.c_str()
    );
    
    pCharacteristic->setValue(txString);
    pCharacteristic->notify();
}

Detection* BluetoothManager::getLatestDetection() {
    if (bufferCount == 0) return nullptr;
    int lastIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    return &detectionBuffer[lastIndex];
}

void BluetoothManager::clearBuffer() {
    bufferIndex = 0;
    bufferCount = 0;
}
