// for attaching 
#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Use UART1

unsigned long lastDisplayTime = 0;
int charactersRead = 0;

void setup() {
    Serial.begin(115200);  // Add this line
    heltec_setup();
    
    // Start GPS serial communication
    GPSSerial.begin(9600, SERIAL_8N1, 5,6);  
    // GPSSerial.begin(115200,SERIAL_8N1,33,34);//for the wireless tracker
    
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "GPS Starting...");
    display.display();
    delay(1000);
}

void loop() {
    display.clear();
    
    while (GPSSerial.available() > 0) {
      char c = GPSSerial.read();
      charactersRead++;
      Serial.print(c); // Add this to see raw data
      gps.encode(c);
    }
    
    // Update display every second
    if (millis() - lastDisplayTime > 1000) {
        // Display debug information
        display.drawString(0, 0, "Chars read: " + String(charactersRead));
        display.drawString(0, 12, "Sentences: " + String(gps.sentencesWithFix()));
        display.drawString(0, 24, "Checksum fail: " + String(gps.failedChecksum()));
        
        if (gps.location.isValid()) {
            display.drawString(0, 36, "Lat: " + String(gps.location.lat(), 6));
            display.drawString(0, 48, "Lon: " + String(gps.location.lng(), 6));
        } else {
            display.drawString(0, 36, "Waiting for fix...");
            // Show why we don't have a fix
            if (charactersRead == 0) {
                display.drawString(0, 48, "No data received!");
            } else if (gps.charsProcessed() < 10) {
                display.drawString(0, 48, "No GPS data found");
            }
        }
        
        display.display();
        lastDisplayTime = millis();

    }
    
    
    heltec_loop();
}
