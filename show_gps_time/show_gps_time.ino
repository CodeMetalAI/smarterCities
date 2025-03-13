#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
unsigned long lastDisplayTime = 0;
uint32_t maxSatsSpotted = 0;  // Track maximum satellites seen at once

void setup() {
  Serial.begin(115200);
  heltec_setup();
  GPSSerial.begin(9600, SERIAL_8N1, 5, 6);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "GPS Starting...");
  display.display();
}

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  if (millis() - lastDisplayTime > 1000) {
    display.clear();
    
    // Time display
    if (gps.time.isValid()) {
      char timeStr[32];
      sprintf(timeStr, "%02d:%02d:%02d UTC", 
        gps.time.hour(),
        gps.time.minute(),
        gps.time.second());
      display.drawString(0, 0, timeStr);
    } else {
      display.drawString(0, 0, "No time data");
    }

    // Track max satellites seen
    if (gps.satellites.isUpdated() && gps.satellites.value() > maxSatsSpotted) {
      maxSatsSpotted = gps.satellites.value();
    }

    // Satellite info
    display.drawString(0, 12, "Sats: " + String(gps.satellites.value()) + 
                             " (Max: " + String(maxSatsSpotted) + ")");
    
    // Fix age and status
    if (gps.satellites.isValid()) {
      display.drawString(0, 24, "Fix age: " + 
                        String(gps.location.age() / 1000.0, 1) + "s");
    } else {
      display.drawString(0, 24, "Searching...");
    }

    // Location (when available)
    if (gps.location.isValid()) {
      char locStr[32];
      sprintf(locStr, "%.6f, %.6f", 
        gps.location.lat(),
        gps.location.lng());
      display.drawString(0, 36, locStr);
      
      // Add altitude if available
      if (gps.altitude.isValid()) {
        display.drawString(0, 48, "Alt: " + 
                         String(gps.altitude.meters(), 1) + "m");
      }
    } else {
      display.drawString(0, 36, "No Fix");
    }

    display.display();
    lastDisplayTime = millis();
  }
  
  heltec_loop();
}