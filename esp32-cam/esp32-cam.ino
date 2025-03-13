/**
 * ESP32 Motion Detection Camera with Broadcast UDP Upload and Deep Sleep
 * 
 * This sketch detects motion using a PIR sensor, captures and uploads
 * images via UDP broadcast, then enters deep sleep mode to save power.
 * 
 * Hardware requirements:
 * - ESP32-CAM or compatible board
 * - PIR motion sensor
 * - Appropriate power supply
 * 
 * Pin connections:
 * - PIR sensor signal pin to GPIO 13 (important: GPIO13 can be used as wake source)
 * - Camera is connected via the default ESP32-CAM pins
 */

// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include "esp_camera.h"
// #include "esp_sleep.h"
// #include "config.h" // Contains WIFI_SSID and WIFI_PASSWORD

// // UDP Settings
// #define UDP_PORT 4210
// const int PACKET_SIZE = 1024;

// // Motion Sensor Pin - IMPORTANT: Must be GPIO13 for deep sleep wake capability
// #define MOTION_SENSOR_PIN 13

// // Camera Settings
// #define CAMERA_MODEL_AI_THINKER // ESP32-CAM

// // Define camera pin constants for AI Thinker model
// #define PWDN_GPIO_NUM     32
// #define RESET_GPIO_NUM    -1
// #define XCLK_GPIO_NUM      0
// #define SIOD_GPIO_NUM     26
// #define SIOC_GPIO_NUM     27
// #define Y9_GPIO_NUM       35
// #define Y8_GPIO_NUM       34
// #define Y7_GPIO_NUM       39
// #define Y6_GPIO_NUM       36
// #define Y5_GPIO_NUM       21
// #define Y4_GPIO_NUM       19
// #define Y3_GPIO_NUM       18
// #define Y2_GPIO_NUM        5
// #define VSYNC_GPIO_NUM    25
// #define HREF_GPIO_NUM     23
// #define PCLK_GPIO_NUM     22

// // Deep sleep settings
// #define DEEP_SLEEP_TIME_SEC 30  // Fallback time to wake if no motion (30 seconds)
// #define FORCED_SLEEP_AFTER_CAPTURE true // Force sleep after capturing
// #define WAKE_TIME_BEFORE_SLEEP 5000 // Stay awake for 5 seconds after capturing image

// // Motion detection settings
// #define MOTION_DEBOUNCE_MS 200   // Debounce time in milliseconds
// #define MOTION_READS 5          // Number of readings for confirmation
// #define MOTION_THRESHOLD 3      // Number of HIGH readings required to confirm motion

// // Function Declarations
// bool initCamera();
// bool captureImage();
// bool sendImageViaBroadcastUDP();
// void enterDeepSleep();
// bool isMotionDetected();

// // Global Variables
// WiFiUDP udp;
// camera_fb_t *fb = NULL;
// unsigned long wakeStartTime = 0;
// unsigned long lastImageTime = 0;
// RTC_DATA_ATTR int bootCount = 0; // Stored in RTC memory, persists during deep sleep

// void setup() {
//   Serial.begin(115200);
//   delay(100); // Give serial time to initialize
  
//   // Increment and print boot count
//   bootCount++;
//   Serial.println("Boot number: " + String(bootCount));
  
//   // Record wake-up time
//   wakeStartTime = millis();
  
//   // Initialize the motion sensor pin with pull-down to prevent floating state
//   pinMode(MOTION_SENSOR_PIN, INPUT_PULLDOWN);
  
//   // Check wake-up reason
//   esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
//   Serial.print("Wake-up reason: ");
  
//   switch(wakeup_reason) {
//     case ESP_SLEEP_WAKEUP_EXT0:
//       Serial.println("External signal using RTC_IO (motion detected)");
//       break;
//     case ESP_SLEEP_WAKEUP_TIMER:
//       Serial.println("Timer");
//       // Check if there's actually motion - if not, go back to sleep
//       if (!isMotionDetected()) {
//         Serial.println("No actual motion detected after timer wake. Going back to sleep.");
//         delay(100);
//         enterDeepSleep();
//         return;
//       }
//       break;
//     default:
//       Serial.println("Initial boot");
//       break;
//   }
  
//   // Print current state of motion pin
//   Serial.print("Current motion sensor pin state: ");
//   Serial.println(digitalRead(MOTION_SENSOR_PIN) ? "HIGH" : "LOW");
  
//   // Initialize camera
//   if (!initCamera()) {
//     Serial.println("Camera initialization failed");
//     enterDeepSleep();
//     return;
//   }
  
//   // Connect to WiFi
//   Serial.printf("Connecting to %s ", WIFI_SSID);
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
//   // Set a timeout for WiFi connection
//   unsigned long wifiStartTime = millis();
//   const unsigned long WIFI_TIMEOUT = 20000; // 20 seconds timeout
  
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
    
//     // Check for timeout
//     if (millis() - wifiStartTime > WIFI_TIMEOUT) {
//       Serial.println("\nWiFi connection timeout. Going to sleep.");
//       enterDeepSleep();
//       return;
//     }
//   }
  
//   Serial.println();
//   Serial.print("WiFi connected with IP: ");
//   Serial.println(WiFi.localIP());
  
//   // Start UDP
//   udp.begin(UDP_PORT);
//   Serial.println("UDP initialized");
  
//   // Capture and send an image
//   if (captureImage()) {
//     if (sendImageViaBroadcastUDP()) {
//       Serial.println("Image sent successfully via UDP broadcast");
//     } else {
//       Serial.println("Failed to send image via UDP broadcast");
//     }
    
//     // Always free the memory after capturing an image
//     if (fb) {
//       esp_camera_fb_return(fb);
//       fb = NULL;
//     }
    
//     lastImageTime = millis();
//   } else {
//     Serial.println("Failed to capture image");
//   }
  
//   // If configured to force sleep after capture
//   if (FORCED_SLEEP_AFTER_CAPTURE) {
//     Serial.printf("Staying awake for %d ms before sleep\n", WAKE_TIME_BEFORE_SLEEP);
//     delay(WAKE_TIME_BEFORE_SLEEP);
//     enterDeepSleep();
//   }
// }

// void loop() {
//   // Only used if not forcing sleep after capture
//   if (!FORCED_SLEEP_AFTER_CAPTURE) {
//     // Check for motion again after a sufficient delay
//     if (isMotionDetected() && (millis() - lastImageTime > 5000)) {
//       Serial.println("Additional motion detected!");
      
//       // Capture and send another image
//       if (captureImage()) {
//         if (sendImageViaBroadcastUDP()) {
//           Serial.println("Image sent successfully via UDP broadcast");
//         } else {
//           Serial.println("Failed to send image via UDP broadcast");
//         }
        
//         // Always free the memory after capturing an image
//         if (fb) {
//           esp_camera_fb_return(fb);
//           fb = NULL;
//         }
        
//         lastImageTime = millis();
//       } else {
//         Serial.println("Failed to capture image");
//       }
//     }
    
//     // Check if we should go to sleep based on time awake
//     if (millis() - wakeStartTime > 30000) { // 30 seconds max awake time
//       Serial.println("Maximum awake time reached, going to sleep");
//       enterDeepSleep();
//     }
//   }
  
//   delay(100); // Small delay to prevent CPU hogging
// }

// // Uses multiple readings to reliably detect motion with debouncing
// bool isMotionDetected() {
//   int motionCount = 0;
  
//   // Take multiple readings with short delays
//   for (int i = 0; i < MOTION_READS; i++) {
//     if (digitalRead(MOTION_SENSOR_PIN) == HIGH) {
//       motionCount++;
//     }
//     delay(MOTION_DEBOUNCE_MS / MOTION_READS);
//   }
  
//   // Return true if enough readings were HIGH
//   return (motionCount >= MOTION_THRESHOLD);
// }

// bool initCamera() {
//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer = LEDC_TIMER_0;
//   config.pin_d0 = Y2_GPIO_NUM;
//   config.pin_d1 = Y3_GPIO_NUM;
//   config.pin_d2 = Y4_GPIO_NUM;
//   config.pin_d3 = Y5_GPIO_NUM;
//   config.pin_d4 = Y6_GPIO_NUM;
//   config.pin_d5 = Y7_GPIO_NUM;
//   config.pin_d6 = Y8_GPIO_NUM;
//   config.pin_d7 = Y9_GPIO_NUM;
//   config.pin_xclk = XCLK_GPIO_NUM;
//   config.pin_pclk = PCLK_GPIO_NUM;
//   config.pin_vsync = VSYNC_GPIO_NUM;
//   config.pin_href = HREF_GPIO_NUM;
//   config.pin_sscb_sda = SIOD_GPIO_NUM;
//   config.pin_sscb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn = PWDN_GPIO_NUM;
//   config.pin_reset = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   config.pixel_format = PIXFORMAT_JPEG;
  
//   // Initial settings - adjust for lower power if needed
//   config.frame_size = FRAMESIZE_VGA; // 640x480
//   config.jpeg_quality = 12; // 0-63, lower number means higher quality
//   config.fb_count = 1;
  
//   // Initialize the camera
//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return false;
//   }
  
//   // Adjust camera settings after initialization if needed
//   sensor_t * s = esp_camera_sensor_get();
//   if (s) {
//     // Adjust brightness, contrast, etc. if needed
//     s->set_brightness(s, 0);     // -2 to 2
//     s->set_contrast(s, 0);       // -2 to 2
//     s->set_saturation(s, 0);     // -2 to 2
//     s->set_special_effect(s, 0); // 0 = No Effect, 1 = Negative, etc.
//   }
  
//   Serial.println("Camera initialized successfully");
//   return true;
// }

// bool captureImage() {
//   // Capture a frame
//   fb = esp_camera_fb_get();
//   if (!fb) {
//     Serial.println("Camera capture failed");
//     return false;
//   }
  
//   Serial.printf("Captured image: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
//   return true;
// }

// bool sendImageViaBroadcastUDP() {
//   if (!fb) {
//     Serial.println("No image buffer to send");
//     return false;
//   }
  
//   // Use broadcast address for UDP sending
//   IPAddress broadcastIP(255, 255, 255, 255);
  
//   // Send image size as header (4 bytes)
//   uint32_t imageSize = fb->len;
//   udp.beginPacket(broadcastIP, UDP_PORT);
//   udp.write((uint8_t*)&imageSize, sizeof(imageSize));
//   udp.endPacket();
  
//   // Small delay to allow receiver to prepare
//   delay(50);
  
//   // Send image data in chunks
//   size_t totalBytesSent = 0;
//   size_t remainingBytes = fb->len;
//   const uint8_t* currentBufferPosition = fb->buf;
  
//   // Packet structure:
//   // [2 bytes packet number][2 bytes total packets][PACKET_SIZE bytes data]
//   uint16_t packetNumber = 0;
//   uint16_t totalPackets = (fb->len + PACKET_SIZE - 1) / PACKET_SIZE; // Ceiling division
  
//   while (remainingBytes > 0) {
//     // Determine the size of this packet
//     size_t bytesToSend = (remainingBytes > PACKET_SIZE) ? PACKET_SIZE : remainingBytes;
    
//     // Begin packet
//     udp.beginPacket(broadcastIP, UDP_PORT);
    
//     // Send packet header
//     udp.write((uint8_t*)&packetNumber, sizeof(packetNumber));
//     udp.write((uint8_t*)&totalPackets, sizeof(totalPackets));
    
//     // Send image data chunk
//     udp.write(currentBufferPosition, bytesToSend);
    
//     // End packet
//     udp.endPacket();
    
//     // Update counters and pointers
//     currentBufferPosition += bytesToSend;
//     totalBytesSent += bytesToSend;
//     remainingBytes -= bytesToSend;
//     packetNumber++;
    
//     // Small delay to prevent network congestion
//     delay(5);
//   }
  
//   Serial.printf("Sent %u bytes in %u packets\n", totalBytesSent, packetNumber);
//   return (totalBytesSent == fb->len);
// }

// void enterDeepSleep() {
//   Serial.println("Preparing for deep sleep...");
//   delay(100); // Give serial time to send
  
//   // Disconnect WiFi to save power
//   WiFi.disconnect(true);
//   WiFi.mode(WIFI_OFF);
  
//   // Turn off peripherals that might be on
//   btStop();
  
//   // Read current state of motion pin before sleep
//   int currentPinState = digitalRead(MOTION_SENSOR_PIN);
//   Serial.print("Motion pin state before sleep: ");
//   Serial.println(currentPinState ? "HIGH" : "LOW");
  
//   // Set up wake sources
  
//   // 1. External wake on GPIO13 (motion sensor) - HIGH level trigger
//   // Only enable if not currently HIGH to prevent immediate wake
//   if (currentPinState == LOW) {
//     esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1); // 1 = HIGH level trigger
//     Serial.println("Enabled motion sensor wake-up trigger");
//   } else {
//     Serial.println("Motion sensor currently HIGH, using only timer wake-up");
//   }
  
//   // 2. Timer wakeup as backup (in case something goes wrong)
//   esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_SEC * 1000000ULL);
//   Serial.printf("Going to deep sleep for %d seconds unless motion is detected\n", DEEP_SLEEP_TIME_SEC);
  
//   delay(200); // Give time for serial to finish sending
  
//   // Enter deep sleep
//   esp_deep_sleep_start();
// }


/**
 * ESP32 Motion Detection Camera with Broadcast UDP Upload
 * 
 * This sketch detects motion using a PIR sensor and uploads 
 * captured images via UDP broadcast when motion is detected.
 * 
 * Hardware requirements:
 * - ESP32-CAM or compatible board
 * - PIR motion sensor
 * - Appropriate power supply
 * 
 * Pin connections:
 * - PIR sensor signal pin to GPIO 12
 * - Camera is connected via the default ESP32-CAM pins
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "config.h" // Contains WIFI_SSID and WIFI_PASSWORD

// UDP Settings
#define UDP_PORT 4210
const int PACKET_SIZE = 1024;

// Motion Sensor Pin
#define MOTION_SENSOR_PIN 12

// Camera Settings
#define CAMERA_MODEL_AI_THINKER // ESP32-CAM

// const unsigned long MOTION_SENSITIVITY = 500; 

// Define camera pin constants for AI Thinker model
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Function Declarations
bool initCamera();
bool captureImage();
bool sendImageViaBroadcastUDP();

// Global Variables
WiFiUDP udp;
camera_fb_t *fb = NULL;
bool motionDetected = false;
unsigned long lastDetectionTime = 0;
const unsigned long COOLDOWN_PERIOD = 1000; // 5 seconds cooldown between detections

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Motion Detection Camera with UDP Broadcast");
  
  // Initialize the motion sensor pin
  pinMode(MOTION_SENSOR_PIN, INPUT);
  
  // Initialize camera
  if (!initCamera()) {
    Serial.println("Camera initialization failed");
    return;
  }
  
  // Connect to WiFi
  Serial.printf("Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  
  // Start UDP
  udp.begin(UDP_PORT);
  Serial.println("UDP initialized");
}

void loop() {
  // Check for motion
  if (digitalRead(MOTION_SENSOR_PIN) == HIGH) {
    unsigned long currentTime = millis();
    
    // Check if we're past the cooldown period
    if (currentTime - lastDetectionTime > COOLDOWN_PERIOD) {
      Serial.println("Motion detected!");
      motionDetected = true;
      lastDetectionTime = currentTime;
      
      // Capture and send image
      if (captureImage()) {
        if (sendImageViaBroadcastUDP()) {
          Serial.println("Image sent successfully via UDP broadcast");
        } else {
          Serial.println("Failed to send image via UDP broadcast");
        }
        
        // Always free the memory after capturing an image
        if (fb) {
          esp_camera_fb_return(fb);
          fb = NULL;
        }
      } else {
        Serial.println("Failed to capture image");
      }
    }
  }
  
  delay(100); // Small delay to prevent CPU hogging
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Initial settings
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 12; // 0-63, lower number means higher quality
  config.fb_count = 1;
  
  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }
  
  // // Adjust camera settings after initialization if needed
  sensor_t * s = esp_camera_sensor_get();


  // if (s) {
  //   // Adjust brightness, contrast, etc. if needed
  //   s->set_brightness(s, 2);     // -2 to 2
  //   s->set_contrast(s, 0);       // -2 to 2
  //   s->set_saturation(s, 0);     // -2 to 2
  //   s->set_special_effect(s, 0); // 0 = No Effect, 1 = Negative, etc.
  //    s->set_gain_ctrl(s, 0); 
  // }

if (s) {
s->set_gain_ctrl(s, 0); // auto gain off (1 or 0)
s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
s->set_agc_gain(s, 0); // set gain manually (0 - 30)
s->set_aec_value(s, 600); // set exposure manually (0-1200)
}
  // if (s) {
  //   s->set_brightness(s, -2);     // Increase from 0 to 2 (max)
  //   s->set_gain_ctrl(s, 1);      // Auto gain on
  //   s->set_exposure_ctrl(s, 1);  // Auto exposure on
  //   s->set_aec2(s, 1);           // Enable auto exposure (AEC DSP)
  //   s->set_gainceiling(s, GAINCEILING_16X); // Maximum gain
  // }

  Serial.println("Camera initialized successfully");
  return true;
}

bool captureImage() {
  // Capture a frame
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  
  Serial.printf("Captured image: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
  return true;
}

bool sendImageViaBroadcastUDP() {
  if (!fb) {
    Serial.println("No image buffer to send");
    return false;
  }
  
  // Use broadcast address for UDP sending
  IPAddress broadcastIP(255, 255, 255, 255);
  
  // Send image size as header (4 bytes)
  uint32_t imageSize = fb->len;
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.write((uint8_t*)&imageSize, sizeof(imageSize));
  udp.endPacket();
  
  // Small delay to allow receiver to prepare
  delay(50);
  
  // Send image data in chunks
  size_t totalBytesSent = 0;
  size_t remainingBytes = fb->len;
  const uint8_t* currentBufferPosition = fb->buf;
  
  // Packet structure:
  // [2 bytes packet number][2 bytes total packets][PACKET_SIZE bytes data]
  uint16_t packetNumber = 0;
  uint16_t totalPackets = (fb->len + PACKET_SIZE - 1) / PACKET_SIZE; // Ceiling division
  
  while (remainingBytes > 0) {
    // Determine the size of this packet
    size_t bytesToSend = (remainingBytes > PACKET_SIZE) ? PACKET_SIZE : remainingBytes;
    
    // Begin packet
    udp.beginPacket(broadcastIP, UDP_PORT);
    
    // Send packet header
    udp.write((uint8_t*)&packetNumber, sizeof(packetNumber));
    udp.write((uint8_t*)&totalPackets, sizeof(totalPackets));
    
    // Send image data chunk
    udp.write(currentBufferPosition, bytesToSend);
    
    // End packet
    udp.endPacket();
    
    // Update counters and pointers
    currentBufferPosition += bytesToSend;
    totalBytesSent += bytesToSend;
    remainingBytes -= bytesToSend;
    packetNumber++;
    
    // Small delay to prevent network congestion
    delay(5);
  }
  
  Serial.printf("Sent %u bytes in %u packets\n", totalBytesSent, packetNumber);
  return (totalBytesSent == fb->len);
}


