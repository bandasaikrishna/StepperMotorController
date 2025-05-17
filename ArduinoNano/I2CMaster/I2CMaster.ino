#include <Wire.h>

#define ESP32_ADDR 0x08  // ESP32's I2C address
int16_t angleToSend = 0;
bool newAngleAvailable = false;
unsigned long lastRequestTime = 0;
const unsigned long requestInterval = 100;  // Request angle every 100ms

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C as Master
  Serial.println("Nano Master Ready");
  Serial.println("Enter angle (0-360) in Serial Monitor:");
}

void requestAngleFromSlave() {
  Wire.requestFrom(ESP32_ADDR, sizeof(float));  // Request 4 bytes (size of float)
  
  if (Wire.available() == sizeof(float)) {
    union {
      byte b[4];
      float f;
    } angleData;
    
    for (int i = 0; i < 4; i++) {
      angleData.b[i] = Wire.read();
    }
    
    Serial.print("Received outAngle from slave: ");
    Serial.println(angleData.f, 4);  // Print with 4 decimal places
  } else {
    Serial.println("Error: Invalid angle data received");
  }
}

void loop() {
  // Read from Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    angleToSend = input.toInt();
    
    if (angleToSend <= 360) {
      newAngleAvailable = true;
      Serial.print("Preparing to send: ");
      Serial.println(angleToSend);
    } else {
      Serial.println("Error: Angle must be 0-360");
    }
  }

  // Send via I2C when new angle is available
  if (newAngleAvailable) {
    Wire.beginTransmission(ESP32_ADDR);
    Wire.write(highByte(angleToSend));  // Send MSB first
    Wire.write(lowByte(angleToSend));   // Send LSB
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Successfully sent: ");
      Serial.println(angleToSend);
    } else {
      Serial.print("Transmission failed. Error code: ");
      Serial.println(error);
    }
    
    newAngleAvailable = false;
  }

  // Request angle from slave at regular intervals
  if (millis() - lastRequestTime >= requestInterval) {
    requestAngleFromSlave();
    lastRequestTime = millis();
  }
}