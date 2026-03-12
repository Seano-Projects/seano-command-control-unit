/*
 * ESP32 Battery Monitor for SEANO
 * 
 * Hardware:
 * - ESP32 DevKit
 * - INA219/INA226 Current/Voltage Sensor Module
 * - Connect to Jetson Nano via Serial
 * 
 * Connections:
 * ESP32 TX (GPIO17) -> Jetson RX (Pin 10)
 * ESP32 RX (GPIO16) -> Jetson TX (Pin 8)
 * ESP32 GND -> Jetson GND
 * 
 * This code reads voltage and current from INA219
 * and sends data to Jetson via Serial in JSON format
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// Serial communication settings
#define SERIAL_BAUDRATE 115200
#define SEND_INTERVAL 1000  // Send data every 1 second (1000ms)

unsigned long lastSendTime = 0;

void setup() {
  // Initialize Serial for communication with Jetson
  Serial.begin(SERIAL_BAUDRATE);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize INA219
  if (!ina219.begin()) {
    Serial.println("{\"error\":\"INA219 not found\"}");
    while (1) {
      delay(1000);
      Serial.println("{\"error\":\"INA219 not found\"}");
    }
  }
  
  // Configure INA219
  // By default the INA219 will be calibrated with a range of 32V, 2A
  // You can change to 16V, 400mA if needed
  ina219.setCalibration_32V_2A();
  
  Serial.println("{\"status\":\"Battery monitor initialized\"}");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send data at specified interval
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Read sensor data
    float voltage = ina219.getBusVoltage_V();
    float current = ina219.getCurrent_mA() / 1000.0;  // Convert to Amperes
    
    // Optional: Add shunt voltage for more accuracy
    // float shuntVoltage = ina219.getShuntVoltage_mV() / 1000.0;
    // voltage = voltage + shuntVoltage;
    
    // Ensure positive current value
    if (current < 0) {
      current = 0;
    }
    
    // Send data in JSON format
    Serial.print("{\"voltage\":");
    Serial.print(voltage, 2);  // 2 decimal places
    Serial.print(",\"current\":");
    Serial.print(current, 2);
    Serial.println("}");
    
    // Alternative simple format (uncomment if preferred):
    // Serial.print("V:");
    // Serial.print(voltage, 2);
    // Serial.print(",A:");
    // Serial.println(current, 2);
  }
  
  delay(10);  // Small delay to prevent CPU hogging
}
