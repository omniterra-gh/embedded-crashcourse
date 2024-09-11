// src/main.cpp
#include <Arduino.h>
#include <Wire.h>

// Define the I2C address of the AHT20 sensor
#define AHT20_ADDRESS 0x38

// Function prototype for reading the AHT20 sensor
bool readAHT20(float *temperature, float *humidity);

void setup() {
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize serial communication for debugging and output
  Serial.begin(9600);
  
  // Wait for 1 second to allow the sensor to power up and stabilize
  delay(1000);
  
  // Initialize the AHT20 sensor
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(0xBE);  // Initialization command
  Wire.write(0x08);  // Calibration command parameter 1
  Wire.write(0x00);  // Calibration command parameter 2
  Wire.endTransmission();
  
  // Wait for 10ms to ensure initialization is complete
  delay(10);
  
  Serial.println("AHT20 Sensor Initialized");
}

void loop() {
  // Variables to store temperature and humidity readings
  float temperature, humidity;
  
  // Attempt to read data from the AHT20 sensor
  if (readAHT20(&temperature, &humidity)) {
    // If successful, print the readings
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);  // Print with 2 decimal places
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity, 2);     // Print with 2 decimal places
    Serial.println(" %");
  } else {
    // If reading fails, print an error message
    Serial.println("Failed to read from AHT20 sensor!");
  }
  
  // Wait for 2 seconds before the next reading
  delay(2000);
}

bool readAHT20(float *temperature, float *humidity) {
  uint8_t data[6];  // Array to store raw data from sensor
  
  // Step 1: Trigger a new measurement
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(0xAC);  // Measurement trigger command
  Wire.write(0x33);  // Measurement command parameter 1
  Wire.write(0x00);  // Measurement command parameter 2
  Wire.endTransmission();
  
  // Wait for 80ms for the measurement to complete
  delay(80);
  
  // Step 2: Read the measurement data
  Wire.requestFrom(AHT20_ADDRESS, 6);  // Request 6 bytes of data
  if (Wire.available() != 6) {
    return false;  // If we don't get 6 bytes, return false to indicate failure
  }
  
  // Read all 6 bytes into our data array
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }
  
  // Step 3: Check the status bit
  if (data[0] & 0x80) {
    return false;  // If the high bit is set, the device is busy, so return false
  }
  
  // Step 4: Convert raw data to temperature and humidity values
  
  // Humidity is stored in bits 12-19 of data[1] and all bits of data[2]
  uint32_t h = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  // Temperature is stored in the lower 4 bits of data[3] and all bits of data[4] and data[5]
  uint32_t t = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  
  // Convert raw values to actual temperature and humidity
  *humidity = (float)h * 100 / 0x100000;  // Formula from datasheet
  *temperature = (float)t * 200 / 0x100000 - 50;  // Formula from datasheet
  
  return true;  // Return true to indicate successful reading
}