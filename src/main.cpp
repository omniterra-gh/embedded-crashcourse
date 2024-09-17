// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#define AHT20_ADDRESS 0x38
#define AHT20_MEASURE_CMD 0xAC
#define AHT20_MEASURE_DATA 0x33
#define AHT20_RESET_CMD 0xBA

// LoRa pin configuration
#define SS_PIN 10
#define RST_PIN 9
#define DIO0_PIN 7

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Wire.begin();

  // Initialize AHT20 sensor
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(AHT20_RESET_CMD);
  Wire.endTransmission();
  delay(20); // Wait for sensor to reset

  // Initialize LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(868.1E6)) { // Change frequency as needed for your region
    Serial.println("LoRa initialization failed. Check your connections.");
    while (1) delay(10);
  }

  // Set LoRa parameters
  LoRa.setSpreadingFactor(12); // Set spreading factor (6-12). Higher SF increases range but decreases data rate
  LoRa.setSyncWord(0xF3);

  Serial.println("AHT20 and LoRa initialized successfully!");
}

void loop() {
  uint8_t data[6];
  
  // Send measure command to AHT20
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(AHT20_MEASURE_CMD);
  Wire.write(AHT20_MEASURE_DATA);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(80); // Wait for measurement to complete

  // Read 6 bytes of data from AHT20
  Wire.requestFrom(AHT20_ADDRESS, 6);
  if (Wire.available() == 6) {
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }
  }

  // Calculate humidity and temperature
  uint32_t humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  uint32_t temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  
  float humidity_float = (float)humidity * 100 / 0x100000;
  float temperature_float = (float)temperature * 200 / 0x100000 - 50;

  // Prepare data string
  String dataString = "Temp: " + String(temperature_float, 1) + " C, Humidity: " + String(humidity_float, 1) + " %";
  
  // Send data via LoRa
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();

  Serial.println("Sent: " + dataString);

  delay(60000); // Wait for 1 minute before next reading
}