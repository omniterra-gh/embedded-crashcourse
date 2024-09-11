#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// 
#define SS 10
#define RST 9
#define DIO0 7

// GPS module connection
#define GPS_RX 20
#define GPS_TX 21

// Enable pin
#define ENABLE_PIN 8

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
void initLoRa();
void initGPS();
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("LoRa GPS Sender");
  // Initialize enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  // Set enable pin HIGH
  digitalWrite(ENABLE_PIN, HIGH);

  // Initialize GPS and Lora
  initLoRa();
  initGPS();
}
//
void initLoRa()
{
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(868.1E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setSyncWord(0x34);
  LoRa.setSpreadingFactor(12);
}
//
void initGPS()
{
  gpsSerial.begin(9600);
}

void loop()
{
  // Read GPS data
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        Serial.println("Sending GPS data via LoRa");

        String dataString = String(gps.location.lat(), 6) + "," +
                            String(gps.location.lng(), 6) + "," +
                            String(gps.altitude.meters(), 2) + "," +
                            String(gps.date.year()) + String(gps.date.month()) + String(gps.date.day()) + "," +
                            String(gps.time.hour()) + String(gps.time.minute()) + String(gps.time.second());

        // Send packet
        LoRa.beginPacket();
        LoRa.print(dataString);
        LoRa.endPacket();

        Serial.println("Sent: " + dataString);

        // Set enable pin LOW
        // digitalWrite(ENABLE_PIN, LOW);

        // Wait before next transmission
        delay(5000);
      }
    }
  }

  // Check for GPS timeout
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring.");
    delay(5000);
  }
}