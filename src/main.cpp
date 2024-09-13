#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
 
#define AHT20_ADDRESS 0x38
#define AHT20_INIT_CMD 0xBE
 
// Add debug mode flag
#define DEBUG_MODE 0
 
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
 
TinyGPSPlus gps;
SoftwareSerial ss(20, 21); // RX, TX
 //prototypes
void initAHT20();
void readAHT20(float &temp, float &hum);
int readA0();
void readGPS(float &lat, float &lon);
void displayData(float temp, float hum, int a0, float lat, float lon);
 
void setup() {
  #if DEBUG_MODE
  Serial.begin(9600);
  #endif
  u8g2.begin();
  Wire.begin();
  ss.begin(9600);
 
  initAHT20();
 
  #if DEBUG_MODE
  Serial.println(F("System initialized"));
  #endif
}
 
void loop() {
  float temperature, humidity, latitude, longitude;
  int analogValue;
 
  readAHT20(temperature, humidity);
  analogValue = readA0();
  readGPS(latitude, longitude);
 
  displayData(temperature, humidity, analogValue, latitude, longitude);
 
  // delay(1000);
}
 
void initAHT20() {
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(AHT20_INIT_CMD);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);  // Wait for sensor to initialize
}
 
void readAHT20(float &temp, float &hum) {
  uint8_t data[6];
 
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(0xAC);  // Trigger measurement
  Wire.write(0x33);
  Wire.write(0x00);
  Wire.endTransmission();
 
  delay(80);  // Wait for measurement to complete
 
  Wire.requestFrom(AHT20_ADDRESS, 6);
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }
 
  hum = ((uint32_t)data[1] << 12 | (uint32_t)data[2] << 4 | (data[3] >> 4)) * 100.0 / 0x100000;
  temp = ((uint32_t)(data[3] & 0x0F) << 16 | (uint32_t)data[4] << 8 | data[5]) * 200.0 / 0x100000 - 50;
}
 
int readA0() {
  return analogRead(A0);
}
 
void readGPS(float &lat, float &lon) {
  unsigned long startTime = millis();
  bool newData = false;
 
  while (millis() - startTime < 1000) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        newData = true;
      }
    }
 
    if (newData) {
      if (gps.location.isValid()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
 
        #if DEBUG_MODE
        Serial.println(F("GPS Debug Information:"));
        Serial.print(F("Latitude: "));
        Serial.println(lat, 6);
        Serial.print(F("Longitude: "));
        Serial.println(lon, 6);
        #endif
      } else {
        #if DEBUG_MODE
        Serial.println(F("Location: INVALID"));
        #endif
      }
 
      #if DEBUG_MODE
      Serial.print(F("Satellites in view: "));
      Serial.println(gps.satellites.value());
      Serial.print(F("HDOP: "));
      Serial.println(gps.hdop.hdop());
 
      if (gps.altitude.isValid()) {
        Serial.print(F("Altitude: "));
        Serial.print(gps.altitude.meters());
        Serial.println(F(" meters"));
      } else {
        Serial.println(F("Altitude: INVALID"));
      }
 
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.printf("Date/Time: %02d/%02d/%04d %02d:%02d:%02d\n",
                      gps.date.month(), gps.date.day(), gps.date.year(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
      } else {
        Serial.println(F("Date/Time: INVALID"));
      }
 
      Serial.println();
      #endif
 
      newData = false;
 
      if (gps.location.isValid()) {
        return;  // Exit the function if we have a valid location
      }
    }
  }
 
  lat = lon = 0.0; // Invalid or no fix
  #if DEBUG_MODE
  Serial.println(F("GPS: No valid data received in time."));
  #endif
}
 
void displayData(float temp, float hum, int a0, float lat, float lon) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
 
    u8g2.setCursor(0, 10);
    u8g2.print(F("Temp: "));
    u8g2.print(temp);
    u8g2.print(F("C"));
 
    u8g2.setCursor(0, 20);
    u8g2.print(F("Hum: "));
    u8g2.print(hum);
    u8g2.print(F("%"));
 
    u8g2.setCursor(0, 30);
    u8g2.print(F("A0: "));
    u8g2.print(a0);
 
    u8g2.setCursor(0, 40);
    u8g2.print(F("Lat: "));
    u8g2.print(lat, 6);
 
    u8g2.setCursor(0, 50);
    u8g2.print(F("Lon: "));
    u8g2.print(lon, 6);
 
    int barWidth = map(a0, 0, 1023, 0, 128);
    u8g2.drawFrame(0, 55, 128, 9);
    u8g2.drawBox(0, 55, barWidth, 9);
  } while (u8g2.nextPage());
}