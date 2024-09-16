#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ADS1X15.h>

#define AHT20_ADDRESS 0x38
#define AHT20_INIT_CMD 0xBE

#define DEBUG_MODE 1
#define GPS_TIMEOUT 2000  // 2 seconds timeout for GPS

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

TinyGPSPlus gps;
SoftwareSerial ss(20, 21); // RX, TX

ADS1115 ADS(0x48);  // Create ADS1115 instance with address 0x48

// Prototypes
bool initAHT20();
bool readAHT20(float &temp, float &hum);
bool initADS1115();
int16_t readADS1115();
bool readGPS(float &lat, float &lon);
void displayData(float temp, float hum, int16_t adcValue, float lat, float lon);

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect
  Serial.println(F("Starting setup..."));
  
  Wire.begin();
  u8g2.begin();
  ss.begin(9600);

  if (!initAHT20()) {
    Serial.println(F("Failed to initialize AHT20"));
  } else {
    Serial.println(F("AHT20 initialized successfully"));
  }

  if (!initADS1115()) {
    Serial.println(F("Failed to initialize ADS1115"));
  } else {
    Serial.println(F("ADS1115 initialized successfully"));
  }

  Serial.println(F("System initialized"));
}

void loop() {
  float temperature, humidity, latitude, longitude;
  int16_t adcValue;

  Serial.println(F("\nStarting sensor readings..."));

  if (!readAHT20(temperature, humidity)) {
    Serial.println(F("Failed to read AHT20"));
    temperature = humidity = NAN;
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.print(F("°C, Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
  }

  adcValue = readADS1115();
  Serial.print(F("ADC Value: "));
  Serial.println(adcValue);

  if (!readGPS(latitude, longitude)) {
    Serial.println(F("Failed to get GPS fix"));
    latitude = longitude = NAN;
  } else {
    Serial.print(F("Latitude: "));
    Serial.print(latitude, 6);
    Serial.print(F(", Longitude: "));
    Serial.println(longitude, 6);
  }

  displayData(temperature, humidity, adcValue, latitude, longitude);

  // delay(5000);  // Increase delay for easier debugging
}

bool initAHT20() {
  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(AHT20_INIT_CMD);
  Wire.write(0x08);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(10);  // Wait for sensor to initialize
  return true;
}

bool readAHT20(float &temp, float &hum) {
  uint8_t data[6];

  Wire.beginTransmission(AHT20_ADDRESS);
  Wire.write(0xAC);  // Trigger measurement
  Wire.write(0x33);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  delay(80);  // Wait for measurement to complete

  if (Wire.requestFrom(AHT20_ADDRESS, 6) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }

  hum = ((uint32_t)data[1] << 12 | (uint32_t)data[2] << 4 | (data[3] >> 4)) * 100.0 / 0x100000;
  temp = ((uint32_t)(data[3] & 0x0F) << 16 | (uint32_t)data[4] << 8 | data[5]) * 200.0 / 0x100000 - 50;

  return !(isnan(temp) || isnan(hum));
}

bool initADS1115() {
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(7);  // fast: 0=8SPS, 4=250SPS, 7=860SPS
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
  return true;
}

int16_t readADS1115() {
  int16_t raw = ADS.readADC(0);  // read from A0
  float volts = ADS.toVoltage(raw);  // convert to voltage

  Serial.print("Raw ADC: ");
  Serial.print(raw);
  Serial.print(", Voltage: ");
  Serial.print(volts, 3);
  Serial.println(" V");

  return raw;
}

bool readGPS(float &lat, float &lon) {
  unsigned long startTime = millis();
  bool newData = false;

  while (millis() - startTime < GPS_TIMEOUT) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        newData = true;
      }
    }

    if (newData && gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      return true;
    }
  }

  return false;
}

void displayData(float temp, float hum, int16_t adcValue, float lat, float lon) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.setCursor(0, 10);
    u8g2.print(F("Temp: "));
    if (isnan(temp)) {
      u8g2.print(F("Error"));
    } else {
      u8g2.print(temp);
      u8g2.print(F("C"));
    }

    u8g2.setCursor(0, 20);
    u8g2.print(F("Hum: "));
    if (isnan(hum)) {
      u8g2.print(F("Error"));
    } else {
      u8g2.print(hum);
      u8g2.print(F("%"));
    }

    u8g2.setCursor(0, 30);
    u8g2.print(F("ADC: "));
    u8g2.print(adcValue);

    u8g2.setCursor(0, 40);
    u8g2.print(F("Lat: "));
    if (isnan(lat)) {
      u8g2.print(F("No Fix"));
    } else {
      u8g2.print(lat, 6);
    }

    u8g2.setCursor(0, 50);
    u8g2.print(F("Lon: "));
    if (isnan(lon)) {
      u8g2.print(F("No Fix"));
    } else {
      u8g2.print(lon, 6);
    }

    int barWidth = map(adcValue, 0, 14846, 0, 128);
    u8g2.drawFrame(0, 55, 128, 9);
    u8g2.drawBox(0, 55, barWidth, 9);
  } while (u8g2.nextPage());
}