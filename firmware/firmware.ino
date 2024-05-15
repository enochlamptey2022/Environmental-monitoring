#define BLYNK_TEMPLATE_ID "TMPL2O7HxsgQU"
#define BLYNK_TEMPLATE_NAME "ENVIRONMENTAL MONITORING"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h> // Include the Blynk library for ESP32

#define BUZZER_PIN  18 // ESP32 pin GPIO18 connected to piezo buzzer
#define TONE_FREQUENCY 1000 // Adjust this value to change the tone frequency
#define TONE_DURATION 500 // Adjust this value to change the duration of each tone
#define DELAY_BETWEEN_TONES 500 // Adjust this value to change the delay between tones

// Blynk authentication token
char auth[] = "C_VK0K3WDageK7vRDYXGqBmnJRkvvTTw";

// Your WiFi credentials
char ssid[] = "iPhone";
char pass[] = "enoch222";


// MQ135 board specific macros
#define Board ("ESP32")
#define Pin (34)       // Analog input 3 of the MQ135 sensor 
#define Type ("MQ-135")  // MQ135
#define Voltage_Resolution (3.3)
#define ADC_Bit_Resolution (12)  // For arduino UNO/MEGA/NANO
#define RatioMQ2CleanAir (9.83)  // RS / R0 = 9.83 ppm

// DHT22 config macros
#define DHT_SENSOR_PIN 14  // ESP32 pin GPIO14 connected to DHT22 sensor
#define DHT_SENSOR_TYPE DHT22
#define OLED_ADDR 0x3C

// Serial Port connections for PM2.5 Sensor
#define RXD2 16  // To sensor TXD
#define TXD2 17  // To sensor RXD

Adafruit_SSD1306 display(128, 64, &Wire, -1);
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

volatile float humi;
volatile float tempC;
volatile float tempF;

volatile uint16_t pm10, pm25, pm100;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  dht_sensor.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  MQ135.setRegressionMethod(1);  
  MQ135.setA(574.25);
  MQ135.setB(-2.222);  
  MQ135.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();  
    calcR0 += MQ135.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }
  MQ135.serialDebug(true);

  // Connect to Wi-Fi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  
  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);
}

void loop() {
  Blynk.run(); // Run Blynk
  
  display.clearDisplay();
  for (int i = 63; i < 66; i++) {
    display.drawFastVLine(i, 0, 64, WHITE);
    display.drawFastHLine(0, i - 32, 128, WHITE);
  }
  displayTemperature();
  displayCO();

  if (readPMSdata(&Serial1)) {
    setPlaceholders(data);
  }
  displayPPM(data);

  // Sound the buzzer based on sensor readings
  if ( pm25 > 12 || MQ135.readSensor() > 9) {
    soundBuzzer();
  }

  // Send sensor data to Blynk
  sendSensorDataToBlynk();

  display.display();
  delay(2000);
}

void displayCO() {
  MQ135.update();
  float coValue = MQ135.readSensor();
  display.setTextSize(2);
  display.setCursor(68, 6);
  if (coValue < 10) {
    display.print(" ");
  }
  display.print(coValue);
  display.setTextSize(1);
  display.setCursor(68, 22);
  display.print("ppm");
}

void setPlaceholders(struct pms5003data data) {
  pm10 = data.pm10_standard;
  pm100 = data.pm100_standard;
  pm25 = data.pm25_standard;
}

void displayPPM(struct pms5003data data) {
  display.setCursor(0, 6);
  display.print("1");
  display.setCursor(28, 6);
  display.print(pm10);
  display.print("ppm");
  display.setCursor(0, 14);
  display.print("2.5");
  display.setCursor(28, 14);
  display.print(pm25);
  display.print("ppm");
  display.setCursor(0, 22);
  display.print("10");
  display.setCursor(28, 22);
  display.print(pm100);
  display.print("ppm");
  Serial.println();
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (standard)");
  Serial.print("PM 1.0: ");
  Serial.print(data.pm10_standard);
  Serial.print("\t\tPM 2.5: ");
  Serial.print(data.pm25_standard);
  Serial.print("\t\tPM 10: ");
  Serial.println(data.pm100_standard);
}

void displayTemperature() {
  readTemperature();
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(tempC);
  display.setCursor(68, 40);
  display.print(humi);
  display.setTextSize(1);
  display.setCursor(0, 56);
  char deg = 248;
  display.print("degrees");
  display.setCursor(68, 56);
  display.print("percent");
}

boolean readPMSdata(Stream *s) {
  if (!s->available()) {
    return false;
  }
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
  if (s->available() < 32) {
    return false;
  }
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
  memcpy((void *)&data, (void *)buffer_u16, 30);
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  return true;
}

void readTemperature() {
  humi = dht_sensor.readHumidity();
  tempC = dht_sensor.readTemperature();
  tempF = dht_sensor.readTemperature(true);
  if (isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");
    Serial.print("  |  ");
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");
  }
}

void soundBuzzer() {
  tone(BUZZER_PIN, TONE_FREQUENCY);
  delay(TONE_DURATION);
  noTone(BUZZER_PIN);
  delay(DELAY_BETWEEN_TONES);
}
void sendSensorDataToBlynk() {
  // Read sensor data
  float coValue = MQ135.readSensor();
  readTemperature();
  readPMSdata(&Serial1);

  // Send sensor data to Blynk
  Blynk.virtualWrite(V0, pm25); // Send PM2.5 data to virtual pin V0
  Blynk.virtualWrite(V1, tempC); // Send temperature data to virtual pin V1
  Blynk.virtualWrite(V2, humi); // Send humidity data to virtual pin V2
  Blynk.virtualWrite(V3, coValue); // Send CO2 data to virtual pin V3

  // Check if PM2.5 exceeds 30
  if (pm25 > 30) {
    Blynk.logEvent("pollution_alert", "Bad air! Make sure you have your inhaler!!!!!"); // Log event to Blynk
  }
}
