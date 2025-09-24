/* Wokwi-ready sketch:
   - ESP32 (SDA=21, SCL=22)
   - MPU6050 (I2C)
   - DS18B20 on GPIO4 with 4.7k pull-up
   - Optional SSD1306 OLED (I2C 0x3C)
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== pins =====
#define SDA_PIN 8
#define SCL_PIN 9
#define DS18_PIN 4

// ===== OLED setup =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== sensor objects =====
Adafruit_MPU6050 mpu;
OneWire oneWire(DS18_PIN);
DallasTemperature ds18(&oneWire);

// ===== algorithm params =====
const int WINDOW = 40;         // number of accel samples per window
float accBuffer[WINDOW];
const float TEMP_THRESHOLD = 60.0;   // degrees Celsius (tune later)
const float VIB_THRESHOLD  = 1.0;    // vibration stdev threshold (tune later)

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Starting simulation...");

  // I2C on ESP32
  Wire.begin(SDA_PIN, SCL_PIN);

  // start sensors
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (1) delay(10);
  } else {
    Serial.println("MPU6050 found.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  ds18.begin();

  // OLED (optional) init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C is common
    Serial.println("OLED not found (optional). Continuing without it.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("ESP32 sensor demo");
    display.display();
  }
}

float compute_stdev(float *arr, int n) {
  float sum = 0;
  for (int i=0;i<n;i++) sum += arr[i];
  float mean = sum / n;
  float var = 0;
  for (int i=0;i<n;i++) var += (arr[i]-mean)*(arr[i]-mean);
  return sqrt(var / n);
}

void loop() {
  // 1) read temperature
  ds18.requestTemperatures();
  float tempC = ds18.getTempCByIndex(0); // first sensor on the bus

  // 2) read accel WINDOW times
  for (int i = 0; i < WINDOW; i++) {
    sensors_event_t a, g, tempEvent;
    mpu.getEvent(&a, &g, &tempEvent);
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float mag = sqrt(ax*ax + ay*ay + az*az);
    accBuffer[i] = mag;
    delay(10); // ~100 Hz sampling (adjust if needed)
  }

  float stdev = compute_stdev(accBuffer, WINDOW);

  // 3) print status
  Serial.print("Temp (C): ");
  Serial.print(tempC, 2);
  Serial.print("  VibStdev: ");
  Serial.println(stdev, 4);

  // 4) show on OLED if present
  if (display.width() > 0) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Temp: ");
    display.print(tempC,1);
    display.println(" C");
    display.print("Vib stdev: ");
    display.println(stdev,3);
    if (tempC > TEMP_THRESHOLD || stdev > VIB_THRESHOLD) {
      display.println(">>> ALERT <<<");
    } else {
      display.println("Status: OK");
    }
    display.display();
  }

  // 5) simple alert on serial
  if (tempC > TEMP_THRESHOLD || stdev > VIB_THRESHOLD) {
    Serial.println("ALERT: motor may be failing!");
  }
  delay(500);
}
