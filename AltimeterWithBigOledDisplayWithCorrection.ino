#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <RTClib.h>
#include <U8g2lib.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

// ===================== PINS =====================
#define SDA_PIN 8
#define SCL_PIN 9
#define BUTTON_PIN 3
#define BATTERY_PIN 0

// ===================== DEVICES =====================
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0);

// ===================== STATE =====================
bool useFeet = false;
float SEA_LEVEL_PRESSURE = 1013.25; // Will update dynamically during setup
float baseAltitude = 0;

// altitude
float smoothedAltitude = 0;
float lastValidAlt = 0;

// vertical speed
float lastAltForSpeed = 0;
unsigned long lastSpeedTime = 0;
float verticalSpeed = 0;
float smoothedSpeed = 0;

// tracking state for landing/climbing logic
bool hasClearedThreshold = false;

// battery
float rawMax = 2750.0;
float rawMin = 2150.0;
float smoothedRaw = 0;

// ===================== BATTERY =====================
int getBatteryPercent() {
  float sum = 0;

  for (int i = 0; i < 20; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }

  float raw = sum / 20.0;

  if (smoothedRaw == 0) smoothedRaw = raw;
  smoothedRaw = smoothedRaw * 0.85 + raw * 0.15;

  float norm = (smoothedRaw - rawMin) / (rawMax - rawMin);
  norm = constrain(norm, 0.0, 1.0);

  return (int)(norm * 100);
}

// ===================== BATTERY UI =====================
void drawBattery(int x, int y, int p) {
  u8g2.drawFrame(x, y, 18, 8);      // outer battery
  u8g2.drawBox(x + 18, y + 2, 2, 4); // battery tip

  int blocks = 0;

  if (p >= 90) {
    blocks = 4;
  } 
  else if (p >= 51) {
    blocks = 3;
  } 
  else if (p >= 31) {
    blocks = 2;
  } 
  else if (p >= 10) {
    blocks = 1;
  } 
  else {
    blocks = 0;
  }

  // draw blocks
  for (int i = 0; i < blocks; i++) {
    u8g2.drawBox(x + 2 + i * 4, y + 2, 3, 4);
  }
}

// ===================== ALTITUDE CALIBRATION =====================
void calibrateZero() {
  // Dynamically grab the current sea level pressure baseline
  float measured_pressure = bmp.readPressure() / 100.0;
  SEA_LEVEL_PRESSURE = measured_pressure;

  float sum = 0;
  for (int i = 0; i < 40; i++) {
    sum += bmp.readAltitude(SEA_LEVEL_PRESSURE);
    delay(10);
  }

  baseAltitude = sum / 40.0;
  smoothedAltitude = 0;
  lastValidAlt = 0;
  hasClearedThreshold = false; // Reset takeoff state
}

// ===================== VERTICAL SPEED =====================
void updateVerticalSpeed(float currentAlt) {
  unsigned long now = millis();
  float dt = (now - lastSpeedTime) / 1000.0;

  if (dt <= 0) return;

  float rawSpeed = (currentAlt - lastAltForSpeed) / dt;

  // filter noise spikes
  if (abs(rawSpeed) < 5.0) {
    smoothedSpeed = smoothedSpeed * 0.85 + rawSpeed * 0.15;
  }

  verticalSpeed = smoothedSpeed;

  lastAltForSpeed = currentAlt;
  lastSpeedTime = now;
}

// ===================== SLEEP =====================
void goToSleep() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(40, 32, "SLEEP");
  u8g2.sendBuffer();
  delay(300);

  u8g2.setPowerSave(1);

  gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  delay(100);
  esp_light_sleep_start();

  u8g2.setPowerSave(0);
}

// ===================== STEPPED DISPLAY PROCESSING =====================
int getSteppedAltitude(float rawMeters, float vSpeed) {
  // 1. Manage climb threshold vs descent logic
  if (!hasClearedThreshold) {
    if (rawMeters >= 50.0) {
      hasClearedThreshold = true; // Trip wire crossed, we are up!
    } else if (vSpeed >= -0.2) {
      // If we haven't crossed 50m and aren't actively coming down hard, lock to 0
      return 0;
    }
  }
  
  // If descending back near or below zero, clean lock back to 0
  if (rawMeters <= 0.5) {
    hasClearedThreshold = false; // Reset for next takeoff
    return 0;
  }

  // 2. Variable meter stepping profiles (Snap processing)
  int steppedMeters = 0;
  if (rawMeters >= 1000.0) {
    // Snap to nearest 10 meters
    steppedMeters = ((int)round(rawMeters / 10.0)) * 10;
  } else {
    // Snap to nearest 5 meters
    steppedMeters = ((int)round(rawMeters / 5.0)) * 5;
  }

  return steppedMeters;
}

// ===================== SETUP =====================
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  if (!bmp.begin(0x76)) {
    while (1) {
      u8g2.clearBuffer();
      u8g2.drawStr(10, 30, "BMP FAIL");
      u8g2.sendBuffer();
      delay(1000);
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16);

  if (!rtc.begin()) {
    while (1) {
      u8g2.clearBuffer();
      u8g2.drawStr(10, 30, "RTC FAIL");
      u8g2.sendBuffer();
      delay(1000);
    }
  }

  calibrateZero();
}

// ===================== LOOP =====================
void loop() {

  // ================= BUTTON =================
  static unsigned long pressStart = 0;
  static unsigned long lastClick = 0;
  static int clickCount = 0;
  static bool pressedState = false;

  bool pressed = (digitalRead(BUTTON_PIN) == LOW);

  if (pressed && !pressedState) {
    pressStart = millis();
    pressedState = true;
  }

  if (!pressed && pressedState) {
    unsigned long duration = millis() - pressStart;

    if (duration > 2500) {
      goToSleep();
    } else {
      clickCount++;
      lastClick = millis();
    }

    pressedState = false;
  }

  if (clickCount > 0 && millis() - lastClick > 450) {

    if (clickCount == 1) {
      useFeet = !useFeet;
    }

    if (clickCount >= 2) {
      calibrateZero();

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.drawStr(0, 28, "RECALIBRATED");
      u8g2.sendBuffer();
      delay(900);
    }

    clickCount = 0;
  }

  // ================= ALTITUDE =================
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE) - baseAltitude;

  if (abs(alt - lastValidAlt) < 80) {
    smoothedAltitude = smoothedAltitude * 0.90 + alt * 0.10;
    lastValidAlt = alt;
  }

  updateVerticalSpeed(smoothedAltitude);

  // Process our brand new meter stepping & threshold rules
  int processedMeters = getSteppedAltitude(smoothedAltitude, verticalSpeed);

  // Convert output units based on processed meters context
  float displayAlt = useFeet ? (float)processedMeters * 3.28084 : (float)processedMeters;
  float displaySpeed = useFeet ? verticalSpeed * 3.28084 : verticalSpeed;

  // ================= BATTERY =================
  int batt = getBatteryPercent();

  // ================= TIME =================
  DateTime now = rtc.now();

  // ================= DISPLAY =================
  u8g2.clearBuffer();

  // ALTITUDE
  char buf[12];
  sprintf(buf, "%d", (int)round(displayAlt));

  u8g2.setFont(u8g2_font_logisoso38_tn);

  int w = u8g2.getStrWidth(buf);
  int uw = u8g2.getStrWidth(useFeet ? "ft" : "m");

  int x = (128 - (w + uw)) / 2;

  u8g2.setCursor(x, 50);
  u8g2.print(buf);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(x + w + 2, 50);
  u8g2.print(useFeet ? "ft" : "m");

  // ================= VERTICAL SPEED =================
  char speedBuf[24];
  const char* unitStr = useFeet ? "ft/s" : "m/s";

  if (verticalSpeed > 0.05) {
    sprintf(speedBuf, "▲ %.1f %s", displaySpeed, unitStr);
  } else if (verticalSpeed < -0.05) {
    sprintf(speedBuf, "▼ %.1f %s", displaySpeed, unitStr);
  } else {
    sprintf(speedBuf, "0.0 %s", unitStr);
  }

  // TOP LEFT CORNER
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 10);
  u8g2.print(speedBuf);

  // ================= TIME =================
  u8g2.setCursor(0, 63);
  if (now.hour() < 10) u8g2.print("0");
  u8g2.print(now.hour());
  u8g2.print(":");
  if (now.minute() < 10) u8g2.print("0");
  u8g2.print(now.minute());

  // DATE
  char d[16];
  sprintf(d, "%02d/%02d/%04d", now.day(), now.month(), now.year());
  u8g2.setCursor(70, 63);
  u8g2.print(d);

  // ================= BATTERY =================
  drawBattery(106, 2, batt);

  u8g2.setCursor(70, 10);
  u8g2.print(batt);
  u8g2.print("%");

  if (batt < 15) {
    u8g2.setCursor(0, 10);
    u8g2.print("LOW");
  }

  u8g2.sendBuffer();

  delay(80);
}