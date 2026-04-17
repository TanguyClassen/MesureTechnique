#include "arduino_secrets.h"
/*
  Arduino IoT Cloud Dashboard
  Hardware : Arduino Nano ESP32
  Sensors  : HX711 load cell amplifier + MMA8451 accelerometer
  Dashboard: Live weight, acceleration XYZ, tilt angles, time-series charts
             + Tare buttons for load cell and accelerometer

  --- Cloud Variables ---
  float  weight_kg     (Read Only)
  float  accel_x       (Read Only)
  float  accel_y       (Read Only)
  float  accel_z       (Read Only)
  float  tilt_x        (Read Only)
  float  tilt_y        (Read Only)
  bool   Tare_weight   (Read/Write)  <-- Push Button widget on dashboard
  bool   Tare_accel    (Read/Write)  <-- Push Button widget on dashboard
*/

#include "thingProperties.h"
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ── HX711 pins ──────────────────────────────────────────────────────────────
#define HX711_DOUT  D2
#define HX711_SCK   D3

// ── Calibration ─────────────────────────────────────────────────────────────
#define CALIBRATION_FACTOR  -7050.0   // <-- CHANGE after calibration
#define TARE_ON_BOOT        true

// ── Update rate ─────────────────────────────────────────────────────────────
#define UPDATE_INTERVAL_MS  500

// ── Accel tare offsets ───────────────────────────────────────────────────────
float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;

// ── Objects ─────────────────────────────────────────────────────────────────
HX711            scale;
Adafruit_MMA8451 mma;

unsigned long lastUpdate = 0;

// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // ── HX711 ──
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  if (TARE_ON_BOOT) {
    Serial.println("Taring scale on boot...");
    scale.tare(10);
    Serial.println("Scale tare done.");
  }

  // ── MMA8451 ──
  if (!mma.begin()) {
    Serial.println("ERROR: MMA8451 not found. Check wiring.");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.println("MMA8451 ready.");
}

// ────────────────────────────────────────────────────────────────────────────
void loop() {
  ArduinoCloud.update();

  if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
    lastUpdate = millis();
    readLoadCell();
    readAccelerometer();
  }
}

// ────────────────────────────────────────────────────────────────────────────
void readLoadCell() {
  if (!scale.is_ready()) return;

  float kg = scale.get_units(5);
  if (kg < 0.002 && kg > -0.002) kg = 0.0;
  weight_kg = kg;

  Serial.print("Weight: ");
  Serial.print(weight_kg, 3);
  Serial.println(" kg");
}

// ────────────────────────────────────────────────────────────────────────────
void readAccelerometer() {
  sensors_event_t event;
  mma.getEvent(&event);

  // Apply tare offsets
  accel_x = event.acceleration.x - accel_offset_x;
  accel_y = event.acceleration.y - accel_offset_y;
  accel_z = event.acceleration.z - accel_offset_z;

  tilt_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
  tilt_y = atan2(-accel_x, accel_z) * 180.0 / M_PI;

  Serial.printf("Accel  X=%.2f  Y=%.2f  Z=%.2f m/s²\n", accel_x, accel_y, accel_z);
  Serial.printf("Tilt   Pitch=%.1f°  Roll=%.1f°\n", tilt_x, tilt_y);
}

// ────────────────────────────────────────────────────────────────────────────
// Called by Arduino Cloud when Tare_weight flips to true from the dashboard
void onTareWeightChange() {
  if (Tare_weight) {
    Serial.println(">>> Taring load cell...");
    scale.tare(10);
    Serial.println(">>> Load cell tare done.");
    Tare_weight = false;   // Reset the button back to false
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Called by Arduino Cloud when Tare_accel flips to true from the dashboard
void onTareAccelChange() {
  if (Tare_accel) {
    Serial.println(">>> Taring accelerometer...");

    // Average several readings for a stable offset
    float sx = 0, sy = 0, sz = 0;
    int samples = 20;
    for (int i = 0; i < samples; i++) {
      sensors_event_t event;
      mma.getEvent(&event);
      sx += event.acceleration.x;
      sy += event.acceleration.y;
      sz += event.acceleration.z;
      delay(10);
    }
    accel_offset_x = sx / samples;
    accel_offset_y = sy / samples;
    accel_offset_z = sz / samples;

    Serial.printf(">>> Accel offsets set: X=%.3f Y=%.3f Z=%.3f\n",
                  accel_offset_x, accel_offset_y, accel_offset_z);

    Tare_accel = false;   // Reset the button back to false
  }
}

/*
  Since AccelX is READ_WRITE variable, onAccelXChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onAccelXChange()  {
  // Add your code here to act upon AccelX change
}
/*
  Since AccelY is READ_WRITE variable, onAccelYChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onAccelYChange()  {
  // Add your code here to act upon AccelY change
}
/*
  Since AccelZ is READ_WRITE variable, onAccelZChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onAccelZChange()  {
  // Add your code here to act upon AccelZ change
}
/*
  Since TiltX is READ_WRITE variable, onTiltXChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTiltXChange()  {
  // Add your code here to act upon TiltX change
}
/*
  Since TiltY is READ_WRITE variable, onTiltYChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTiltYChange()  {
  // Add your code here to act upon TiltY change
}
/*
  Since WeightKg is READ_WRITE variable, onWeightKgChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onWeightKgChange()  {
  // Add your code here to act upon WeightKg change
}