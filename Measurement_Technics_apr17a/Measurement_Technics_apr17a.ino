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

#include "arduino_secrets.h"
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
#define CALIBRATION_FACTOR  -7050.0
#define TARE_ON_BOOT        true

// ── Update rates ─────────────────────────────────────────────────────────────
#define SERIAL_INTERVAL_MS  100     // ~10 Hz display
#define CLOUD_INTERVAL_MS   1000    // 1 Hz cloud sync

// ── Circular buffer (last 30 lines) ──────────────────────────────────────────
#define BUFFER_LINES 30
char lineBuffer[BUFFER_LINES][110];
uint8_t bufHead  = 0;
uint8_t bufCount = 0;

// ── ANSI escape codes ─────────────────────────────────────────────────────────
#define ANSI_CLEAR    "\033"
#define ANSI_HOME     "\033"
#define ANSI_BOLD     "\033"
#define ANSI_RESET    "\033"

// ── Accel tare offsets ───────────────────────────────────────────────────────
float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;

// ── Objects ──────────────────────────────────────────────────────────────────
HX711            scale;
Adafruit_MMA8451 mma;

unsigned long lastSerial = 0;
unsigned long lastCloud  = 0;

// ─────────────────────────────────────────────────────────────────────────────
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

  Serial.print(ANSI_CLEAR);
  Serial.print(ANSI_HOME);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  ArduinoCloud.update();

  unsigned long now = millis();

  if (now - lastSerial >= SERIAL_INTERVAL_MS) {
    lastSerial = now;
    readLoadCell();
    readAccelerometer();
    pushLine();
    redrawScreen();
  }

  if (now - lastCloud >= CLOUD_INTERVAL_MS) {
    lastCloud = now;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void readLoadCell() {
  if (!scale.is_ready()) return;
  float kg = scale.get_units(1);
  if (kg > -0.002 && kg < 0.002) kg = 0.0;
  weight_kg = kg;
}

// ─────────────────────────────────────────────────────────────────────────────
void readAccelerometer() {
  sensors_event_t event;
  mma.getEvent(&event);

  accel_x = event.acceleration.x - accel_offset_x;
  accel_y = event.acceleration.y - accel_offset_y;
  accel_z = event.acceleration.z - accel_offset_z;

  tilt_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
  tilt_y = atan2(-accel_x, accel_z) * 180.0 / M_PI;
}

// ─────────────────────────────────────────────────────────────────────────────
void pushLine() {
  float norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
  snprintf(lineBuffer[bufHead], sizeof(lineBuffer[0]),
    "  %9.3f  | %9.3f | %9.3f | %9.3f |  %9.3f  | %8.2f | %7.2f",
    weight_kg, accel_x, accel_y, accel_z, norm, tilt_x, tilt_y);
  bufHead = (bufHead + 1) % BUFFER_LINES;
  if (bufCount < BUFFER_LINES) bufCount++;
}

// ─────────────────────────────────────────────────────────────────────────────
void redrawScreen() {
  Serial.print(ANSI_HOME);

  Serial.print(ANSI_BOLD);
  Serial.println(F("Weight(kg) |  Accel X  |  Accel Y  |  Accel Z  |  Norm(m/s2) | Pitch(deg) | Roll(deg)"));
  Serial.print(ANSI_RESET);
  Serial.println(F(" -----------+-----------+-----------+-----------+-------------+------------+----------"));

  uint8_t start = (bufCount < BUFFER_LINES) ? 0 : bufHead;
  for (uint8_t i = 0; i < bufCount; i++) {
    uint8_t idx = (start + i) % BUFFER_LINES;
    Serial.println(lineBuffer[idx]);
  }

  for (uint8_t i = bufCount; i < BUFFER_LINES; i++) {
    Serial.println(F(""));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void onTareWeightChange() {
  if (Tare_weight) {
    scale.tare(10);
    Tare_weight = false;
  }
}

void onTareAccelChange() {
  if (Tare_accel) {
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < 20; i++) {
      sensors_event_t event;
      mma.getEvent(&event);
      sx += event.acceleration.x;
      sy += event.acceleration.y;
      sz += event.acceleration.z;
      delay(10);
    }
    accel_offset_x = sx / 20.0;
    accel_offset_y = sy / 20.0;
    accel_offset_z = sz / 20.0;
    Tare_accel = false;
  }
}

void onAccelXChange()   {}
void onAccelYChange()   {}
void onAccelZChange()   {}
void onTiltXChange()    {}
void onTiltYChange()    {}
void onWeightKgChange() {}