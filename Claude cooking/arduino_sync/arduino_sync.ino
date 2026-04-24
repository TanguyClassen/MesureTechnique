#include "arduino_secrets.h"
#include "thingProperties.h"
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define HX711_DOUT   D2
#define HX711_SCK    D3
#define LED_SYNC_PIN D13    // Connect LED + 220Ω resistor to GND

// ── Calibration ───────────────────────────────────────────────────────────────
#define CALIBRATION_FACTOR  -7050.0
#define TARE_ON_BOOT        true

// ── Rates ─────────────────────────────────────────────────────────────────────
#define SERIAL_INTERVAL_MS  50      // 20 Hz serial stream
#define CLOUD_INTERVAL_MS   1000    // 1 Hz cloud sync
#define LED_FLASH_MS        400     // LED on duration for sync flash

// ── Circular buffer ───────────────────────────────────────────────────────────
#define BUFFER_LINES 30
char     lineBuffer[BUFFER_LINES][110];
uint8_t  bufHead  = 0;
uint8_t  bufCount = 0;

// ── ANSI ──────────────────────────────────────────────────────────────────────
#define ANSI_HOME  "\033[H"
#define ANSI_CLEAR "\033[2J"
#define ANSI_BOLD  "\033[1m"
#define ANSI_RST   "\033[0m"

// ── Accel offsets ─────────────────────────────────────────────────────────────
float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;

// ── Objects ───────────────────────────────────────────────────────────────────
HX711            scale;
Adafruit_MMA8451 mma;

unsigned long lastSerial    = 0;
unsigned long lastCloud     = 0;
unsigned long ledOffTime    = 0;
bool          ledActive     = false;

// ─────────────────────────────────────────────────────────────────────────────
void triggerSync() {
  unsigned long ts = millis();
  // 1. Flash LED
  digitalWrite(LED_SYNC_PIN, HIGH);
  ledActive  = true;
  ledOffTime = ts + LED_FLASH_MS;
  // 2. Mark in serial stream — Python will detect this line
  Serial.print("S,");
  Serial.println(ts);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_SYNC_PIN, OUTPUT);
  digitalWrite(LED_SYNC_PIN, LOW);

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  if (TARE_ON_BOOT) {
    scale.tare(10);
  }

  if (!mma.begin()) {
    Serial.println("ERROR: MMA8451 not found.");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);

  Serial.print(ANSI_CLEAR);
  Serial.print(ANSI_HOME);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  ArduinoCloud.update();

  unsigned long now = millis();

  // ── LED timer ──
  if (ledActive && now >= ledOffTime) {
    digitalWrite(LED_SYNC_PIN, LOW);
    ledActive = false;
  }

  // ── Serial command from Python ('SYNC\n') ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "SYNC") triggerSync();
  }

  // ── Sensor read + serial ──
  if (now - lastSerial >= SERIAL_INTERVAL_MS) {
    lastSerial = now;
    readLoadCell();
    readAccelerometer();
    pushLine(now);
    redrawScreen();
    // Also emit raw data line for Python capture
    Serial.print("D,");
    Serial.print(now);       Serial.print(",");
    Serial.print(weight_kg, 4); Serial.print(",");
    Serial.print(accel_x, 4);  Serial.print(",");
    Serial.print(accel_y, 4);  Serial.print(",");
    Serial.print(accel_z, 4);  Serial.print(",");
    Serial.print(tilt_x, 3);   Serial.print(",");
    Serial.println(tilt_y, 3);
  }

  // ── Cloud sync ──
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

void readAccelerometer() {
  sensors_event_t event;
  mma.getEvent(&event);
  accel_x = event.acceleration.x - accel_offset_x;
  accel_y = event.acceleration.y - accel_offset_y;
  accel_z = event.acceleration.z - accel_offset_z;
  tilt_x  = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * 180.0 / M_PI;
  tilt_y  = atan2(-accel_x, accel_z) * 180.0 / M_PI;
}

void pushLine(unsigned long now) {
  float norm = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  snprintf(lineBuffer[bufHead], sizeof(lineBuffer[0]),
    "  %9.3f  | %9.3f | %9.3f | %9.3f |  %9.3f  | %8.2f | %7.2f",
    weight_kg, accel_x, accel_y, accel_z, norm, tilt_x, tilt_y);
  bufHead = (bufHead + 1) % BUFFER_LINES;
  if (bufCount < BUFFER_LINES) bufCount++;
}

void redrawScreen() {
  Serial.print(ANSI_HOME);
  Serial.print(ANSI_BOLD);
  Serial.println(F("  Weight(kg) |  Accel X  |  Accel Y  |  Accel Z  |  Norm(m/s2) | Pitch(deg) | Roll(deg)"));
  Serial.print(ANSI_RST);
  Serial.println(F(" -----------+-----------+-----------+-----------+-------------+------------+----------"));
  uint8_t start = (bufCount < BUFFER_LINES) ? 0 : bufHead;
  for (uint8_t i = 0; i < bufCount; i++) {
    Serial.println(lineBuffer[(start + i) % BUFFER_LINES]);
  }
  for (uint8_t i = bufCount; i < BUFFER_LINES; i++) Serial.println();
}

// ── Cloud callbacks ───────────────────────────────────────────────────────────
void onTareWeightChange() {
  if (Tare_weight) { scale.tare(10); Tare_weight = false; }
}

void onTareAccelChange() {
  if (Tare_accel) {
    float sx=0, sy=0, sz=0;
    for (int i=0; i<20; i++) {
      sensors_event_t e; mma.getEvent(&e);
      sx += e.acceleration.x; sy += e.acceleration.y; sz += e.acceleration.z;
      delay(10);
    }
    accel_offset_x = sx/20.0; accel_offset_y = sy/20.0; accel_offset_z = sz/20.0;
    Tare_accel = false;
  }
}

void onAccelXChange()   {}
void onAccelYChange()   {}
void onAccelZChange()   {}
void onTiltXChange()    {}
void onTiltYChange()    {}
void onWeightKgChange() {}
