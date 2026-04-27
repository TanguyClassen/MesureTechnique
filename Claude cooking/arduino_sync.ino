/*
  ME-301 Measurement Lab — Pure Serial Sketch
  NO Arduino Cloud — runs standalone over USB serial

  Data streams:
    W,<ms>,<weight_kg>                       up to 80 Hz  (needs RATE pin HIGH on HX711)
    A,<ms>,<ax>,<ay>,<az>,<pitch>,<roll>     up to 200 Hz
    S,<ms>                                   sync LED flash

  Serial commands (send from Python):
    SYNC      → flash LED, emit S,<ms>
    TARE_W    → tare load cell
    TARE_A    → tare accelerometer

  ── HX711 RATE pin ────────────────────────────────────────────────────────────
  Connect HX711 RATE pin to:
    GND  →  10 SPS  (slow, stable)
    VCC  →  80 SPS  (fast, recommended) ← do this for best results
  ─────────────────────────────────────────────────────────────────────────────
*/

#include <HX711.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define HX711_DOUT   D2
#define HX711_SCK    D3
#define LED_SYNC_PIN D4    // LED + 220Ω to GND

// ── Sample intervals ──────────────────────────────────────────────────────────
// At 80 SPS the HX711 produces a new sample every 12.5 ms
// Going below 13 ms won't give new data — it'll just repeat the last reading
#define WEIGHT_INTERVAL_MS   13     // ~76 Hz  (set RATE pin HIGH for this to work)
#define ACCEL_INTERVAL_MS     5     // 200 Hz  (MMA8451 ODR set to 400 Hz below)
#define SERIAL_STATUS_MS   2000     // print sample rates every 2 s
#define LED_FLASH_MS        300

// ── Calibration ───────────────────────────────────────────────────────────────
#define CALIBRATION_FACTOR  -7050.0
#define TARE_ON_BOOT        true

// ── Accel tare offsets ────────────────────────────────────────────────────────
float off_x = 0, off_y = 0, off_z = 0;

// ── Objects ───────────────────────────────────────────────────────────────────
HX711            scale;
Adafruit_MMA8451 mma;

// ── Timers ────────────────────────────────────────────────────────────────────
unsigned long t_weight = 0;
unsigned long t_accel  = 0;
unsigned long t_status = 0;
unsigned long t_led    = 0;
bool          led_on   = false;

// ── Rate counters ─────────────────────────────────────────────────────────────
uint32_t cnt_w = 0, cnt_a = 0;
unsigned long t_rate_w = 0, t_rate_a = 0;

// ─────────────────────────────────────────────────────────────────────────────
void triggerSync() {
  unsigned long ts = millis();
  digitalWrite(LED_SYNC_PIN, HIGH);
  led_on = true;
  t_led  = ts + LED_FLASH_MS;
  Serial.print("S,"); Serial.println(ts);
}

void tare_weight() {
  scale.tare(10);
  Serial.println("INFO,tare_weight_done");
}

void tare_accel() {
  float sx=0, sy=0, sz=0;
  for (int i=0; i<30; i++) {
    sensors_event_t e; mma.getEvent(&e);
    sx += e.acceleration.x;
    sy += e.acceleration.y;
    sz += e.acceleration.z;
    delay(5);
  }
  off_x = sx/30.0; off_y = sy/30.0; off_z = sz/30.0;
  Serial.println("INFO,tare_accel_done");
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(460800);   // Higher baud = more headroom for fast streaming
  delay(800);

  pinMode(LED_SYNC_PIN, OUTPUT);
  digitalWrite(LED_SYNC_PIN, LOW);

  // ── HX711 ──
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CALIBRATION_FACTOR);
  if (TARE_ON_BOOT) {
    Serial.println("INFO,taring...");
    scale.tare(10);
    Serial.println("INFO,tare_done");
  }

  // ── MMA8451 — set to 400 Hz ODR ──
  if (!mma.begin()) {
    Serial.println("ERROR,MMA8451_not_found");
    while (1);
  }
  mma.setRange(MMA8451_RANGE_2_G);
  // Force highest output data rate: 400 Hz
  mma.setDataRate(MMA8451_DATARATE_400_HZ);
  Serial.println("INFO,ready");

  t_weight = t_accel = t_status = millis();
  t_rate_w = t_rate_a = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── LED off timer ──
  if (led_on && now >= t_led) {
    digitalWrite(LED_SYNC_PIN, LOW);
    led_on = false;
  }

  // ── Serial commands ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if      (cmd == "SYNC")   triggerSync();
    else if (cmd == "TARE_W") tare_weight();
    else if (cmd == "TARE_A") tare_accel();
  }

  // ── Weight stream ──
  if (now - t_weight >= WEIGHT_INTERVAL_MS) {
    t_weight = now;
    if (scale.is_ready()) {
      float kg = scale.get_units(1);
      if (kg > -0.002 && kg < 0.002) kg = 0.0;
      Serial.print("W,"); Serial.print(now);
      Serial.print(","); Serial.println(kg, 4);
      cnt_w++;
    }
  }

  // ── Accel stream ──
  if (now - t_accel >= ACCEL_INTERVAL_MS) {
    t_accel = now;
    sensors_event_t ev;
    mma.getEvent(&ev);
    float ax = ev.acceleration.x - off_x;
    float ay = ev.acceleration.y - off_y;
    float az = ev.acceleration.z - off_z;
    float pitch = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;
    float roll  = atan2(-ax, az)                 * 180.0 / M_PI;
    Serial.print("A,"); Serial.print(now);
    Serial.print(","); Serial.print(ax, 3);
    Serial.print(","); Serial.print(ay, 3);
    Serial.print(","); Serial.print(az, 3);
    Serial.print(","); Serial.print(pitch, 2);
    Serial.print(","); Serial.println(roll, 2);
    cnt_a++;
  }

  // ── Emit actual measured sample rates every 2 s ──
  if (now - t_status >= SERIAL_STATUS_MS) {
    float dt = (now - t_status) / 1000.0;
    float sr_w = cnt_w / dt;
    float sr_a = cnt_a / dt;
    Serial.print("RATE,"); Serial.print(sr_w, 1);
    Serial.print(",");     Serial.println(sr_a, 1);
    cnt_w = 0; cnt_a = 0;
    t_status = now;
  }
}
