#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Sampling time
const unsigned long sampleInterval = 20;   // 50 Hz
unsigned long previousMillis = 0;

void setup(void) {
  Serial.begin(9600);
  delay(1000);

  // Try default address
  if (!mma.begin()) {
    Serial.println("Could not find a valid MMA8451 sensor, check wiring!");
    while (1);
  }

  // Set range: 2G, 4G or 8G
  mma.setRange(MMA8451_RANGE_2_G);

  Serial.println("MMA8451 found!");
  Serial.println("time_ms,ax_ms2,ay_ms2,az_ms2,amag_ms2");
}

void loop(void) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= sampleInterval) {
    previousMillis = currentMillis;

    sensors_event_t event;
    mma.getEvent(&event);

    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;

    float amag = sqrt(ax * ax + ay * ay + az * az);

    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(ax, 4);
    Serial.print(",");
    Serial.print(ay, 4);
    Serial.print(",");
    Serial.print(az, 4);
    Serial.print(",");
    Serial.println(amag, 4);
  }
}
