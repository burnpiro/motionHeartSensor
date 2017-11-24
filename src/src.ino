#include "helpers.h"


void setup() {
  Wire.begin();
  Wire.pins(D1,D2);
  setupSensor();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
  yield();
}

void loop() {
    static unsigned long l = 0;
    unsigned long t;

    t = millis();

    if((t - l) > 100) {
        getMotionSensorData();
        yield();
        getPulseSensorData();
        yield();
        Serial.println(sensorData + " " + Pulse);
        l = t;
        yield();
    }
}
