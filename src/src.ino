#include "helpers.h"

void setup() {
  Wire.begin();
  Wire.pins(D1,D2);
  setupSensor(MPU9250_ADDRESS);
  setupSensor(MPU9250_ADDRESS2);
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
        sensorData[0] = getMotionSensorData(MPU9250_ADDRESS);
        sensorData[1] = getMotionSensorData(MPU9250_ADDRESS2);
        yield();
        getPulseSensorData();
        yield();
        Serial.println(sensorData[0] + " // " + sensorData[1] + " // " + Pulse);
        l = t;
        yield();
    }
}
