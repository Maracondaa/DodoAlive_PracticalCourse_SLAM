#include "Arduino.h"
#include "Arduino_BHY2.h"

SensorQuaternion rotation(SENSOR_ID_RV);
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);

const int OUTPUT_RATE = 50; // Hz

void setup() {
  Serial.begin(460800);
  while (!Serial);

  BHY2.begin();

  // start sensors at same rate
  rotation.begin(OUTPUT_RATE);
  accel.begin(OUTPUT_RATE);
  gyro.begin(OUTPUT_RATE);
}

void loop() {

  BHY2.update();

  // only publish when new data exists
  if (rotation.dataAvailable() && accel.dataAvailable() && gyro.dataAvailable()) {

    float qx = rotation.x();
    float qy = rotation.y();
    float qz = rotation.z();
    float qw = rotation.w();

    float ax = accel.x();
    float ay = accel.y();
    float az = accel.z();

    float gx = gyro.x();
    float gy = gyro.y();
    float gz = gyro.z();

    // reject clearly invalid packets
    if (qx == 0 && qy == 0 && qz == 0 && qw == 0) return;
    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) return;

    // format: qx,qy,qz,qw,ax,ay,az,gx,gy,gz
    Serial.print(qx); Serial.print(",");
    Serial.print(qy); Serial.print(",");
    Serial.print(qz); Serial.print(",");
    Serial.print(qw); Serial.print(",");

    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");

    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
}

