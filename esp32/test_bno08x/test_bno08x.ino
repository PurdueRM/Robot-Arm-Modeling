#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("BNO08x I2C test");

  // Start I2C on ESP32 (SDA=21, SCL=22)
  Wire.begin(21, 22);

  // Initialize IMU over I2C
  if (myIMU.begin() == false)
  {
    Serial.println("IMU not detected. Check wiring.");
    while (1);
  }

  Serial.println("IMU connected!");

  // Enable rotation vector (quaternion)
  myIMU.enableRotationVector(50); // 50 Hz
}

void loop()
{
  if (myIMU.dataAvailable())
  {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float ax = myIMU.getAccelX();
    float ay = myIMU.getAccelY();
    float az = myIMU.getAccelZ();

    Serial.print("Quat: ");
    Serial.print(qw, 4); Serial.print(", ");
    Serial.print(qx, 4); Serial.print(", ");
    Serial.print(qy, 4); Serial.print(", ");
    Serial.print(qz, 4);
    Serial.print(" Accel: ");
    Serial.print(ax, 4); Serial.print(", ");
    Serial.print(ay, 4); Serial.print(", ");
    Serial.println(az, 4);
  }
}