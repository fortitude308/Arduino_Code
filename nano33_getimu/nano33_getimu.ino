/*
取得Nano 33 BLE Sense的IMU資料
這邊我用的Arduino_LSM9DS1版本是FemmeVerbeek的2.0版本
似乎有個同名的Arduino_LSM9DS1，但是不一樣，需要注意
這裡只單純紀錄如何取得ACC、GYRO的資料，詳細設定請參考https://github.com/FemmeVerbeek/Arduino_LSM9DS1

*/

#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
}
