/*
取得Nano 33 BLE Sense的IMU資料
這邊我用的Arduino_LSM9DS1版本是FemmeVerbeek的2.0版本
似乎有個同名的Arduino_LSM9DS1，但是不一樣，需要注意
這裡只單純紀錄如何取得ACC、GYRO的資料，詳細設定請參考https://github.com/FemmeVerbeek/Arduino_LSM9DS1
setAccelFS(.): 0=±2g、1=±4g、2=±8g、3=±16g  (Github上文件似乎有誤，Acc的scale要確認)
setAccelODR(.):0=off、1=10Hz、2=50Hz、3=119Hz、4=238Hz、5=476Hz (Gyro和Acc共用)
*/ 

#include <Arduino_LSM9DS1.h>

float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements
float m_x, m_y, m_z; // magnetometer measurements

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
   // Accelerometer code
   IMU.setAccelFS(2);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(0.000535, -0.013943, -0.025533);
   IMU.setAccelSlope (1.000535, 0.986057, 0.998086);
   
   // Gyroscope code
   IMU.setGyroFS(2);
   //IMU.setGyroODR(5);
   IMU.setGyroOffset (0.784119, -0.122986, 0.644989);
   IMU.setGyroSlope (1.149049, 1.159496, 1.128838);

   // Magnetometer
   IMU.setMagnetFS(0);  
   IMU.setMagnetODR(8); 
   IMU.setMagnetOffset(0,0,0);  //  uncalibrated
   IMU.setMagnetSlope (1,1,1);  //  uncalibrated

   // Madgwick Filter需要的輸入單位為: 加速度(g), 角速度(degree/sec), 磁力(MicroTesla)
   IMU.accelUnit =  GRAVITY;  //  GRAVITY   OR  METERPERSECOND2 
   IMU.gyroUnit  = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  
   IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA
}

void loop() {

  IMU.readAccel(a_x, a_y, a_z);  // alias IMU.readAcceleration  in library version 1.01
  IMU.readGyro(w_x, w_y, w_z);   // alias IMU.readGyroscope
  IMU.readMagnet(m_x, m_y, m_z);

  Serial.println(a_x);
}
