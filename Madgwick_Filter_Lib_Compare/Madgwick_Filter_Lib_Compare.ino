/*
  比較MadgwickAHRS寫好的library，測試6DoF與9DoF
 */

#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>


Madgwick filter1, filter2;
unsigned long microsPerReading, microsPrevious;
int sample_rate = 100; // 100Hz取樣率

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin())
  { Serial.println("Failed to initialize IMU!");
   while (1);
  }
  
   // Accelerometer code
   IMU.setAccelFS(0);  //最精細
   IMU.setAccelODR(5); //回傳最快
   IMU.setAccelOffset(0.000535, -0.013943, -0.025533);
   IMU.setAccelSlope (1.000535, 0.986057, 0.998086);
   
   // Gyroscope code
   IMU.setGyroFS(0);
   IMU.setGyroODR(5);
   IMU.setGyroOffset (0.784119, -0.122986, 0.644989);
   IMU.setGyroSlope (1.149049, 1.159496, 1.128838);
   
   // Magnetometer
   IMU.setMagnetFS(0);  //
   IMU.setMagnetODR(8); 
   IMU.setMagnetOffset(0,0,0);  //  uncalibrated
   IMU.setMagnetSlope (1,1,1);  //  uncalibrated
   

  // Madgwick單位為: 加速度(g), 角速度(degree/sec), 磁力(MicroTesla)
  IMU.accelUnit =  GRAVITY;  //  GRAVITY   OR  METERPERSECOND2 
  IMU.gyroUnit  = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  
  IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA
  
  filter1.begin(sample_rate); // 初始化Madgwick
  filter2.begin(sample_rate); // 初始化Madgwick



  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / sample_rate;
  microsPrevious = micros();
}

void loop() {

  float a_x, a_y, a_z; // accelerometer measurements
  float w_x, w_y, w_z; // gyroscope measurements
  float m_x, m_y, m_z; // magnetometer measurements

  float roll_IMU, pitch_IMU, yaw_IMU, roll_AHRS, pitch_AHRS, yaw_AHRS;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    IMU.readAccel(a_x, a_y, a_z);  // alias IMU.readAcceleration  in library version 1.01
    IMU.readGyro(w_x, w_y, w_z);   // alias IMU.readGyroscope
    IMU.readMagnet(m_x, m_y, m_z);

    // update the filter, which computes orientation
    // 負號因為NANO 33 BLE 的 sensor XYZ 非右手定則，加負號滿足
    // Arduino Madgwick library: AHRS -> filter.update, IMU -> filter.updateIMU
    filter1.updateIMU(-w_x, w_y, w_z, -a_x, a_y, a_z); 
    filter2.update(-w_x, w_y, w_z, -a_x, a_y, a_z, m_x, m_y, m_z); 

    // print the heading, pitch and roll
    roll_IMU = filter1.getRoll();
    pitch_IMU = filter1.getPitch();
    yaw_IMU = filter1.getYaw();

    roll_AHRS = filter2.getRoll();
    pitch_AHRS = filter2.getPitch();
    yaw_AHRS = filter2.getYaw();
    
//    Serial.print("Orientation: ");
//    Serial.print(yaw);

    Serial.print("Roll_IMU:");
    Serial.print(roll_IMU);
    Serial.print(",");
    Serial.print("Roll_AHRS:");
    Serial.println(roll_AHRS);

//    Serial.print("Yaw_IMU:");
//    Serial.print(yaw_IMU);
//    Serial.print(",");
//    Serial.print("Yaw_AHRS:");
//    Serial.println(yaw_AHRS);


    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}
