/*
  把Madgwick filter的函式自己拉出來寫看看
 */

#include <MadgwickAHRS.h>
#include <Arduino_LSM9DS1.h>


Madgwick filter;
unsigned long microsPerReading, microsPrevious;
int sample_rate = 100; // 100Hz取樣率

float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements
float m_x, m_y, m_z; // magnetometer measurements
float roll_IMU, pitch_IMU, yaw_IMU;


// 以下為自行改寫的Madgwick filter變數
#define betaDef         0.1f            // 2 * proportional gain
float beta = betaDef;
float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
float invSampleFreq = 1.0 / sample_rate;
float anglesComputed = 0;

float roll_HSJ; //自己拉出library計算的ROLL


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
 

  // Madgwick單位為: 加速度(g), 角速度(degree/sec), 磁力(MicroTesla)
  IMU.accelUnit =  GRAVITY;  //  GRAVITY   OR  METERPERSECOND2 
  IMU.gyroUnit  = DEGREEPERSECOND;  //   DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND  


  filter.begin(sample_rate); // 初始化Madgwick


  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / sample_rate;
  microsPrevious = micros();
}

void loop() {  
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
    filter.updateIMU(-w_x, w_y, w_z, -a_x, a_y, a_z);

    // print the heading, pitch and roll
    roll_IMU = filter.getRoll();
    Serial.print("Roll_IMU:");
    Serial.print(roll_IMU);
    Serial.print(",");
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
    
    // 以下為自行改寫之Madgwick filter
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    float gx = -w_x;  //右手定則
    float gy = w_y;
    float gz = w_z;

    float ax = -a_x;  //右手定則
    float ay = a_y;
    float az = a_z;    
    
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
    
      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;
    
      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
    
      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    roll_HSJ = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578;

    Serial.print("Roll_HSJ:");
    Serial.println(roll_HSJ);



    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}



float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}
