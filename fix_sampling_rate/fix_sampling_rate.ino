/*
固定取樣時間練習
*/

unsigned long t0, preval=0;
long sampling_rate = 1000; //sampling rate in Hz
long micro_sec = 1000000/sampling_rate; 
// 1 Hz = 10^6 micro second
// 10Hz = 10^5 micro second



void setup() {
  Serial.begin(115200);
    while (!Serial);
}

void loop() {
  // 如果當下的micro時間減去先前紀錄的時間，就執行if內條件
  if (micros() - preval >= micro_sec) 
  {
    preval += micro_sec; //紀錄下一個停頓點的值
    Serial.println(micros());
   }
}
