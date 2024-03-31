#include "imu.h"
MPU9250_asukiaaa imu;

void setup_imu() {
#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  imu.setWire(&Wire);
#endif
  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();
}

void get_acc(acc* ac){
  imu.accelUpdate();
  ac->ax = imu.accelX();
  ac->ay = imu.accelY();
  ac->az = imu.accelZ();
}
void get_gyr(gyr* gr){
  imu.gyroUpdate();
  gr->gx = imu.gyroX();
  gr->gy = imu.gyroY();
  gr->gz = imu.gyroZ();
}
mag get_mag(){
  mag mg;
  imu.magUpdate();
  mg.mx = imu.magX();
  mg.my = imu.magY();
  mg.mz = imu.magZ();
  return mg;
}
void cal_gyr_offset(gyr* gr){
  const int N = 300;
  float data[3][N];
  for(int i = 0; i < N; i++){
    get_gyr(gr);
    data[0][i] = gr->gx;
    data[1][i] = gr->gy;
    data[2][i] = gr->gz;
    delay(10);
  }
  float offset[3] = {0,0,0};
  for(int i = 0; i < N; i++){
    offset[0]+= data[0][i];
    offset[1]+= data[1][i];
    offset[2]+= data[2][i];
  }
  gr->gxo = offset[0]/N;
  gr->gyo = offset[1]/N;
  gr->gzo = offset[2]/N;
}
void sample_mag(){
  mag mg;
  for(int i = 0; i < 1000; i++){
    mg = get_mag();
    Serial.print(mg.mx); Serial.print("\t"); Serial.print(mg.my); Serial.print("\t"); Serial.println(mg.mz);
    delay(15);
  }
}