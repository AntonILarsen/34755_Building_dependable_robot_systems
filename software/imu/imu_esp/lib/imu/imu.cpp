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

uint8_t get_acc(){
    return (imu.accelUpdate() == 0 ? 1 : 0);
}
gyr get_gyr(){
  gyr gr;
  imu.gyroUpdate();
  gr.gx = imu.gyroX();
  gr.gy = imu.gyroY();
  gr.gz = imu.gyroZ();
  return gr;
}
mag get_mag(){
  mag mg;
  imu.magUpdate();
  mg.mx = imu.magX();
  mg.my = imu.magY();
  mg.mz = imu.magZ();
  return mg;
}
float cal_gyr_offset(uint8_t axis){
  float data[300];
  for(int i = 0; i < 300; i++){
    gyr gr = get_gyr();
    if(axis == XAXIS){
      data[i] = gr.gx;
    }else if(axis == YAXIS){
      data[i] = gr.gy;
    }else if(axis == ZAXIS){
      data[i] = gr.gz;
    }else{
      return -3000;
    }
    delay(10);
  }
  float offset = 0;
  for(int i = 0; i < 300; i++){
    offset+= data[i];
  }
  offset = offset/300;
  return offset;
}
void sample_mag(){
  mag mg;
  for(int i = 0; i < 1000; i++){
    mg = get_mag();
    Serial.print(mg.mx); Serial.print("\t"); Serial.print(mg.my); Serial.print("\t"); Serial.println(mg.mz);
    delay(15);
  }
}

