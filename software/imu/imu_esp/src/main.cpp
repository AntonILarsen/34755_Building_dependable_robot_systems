#include <Arduino.h>
#include "timer0.h"
#include "imu.h"
#include <math.h>
#include "my_ekf.h"
#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
#define T_ISR 0.01         // Timer period
#define PRE_SCALE 80       // Prescaler
#define APB_CLK_F 80e6     // APB (advanced peripheral bus) clock frequency
#define BAUD_RATE 115200
uint8_t isr_flag = 0;
uint16_t isr_cnt = 0;
uint8_t isr_done = 0;
uint8_t acc_cnt = 0;
uint8_t gyr_cnt = 0;
uint8_t mag_cnt = 0;
float gz_offset = 0;
#define CNT_MAX 30/T_ISR
//#define CAL_MAG 1
unsigned long t1 = 0;
void update();
void setup()
{
  Serial.begin(115200);
  setup_imu();
  gz_offset = cal_gyr_offset(ZAXIS);
  Serial.println(gz_offset);
  uint64_t ticks = setup_timer(PRE_SCALE, T_ISR, APB_CLK_F);
  t1 = micros();
}
void loop()
{
  if(isr_flag && !isr_done){
    isr_flag = 0;
    #ifndef CAL_MAG
      update();
    #endif
    #ifdef CAL_MAG
      sample_mag();
      isr_done = 1;
    #endif
  }
}
void update(){
  isr_cnt++;
  mag mg = get_mag();
  gyr gr = get_gyr();
  float dir = kalman(mg.mx, mg.my, mg.mz, gr.gz-gz_offset);
  if(isr_cnt == 5){
    Serial.println(dir*180/PI);
    isr_cnt = 0;
  }
}

