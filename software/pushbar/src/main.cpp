#include <Arduino.h>
#include "timer0.h"
#include "imu.h"
#include <math.h>
#include "my_ekf.h"
#include "servo_ctrl.h"
#include <MPU9250_asukiaaa.h>
#include <ESP32Encoder.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
#define T_ISR 0.01         // Timer period
#define PRE_SCALE 80       // Prescaler
#define APB_CLK_F 80e6     // APB (advanced peripheral bus) clock frequency
#define BAUD_RATE 115200
#define PWM_MAX 1024
#define PWM_FREQ 70000
#define LOW_PWM 158
#define HIGH_PWM 181
#define CNT_ROT 1000
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
uint8_t enA = 14, in1 = 27, in2 = 26; // enA -> pwm_pin = 4; dir_pin = 16,
uint8_t encoder1 = 32, encoder2 = 33;
ESP32Encoder encoder;
uint8_t stopped = 0;
// 11.3
void update();
void setup()
{
  Serial.begin(115200);
  setup_imu();
  gz_offset = cal_gyr_offset(ZAXIS);
  //Serial.println(gz_offset);
  uint64_t ticks = setup_timer(PRE_SCALE, T_ISR, APB_CLK_F);
  t1 = micros();
  //pinMode(pwm_pin, OUTPUT);
  //pinMode(dir_pin, OUTPUT);
  //pinMode(in2_pin, OUTPUT);
  //ledcSetup(0,PWM_FREQ,8);
  pinMode(enA, OUTPUT);// ledcSetup(0,PWM_FREQ,8); // B4
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //ledcAttachPin(in1_pin,0);
  //ledcAttachPin(pwm_pin,0); // B4
  //ledcWrite(0,0);
  //digitalWrite(dir_pin,0);
  analogWrite(enA, 0); // ledcWrite(0,LOW_PWM); // B4
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(encoder1, encoder2); // Encoder pins
  encoder.clearCount();
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
  static int32_t set_cnt = 0;
  static float t = 0;
  //set_cnt = 2*CNT_ROT*sin(2*PI*0.2*t);
  t+=T_ISR;
  static uint8_t down = 0;
  /*
  if(set_cnt >= 2*CNT_ROT){
    down = 1;
  }
  if(set_cnt <= 0){
    down = 0;
  }
  if(down){
    set_cnt-=(2*CNT_ROT)/200;
  }else{
    set_cnt+=(2*CNT_ROT)/200;
  }
  */
  
  if(t >= 10){
    t = 0;
    set_cnt= (set_cnt == 0) ? 1.5*  4*800 : 0; //2*CNT_ROT : 0;
  }

  
  int32_t cnt_n = (int32_t)encoder.getCount();
  static int32_t cnt_p = cnt_n;
  int32_t e = set_cnt-(cnt_n);
  //float u = (float)(e)*2.5;
  //float u = 0.35*e;
  //float u = pictrl(e,0.45*0.35,0.85*0.08,T_ISR);
  float u = pictrl(e,0.1*0.08,4*0.15,T_ISR);
  Serial.print(e); Serial.print("\t"); Serial.print(cnt_n); Serial.print("\t"); Serial.print(set_cnt); Serial.print("\t"); Serial.println(u);
  if(u < 0){
    //digitalWrite(dir_pin,1);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }else{
    //digitalWrite(dir_pin,0);
    	digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
  }
  u = abs(u)*255.0/12;
  if(u > 255){
    u = 255;
  }
  analogWrite(enA, (int) u);//ledcWrite(0,(int) (u));
  
  
  isr_cnt++;
  /*
  if(isr_cnt >= 300){
    if(set_cnt > 0){
      set_cnt = 0;
    }else{
      set_cnt = CNT_ROT;
    }
    isr_cnt
    */
  //For TF estimation
  /*
  if(isr_cnt >  150 && !stopped){
    ledcWrite(0,HIGH_PWM);
    stopped=1;
  }
  if(!stopped){
    Serial.print(LOW_PWM); Serial.print("\t"); Serial.println(cnt_n-cnt_p);
  }else{
    Serial.print(HIGH_PWM); Serial.print("\t"); Serial.println(cnt_n-cnt_p);
  }
  */
  cnt_p = cnt_n;



  

  /*
  isr_cnt++;
  mag mg = get_mag();
  gyr gr = get_gyr();
  float dir = kalman(mg.mx, mg.my, mg.mz, gr.gz-gz_offset);
  if(isr_cnt == 5){
    Serial.println(dir*180/PI);
    isr_cnt = 0;
  }
  */
}

