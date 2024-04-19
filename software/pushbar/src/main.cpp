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
#define T_MOT 0.010        // Motor sampling period
#define T_CPF 0.001        // Complementary filter period 
#define PRE_SCALE 80       // Prescaler
#define APB_CLK_F 80e6     // APB (advanced peripheral bus) clock frequency
#define BAUD_RATE 115200
#define PWM_MAX 1024
#define PWM_FREQ 20000
#define LOW_PWM 158
#define HIGH_PWM 181
#define CNT_ROT 1000
#define LINE_FEED 0x0A
#define RQST_ANGLE 0x01
#define WRTE_ANGLE 0x02
#define RQST_ANGLE_ACK 0x03
#define WRTE_ANGLE_ACK 0x04
#define READ_PITCH 0x05
#define READ_ROLL 0x06
#define READ_PITCH_ACK 0x07
#define READ_ROLL_ACK 0x08
#define TX_ERR 0x19
#define KP 30
#define TAUI 0.05
#define TAUD 0.07
#define ALPHA 0.15
#define LOCAL_REF 0
#define TAU_CPF 0.5
#define DEBUG 0
uint8_t isr_flag = 0;
uint8_t enA = 14, in1 = 27, in2 = 26; 
uint8_t encoder1 = 32, encoder2 = 33;
uint8_t vcc_imu = 16, gnd_imu = 17;
float theta_r = 0.0;   // Angle reference
float theta = 0.0;     // Measured angle
uint8_t cmd_available = 0;
uint8_t bfr[6];
uint8_t bfr_idx = 0;
ESP32Encoder encoder;
float pitch = 0, roll = 0;
gyr gr;
acc ac;
uint8_t cntms = 0;
void update_motor();
void update_cpf();
void setup()
{
  Serial.begin(115200);
  uint64_t ticks = setup_timer(PRE_SCALE, T_CPF, APB_CLK_F);
  ledcSetup(0,PWM_FREQ,8); 
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(vcc_imu, OUTPUT);
  pinMode(gnd_imu, OUTPUT);
  digitalWrite(vcc_imu, HIGH);
  digitalWrite(gnd_imu, LOW);
  setup_imu();
  cal_gyr_offset(&gr);
  //Serial.print("Got offsets: gxo = "); Serial.print(gr.gxo); Serial.print(", gyo = "); Serial.print(gr.gyo); Serial.print(", gzo = "); Serial.println(gr.gzo);
  delay(2000);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, 0);
  ledcAttachPin(enA,0);
  ledcWrite(0,127);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(encoder1, encoder2);
  encoder.clearCount();
}
void loop()
{ 
  if(isr_flag){
    cntms++;
    isr_flag = 0;
    update_cpf();
    if(cntms == 10){
      cntms = 0;
      update_motor();
    }
  }
  if(cmd_available){
    cmd_available = 0;
    uint8_t buf[6];
    for(int i = 0; i < 6; i++){ // Make a copy of bfr in case new command comes quickly
      buf[i] = bfr[i];
    }
    uint8_t cmd = buf[0];
    
    if(cmd == RQST_ANGLE){
      if(buf[1] != LINE_FEED){ // Missing line feed
        Serial.write(TX_ERR);
        Serial.write(LINE_FEED);
      }else{
        uint8_t *p = (uint8_t*)&theta;
        Serial.write(RQST_ANGLE_ACK);
        Serial.write(p[0]);
        Serial.write(p[1]);
        Serial.write(p[2]);
        Serial.write(p[3]);
        Serial.write(LINE_FEED);
      }
    }else if(cmd == WRTE_ANGLE){
      if(buf[5] != LINE_FEED){ // Missing line feed
        Serial.write(TX_ERR);
        Serial.write(LINE_FEED);
      }else{
        theta_r = *((float *)(bfr+1));
        Serial.write(WRTE_ANGLE_ACK);
        Serial.write(LINE_FEED);
      }
    }else if(cmd == READ_PITCH){
      if(buf[1] != LINE_FEED){ // Missing line feed
        Serial.write(TX_ERR);
        Serial.write(LINE_FEED);
      }else{
        uint8_t *p = (uint8_t*)&pitch;
        Serial.write(READ_PITCH_ACK);
        Serial.write(p[0]);
        Serial.write(p[1]);
        Serial.write(p[2]);
        Serial.write(p[3]);
        Serial.write(LINE_FEED);
      }
    }else if(cmd == READ_ROLL){
      if(buf[1] != LINE_FEED){ // Missing line feed
        Serial.write(TX_ERR);
        Serial.write(LINE_FEED);
      }else{
        uint8_t *p = (uint8_t*)&roll;
        Serial.write(READ_ROLL_ACK);
        Serial.write(p[0]);
        Serial.write(p[1]);
        Serial.write(p[2]);
        Serial.write(p[3]);
        Serial.write(LINE_FEED);
      }
    }else{ // Unrecognized command
      Serial.write(TX_ERR);
      Serial.write(LINE_FEED);
    }
  }
}
void update_cpf(){
  static int cnt_cpf = 0;
  const float b0 = T_CPF, b1 = T_CPF;
  const float a0 = T_CPF+2*TAU_CPF, a1 = T_CPF-2*TAU_CPF;
  get_gyr(&gr);
  get_acc(&ac);
  float acc_angle_roll, acc_angle_pitch;
  #if USE_ATAN2 == 1
    acc_angle_pitch = 180/M_PI*atan2(ac.ax,sqrt(ac.ay*ac.ay+ac.az*ac.az));
    acc_angle_roll = 180/M_PI*atan2(ac.ay,sqrt(ac.ax*ac.ax+ac.az*ac.az));
  #else
    acc_angle_pitch = 180/M_PI*atan(ac.ax/sqrt(ac.ay*ac.ay+ac.az*ac.az));
    acc_angle_roll = 180/M_PI*atan(ac.ay/sqrt(ac.ax*ac.ax+ac.az*ac.az));
  #endif
  static float roll_in_p = 0, pitch_in_p = 0;

  pitch = 1/a0*(b0*(acc_angle_pitch + TAU_CPF*gr.gy) + b1*pitch_in_p - a1*pitch);
  roll =  1/a0*(b0*(acc_angle_roll  + TAU_CPF*gr.gx) + b1*roll_in_p  - a1*roll);
  
  pitch_in_p = acc_angle_pitch + TAU_CPF*gr.gy;
  roll_in_p = acc_angle_roll  + TAU_CPF*gr.gx;
  #if DEBUG == 1
    cnt_cpf++;
    if(cnt_cpf == 10){
      cnt_cpf = 0;
      Serial.print("Pitch (cpf) = "); Serial.print(pitch); Serial.print("\t Roll (cpf) = "); Serial.println(roll);
    }
  #endif
}
void update_motor(){
  if(LOCAL_REF){
    static float t = 0;
    if(t >= 4){
      t = 0;
      theta_r = (theta_r > M_PI) ? 0 : 2*M_PI;
    }
    t = t+=T_MOT;
  }
  theta = ((int32_t)encoder.getCount())/(4.0*800.0)*2*M_PI; // Convert counts to angle
  float e = theta_r - theta;
  float u = pidctrl(e,KP,TAUI,TAUD,ALPHA,T_MOT);
  if(u < 0){ 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  u = abs(u)*255.0/12; // Convert voltage to PWM 
  if(u > 255){
    u = 255;
  }
  ledcWrite(0,(int) (u));
}
void serialEvent() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    bfr[bfr_idx] = b;
    bfr_idx++;
    if(b == LINE_FEED){
      cmd_available = 1;
      bfr_idx = 0;
    }
  }
}


