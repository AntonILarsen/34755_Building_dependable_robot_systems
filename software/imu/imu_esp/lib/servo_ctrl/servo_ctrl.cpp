#include "servo_ctrl.h"
float pictrl(float e, float Kp, float tau, float Ts){
  float b1 = Ts/(2*tau)+1;
  float b0 = Ts/(2*tau)-1;
  float a1 = 1;
  float a0 = -1;
  static float x = e, xp = e, y = 0, yp = 0;
  x = e;
  y = 1/a1*(Kp*(b1*x+b0*xp)-a0*yp);
  if(y >= 12){
    y = 12;
  }else if(y <= -12){
    y = -12;
  }
  //0.21296791443850267379679144385027*x - 0.23703208556149732620320855614973*xp + yp;
  yp = y;
  xp = x;
  return y;
}

float lpfilt(float u, float tau, float Ts){
  float b1 = Ts;
  float b0 = Ts;
  float a1 = Ts+2*tau;
  float a0 = Ts-2*tau;
  static float x = u, xp = u, y = 0, yp = 0;
  x = u;
  y = 1/a1*(b1*x+b0*xp-a0*yp);
  //0.21296791443850267379679144385027*x - 0.23703208556149732620320855614973*xp + yp;
  yp = y;
  xp = x;
  return y;
}