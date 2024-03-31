#include "servo_ctrl.h"
#include <Arduino.h>
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
float pidctrl(float e, float Kp, float taui, float taud, float al, float Ts){
  const float b0 = 0.5*Kp*(Ts + 2.0*taud)*(Ts + 2.0*taui);
  const float b1 = 0.5*Kp*(Ts - 2.0*taud)*(Ts + 2.0*taui) + 0.5*Kp*(Ts + 2.0*taud)*(Ts - 2.0*taui);
  const float b2 = 0.5*Kp*(Ts - 2.0*taud)*(Ts - 2.0*taui);
  const float a0 = taui*(Ts + 2.0*al*taud);
  const float a1 = taui*(Ts - 2.0*al*taud) - 1.0*taui*(Ts + 2.0*al*taud);
  const float a2 = -1.0*taui*(Ts - 2.0*al*taud);
  static float x = e, xp = 0, xpp = 0, y = 0, yp = 0, ypp = 0;
  x = e;
  y = 1/a0*(b0*x + b1*xp + b2*xpp - a1*yp - a2*ypp);
  if(y >= 12){
    y = 12;
  }else if(y <= -12){
    y = -12;
  }
  ypp = yp;
  yp = y;
  xpp = xp;
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