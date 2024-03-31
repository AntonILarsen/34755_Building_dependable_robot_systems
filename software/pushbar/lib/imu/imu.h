#include "Arduino.h"
#include <MPU9250_asukiaaa.h>
#include <math.h>
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#ifdef _ESP32_HAL_I2C_H_
  #define SDA_PIN 21
  #define SCL_PIN 22
#endif
#define USE_ATAN2 1
typedef struct mag{
  float mx, my, mz;
} mag; 
typedef struct gyr{
  float gx, gy, gz;
  float gxo, gyo, gzo;
} gyr;
typedef struct acc{
  float ax, ay, az;
  float axo, ayo, azo;
} acc;
void setup_imu();
void get_acc(acc* ac);
void get_gyr(gyr* gr);
mag get_mag();

void cal_gyr_offset(gyr* gr);
void sample_mag();