#include "Arduino.h"
#include <MPU9250_asukiaaa.h>
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#ifdef _ESP32_HAL_I2C_H_
  #define SDA_PIN 21
  #define SCL_PIN 22
#endif
typedef struct mag{
  float mx, my, mz;
} mag; 
typedef struct gyr{
  float gx, gy, gz;
} gyr;
void setup_imu();
uint8_t get_acc();
gyr get_gyr();
mag get_mag();

float cal_gyr_offset(uint8_t axis);
void sample_mag();