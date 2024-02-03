/*  
 * 
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */


#ifndef SGPIOD_H
#define SGPIOD_H

#include <gpiod.h>
#include "utime.h"


/**
 * Class to help access to GPIO pins on the Raspberry
 *
 * Requires that gpiod and libgpiod-dev are installed
 */
class SGpiod
{
public:
  /** setup and request data */
  void setup();
  /**
   * regular update tick */
  void tick();
  /**
   * terminate */
  void terminate();
  /**
   * Read one of the available pins
   * \param pin is one of 13 (start), 6 (stop)
   * \returns pin value, ot -1 if pin can not be reserved */
  int readPin(const int pin);
  /**
  * to listen to pins */
  void run();

protected:
  int getPinIndex(int pinNumber);

private:
  // base
  // use one pin only, the START switch.
  static const int MAX_PINS = 1;
  const char *chipname = "gpiochip0";
  std::string start_script_name = "/home/local/mission_start.bash";
//   std::string stop_script_name = "/home/local/mission_stop.bash";
  struct gpiod_chip *chip;
  struct gpiod_line *pins[MAX_PINS];
  //
  int pinNumber[MAX_PINS] = {13 /*,6*/};
  int in_pin_value[MAX_PINS] = {-1};
  bool isOK = false;
  // logfile
  FILE * logfile = nullptr;
  float pinReadTook = 0;

private:
  static void runObj(SGpiod * obj)
  { // called, when thread is started
      // transfer to the class run() function.
      obj->run();
  }
  std::thread * th1;
};

/**
 * Make this visible to the rest of the software */
extern SGpiod gpio;

#endif
