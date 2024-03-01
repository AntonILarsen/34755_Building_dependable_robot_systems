/*  
 * 
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
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

#include <string>
#include <string.h>
#include <unistd.h>
#include "uservice.h"
#include "sgpiod.h"

// https://github.com/brgl/libgpiod/blob/master/bindings/cxx/gpiod.hpp
#include "gpiod.h"

// create value
SGpiod gpio;


// Bridge class:
void SGpiod::setup()
{ // ensure default values
  chip = gpiod_chip_open_by_name(chipname);
  if (chip != nullptr)
  { // get pin handles
    for (int i = 0; i < MAX_PINS; i++)
    { // get handle to relevant pins and set output as specified
      pins[i] = gpiod_chip_get_line(chip, pinNumber[i]);
      if (false)
        // try not to reserve input pin (is also used by others)
        gpiod_line_request_input(pins[i], "ip_disp_in");
    }
  }
  else
  {
    printf("# SGpiod::setup there is no GPIO chip found\n");
  }
  // logfiles
  if (true)
  { // open logfile
    std::string fn = "log_gpio.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% gpio logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tPin %d (start)\n", pinNumber[0]);
//     fprintf(logfile, "%% 3 \tPin %2d (stop)\n", pinNumber[1]);
    if (chip == nullptr)
      fprintf(logfile, "%% No GPIO chip found\n");
  }
  if (not service.stop)
    // start listen to the keyboard
    th1 = new std::thread(runObj, this);
}

void SGpiod::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
    printf("# SGpiod:: logfile closed\n");
  }
  if (chip != nullptr)
  {
    for (int i = 0; i < MAX_PINS; i++)
      gpiod_line_release(pins[i]);
  }
}


int SGpiod::getPinIndex(int pinNumber)
{ //   int pinNumber[MAX_PINS] = {13, 6, 12, 16, 19, 26, 21, 20};
  int result = -1;
  switch (pinNumber)
  {
    case 13: result = 0; break; // start
    case  6: result = 1; break; // not used
    default:
      break;
  }
  return result;
}

int SGpiod::readPin(const int pin)
{
  int idx = getPinIndex(pin);
  int val = -1;
  UTime t("now");
  if (chip != nullptr and service.teensyFileFree)
  {
    if (idx >= 0)
    {
      int err = gpiod_line_request_input(pins[idx], "ip_disp_in");
      if (err == 0)
      {
        val = gpiod_line_get_value(pins[idx]);
        gpiod_line_release(pins[idx]);
        pinReadTook = t.getTimePassed();
      }
      else
        printf("# SGpiod::readPin: pin %d is reserved by others - fine (last OK took %g sec)\n", pin, pinReadTook);
    }
    else
      printf("# SGpiod::readPin: pin %d is not valid, use one of: %d\n", pin,
          pinNumber[0]);
  }
  return val;
}

void SGpiod::run()
{
  int pv[MAX_PINS] = {false};
  bool changed = true;
  UTime t;
  int loop = 0;
  bool firstRead = true;
  while (not service.stop and chip != nullptr)
  {
    loop++;
    changed = false;
    for (int i = 0; i < MAX_PINS; i++)
    { // check all pins on the list
      pv[i] = readPin(pinNumber[i]);
      if (pv[i] >= 0)
      { // valid read
//         printf("# Pin %d read: first=%d, new=%d, old=%d\n",
//                i, firstRead, pv[i], in_pin_value[i]);
        if (firstRead)
          in_pin_value[i] = pv[i];
        else if (pv[i] != in_pin_value[i])
        { // pin has changed value
          in_pin_value[i] = pv[i];
          changed = true;
          if (i == 0 and pv[0] == false)
          { // call start script
            bool fileOK = service.file_exists(start_script_name);
            if (fileOK)
            { // start script exist
              std::string cmd = "bash " + start_script_name;
              printf("# SGpio:: start: %s\n", cmd.c_str());
              std::string o = service.exec(cmd);
            }
          }
  // stop will not work,  as "raubase" has the GPIO pin
  //         if (i == 1 and pv[1] == true)
  //         { // call stop script
  //           bool fileOK = service.file_exists(stop_script_name);
  //           if (fileOK)
  //           {
  //             std::string cmd = "bash " + stop_script_name;
  //             std::string o = service.exec(cmd);
  //             printf("# SGpio:: stop: %s\n", o.c_str());
  //           }
  //         }
        }
      }
      if (firstRead and pv[0] >= 0)
      {
        firstRead = false;
      }
    }
    if (changed and logfile != nullptr)
    {
      t.now();
      fprintf(logfile,"%lu.%04ld %d\n", t.getSec(), t.getMicrosec()/100,
              pv[0]);
    }
    usleep(25000);
  }
  printf("# Gpio::run terminated\n");
}

