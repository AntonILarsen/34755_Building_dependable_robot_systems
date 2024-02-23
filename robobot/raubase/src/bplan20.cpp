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
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"

#include "bplan20.h"

// create class object
BPlan20 plan20;

void BPlan20::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan20"].has("log"))
  { // no data yet, so generate some default values
    ini["plan20"]["log"] = "true";
    ini["plan20"]["run"] = "false";
    ini["plan20"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan20"]["print"] == "true";
  //
  if (ini["plan20"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan20.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan20 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan20::~BPlan20()
{
  terminate();
}

void BPlan20::run()
{
  if (not setupDone)
    setup();
  if (ini["plan20"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  int passedgates = 0;
  bool OnTrack1 = true;
  //
  toLog("Plan20 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission
    case 10:
      pose.resetPose();
      mixer.setEdgeMode(rightEdge, 0); // 0 offset.
      mixer.setVelocity(0.5);
      tolog("Distance_Sensor_Go: %d", dist[0]);

      if (dist[0] < 900000)
      {
        bool nogate = true;
      }
      if (dist[0] > 900000 &nogate = true) // gate detected (guess of sensor value at gate.)
      {
        passedgates += 1;
        nogate = false;
        if passedgates
          = 1
          {
            int firstgatetime = t.getTimePassed();
          }
      }
      if (t.getTimePassed() >= firstgatetime + 4)
      { // få styr på tidsenheden.
        if (dist[0] > 90000)
        {
          OnTrack = true;
        }

        while (t.getTimePassed < firstgatetime + 6)
        {
          mixer.setVelocity(0);
          tolog("distance sensor: %d", dist[0])
        }
        if (dist[0] > 90000)
        {
          OnTrack = true;
          mixer.setVelocity(0.5);
        }
        else
          On Track = false;
        state 12;
      }

      if (passedgates = 3)
      {
        state = 11;
      }

      break;
    case 11: // wait for distance
      if (pose.dist >= 1.0)
      { // done, and then
        finished = true;
      }
      else if (t.getTimePassed() > 10)
        lost = true;
      break;
    default:
      toLog("Unknown state");
      lost = true;
      break;
    case 12: //stepbro drives backwards for 6 secodns, assuming it can follow the line while driving backwards.
      mixer.setVelocity(-0.5) 
          int thirdgatepassed = t.getTimePassed();
      if (t.getTimePassed < thirdgatepassed + 6)
      {
          finished = true;
      }
    }
    if (state != oldstate)
    {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan20 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan20 finished");
}

void BPlan20::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan20::toLog(const char *message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
           oldstate,
           message);
  }
}
