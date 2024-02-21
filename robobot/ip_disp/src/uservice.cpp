/* #***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jcan@dtu.dk
 #*
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */


#include <stdio.h>
#include <signal.h>
#include "CLI/CLI.hpp"
#include <filesystem>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <netpacket/packet.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "uservice.h"
#include "steensy.h"
#include "sgpiod.h"

#define REV "$Id: uservice.cpp 591 2024-01-26 13:39:56Z jcan $"
// define the service class
UService service;
// make a configuration structure

void signal_callback_handler(int signum)
{ // called when pressing ctrl-C
  std::cout << "Caught signal " << signum << std::endl;
  service.terminate();
  exit(signum);
}

bool UService::setup(int argc,char **argv)
{ // Interrupt signal handler
  signal(SIGINT, signal_callback_handler);
  //
  CLI::App cli{"IP_display app"};
  // cli.add_option("-d,--device", service.usbDev, "USB device name for Teensy (default is /dev/ttyACM0)");
  // add reply to version request
  bool version{false};
  cli.add_flag("-v,--version", version, "Latest SVN version (for uservice.cpp)");
  bool api_list{false};
  cli.add_flag("-a,--api-list", api_list, "List available modules");
  cli.allow_windows_style_options();
  CLI11_PARSE(cli, argc, argv);
  // // generate filename with date/time included
  UTime t("now");
  const int MSL = 100;
  char s[MSL];
  t.getForFilename(s);
  std::string fn = "log_ip_disp_";
  fn.append(s);
  fn.append(".txt");
  // open logfile and write legend
  logfile = fopen(fn.c_str(), "w");
  fprintf(logfile, "%% IP display service Teensy\n");
  fprintf(logfile, "%% 1 \tTime (sec) from system\n");
  fprintf(logfile, "%% 2 \tCPU temperature (deg C)\n");
  fprintf(logfile, "%% 3 \t%% (to help import to Matlab)\n");
  fprintf(logfile, "%% 4 \tMessage string (possibly more lines)\n");
  //
  bool theEnd = false;
  //
  if (version)
  {
    printf("RAUBASE SVN service version%s\n", getVersionString().c_str());
    theEnd = true;
  }
  if (api_list)
  {
    printf("Available API modules:\n");
    printf("\tteensy1  Communication withe Teensy\n");
    theEnd = true;
  }
  if (not theEnd)
  { // open the main data source/destination
    teensy1.setup();
    gpio.setup();
    setupComplete = true;
    usleep(2000);
    //
  }
  return not theEnd;
}

bool UService::decode(const char* msg, UTime&)
{ // decode messages from Teensy
  bool used = true;
  if      (false) {}
  //
  else if (strncmp(msg, "dname ", 6) == 0)
  {
    const char * p1 = strrchr(msg, ' ');
    while (*p1 == ' ')
      p1++;
    saveRobotName(p1);
    toLog(msg);
//     got_id = true;
    printf("# UService::decode: got robot ID: %s\n", msg);
  }
  else if (strncmp(msg, "hbt ", 4) == 0)
  {
    toLog(msg);
  }//
  else
    used = false;
  return used;
}


void UService::terminate()
{ // Terminate modules (especially threads and log files)
  if (terminating or not setupComplete)
    return;
  printf("# --------- terminating -----------\n");
  terminating = true;
  stop = true; // stop all threads, when finished current activity
  // terminate sensors before Teensy
  teensy1.terminate();
  gpio.terminate();
  // service must be the last to close
  if (logfile != nullptr)
  {
    std::string tm = exec("date");
    toLog(tm.c_str());
    toLog("Closed log");
    fclose(logfile);
    printf("# UService:: logfile closed\n");
  }
}

std::string UService::getVersionString()
{
  // #define REV "$Id: uservice.cpp 591 2024-01-26 13:39:56Z jcan $"
  std::string ver = REV;
  int n1 = ver.find(' ', 10);
  int n2 = ver.rfind("Z ");
  std::string part = ver.substr(n1, n2-n1);
  return part;
}

////////// run //////////////

void UService::run()
{ // potential keyboard input
  bool ip_list_changed;
  bool ip_sentToTeensy = false;
  int loop = 0;
  int userCnt = 0;
  bool ipRemoved = false;
  std::string tm = exec("date");
  toLog(tm.c_str());
  while (not stop)
  {
    findIPs();
    // update message
    ip_list_changed = updateIPlist();
    // tell Regbot display about the host IP
    float t = measureCPUtemp();
    if (fabsf(t - cpuTemp) > 1)
    {
      cpuTemp = t;
      toLog("");
    }
    if (true)
    { // check if someone is using the Teensy using lsof (list open files)
      bool fileOK = file_exists(teensy1.usbDevName);
      if (fileOK)
      { // file exist, is it open?
        std::string cmd = "lsof " + teensy1.usbDevName;
        std::string s = exec(cmd);
                teensyFileFree = s.size() < 10;
        if (not teensyFileFree)
        {  // someone is using the Teensy, so the display is
          // used for something else
          if (not ipRemoved)
          {
            printf("Someone is using the Teensy\n");
            toLog("Teensy occupied");
          }
          ipRemoved = true;
        }
        else if (ipRemoved)
          printf("Teensy is free again\n");
      }
      else
      {
        if (not ipRemoved)
          printf("Someone removed the Teensy\n");
        ipRemoved = true;
      }
    }
    int n;
    std::string users = getUsers(&n);
    if (n != userCnt)
    { // number of users has changed, so note
      toLog(users.c_str());
      userCnt = n;
    }
    //
    if (ip_list_changed)
      ip_sentToTeensy = false;
    if (not ip_sentToTeensy or ipRemoved)
    { // tell Teensy and show IP on display
      // but don't if a mission is running
      if (teensyFileFree)
      { // device present and no user
        toLog("Teensy available");
        printf("# Teensy available\n");
        teensy1.openToTeensy();
        if (teensy1.teensyConnectionOpen)
        {
          const int MSL = 200;
          char s[MSL];
          snprintf(s, MSL, "disp %s\n", ip4list);
          teensy1.send(s);
          // request also robot name
          usleep(2000);
          teensy1.send("idi\n");
          // allow communication to finish
          // handled by Teensy thread
          sleep(3);
          toLog("IP updated to Teensy display, stopping connection to Teensy");
          teensy1.stopUSB = true;
          ip_sentToTeensy = true;
          ip_list_changed = false;
          ipRemoved = false;
        }
      }
    }
    sleep(2);
    loop++;
  }
}

bool UService::findIPs()
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  bool changed = false;
  //
  getifaddrs(&ifAddrStruct);
  ipsCnt = 0;
//   macCnt = 0;
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
  { // all interface names and types
    if (ifa->ifa_addr->sa_family == AF_INET)
    { // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      if (strcmp(ifa->ifa_name, "lo") != 0)
      { // not loop back - loopback is skipped
        int sl = strlen(addressBuffer);
        if (sl > 0 and strncmp(ips[ipsCnt], addressBuffer, sl) != 0)
        {
          changed = true;
          snprintf(ips[ipsCnt], MHL, "%s", addressBuffer);
          printf("#### found IP %d: %s %s\n", ipsCnt, ifa->ifa_name, ips[ipsCnt]);
        }
        if (ipsCnt < MIPS - 1)
          ipsCnt++;
      }
    }
  }
  return changed;
}

bool UService::updateIPlist()
{ // update IP list
  int n = 0;
  char * p1 = ip4list;
  bool changed = false;
  for (int i = 0; i < ipsCnt; i++)
  {
    snprintf(p1, MHL2 - n, " %s", ips[i]);
    n += strlen(p1);
    p1 = &ip4list[n];
  }
  // see if it has changed
  if (strcmp(ip4list, ip4listLast) != 0)
  { // make IP list message
    toLog(ip4list);
    changed = true;
    strncpy(ip4listLast, ip4list, MHL2);
  }
  return changed;
}

float UService::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
    //     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s);
    fclose(temp);
  }
  return t/1000.0;
}

void UService::toLog(const char* message)
{
  if (logfile != nullptr)
  {
    UTime t("now");
    fprintf(logfile, "%lu.%04ld %.1f %% %s\n", t.getSec(), t.getMicrosec()/100,
            cpuTemp,
            message);
  }
}

std::string UService::exec(std::string cmd)
{
  std::array<char, 100> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

std::string UService::getUsers(int* userCnt)
{
  std::string users = exec("w");
  const char * p1 = strstr(users.c_str(), ", ");
  if (p1 != nullptr)
  {
    p1 += 2;
    int n = strtol(p1, nullptr, 10);
//     printf("Found %d users (in %s)\n", n, users.c_str());
    *userCnt = n;
  }
  else
    printf("Failed to find user count (%s)\n", users.c_str());
  return users;
}

void UService::saveRobotName(const char * newName)
{
  //   printf("# UTeensy::renameHostTest: hertil\n");
  const int MNL = 32;
  char nn[MNL];
  int nnCnt = 0;
//   const char p1 = newName;
  for (int i = 0; i < (int)strlen(newName); i++)
  { // use non white only
    if (newName[i] > ' ')
    { // and use lower case only
      nn[nnCnt++] = tolower(newName[i]);
    }
  }
  nn[nnCnt] = '\0';
  printf("UService::saveRobotName: saves '%s' to robotname\n", nn);
  FILE * sc;
  sc = fopen("robotname","w");
  fprintf(sc,"%s", nn);
  fclose(sc);
}
