/**
 * Interface with Teensy
 *  
 #***************************************************************************
 #*   Copyright (C) 2017-2023 by DTU
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

#include <sys/time.h>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>

#include "steensy.h"
#include "uservice.h"

using namespace std;

STeensy teensy1;

bool UOutQueue::setMessage(const char* message)
{ // add a '!' to request confirmation of this message
  msg[3] = '!';
  len = strnlen(message, MML);
  bool isOK = len + 5 < MML;
  if (isOK)
  {
    strncpy(&msg[4], message, len);
    len += 4;
    if (msg[len-1] != '\n')
    { // add a \n if it is not there
      msg[len++] = '\n';
    }
    // terminate string
    msg[len] = '\0';
    // add crc in front
    const int MCL = 4;
    char cc[MCL];
    teensy1.generateCRC(&msg[3], cc);
    strncpy(msg, cc, 3);
  }
  else
    printf("# STeensy::UOutQueue::setMessage: messages longer than %d chars are not allowed! '%s'\n", MML, message);
  // debug
//   printf("# STeensy:: set new message (ok=%d) to %s", isOK, msg);
  // debug end
  return isOK;
}



void STeensy::setup()
{
  teensyConnectionOpen = false;
  // get ini-file values
  usbDevName = "/dev/ttyACM0";
  //
  if (true)
  { // open log file and write the header - else no logging
    std::string fn = "log_teensy.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% teensy communication to/from Teensy\n");
    fprintf(logfile, "%% 1 \tTime (sec) from system\n");
    fprintf(logfile, "%% 2 \t(Tx) Send to Teensy\n");
    fprintf(logfile, "%%   \t(Rx) Received from Teensy\n");
    fprintf(logfile, "%%   \t(Qu N) Put in queue to Teensy, now queue size N\n");
    fprintf(logfile, "%% 3 \tMessage string queued, send or received\n");
  }
  // start thread and open teensy connection
  th1 = new std::thread(runObj, this);
  usleep(100000);
  // start the receive thread (that also sends the queue elements)
//   initialized = true;
}

void STeensy::terminate()
{ // wait for last message to be processed
  send("leave\n", true);
  send("disp disconnected\n", true);
  // wait until output queue is empty
  UTime t("now");
  while (outQueue.size() > 0 and t.getTimePassed() < 1)
    usleep(1000);
  stopUSB = true;
  if (th1 != nullptr)
  {
    th1->join();
//     printf("# STeensy:: read thread closed\n");
  }
  // close logfile if open
  if (logfile != nullptr)
  {
    dataLock.lock();
    fprintf(logfile, "%% end of file\n");
    fclose(logfile);
    logfile = nullptr;
    printf("# STeensy:: logfile closed\n");
    dataLock.unlock(); // ensure consistency
  }
}

/**
  * send a string to the serial port (Teensy) */
bool STeensy::send(const char* message, bool direct)
{
  bool sendOK = false;
  if (direct)
  {
    sendOK = sendDirect(message);
  }
  else
  { // using queue is default
//     printf(" STeensy::sending into queue : %s\n", message);
    sendToQueue(message);
  }
  return sendOK;
}

void STeensy::sendToQueue(const char* message)
{
  // debug
//   if (strncmp(message, "sub enc", 7) == 0)
//     printf("# STeensy 'sub enc' just before queue %s", message);
  // debug end
  outQueue.push(UOutQueue(message));
  dataLock.lock(); // ensure consistency
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld Qu %d %s",
            outQueue.back().queuedAt.getSec(),
            outQueue.back().queuedAt.getMicrosec()/100,
            (int)outQueue.size(),
            outQueue.back().msg);
  }
//   printf("# STeensy::sendToQueue: added '%s' tx-queue, now size %d\n", outQueue.back().msg, (int)outQueue.size());
  dataLock.unlock();
}

bool STeensy::generateCRC(const char * cmd, char * crc)
{
  int n = strlen(cmd);
  // add CRC code
  const int MCL = 4;
//   char crc[MCL];
  const char * p1 = cmd;
  bool gotNewline = false;
  int sum = 0;
  for (int i = 0; i < n; i++)
  { // do not count \t, \r, \n etc
    // as these gives problems for systems with auto \n or \n\r or similar
    if (*p1 >= ' ')
      sum += *p1;
    if (*p1 == '\n')
    { // one newline is allowed only, the rest is ignored
      gotNewline = true;
      break;
    }
    p1++;
  }
  snprintf(crc, MCL, ";%02d", (sum % 99) + 1);
  return gotNewline;
}

bool STeensy::sendDirect(const char* message)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
  int timeoutMs = 100;
//   printf("# STeensy try to send timeout=%dms, confirmSend=%d, msg:%s", timeoutMs, confirmSend, message);
  int t = 0;
  bool lostConnection = false;
  bool sendOK = false;
  std::string cmd = message;
  // remove any source information as this is not relevant for the Teensy
  if (teensyConnectionOpen and cmd[0] != '#')
  { // simulator or not
    // add CRC code
    const int MCL = 4;
    char crc[MCL];
    bool gotNewline = generateCRC(message, crc);
    cmd.insert(0, crc);
    if (not gotNewline)
      cmd.append(1, '\n');
    int n = cmd.size();
    //
    sendLock.lock();
    // may have been closed in the meantime
    if (teensyConnectionOpen)
    {
      sendCnt++;
//       printf("# STeensy sending directly CRC:'%s', msg:'%s'\n", crc, cmd);
//       int m = write(usbport, crc, 3);
      int d = 0;
      int m;
      while ((d < n) and (t < timeoutMs))
      { // want to send n bytes to usbport within timeout period
        m = write(usbport, &cmd[d], n - d);
        if (m < 0)
        { // error - an error occurred while sending
          switch (errno)
          { // may be an error, or just nothing send (buffer full)
            case EAGAIN:
              //not all send - just continue
              printf("STeensy::sendDirect: waiting - nothing send %d/%d\n", d, n);
              usleep(1000);
              t += 1;
              break;
            default:
              perror("STeensy::sendDirect (closing connection): ");
              lostConnection = true;
              break;
          }
          // dump the rest on most errors
          if (lostConnection)
            break;
        }
        else
          // count bytes send
          d += m;
      }
      sendOK = d == n;
      dataLock.lock();
      if (logfile != nullptr)
      {
        UTime t;
        t.now();
        fprintf(logfile, "%lu.%04ld Txd %s", t.getSec(), t.getMicrosec()/100, cmd.c_str());
        if (not gotNewline)
          fprintf(logfile, "\n");
      }
      dataLock.unlock();
      // include a short break to ensure that Teensy do not get overloaded
      usleep(500);
    }
    if (lostConnection)
    {
      closeUSB();
    }
    else
      lastTxTime.now();
    sendLock.unlock();
  }
//  printf("# STeensy:: send finished\n");
  return sendOK;
}

////////////////////////////////////////////////////////////////////////

void STeensy::closeUSB()
{
  if (teensyConnectionOpen)
  {
    teensyConnectionOpen = false;
    // tell Teensy that we leave
//     printf("# STeensy::run - no relevant activity, shutting down\n");
//     printf("# STeensy::run but open=%d, gotAct=%d, lastTime=%f, just=%d, justTime=%g\n",
//           teensyConnectionOpen, gotActivityRecently, lastRxTime.getTimePassed(), justConnected, justConnectedTime.getTimePassed());
    // then close the connection (after 100ms)
    usleep(100000);
    close(usbport);
    usbport = -1;
    justConnected = false;
    // stop the tx queue and empty any remaining
    confirmSend = false;
    while (not outQueue.empty())
      outQueue.pop();
    //
    if (logfile != nullptr)
    {
      UTime t;
      t.now();
      fprintf(logfile, "%lu.%04ld USB %s closed\n", t.getSec(), t.getMicrosec()/100, usbDevName.c_str());
    }
  }
}



/**
  * receive thread */
void STeensy::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  int readIdleLoops = 0;
  UTime t, terr;
  t.now();
  terr.now();
  const int MTS = 10;
  UTime tit[MTS];
  UTime msgTime;
  float titsum[MTS] = {0};
  // get robot name
  tit[9].now();
//   bool ntpUpdate = false;
  while (not service.stop)
  { // handle Teensy connection
    if (stopUSB) // and (service.got_id or t.getTimePassed() > 10))
    {
      closeUSB();
      stopUSB = false;
      printf("# STeensy::run: Closed connection to Teensy\n");
    }
    if (teensyConnectionOpen)
    { // we are connected
      //
      if (justConnected)
      { // no name is received yet, so try again
        tit[2].now();
        // justconnected flag is cleared when receiving a 'dname' message from Teensy
        send("hbti\n", true); // this may be lost - and no problem
        send("leave\n", true); // stop any old subscriptions
        justConnected = false;
        t.now();
        titsum[2] += tit[2].getTimePassed();
      }
      if (gotActivityRecently and lastRxTime.getTimePassed() > 2)
      { // are loosing data - may be just temporarily
        gotActivityRecently = false;
      }
      // read from USB
      tit[3].now();
      //
      n = read(usbport, &rx[rxCnt], 1);
      if (n < 0 and errno == EAGAIN)
      { // no data
        n = 0;
      }
      else if (n < 0)
      { // other error - close connection
        perror("Teensy::run port error");
        usleep(100000);
        sendLock.lock();
        // don't close while sending
        closeUSB();
        sendLock.unlock();
      }
      titsum[3] += tit[3].getTimePassed();
      //
      // got a character - assemble to a text line
      if (n == 1)
      { // got a new character
        tit[4].now(); // timing
        //
        if (rxCnt > 0)
        {
          rxCnt++;
        }
        else if (rx[0] == ';')
        { // first character in a new message
          msgTime.now();
          rxCnt = 1;
        }
        // check for end of message, i.e. a new-line
        if (rx[rxCnt-1] == '\n')
        { // terminate string - end of new line
          rx[rxCnt] = '\0';
          // save to logfile if open
          dataLock.lock();
          if (logfile != nullptr)
          {
            fprintf(logfile, "%lu.%04ld Rx %s", msgTime.getSec(), msgTime.getMicrosec()/100, rx);
          }
          dataLock.unlock();
          // handle this message line
          if (crcCheck(rx))
          { // got (at least) one valid message
            const char * okMsg = &rx[3];
            // check if this is a confirm message
            if (strncmp(okMsg, "confirm", 7) == 0)
            { // release next message
              confirmSend = true;
//               printf("# STeensy::run: received a confirm: '%s'\n", rx);
              messageConfirmed(rx);
            }
            else
            {
              decode(okMsg, msgTime);
            }
          }
          else
            printf("# Teenst message discarded (crc-error) %s\n", rx);
          // set activity timeer
          gotActivityRecently = true;
          lastRxTime.now();
          // reset receive buffer
          rxCnt = 0;
          n = 0;
          gotCnt++;
        }
        titsum[4] += tit[4].getTimePassed();
      }
      else if (n == 0)
      {  // no data, so wait a bit
        tit[5].now();
        //
        usleep(1000);
        readIdleLoops++;
        titsum[5] += tit[5].getTimePassed();
      }
      else
      { // debug n!= 0 and n!= 1
        tit[6].now();
        printf("# Teensy::run: got n=%d chars, when asking for 1!\n", n);
        fflush(nullptr);
        titsum[6] += tit[6].getTimePassed();
      }
      if (not outQueue.empty())
      { // got the first confirm
//         printf("#STeensy:: que not empty\n");
        tit[7].now();
        if (not outQueue.front().isSend)
        { // new message to send
          sendLock.lock();
          if (teensyConnectionOpen)
          { // send queued message to Teensy
            write(usbport, outQueue.front().msg, outQueue.front().len);
            outQueue.front().sendAt.now();
            outQueue.front().isSend = true;
            outQueue.front().resendCnt++;
            if (logfile != nullptr)
            {
              fprintf(logfile, "%lu.%04ld Tx %s",
                      outQueue.front().sendAt.getSec(),
                      outQueue.front().sendAt.getMicrosec()/100,
                      outQueue.front().msg);
            }
//             printf("# STeensy::from queue: %lu.%04ld Tx '%s'",
//                    outQueue.front().sendAt.getSec(),
//                    outQueue.front().sendAt.getMicrosec()/100,
//                    outQueue.front().msg);
          }
          sendLock.unlock();
        }
        else
        { // waiting for confirmation - check for too old
//           printf("# STeensy:: is send - waiting for confirm\n");
          float dt = outQueue.front().sendAt.getTimePassed();
          if (dt > confirmTimeout)
          {
            // debug
            printf("# STeensy::run confirm timeout after %.5f sec (retry=%d, queue=%d):%s\n",
                    outQueue.front().sendAt.getTimePassed(),
                    outQueue.front().resendCnt,
                    (int)outQueue.size(),
                    outQueue.front().msg);
            // debug end
            if (outQueue.front().resendCnt < 13)
            { // just try again
              outQueue.front().isSend = false;
              confirmRetryCnt++;
            }
            else
            { // remove from queue
              outQueue.pop();
              confirmRetryDump++;
            }
          }
        }
        titsum[7] += tit[7].getTimePassed();
      }
    } // connected
    else
    { // nothing to do
      usleep(100000);
    }
//     ntpUpdate = false;
//     if (tit[9].getTimePassed() > 2.0)
//     { // likely NTP update received
//       titsum[9]+= tit[9].getTimePassed();
//       // don't close connection based on a NPT update
//       ntpUpdate = true;
//       printf("# NTP update? time glitch of %.3f sec\n", tit[9].getTimePassed());
//       fflush(nullptr);
//     }
    tit[9].now();
  }
  closeUSB();
}  

bool STeensy::crcCheck(const char* msg)
{ // not really a standard CRC check, just modulus of all visible characters
  bool dataOK = false;
  if (msg[0] == ';')
  { // there is a CRC check code
    if (isdigit(msg[1]) and isdigit(msg[2]))
    {
      const char * p1 = &msg[3];
      int sum = 0;
      int m = strlen(p1);
      for (int i = 0; i < m; i++)
      { // sum all visible characters
        if (*p1 >= ' ')
          sum += *p1;
        p1++;
      }
      int q1 = (sum % 99) + 1;
      int q2 = (msg[1] - '0') * 10 + msg[2] - '0';
      if (q1 != q2)
        printf("# UHandler::handleCommand: CRC check failed (from Teensy) q1=%d != q2=%d (msg=%s\n", q1, q2, msg);
      dataOK = true;
    }
  }
  return dataOK;
}


void STeensy::messageConfirmed(const char* confirm)
{ // got a confirm message
  // test for first message in tx queue
  // remove if a match - else ignore
  if (not outQueue.empty())
  {
    if (outQueue.front().isSend)
    { // this message is send, but is it equal
      bool eq = outQueue.front().compare(&confirm[11]);
      if (eq)
      {
        if (outQueue.front().resendCnt > 1)
        {
          printf("# STeensy::run: Confirm OK after %d retry and %.4fs: send'%s'",
                  outQueue.front().resendCnt,
                  outQueue.front().queuedAt.getTimePassed(),
                  outQueue.front().msg);
        }
        outQueue.pop();
      }
      else
      { // no match
        confirmMismatchCnt++;
      }
    }
  }
}


/**
  * Open the connection.
  * \returns true if successful */
bool STeensy::openToTeensy()
{
   // should use USB port
  if (usbport != -1)
  {
    printf("# Teensy::openToTeensy device %s is open already\n", usbDevName.c_str());
  }
  else
  { // not open already - try
//     printf("# Teensy::openToTeensy '%s' - opening\n", usbDevName);
    // make reservation
    usbport = open(usbDevName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (usbport == -1)
    { // open failed
      if (connectErrCnt < 5)
      { // don't spam with too many error messages
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# STeensy::openToTeensy open '%s' failed:",  usbDevName.c_str());
        perror(s);
      }
      // wait a bit before re-connection
      usleep(300000);
      connectErrCnt++;
    }
    else
    { // set connection to non-blocking
//       printf("# STeensy::openToTeensy opened '%s'\n", usbDevName);
      int flags;
      if (-1 == (flags = fcntl(usbport, F_GETFL, 0)))
        flags = 0;
      fcntl(usbport, F_SETFL, flags | O_NONBLOCK);
  // #ifdef armv7l
      struct termios options;
      tcgetattr(usbport, &options);
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
      options.c_iflag = IGNPAR;
      options.c_oflag = 0;
      options.c_lflag = 0;
      tcsetattr(usbport, TCSANOW, &options);
  // #endif
      tcflush(usbport, TCIFLUSH);
      connectErrCnt = 0;
    }
    teensyConnectionOpen = usbport != -1;
    if (teensyConnectionOpen)
    { // request base data
//       printf("# STeensy::run - just connected to '%s'\n", usbDevName);
      justConnected = true;
      justConnectedTime.now();
      //         initMessageTypes();
      // assume there is activity - in order not to
      // get an error right away
      gotActivityRecently = true;
      lastRxTime.now();
      if (logfile != nullptr)
      {
        UTime t;
        t.now();
        fprintf(logfile, "%lu.%04ld USB %s opened\n", t.getSec(), t.getMicrosec()/100, usbDevName.c_str());
      }
    }
  }
  return teensyConnectionOpen;
}


bool STeensy::decode(const char * msg, UTime & msgTime)
{
  // debug
  if (false)
  {
    const int MSL = 100;
    char s[MSL];
    msgTime.getTimeAsString(s, true);
    printf("# STeensy:: got %s for decoding:%s", s, msg);
  }
  // debug end
  bool used = true;
//   const char * p1 = msg;

  if      (service.decode(msg, msgTime))
  { // nothing to do here
  }
  else if (msg[0] == '#')
  { // service message - just ignored
    printf("# UTeensy:: service message from Teensy: %s", msg);
  }
  else
  {
    printf(" UTeensy:: unused Teensy message: %s", msg);
    used = false;
  }
  return used;
}

int STeensy::getTeensyCommError(int& retryCnt)
{
  retryCnt = confirmRetryCnt;
  return confirmRetryDump;
}

int STeensy::getTeensyCommQueueSize()
{
  return outQueue.size();
}

