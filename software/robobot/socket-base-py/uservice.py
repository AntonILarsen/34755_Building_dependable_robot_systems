 #/***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jca@elektro.dtu.dk
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


from functools import partial
import time
import socket

import signal
import sys


# NB! this part is not used


class UService:
    conn = []
    addr = []
    def setup(self):
        # allow close down on ctrl-C
        signal.signal(signal.SIGINT, signal_handler)
        #self.socketSetup(25001)
        #

    def runSocketService(self, port):
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", port))
        s.listen()
        # accept will block
        self.conn, self.addr = s.accept()
        with self.conn:
          print(f"Connect accepted {self.addr}")
          while True:
              data = self.conn.recv(1024)
              if not data:
                  break
              #conn.sendall(data)
              self.decode(data)
          pass
      pass


    def decode(self, msg):
        used = False # state.decode(msg, msgTime)
        if not used:
          self.conn.sendall("# message not used")
          print("# Service:: message not used " + msg)
        return used

    def shutdown(self):
        #state.terminate()
        print("# terminated")

# create the service object
service = UService()

