#!/usr/bin/python3

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


import socketserver
import socket
import signal
import sys
import threading
import time
import select
#from netifaces import interfaces, ifaddresses, AF_INET
#import netifaces

#
# based on example found in
# https://docs.python.org/3/library/socketserver.html#module-socketserver
#
#

## function to handle ctrl-C and reasonable shutdown
def signal_handler(sig, frame):
    print('UService:: You pressed Ctrl+C!')
    server.stop = True

#################################################################
#################################################################
#################################################################

#class MyTCPHandler(socketserver.BaseRequestHandler):
class MyTCPHandler(socketserver.StreamRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and
    override the handle() method to implement communication to the
    client.
    """
    def handle(self):
        # A client is connected, ready to be handled
        stop = False
        print("Got new client from {}".format(self.client_address[0]))
        self.send("Welcome to python vision server (send 'help' or 'quit' to quit)")
        # use poll to allow nice shutdown with ctrl-c even
        poll_obj = select.poll()
        poll_obj.register(self.rfile, select.POLLIN)
        while not stop and not server.stop:
            # test if a command is available (wait up to 100 ms for a command)
            poll_result = poll_obj.poll(100)
            if (poll_result):
                # there is data, get a full line
                self.data = self.rfile.readline().strip()
                try:
                    got = str(self.data, 'utf-8')
                    print("{} wrote: '{}'".format(self.client_address[0], got))
                except:
                    got = "invalid string"
                # split into individual words/numbers (space separated)
                gg = got.split()
                #print("# number of parameters are " + str(len(gg)))
                if len(gg) > 0:
                    if gg[0] == "quit":
                        stop = True
                    elif gg[0] == "off":
                        self.send("Server shutdown")
                        stop = True
                        server.stop = True
                    elif gg[0] == "aruco":
                        self.arucoRequest(gg)
                    elif gg[0] == "golf":
                        self.golfRequest(gg)
                    elif gg[0] == "help":
                        self.send("python vision server")
                        self.send("  quit:  Closes this connection to server")
                        self.send("  off:   Server shutdown")
                        self.send("  aruco: Report any visible ArUco codes (not implemented)")
                        self.send("  golf:  Report any visible golf balls in image (not implemented)")
                        self.send("  help:  This message")
                    else:
                        self.send(str(self.data.upper()) + ", try again, got " + got)
                    #self.request.sendall(bytes("hej - fint, prøv igen\n", "utf-8"))
            else:
                time.sleep(0.05)
            pass
        print("Client on {} hung up".format(self.client_address[0]))
        pass

    def send(self, msg):
        self.request.sendall(bytes(msg + "\r\n", "utf-8"))
        pass

    def arucoRequest(self, gg):
        id = 66
        if len(gg) > 1:
          id = int(gg[1]);
        print("Looking for ArUco ID " + str(id))
        # do ArUco analysis
        # findArUcos(id)
        # assumed result of Aruco position in cam view
        found = True
        x = 2.2   # (forward) relative to robot/camera
        y = 0.33  # (left)
        h = 0.3   # orientation (radians) around z (up)
        self.send("arucopos {} {} {} {} id {}".format(int(found),x,y,h, id))

    def golfRequest(self, gg):
        # find balls
        # [found, x, y] = findGolfBalls()
        # assumed position of first golf ball out of 3 in cam view
        found = 3
        x = [0.38,  1.22, 1.43]
        y = [0.04, -0.56, 1.12]
        if found == 0:
          self.send("golfpos 0 0 0 0")
        else:
          for i in range(0,found):
            self.send("golfpos {} {} {} {}".format(found, i, x[i],y[i]))


#################################################################
#################################################################
#################################################################

# class for server daemon,
# - a bit overkill, but can be extended with some exception handling functions
class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass


#################################################################
#################################################################
#################################################################

class UServer:
  socserver = []
  stop = False
  port = 25001
  #
  def run(self):
    # open server on then local network where the hostname is registered.
    self.socserver = ThreadedTCPServer((socket.gethostname() + ".local", self.port), MyTCPHandler)
    # run listen to port as a thread (daemon)
    serverThread = threading.Thread(target = self.socserver.serve_forever)
    serverThread.daemon = True
    serverThread.start()
    if True:
        # Opens the socket on 127.0.0.0, accessible on this host only
        # (both can run simultaneously).
        self.socserver1 = ThreadedTCPServer(("localhost", self.port), MyTCPHandler) # as server:
        serverThread1 = threading.Thread(target = self.socserver1.serve_forever)
        serverThread1.daemon = True
        serverThread1.start()
    if False:
        # Opens the socket on 169.254.x.x, accessible on cable-to-PC hookup.
        # probably not useful
        self.socserver2 = ThreadedTCPServer(("169.254.172.149", self.port), MyTCPHandler) # as server:
        serverThread2 = threading.Thread(target = self.socserver2.serve_forever)
        serverThread2.daemon = True
        serverThread2.start()
    # waiting for clients (handled in separate handle-callback)
    while not self.stop:
        time.sleep(0.2)
        pass
    self.socserver.shutdown()
  #
  #
  #def ip4_addresses(self):
    #ip_list = []
    #print("Interfaces {}".format(str(netifaces.interfaces())))
    #for interface in netifaces.interfaces():
        ##for link in ifaddresses(interface)[AF_INET]:
        #for link in netifaces.ifaddresses(interface):
            ##ip_list.append(link['addr'])
            #cc = link
            #print("# IP : {} {}".format(str(interface), str(cc)))
    #return ip_list
  #
  pass


# make the server class
server = UServer()


if __name__ == "__main__":
    #allow for ctrl-C
    signal.signal(signal.SIGINT, signal_handler)
    # welcome message
    print("# Server starting on port " + str(server.port))
    server.run()
    print("# Server on port " + str(server.port) + " terminated")
