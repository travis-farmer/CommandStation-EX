/*
 *  © 2021, Chris Harlow. All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef WifiInterface_h
#define WifiInterface_h
#include <SPI.h>
#include <WiFiNINA.h>
#include "FSH.h"
#include "RingStream.h"

#define MAX_NINA_BUFFER 512
#define OUTBOUND_RING_SIZE 2048

class WifiInterface
{

public:
  static bool setup(long serial_link_speed, // ignored
                    const FSH *wifiESSID,
                    const FSH *wifiPassword,
                    const FSH *hostname,
                    const int port = 2560); // ignored
  static void loop();

private:
  static WifiInterface *singleton;

  WifiInterface(long serial_link_speed,
                const FSH *wifiESSID,
                const FSH *wifiPassword,
                const FSH *hostname,
                const int port);

  enum INBOUND_STATE : byte
  {
    INBOUND_BUSY, // keep calling in loop()
    INBOUND_IDLE  // Nothing happening, outbound may xcall CIPSEND
  };

  enum LOOP_STATE : byte
  {
    ANYTHING,  // ready for +IPD, n CLOSED, n CONNECTED, busy etc...
    SKIPTOEND, // skip to newline

    // +IPD,client,length:data
    IPD,             // got +
    IPD1,            // got +I
    IPD2,            // got +IP
    IPD3,            // got +IPD
    IPD4_CLIENT,     // got +IPD,  reading cient id
    IPD5,            // got +IPD,c
    IPD6_LENGTH,     // got +IPD,c, reading length
    IPD_DATA,        // got +IPD,c,ll,: collecting data
    IPD_IGNORE_DATA, // got +IPD,c,ll,: ignoring the data that won't fit inblound Ring

    GOT_CLIENT_ID, // clientid prefix to CONNECTED / CLOSED
    GOT_CLIENT_ID2 // clientid prefix to CONNECTED / CLOSED
  };

  LOOP_STATE loopState = ANYTHING;
  int runningClientId; // latest client inbound processing data or CLOSE
  int dataLength;      // dataLength of +IPD
  int clientPendingCIPSEND = -1;
  int currentReplySize;
  bool pendingCipsend;
  void purgeCurrentCIPSEND();

  static WiFiServer server;
  static bool connected;
  //static RingStream *outboundRing;
  void loop1();
  INBOUND_STATE loop2();
};
#endif
