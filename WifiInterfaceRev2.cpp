/*
 *  © 2021, Chris Harlow, Anthony Williams. All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
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
 * 
 */
#ifdef ARDUINO_AVR_UNO_WIFI_REV2
// This code is ONLY compiled on a unoWifiRev2 processor which uses a different architecture

#include "WifiInterfaceRev2.h"
#include "DIAG.h"
#include "CommandDistributor.h"

WifiInterface * WifiInterface::singleton;

WiFiServer WifiInterface::server(2560);
bool WifiInterface::connected = false;
/**
 * @brief Setup Wifi Connection
 * 
 */

bool WifiInterface::setup(long serial_link_speed,
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port)
{
  singleton = new WifiInterface(serial_link_speed, wifiESSID, wifiPassword, hostname, port);
  //return singleton->connected;
}

WifiInterface::WifiInterface(long serial_link_speed,
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port)
{
  (void)serial_link_speed;
  (void)port;     // obsolete
  (void)hostname; // To be implemented

  if (WiFi.status() == WL_NO_MODULE)
  {
    DIAG(F("Wifi- hardware failed\n"));
  }
  DIAG(F("Wifi Firmware=%s  expected=%S"), WiFi.firmwareVersion(), F(WIFI_FIRMWARE_LATEST_VERSION));

  int status = WL_IDLE_STATUS;
  int attempts = 4;
  while (status != WL_CONNECTED)
  {
    if (attempts-- <= 0)
    {
      DIAG(F("\nFAILED - No Wifi\n"));
    }
    DIAG(F("\nAttempting to connect to %s\n"), wifiESSID);
    status = WiFi.begin(wifiESSID, wifiPassword);
    // wait 10 seconds for connection:
    delay(10000);
  }

  server.begin(); // start the server on port 2560

  IPAddress ip = WiFi.localIP();
  LCD(4, F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
  LCD(5, F("Port:2560"));
}

void WifiInterface::loop() {
  singleton->loop1();
}

/**
 * @brief Main loop for the WifiInterfaceRev2
 * 
 */
void WifiInterface::loop1()
{

  if (loop2() != INBOUND_IDLE)
    return;
  // WiFiClient client = server.available(); // listen for incoming clients
  // if (client)
  // {
  //   // read bytes from a client
  //   byte buffer[MAX_NINA_BUFFER];
  //   int count = client.read(buffer, MAX_NINA_BUFFER - 1);
  //   buffer[count] = '\0'; // terminate the string properly
  //   if (Diag::WIFI)
  //     DIAG(F("WIFI:%e\n"), buffer);
  //   // TEMPORARY - Assume all clients are client 1, this will confuse WiThrottle!
  //   outboundRing->mark(1);
  //   // TEMPORARY - Assume all clients are client 1, this will confuse WiThrottle!
  //   CommandDistributor::parse(1, buffer, outboundRing);
  //   outboundRing->commit();
  //   int socketOut = outboundRing->read();
  //   if (socketOut >= 0)
  //   {
  //     int count = outboundRing->count();
  //     if (Diag::WIFI)
  //       DIAG(F("Wifi Reply count=:%d\n"), count);
  //     for (; count > 0; count--)
  //       client.write(outboundRing->read());
  //     client.flush(); //maybe
  //   }
  // }
}

WifiInterface::INBOUND_STATE WifiInterface::loop2()
{
  WiFiClient client = server.available(); // listen for incoming clients
  if (client)
  {
    while (client.available())
    {
      int ch = client.read();

      // echo the char to the diagnostic stream in escaped format
      if (Diag::WIFI)
      {
        // DIAG(F(" %d/"), loopState);
        StringFormatter::printEscape(ch); // DIAG in disguise
      }

      switch (loopState)
      {
      case ANYTHING: // looking for +IPD, > , busy ,  n,CONNECTED, n,CLOSED, ERROR, SEND OK

        if (ch == '+')
        {
          loopState = IPD;
          break;
        }

        if (ch == '>')
        {
          if (Diag::WIFI)
            DIAG(F("[XMIT %d]"), currentReplySize);
          // for (int i = 0; i < currentReplySize; i++)
          // {
          //   int cout = outboundRing->read();
          //   client.print(cout);
          //   if (Diag::WIFI)
          //     StringFormatter::printEscape(cout); // DIAG in disguise
          // }
          clientPendingCIPSEND = -1;
          pendingCipsend = false;
          loopState = SKIPTOEND;
          break;
        }

        if (ch == 'R')
        { // Received ... bytes
          loopState = SKIPTOEND;
          break;
        }

        if (ch == 'S')
        { // SEND OK probably
          loopState = SKIPTOEND;
          break;
        }

        if (ch == 'b')
        { // This is a busy indicator... probabaly must restart a CIPSEND
          pendingCipsend = (clientPendingCIPSEND >= 0);
          loopState = SKIPTOEND;
          break;
        }

        if (ch >= '0' && ch <= '9')
        {
          runningClientId = ch - '0';
          loopState = GOT_CLIENT_ID;
          break;
        }

        if (ch == 'E' || ch == 'l')
        { // ERROR or "link is not valid"
          if (clientPendingCIPSEND >= 0)
          {
            // A CIPSEND was errored... just toss it away
            //purgeCurrentCIPSEND();
          }
          loopState = SKIPTOEND;
          break;
        }

        break;

      case IPD: // Looking for I   in +IPD
        loopState = (ch == 'I') ? IPD1 : SKIPTOEND;
        break;

      case IPD1: // Looking for P   in +IPD
        loopState = (ch == 'P') ? IPD2 : SKIPTOEND;
        break;

      case IPD2: // Looking for D   in +IPD
        loopState = (ch == 'D') ? IPD3 : SKIPTOEND;
        break;

      case IPD3: // Looking for ,   After +IPD
        loopState = (ch == ',') ? IPD4_CLIENT : SKIPTOEND;
        break;

      case IPD4_CLIENT: // reading connection id
        if (ch >= '0' || ch <= '9')
        {
          runningClientId = ch - '0';
          loopState = IPD5;
        }
        else
          loopState = SKIPTOEND;
        break;

      case IPD5: // Looking for ,   After +IPD,client
        loopState = (ch == ',') ? IPD6_LENGTH : SKIPTOEND;
        dataLength = 0; // ready to start collecting the length
        break;

      case IPD6_LENGTH: // reading for length
        if (ch == ':')
        {
          if (dataLength == 0)
          {
            loopState = ANYTHING;
            break;
          }
          if (Diag::WIFI)
            DIAG(F("\nWifi inbound data(%d:%d):"), runningClientId, dataLength);
          // if (server.freeSpace() <= (dataLength + 1))
          // {
          //   // This input would overflow the inbound ring, ignore it
          //   loopState = IPD_IGNORE_DATA;
          //   if (Diag::WIFI)
          //     DIAG(F("\nWifi OVERFLOW IGNORING:"));
          //   break;
          // }
          // server.mark(runningClientId);
          loopState = IPD_DATA;
          break;
        }
        dataLength = dataLength * 10 + (ch - '0');
        break;

      case IPD_DATA: // reading data
        server.write(ch);
        dataLength--;
        if (dataLength == 0)
        {
          //server.commit();
          loopState = ANYTHING;
        }
        break;

      case IPD_IGNORE_DATA: // ignoring data that would not fit in inbound ring
        dataLength--;
        if (dataLength == 0)
          loopState = ANYTHING;
        break;

      case GOT_CLIENT_ID: // got x before CLOSE or CONNECTED
        loopState = (ch == ',') ? GOT_CLIENT_ID2 : SKIPTOEND;
        break;

      case GOT_CLIENT_ID2: // got "x,"
        if (ch == 'C')
        {
          // got "x C" before CLOSE or CONNECTED, or CONNECT FAILED
          //if (runningClientId == clientPendingCIPSEND)
            //purgeCurrentCIPSEND();
        }
        loopState = SKIPTOEND;
        break;

      case SKIPTOEND: // skipping for /n
        if (ch == '\n')
          loopState = ANYTHING;
        break;
      } // switch
    }
  } // available
  return (loopState == ANYTHING) ? INBOUND_IDLE : INBOUND_BUSY;
}

// void WifiInboundHandler::purgeCurrentCIPSEND() {
//          // A CIPSEND was sent but errored... or the client closed just toss it away
//          if (Diag::WIFI) DIAG(F("Wifi: DROPPING CIPSEND=%d,%d\n"),clientPendingCIPSEND,currentReplySize);
//          for (int i=0;i<=currentReplySize;i++) outboundRing->read();
//          pendingCipsend=false;  
//          clientPendingCIPSEND=-1;
// }

#endif
