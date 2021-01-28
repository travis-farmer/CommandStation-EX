/*
 *  © 2020,Anthony Williams All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
 *
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
 *  Uno Wifi Interface added by Anthony Williams
 */
#ifndef UnoWifiInterface_h
#define UnoWifiInterface_h
#endif

#include <SPI.h>
#include <WiFiNINA.h>
#include "DCCEXParser.h"

#define MAX_WIFI_BUFFER 512

class UnoWifiInterface {
    public:
     
     static void setup();       
     static void loop();
   
 private:
     static UnoWifiInterface * singleton;
     bool connected;
     UnoWifiInterface();
     void loop2();
    WifiServer * server;
    WifiClient clients[MAX_SOCK_NUM];                // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on the Shield
    uint8_t buffer[MAX_WIFI_BUFFER+1];                    // buffer used by TCP for the recv
    
  
}