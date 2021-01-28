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
#ifndef UnoWifiInterface_cpp
#define UnoWifiInterface_cpp
#endif

#include "config.h"
#include "defines.h" // This should be changed to DCCEX.h when possible
#if WIFI_ON == true
#include "UnoWifiInterface.h"
#include "DIAG.h"
#include "CommandDistributor.h"

UnoWifiInterface * UnoWifiInterface::singleton=NULL;
char  ReplyBuffer[] = "acknowledged";
/**
 * @brief Setup Ethernet Connection
 * 
 */
void UnoWifiInterface::setup()
{
    singleton=new UnoWifiInterface();
    if (!singleton->connected) singleton=NULL; 
};


/**
 * @brief Aquire IP Address from DHCP and start server
 * 
 * @return true 
 * @return false 
 */
UnoWifiInterface::UnoWifiInterface()
{
    
    
    DIAG(F("\n+++++ Uno Wifi Setup "));
    if (WiFi.status() == WL_NO_MODULE) {

    DIAG(F("Communication with WiFi module failed!"));
    return;
  }

  String fv = WiFi.firmwareVersion();

  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {

    DIAG(F("Please upgrade the firmware"));

  }
   

   while (status != WL_CONNECTED) {

    DIAG(F("Attempting to connect to SSID: %S", WIFI_SSID));

    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // wait 10 seconds for connection:

    delay(10000);

  }

  DIAG(F("Connected to wifi"));
 IPAddress ip = WiFi.localIP();
 DIAG(F("IP Address %S", ip))
  // if you get a connection, report back via serial:
    
  
    #ifdef IP_ADDRESS
   
    #else

    #endif
}

/**
 * @brief Main loop for the EthernetInterface
 * 
 */
void UnoWifiInterface::loop()
{
    if (!singleton) return;
   singleton->loop2();

}

 void UnoWifiInterface::loop2()
{   

}

#endif