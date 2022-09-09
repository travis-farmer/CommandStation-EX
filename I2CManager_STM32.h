/*
 *  © 2022 Paul M Antoine
 *  © 2021, Neil McKechnie
 *  All rights reserved.
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

#ifndef I2CMANAGER_STM32_H
#define I2CMANAGER_STM32_H

#include <Arduino.h>
#include "I2CManager.h"

#include <wiring_private.h>
#include <utility/twi.h>

/*****
 *  Base address for  
 */

/***************************************************************************
 *  Interrupt handler.
 *  Later we may wish to allow use of an alternate I2C bus, or more than one I2C
 *  bus on the STM32 architecture 
 ***************************************************************************/
#if defined(I2C_USE_INTERRUPTS) && defined(ARDUINO_STM32)
// set interrupt callbacks here
#endif

// Declare an I2C structure, as used by the STM32 TWI driver
i2c_t i2c;

/***************************************************************************
 *  Set I2C clock speed
 ***************************************************************************/
void I2CManagerClass::I2C_setClock(uint32_t i2cClockSpeed) {
  i2c_setTiming(&i2c, i2cClockSpeed);
  return;
}

/***************************************************************************
 *  Initialise I2C pins and modes
 ***************************************************************************/
void I2CManagerClass::I2C_init()
{
  i2c.sda = digitalPinToPinName(SDA);
  i2c.scl = digitalPinToPinName(SCL);
  i2c.__this = &i2c;
  i2c.isMaster = true;
  i2c.generalCall = false;
  i2c_custom_init(&i2c, I2C_FREQ, I2C_ADDRESSINGMODE_7BIT, (0x01 << 1));

#if defined(I2C_USE_INTERRUPTS)
  // // Setting NVIC
  // NVIC_EnableIRQ(SERCOM3_IRQn);
  // NVIC_SetPriority (SERCOM3_IRQn, 0);  /* set Priority */

  // // Enable all interrupts
  // s->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR;
#endif

  // // Calculate baudrate and set default rate for now
  // s->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * I2C_FREQ) - 7 / (2 * 1000);

  // // Enable the I2C master mode and wait for sync
  // s->I2CM.CTRLA.bit.ENABLE = 1 ;
  // while (s->I2CM.SYNCBUSY.bit.ENABLE != 0);

  // // Setting bus idle mode and wait for sync
  // s->I2CM.STATUS.bit.BUSSTATE = 1 ;
  // while (s->I2CM.SYNCBUSY.bit.SYSOP != 0);

  // // Set SDA/SCL pins as outputs and enable pullups, at present we assume these are
  // // the default ones for SERCOM3 (see assumption above)
  // pinPeripheral(PIN_WIRE_SDA, g_APinDescription[PIN_WIRE_SDA].ulPinType);
  // pinPeripheral(PIN_WIRE_SCL, g_APinDescription[PIN_WIRE_SCL].ulPinType);

  // // Enable the SCL and SDA pins on the sercom: includes increased driver strength,
  // // pull-up resistors and pin multiplexer
	// PORT->Group[g_APinDescription[PIN_WIRE_SCL].ulPort].PINCFG[g_APinDescription[PIN_WIRE_SCL].ulPin].reg =  
	// 	PORT_PINCFG_DRVSTR | PORT_PINCFG_PULLEN | PORT_PINCFG_PMUXEN;  
  // PORT->Group[g_APinDescription[PIN_WIRE_SDA].ulPort].PINCFG[g_APinDescription[PIN_WIRE_SDA].ulPin].reg = 
	// 	PORT_PINCFG_DRVSTR | PORT_PINCFG_PULLEN | PORT_PINCFG_PMUXEN;
}

/***************************************************************************
 *  Initiate a start bit for transmission.
 ***************************************************************************/
void I2CManagerClass::I2C_sendStart() {
  bytesToSend = currentRequest->writeLen;
  bytesToReceive = currentRequest->readLen;

  // We may have initiated a stop bit before this without waiting for it.
  // Wait for stop bit to be sent before sending start.
  while (s->I2CM.STATUS.bit.BUSSTATE == 0x2);

  // If anything to send, initiate write.  Otherwise initiate read.
  if (operation == OPERATION_READ || ((operation == OPERATION_REQUEST) && !bytesToSend))
  {
    // Send start and address with read/write flag or'd in
    s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1) | 1;
  }
  else {
    // Wait while the I2C bus is BUSY
    while (s->I2CM.STATUS.bit.BUSSTATE != 0x1);
    s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1ul) | 0;
  }
}

/***************************************************************************
 *  Initiate a stop bit for transmission (does not interrupt)
 ***************************************************************************/
void I2CManagerClass::I2C_sendStop() {
  s->I2CM.CTRLB.bit.CMD = 3; // Stop condition
}

/***************************************************************************
 *  Close I2C down
 ***************************************************************************/
void I2CManagerClass::I2C_close() {
  I2C_sendStop();
}

/***************************************************************************
 *  Main state machine for I2C, called from interrupt handler or,
 *  if I2C_USE_INTERRUPTS isn't defined, from the I2CManagerClass::loop() function
 *  (and therefore, indirectly, from I2CRB::wait() and I2CRB::isBusy()).
 ***************************************************************************/
void I2CManagerClass::I2C_handleInterrupt() {

  if (s->I2CM.STATUS.bit.ARBLOST) {
    // Arbitration lost, restart
    I2C_sendStart();   // Reinitiate request
  } else if (s->I2CM.STATUS.bit.BUSERR) {
    // Bus error
    state = I2C_STATUS_BUS_ERROR;
  } else if (s->I2CM.INTFLAG.bit.MB) {
    // Master write completed
    if (s->I2CM.STATUS.bit.RXNACK) {
      // Nacked, send stop.
      I2C_sendStop();
      state = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
    } else if (bytesToSend) {
      // Acked, so send next byte
      if (currentRequest->operation == OPERATION_SEND_P)
        s->I2CM.DATA.bit.DATA = GETFLASH(currentRequest->writeBuffer + (txCount++));
      else
        s->I2CM.DATA.bit.DATA = currentRequest->writeBuffer[txCount++];
      bytesToSend--;
    } else if (bytesToReceive) {
      // Last sent byte acked and no more to send.  Send repeated start, address and read bit.
        s->I2CM.ADDR.bit.ADDR = (currentRequest->i2cAddress << 1) | 1;
    } else {
      // No more data to send/receive. Initiate a STOP condition.
      I2C_sendStop();
      state = I2C_STATUS_OK; // Done
    }
  } else if (s->I2CM.INTFLAG.bit.SB) {
    // Master read completed without errors
    if (bytesToReceive) {
      currentRequest->readBuffer[rxCount++] = s->I2CM.DATA.bit.DATA;  // Store received byte
      bytesToReceive--;
    } else { 
      // Buffer full, issue nack/stop
      s->I2CM.CTRLB.bit.ACKACT = 1;
      I2C_sendStop();
      state = I2C_STATUS_OK;
    }
    if (bytesToReceive) {
      // PMA - I think Smart Mode means we have nothing to do...
      // More bytes to receive, issue ack and start another read
    }
    else
    {
      // Transaction finished, issue NACK and STOP.
      s->I2CM.CTRLB.bit.ACKACT = 1;
      I2C_sendStop();
      state = I2C_STATUS_OK;
    }
  }
}

#endif /* I2CMANAGER_STM32_H */
