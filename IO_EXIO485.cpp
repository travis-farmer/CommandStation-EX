/*
 *  © 2024, Travis Farmer. All rights reserved.
 *  © 2024, https://github.com/appnostic-io/EXIO_SC16IS7XX_Arduino_Library
 * 
 *  This file is part of DCC++EX API
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

#include "IO_EXIO485.h"
#include "defines.h"

bool EXIO_SC16IS7XX::_initialized = false;

/*** REGISTERS *****************************************************/

/**
 * @brief writes to the register of the device via i2c or spi
 * @param channel
 * @param reg_addr
 * @param val
 */
void EXIO_SC16IS7XX::writeRegister(uint8_t reg_addr, uint8_t val)
{
  uint8_t buf[] = {reg_addr, val};
  I2CManager.write(device_address,buf,2);
}

/**
 * @brief reads a register from the device via i2c or spi
 * @param channel
 * @param reg_addr
 * @return
 */
uint8_t EXIO_SC16IS7XX::readRegister(uint8_t reg_addr)
{
  uint8_t write_buf[1] = {reg_addr};
  uint8_t read_buf[1];
  I2CManager.read(device_address, write_buf, 1, read_buf, 1);

  return read_buf[0];
}

/*** CONFIG *******************************************************/

/**
 * @brief sets the crystal frequency in hertz. nos8007 has a 14.7456MHz XTAL
 * @note not normally called for nos8007. defaults to 147456000 (Hz)
 * @param frequency
 */
void EXIO_SC16IS7XX::setCrystalFrequency(uint32_t frequency)
{
    crystal_frequency = frequency;
}

/**
 * @brief gets the xtal frequency in hertz.
 * @return
 */
uint32_t EXIO_SC16IS7XX::getCrystalFrequency()
{
    return crystal_frequency;
}

/*** DEVICE *******************************************************/

/**
 * @brief derived function to reset the device
 */
void EXIO_SC16IS7XX::resetDevice()
{
    uint8_t reg;

    reg = readRegister(SC16IS7XX_REG_IOCONTROL << 3);
    reg |= 0x08;
    writeRegister(SC16IS7XX_REG_IOCONTROL << 3, reg);
}

/*** I2C *********************************************************/

/**
 * @brief begins an i2c session for the target address
 * @param addr
 * @return
 */
bool EXIO_SC16IS7XX::begin_i2c(uint8_t addr)
{

    if ((addr >= 0x48) && (addr <= 0x57))
    {
        device_address = addr;
    }
    else
    {
        device_address = (addr >> 1);
    }

    if (_initialized == true)
    {
        return true; // i2c already running
    }

    I2CManager.begin();
    bool retVal = I2CManager.exists(device_address);
    return retVal;
}

/**
 * @brief shorthand method to start i2c as nos8007 default address
 * @return
 */
bool EXIO_SC16IS7XX::begin_i2c()
{
    return begin_i2c(0X90);
}


void EXIO_SC16IS7XX::setPortState(uint8_t state)
{
    writeRegister(SC16IS7XX_REG_IOSTATE << 3, state);
}

uint8_t EXIO_SC16IS7XX::getPortState()
{
    return readRegister(SC16IS7XX_REG_IOSTATE << 3);
}

void EXIO_SC16IS7XX::setPortMode(uint8_t mode)
{
    writeRegister(SC16IS7XX_REG_IODIR << 3, mode);
}

uint8_t EXIO_SC16IS7XX::getPortMode()
{
    return readRegister(SC16IS7XX_REG_IODIR << 3);
}

void EXIO_SC16IS7XX::setGPIOLatch(bool enabled)
{
    uint8_t tmp_iocontrol;

    tmp_iocontrol = readRegister(SC16IS7XX_REG_IOCONTROL << 3);
    if (enabled == false)
    {
        tmp_iocontrol &= 0xFE;
    }
    else
    {
        tmp_iocontrol |= 0x01;
    }
    writeRegister(SC16IS7XX_REG_IOCONTROL << 3, tmp_iocontrol);
}

/**
 * @brief constructor for SC16IS752
 * @param channel
 */
EXIO_SC16IS752::EXIO_SC16IS752(uint8_t channel)
{
    EXIO_SC16IS752::channel = channel;
    EXIO_SC16IS752::peek_flag = 0;
}

/**
 * @brief   slight modification of the register write function to
 *          allow for the separate channels of the SC16IS752
 * @note    uses EXIO_SC16IS7XX::writeRegister
 * @param channel 0 or 1
 * @param reg_addr
 * @param val
 */
void EXIO_SC16IS752::writeRegister(uint8_t channel, uint8_t reg_addr, uint8_t val)
{
    EXIO_SC16IS7XX::writeRegister((reg_addr << 3 | channel << 1), val);
}

/**
 * @brief   slight modification of the register read function to
 *          allow for the separate channels of the SC16IS752
 * @note    uses EXIO_SC16IS7XX::readRegister
 * @param channel
 * @param reg_addr
 * @return
 */
uint8_t EXIO_SC16IS752::readRegister(uint8_t channel, uint8_t reg_addr)
{
    return EXIO_SC16IS7XX::readRegister((reg_addr << 3 | channel << 1));
}

/*** DERIVED FUNCTIONS **********************************************/

/**
 * @brief tests the device to check if it is online
 * @return
 */
bool EXIO_SC16IS752::ping()
{
    writeRegister(SC16IS752_CHANNEL_A, SC16IS7XX_REG_SPR, 0x55);

    if (readRegister(SC16IS752_CHANNEL_A, SC16IS7XX_REG_SPR) == 0x55)
    {
        return true;
    }

    writeRegister(SC16IS752_CHANNEL_A, SC16IS7XX_REG_SPR, 0xAA);

    if (readRegister(SC16IS752_CHANNEL_A, SC16IS7XX_REG_SPR) == 0xAA)
    {
        return true;
    }

    writeRegister(SC16IS752_CHANNEL_B, SC16IS7XX_REG_SPR, 0x55);

    if (readRegister(SC16IS752_CHANNEL_B, SC16IS7XX_REG_SPR) == 0x55)
    {
        return true;
    }

    writeRegister(SC16IS752_CHANNEL_B, SC16IS7XX_REG_SPR, 0xAA);

    if (readRegister(SC16IS752_CHANNEL_B, SC16IS7XX_REG_SPR) == 0xAA)
    {
        return true;
    }

    return false;
}

/*** UART CONFIGURATION *****************************************/

/**
 * @brief enables fifo buffer
 * @param enabled
 */
void EXIO_SC16IS752::setFIFO(bool enabled)
{
    settings.fifo = enabled;

    uint8_t tmp_fcr;

    tmp_fcr = readRegister(channel, SC16IS7XX_REG_FCR);

    if (enabled == false)
    {
        tmp_fcr &= 0xFE;
    }
    else
    {
        tmp_fcr |= 0x01;
    }

    writeRegister(channel, SC16IS7XX_REG_FCR, tmp_fcr);
}

/**
 * @brief resets tx or rx fifo buffer
 * @param rx
 */
void EXIO_SC16IS752::resetFIFO(bool rx)
{
    uint8_t tmp_fcr;

    tmp_fcr = readRegister(channel, SC16IS7XX_REG_FCR);

    if (rx == false)
    {
        tmp_fcr |= 0x04;
    }
    else
    {
        tmp_fcr |= 0x02;
    }
    writeRegister(channel, SC16IS7XX_REG_FCR, tmp_fcr);
}

void EXIO_SC16IS752::setFIFOTriggerLevel(bool rx, uint8_t length)
{
    uint8_t tmp_reg;

    tmp_reg = readRegister(channel, SC16IS7XX_REG_MCR);
    tmp_reg |= 0x04;
    writeRegister(channel, SC16IS7XX_REG_MCR, tmp_reg); // SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

    tmp_reg = readRegister(channel, SC16IS7XX_REG_EFR);
    writeRegister(channel, SC16IS7XX_REG_EFR, tmp_reg | 0x10); // set ERF[4] to '1' to use the  enhanced features
    if (rx == false)
    {
        writeRegister(channel, SC16IS7XX_REG_TLR, length << 4); // Tx FIFO trigger level setting
    }
    else
    {
        writeRegister(channel, SC16IS7XX_REG_TLR, length); // Rx FIFO Trigger level setting
    }
    writeRegister(channel, SC16IS7XX_REG_EFR, tmp_reg); // restore EFR register
}

/**
 * @brief sets the baud rate. nos8007 has been tested to 921600
 * @param baudRate
 */
void EXIO_SC16IS752::setBaudrate(uint32_t baudRate)
{
    settings.baud = baudRate;

    uint16_t divisor;
    uint8_t prescaler;
    uint8_t tmp_lcr;

    if ((readRegister(channel, SC16IS7XX_REG_MCR) & 0x80) == 0)
    {
        prescaler = 1;
    }
    else
    {
        prescaler = 4;
    }

    divisor = (getCrystalFrequency() / prescaler) / (baudRate * 16);

    tmp_lcr = readRegister(channel, SC16IS7XX_REG_LCR);
    tmp_lcr |= 0x80;
    writeRegister(channel, SC16IS7XX_REG_LCR, tmp_lcr);

    // write to DLL
    writeRegister(channel, SC16IS7XX_REG_DLL, (uint8_t)divisor);

    // write to DLH
    writeRegister(channel, SC16IS7XX_REG_DLH, (uint8_t)(divisor >> 8));
    tmp_lcr &= 0x7F;
    writeRegister(channel, SC16IS7XX_REG_LCR, tmp_lcr);
}

/**
 * @brief sets the line parameters
 * @param bits
 * @param parity
 * @param stopBits
 */
void EXIO_SC16IS752::setLine(uint8_t bits, uint8_t parity, uint8_t stopBits)
{
    uint8_t tmp_lcr;

    settings.bits = bits;
    settings.parity = parity;
    settings.stopBits = stopBits;

    tmp_lcr = readRegister(channel, SC16IS7XX_REG_LCR);
    tmp_lcr &= 0xC0; // Clear the lower six bit of LCR (LCR[0] to LCR[5]

    // data bit length
    switch (settings.bits)
    {
    case 5:
        break;
    case 6:
        tmp_lcr |= 0x01;
        break;
    case 7:
        tmp_lcr |= 0x02;
        break;
    case 8:
        tmp_lcr |= 0x03;
        break;
    default:
        tmp_lcr |= 0x03;
        break;
    }

    // stop bits
    if (settings.stopBits == 2)
    {
        tmp_lcr |= 0x04;
    }

    // parity
    switch (parity)
    {
    case 0: // no parity
        break;
    case 1: // odd parity
        tmp_lcr |= 0x08;
        break;
    case 2: // even parity
        tmp_lcr |= 0x18;
        break;
    case 3: // force '1' parity
        tmp_lcr |= 0x03;
        break;
    case 4: // force '0' parity
        break;
    default:
        break;
    }

    writeRegister(channel, SC16IS7XX_REG_LCR, tmp_lcr);
}

uint8_t EXIO_SC16IS752::FIFOAvailableData()
{
    if (fifo_available == 0)
    {
        fifo_available = readRegister(channel, SC16IS7XX_REG_RXLVL);
    }
    return fifo_available;
}

uint8_t EXIO_SC16IS752::FIFOAvailableSpace()
{
    return readRegister(channel, SC16IS7XX_REG_TXLVL);
}

int EXIO_SC16IS752::read()
{
    volatile uint8_t val;

    if (FIFOAvailableData() == 0)
    {
        return -1;
    }
    else
    {
        if (fifo_available > 0)
        {
            --fifo_available;
        }
        val = readRegister(channel, SC16IS7XX_REG_RHR);
        return val;
    }
}

int EXIO_SC16IS752::available()
{
    return readRegister(channel, SC16IS7XX_REG_RXLVL);
}

int EXIO_SC16IS752::peek()
{
    if (peek_flag == 0)
    {
        peek_buf = read();
        if (peek_buf >= 0)
        {
            peek_flag = 1;
        }
    }

    return peek_buf;
}

size_t EXIO_SC16IS752::write(uint8_t val)
{
    uint8_t tmp_lsr;

    do
    {
        tmp_lsr = readRegister(channel, SC16IS7XX_REG_LSR);
    } while ((tmp_lsr & 0x20) == 0);

    writeRegister(channel, SC16IS7XX_REG_THR, val);

    return 1;
}

size_t EXIO_SC16IS752::write(const uint8_t *buf, size_t size)
{
    for (int i = 0; i < size; i++)
    {
        write(buf[i]);
    }
    return size;
}

void EXIO_SC16IS752::flush()
{
    uint8_t tmp_lsr;

    do
    {
        tmp_lsr = readRegister(channel, SC16IS7XX_REG_LSR);
    } while ((tmp_lsr & 0x20) == 0);
}



static const byte PAYLOAD_FALSE = 0;
static const byte PAYLOAD_NORMAL = 1;
static const byte PAYLOAD_STRING = 2;


/************************************************************
 * EXIO485 implementation
 ************************************************************/

// Constructor for EXIO485
EXIO485::EXIO485(uint8_t busNo, uint8_t i2c_addr, unsigned long baud, uint32_t xtal_freq) {
  _i2c_addr = i2c_addr;
  _xtal_freq = xtal_freq;
  _baud = baud;
  
  _busNo = busNo;
  _retryTime = 2000000UL; // 1 second
  bufferLength=0;
  inCommandPayload=PAYLOAD_FALSE;
  // Add device to HAL device chain
  IODevice::addDevice(this);
  
  // Add bus to EXIO485 chain.
  _nextBus = _busList;
  _busList = this;
}

// CRC-16 implementation
uint16_t EXIO485::crc16(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      bool bit = ((crc & 0x0001) != 0);
      crc >>= 1;
      if (bit) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}


/* -= _loop =-
//
// Main loop function for EXIO485.
// Work through list of nodes.  For each node, in separate loop entries
// When the slot time has finished, move on to the next device.
*/

void EXIO485::_loop(unsigned long currentMicros) {
  _currentMicros = currentMicros;
  if (_currentNode == NULL) _currentNode = _nodeListStart;
  if (!hasTasks() && _currentNode->isInitialised()) { // no tasks? lets poll for data
    uint8_t buffA[3];
    buffA[0] = (_currentNode->getNodeID());
    buffA[1] = (0);
    buffA[2] = (EXIORDD);
    addTask(buffA, 3, EXIORDD);
    uint8_t buffB[3];
    buffB[0] = (_currentNode->getNodeID());
    buffB[1] = (0);
    buffB[2] = (EXIORDAN);
    addTask(buffB, 3, EXIORDAN);
    _currentNode = _currentNode->getNext();
  }
  
  if ( hasTasks()){ // do we have any tasks on the docket
    
    if (CurrentTaskID == -1) CurrentTaskID = getNextTaskId();
    Task* currentTask = getTaskById(CurrentTaskID);
    if (currentTask == nullptr) return; // dead task

    if (!currentTask->completed && _currentMicros - _cycleStartTimeA >= _retryTime) { // after CRC pass, timer is reset
      
      if (currentTask->currentRetryTimer == 0UL) { // first trigger
        currentTask->currentRetryTimer = _currentMicros; // set timer
      } else if (_currentMicros - currentTask->currentRetryTimer >= _retryTime) {
        currentTask->currentRetryTimer = _currentMicros; // reset timer
        currentTask->rxMode = false; // resend data
        DIAG(F("EX-IOExplorer485: Fail RX, Retry TX. Task: %d"), CurrentTaskID);
      }
    }

    if (!currentTask->rxMode) {
      currentTask->crcPassFail = 0;
      uint16_t response_crc = crc16((uint8_t*)currentTask->commandArray, currentTask->byteCount-1);
      
      // Send response data with CRC
      ExtSerialA.write(0xFE);
      ExtSerialA.write(0xFE);
      ExtSerialA.write(response_crc >> 8);
      ExtSerialA.write(response_crc & 0xFF);
      ExtSerialA.write(currentTask->byteCount);
      for (int i = 0; i < currentTask->byteCount; i++) {
        ExtSerialA.write(currentTask->commandArray[i]);
      }
      ExtSerialA.write(0xFD);
      ExtSerialA.write(0xFD);
      ExtSerialA.flush();
      // delete task command after sending, for now
      currentTask->rxMode = true;
      
    } else {
      if ( ExtSerialA.available()) {
        int curByte = ExtSerialA.read();
        
        if (curByte == 0xFE && flagStart == false) flagStart = true;
        else if ( curByte == 0xFE && flagStart == true) {
          flagProc = false;
          byteCounter = 0;
          flagStarted = true;
          flagStart = false;
          flagEnded = false;
          rxStart = true;
          rxEnd = false;
          crcPass = false;
          memset(received_data, 0, ARRAY_SIZE);
        }else if (flagStarted) {
          crc[0] = curByte;
          byteCounter++;
          flagStarted = false;
        } else if (byteCounter == 1) {
          crc[1] = curByte;
          received_crc  = (crc[0] << 8) | crc[1];
          byteCounter++;
        } else if (byteCounter == 2) {
          byteCount = curByte;
          byteCounter++;
        } else if (flagEnded == false && byteCounter >= 3) {
          received_data[byteCounter-3] = curByte;
          byteCounter++;
        }
        if (curByte == 0xFD && flagEnd == false) flagEnd = true;
        else if ( curByte == 0xFD && flagEnd == true) {
          flagEnded = true;
          flagEnd = false;
          rxEnd = true;
          byteCount = byteCounter;
          byteCounter = 0;
        }
        if (flagEnded) {
          calculated_crc = crc16((uint8_t*)received_data, byteCount-6);
          if (received_crc == calculated_crc) {
            crcPass = true;
          }
          flagEnded = false;
        }
      }
      // Check CRC validity
      if (crcPass) {
        // Data received successfully, process it (e.g., print)
        int nodeTo = received_data[0];
        if (nodeTo == 0) { // for master.
          flagProc = true;
          
        }
      }
      
      if (flagProc) {
        crcPass = false;
        int nodeFr = received_data[1];
        EXIO485node *node = findNode(nodeFr);
        int AddrCode = received_data[2];
        
        switch (AddrCode) {
          case EXIOPINS:
            {node->setnumDigitalPins(received_data[3]);
            node->setnumAnalogPins(received_data[4]);

            // See if we already have suitable buffers assigned
            if (node->getnumDigialPins()>0) {
              size_t digitalBytesNeeded = (node->getnumDigialPins() + 7) / 8;
              if (node->getdigitalPinBytes() < digitalBytesNeeded) {
                // Not enough space, free any existing buffer and allocate a new one
                if (node->cleandigitalPinStates(digitalBytesNeeded)) {
                  node->setdigitalPinBytes(digitalBytesNeeded);
                } else {
                  DIAG(F("EX-IOExpander485 node:%d ERROR alloc %d bytes"), nodeFr, digitalBytesNeeded);
                  node->setdigitalPinBytes(0);
                }
              }
            }
            
            if (node->getnumAnalogPins()>0) {
              size_t analogueBytesNeeded = node->getnumAnalogPins() * 2;
              if (node->getanalogPinBytes() < analogueBytesNeeded) {
                // Free any existing buffers and allocate new ones.
                
                if (node->cleanAnalogStates(analogueBytesNeeded)) {
                  node->setanalogPinBytes(analogueBytesNeeded);
                } else {
                  DIAG(F("EX-IOExpander485 node:%d ERROR alloc analog pin bytes"), nodeFr);
                  node->setanalogPinBytes(0);
                }
              }
            }
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;}
          case EXIOINITA: {
            for (int i = 0; i < node->getnumAnalogPins(); i++) {
              node->setanalogPinMap(received_data[i+3], i);
            }
            
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;
          }
          case EXIOVER: {
            node->setMajVer(received_data[3]);
            node->setMinVer(received_data[4]);
            node->setPatVer(received_data[5]);
            DIAG(F("EX-IOExpander485: Found node %d v%d.%d.%d"),node->getNodeID(), node->getMajVer(), node->getMinVer(), node->getPatVer());
            node->setInitialised();
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;
          }
          case EXIORDY: {
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;
          }
          case EXIOERR: {
            markTaskCompleted(CurrentTaskID);
            DIAG(F("EX-IOExplorer485: Some sort of error was received...")); // ;-)
            flagProc = false;
            break;
          }
          case EXIORDAN: {
            for (int i = 0; i < node->_numAnaloguePins; i++) {
              node->setanalogInputBuffer(received_data[i+3], i);
            }
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;
          }
          case EXIORDD: {
            for (int i = 0; i < (node->_numDigitalPins+7)/8; i++) {
              node->setdigitalInputStates(received_data[i+3], i);
            }
            markTaskCompleted(CurrentTaskID);
            flagProc = false;
            break;
          }
        }
        _cycleStartTimeA = currentMicros; // reset timer so we do not resend data
      }
    }
  }
}

// Link to chain of EXIO485 instances, left over from EXIO485 template.
EXIO485 *EXIO485::_busList = NULL;


/************************************************************
 * EXIO485node implementation
 ************************************************************/

/* -= EXIO485node =-
//
// Constructor for EXIO485node object
*/
EXIO485node::EXIO485node(VPIN firstVpin, int nPins, uint8_t nodeID) {
  _firstVpin = firstVpin;
  _nPins = nPins;
  _busNo = 0;
  _nodeID = nodeID;
  _initialised = false;
  memset(resFlag, 0, 255);
  if (_nodeID > 252) _nodeID = 252; // cannot have a node with the frame flags
  if (_nodeID < 1) _nodeID = 1; // cannot have a node with the master ID

  // Add this device to HAL device list
  IODevice::addDevice(this);
  _display();
  // Add EXIO485node to EXIO485 object.
  EXIO485 *bus = EXIO485::findBus(_busNo);
  if (bus != NULL) {
    bus->addNode(this);
    return;
  }
  
}

bool EXIO485node::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
    if (paramCount != 1) return false;
    int pin = vpin - _firstVpin;
    
    uint8_t pullup = (uint8_t)params[0];
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIODPUP);
    buff[3] = (pin);
    buff[4] = (pullup);
    EXIO485 *bus = EXIO485::findBus(0);
    bus->setBusy();
    bus->addTask(buff, 5, EXIODPUP);
    
    return true;
  }

  int EXIO485node::_configureAnalogIn(VPIN vpin) {
    int pin = vpin - _firstVpin;
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOENAN);
    buff[3] = (pin);
    buff[4] = lowByte(_firstVpin);
    buff[5] = highByte(_firstVpin);
    EXIO485 *bus = EXIO485::findBus(0);
    bus->setBusy();
    bus->addTask(buff, 6, EXIOENAN);
    
    return false;
  }

void EXIO485node::_begin() {
  uint8_t buff[ARRAY_SIZE];
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOINIT);
  buff[3] = (_nPins);
  buff[4] = ((_firstVpin & 0xFF));
  buff[5] = ((_firstVpin >> 8));
  EXIO485 *bus = EXIO485::findBus(0);
  bus->setBusy();
  bus->addTask(buff, 6, EXIOINIT);
  
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOINITA);
  bus->setBusy();
  bus->addTask(buff, 3, EXIOINITA);
  
  buff[0] = (_nodeID);
  buff[1] = (0);
  buff[2] = (EXIOVER);
   bus->setBusy();
  bus->addTask(buff, 3, EXIOVER);

#ifdef DIAG_IO
  _display();
#endif
}

int EXIO485node::_read(VPIN vpin) {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }
void EXIO485node::_write(VPIN vpin, int value) {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOWRD);
    buff[3] = (pin);
    buff[4] = (value);
    EXIO485 *bus = EXIO485::findBus(0);
    bus->setBusy();
    bus->addTask(buff, 5, EXIOWRD);
    
  }

  int EXIO485node::_readAnalogue(VPIN vpin) {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    for (uint8_t aPin = 0; aPin < _numAnaloguePins; aPin++) {
      if (_analoguePinMap[aPin] == pin) {
        uint8_t _pinLSBByte = aPin * 2;
        uint8_t _pinMSBByte = _pinLSBByte + 1;
        return (_analogueInputStates[_pinMSBByte] << 8) + _analogueInputStates[_pinLSBByte];
      }
    }
    return -1;  // pin not found in table
  }

  void EXIO485node::_writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) {
    int pin = vpin - _firstVpin;
    uint8_t buff[ARRAY_SIZE];
    buff[0] = (_nodeID);
    buff[1] = (0);
    buff[2] = (EXIOWRAN);
    buff[3] = (pin);
    buff[4] = lowByte(value);
    buff[5] = highByte(value);
    buff[6] = (profile);
    buff[7] = lowByte(duration);
    buff[8] = highByte(duration);
    EXIO485 *bus = EXIO485::findBus(0);
    bus->setBusy();
    bus->addTask(buff, 9, EXIOWRAN);
    
  }