/*
 *  © 2024, Chris Harlow. All rights reserved.
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

/*
* The IO_DS1307 device driver is used to interface a standalone realtime clock. 
* The clock will announce every minute (which will trigger EXRAIL ONTIME events).
* Seconds, and Day/date info is ignored, except that the announced hhmm time
* will attempt to synchronize with the 0 seconds of the clock. 
* An analog read in EXRAIL (IFGTE(vpin, value) etc will check against the hh*60+mm time.
* The clock can be easily set by an analog write to the vpin using 24 hr clock time
* with the command <z vpin hh mm ss> 
*/

#include "IO_afrt7seg.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "CommandDistributor.h"


#ifndef _BV
#define _BV(bit) (1 << (bit)) ///< Bit-value if not defined by Arduino
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  } ///< 16-bit var swap
#endif

static const PROGMEM uint8_t sevensegfonttable[] = {

    0b00000000, // (space)
    0b10000110, // !
    0b00100010, // "
    0b01111110, // #
    0b01101101, // $
    0b11010010, // %
    0b01000110, // &
    0b00100000, // '
    0b00101001, // (
    0b00001011, // )
    0b00100001, // *
    0b01110000, // +
    0b00010000, // ,
    0b01000000, // -
    0b10000000, // .
    0b01010010, // /
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b00001001, // :
    0b00001101, // ;
    0b01100001, // <
    0b01001000, // =
    0b01000011, // >
    0b11010011, // ?
    0b01011111, // @
    0b01110111, // A
    0b01111100, // B
    0b00111001, // C
    0b01011110, // D
    0b01111001, // E
    0b01110001, // F
    0b00111101, // G
    0b01110110, // H
    0b00110000, // I
    0b00011110, // J
    0b01110101, // K
    0b00111000, // L
    0b00010101, // M
    0b00110111, // N
    0b00111111, // O
    0b01110011, // P
    0b01101011, // Q
    0b00110011, // R
    0b01101101, // S
    0b01111000, // T
    0b00111110, // U
    0b00111110, // V
    0b00101010, // W
    0b01110110, // X
    0b01101110, // Y
    0b01011011, // Z
    0b00111001, // [
    0b01100100, //
    0b00001111, // ]
    0b00100011, // ^
    0b00001000, // _
    0b00000010, // `
    0b01011111, // a
    0b01111100, // b
    0b01011000, // c
    0b01011110, // d
    0b01111011, // e
    0b01110001, // f
    0b01101111, // g
    0b01110100, // h
    0b00010000, // i
    0b00001100, // j
    0b01110101, // k
    0b00110000, // l
    0b00010100, // m
    0b01010100, // n
    0b01011100, // o
    0b01110011, // p
    0b01100111, // q
    0b01010000, // r
    0b01101101, // s
    0b01111000, // t
    0b00011100, // u
    0b00011100, // v
    0b00010100, // w
    0b01110110, // x
    0b01101110, // y
    0b01011011, // z
    0b01000110, // {
    0b00110000, // |
    0b01110000, // }
    0b00000001, // ~
    0b00000000, // del
};

static const PROGMEM uint16_t alphafonttable[] = {

    0b0000000000000001, 0b0000000000000010, 0b0000000000000100,
    0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
    0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
    0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
    0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
    0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
    0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
    0b0011101000000000, 0b0001011100000000,
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000001001, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,

};

void Adafruit_LEDBackpack::setDisplayState(bool state) {
  uint8_t buffer;
  if (state)
    buffer = HT16K33_BLINK_CMD | 1;
  else
    buffer = HT16K33_BLINK_CMD;
  I2CManager.write(_address, buffer, 1);
}

void Adafruit_LEDBackpack::setBrightness(uint8_t b) {
  if (b > 15)
    b = 15; // limit to max brightness
  uint8_t buffer = HT16K33_CMD_BRIGHTNESS | b;
  I2CManager.write(_address, buffer, 1);
}

void Adafruit_LEDBackpack::blinkRate(uint8_t b) {
  if (b > 3)
    b = 0; // turn off if not sure
  uint8_t buffer = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
  I2CManager.write(_address, buffer, 1);
}

Adafruit_LEDBackpack::Adafruit_LEDBackpack(void) {}

bool Adafruit_LEDBackpack::begin(uint8_t _addr) {
  _address = _addr;
  I2CManager.begin();
  // turn on oscillator
  uint8_t buffer[1] = {0x21};
  I2CManager.write(_address, buffer, 1);

  // internal RAM powers up with garbage/random values.
  // ensure internal RAM is cleared before turning on display
  // this ensures that no garbage pixels show up on the display
  // when it is turned on.
  clear();
  writeDisplay();

  blinkRate(HT16K33_BLINK_OFF);

  setBrightness(15); // max brightness

  return true;
}

void Adafruit_LEDBackpack::writeDisplay(void) {
  uint8_t buffer[17];

  buffer[0] = 0x00; // start at address $00

  for (uint8_t i = 0; i < 8; i++) {
    buffer[1 + 2 * i] = displaybuffer[i] & 0xFF;
    buffer[2 + 2 * i] = displaybuffer[i] >> 8;
  }

  I2CManager.write(_address, buffer, 17);
}

void Adafruit_LEDBackpack::clear(void) {
  for (uint8_t i = 0; i < 8; i++) {
    displaybuffer[i] = 0;
  }
}

/******************************* 7 SEGMENT OBJECT */

Adafruit_7segment::Adafruit_7segment(void) { position = 0; }

void Adafruit_7segment::print(const String &c) { write(c.c_str(), c.length()); }

void Adafruit_7segment::print(const char c[]) { write(c, strlen(c)); }

void Adafruit_7segment::print(char c) { write(c); }

void Adafruit_7segment::print(unsigned long n, int base) {
  if (base == 0)
    write(n);
  else
    printNumber(n, base);
}

void Adafruit_7segment::print(unsigned char b, int base) {
  print((unsigned long)b, base);
}

void Adafruit_7segment::print(int n, int base) { print((long)n, base); }

void Adafruit_7segment::print(unsigned int n, int base) {
  print((unsigned long)n, base);
}

void Adafruit_7segment::println(void) { position = 0; }

void Adafruit_7segment::println(const String &c) {
  print(c);
  println();
}

void Adafruit_7segment::println(const char c[]) {
  print(c);
  println();
}

void Adafruit_7segment::println(char c) {
  print(c);
  println();
}

void Adafruit_7segment::println(unsigned char b, int base) {
  print(b, base);
  println();
}

void Adafruit_7segment::println(int n, int base) {
  print(n, base);
  println();
}

void Adafruit_7segment::println(unsigned int n, int base) {
  print(n, base);
  println();
}

void Adafruit_7segment::println(long n, int base) {
  print(n, base);
  println();
}

void Adafruit_7segment::println(unsigned long n, int base) {
  print(n, base);
  println();
}

void Adafruit_7segment::println(double n, int digits) {
  print(n, digits);
  println();
}

void Adafruit_7segment::print(double n, int digits) { printFloat(n, digits); }

size_t Adafruit_7segment::write(char c) {

  uint8_t r = 0;

  if (c == '\n')
    position = 0;
  if (c == '\r')
    position = 0;

  if ((c >= ' ') && (c <= 127)) {
    writeDigitAscii(position, c);
    r = 1;
  }

  position++;
  if (position == 2)
    position++;

  return r;
}

size_t Adafruit_7segment::write(const char *buffer, size_t size) {
  size_t n = 0;

  while (n < size) {
    write(buffer[n]);
    n++;
  }

  // Clear unwritten positions
  for (uint8_t i = position; i < 5; i++) {
    writeDigitRaw(i, 0x00);
  }

  return n;
}

void Adafruit_7segment::writeDigitRaw(uint8_t d, uint8_t bitmask) {
  if (d > 4)
    return;
  displaybuffer[d] = bitmask;
}

void Adafruit_7segment::drawColon(bool state) {
  if (state)
    displaybuffer[2] = 0x2;
  else
    displaybuffer[2] = 0;
}

void Adafruit_7segment::writeColon(void) {
  uint8_t buffer[3];

  buffer[0] = 0x04; // start at address $02
  buffer[1] = displaybuffer[2] & 0xFF;
  buffer[2] = displaybuffer[2] >> 8;

  I2CManager.write(_address, buffer, 3);
}

void Adafruit_7segment::writeDigitNum(uint8_t d, uint8_t num, bool dot) {
  if (d > 4 || num > 15)
    return;

  if (num >= 10) { // Hex characters
    switch (num) {
    case 10:
      writeDigitAscii(d, 'a', dot);
      break;
    case 11:
      writeDigitAscii(d, 'B', dot);
      break;
    case 12:
      writeDigitAscii(d, 'C', dot);
      break;
    case 13:
      writeDigitAscii(d, 'd', dot);
      break;
    case 14:
      writeDigitAscii(d, 'E', dot);
      break;
    case 15:
      writeDigitAscii(d, 'F', dot);
      break;
    }
  }

  else
    writeDigitAscii(d, num + 48, dot); // use ASCII offset
}

void Adafruit_7segment::writeDigitAscii(uint8_t d, uint8_t c, bool dot) {
  if (d > 4)
    return;

  uint8_t font = pgm_read_byte(sevensegfonttable + c - 32);

  writeDigitRaw(d, font | (dot << 7));
}

void Adafruit_7segment::print(long n, int base) { printNumber(n, base); }

void Adafruit_7segment::printNumber(long n, uint8_t base) {
  printFloat(n, 0, base);
}

void Adafruit_7segment::printFloat(double n, uint8_t fracDigits, uint8_t base) {
  uint8_t numericDigits = 4; // available digits on display
  bool isNegative = false;   // true if the number is negative

  // is the number negative?
  if (n < 0) {
    isNegative = true; // need to draw sign later
    --numericDigits;   // the sign will take up one digit
    n *= -1;           // pretend the number is positive
  }

  // calculate the factor required to shift all fractional digits
  // into the integer part of the number
  double toIntFactor = 1.0;
  for (int i = 0; i < fracDigits; ++i)
    toIntFactor *= base;

  // create integer containing digits to display by applying
  // shifting factor and rounding adjustment
  uint32_t displayNumber = n * toIntFactor + 0.5;

  // calculate upper bound on displayNumber given
  // available digits on display
  uint32_t tooBig = 1;
  for (int i = 0; i < numericDigits; ++i)
    tooBig *= base;

  // if displayNumber is too large, try fewer fractional digits
  while (displayNumber >= tooBig) {
    --fracDigits;
    toIntFactor /= base;
    displayNumber = n * toIntFactor + 0.5;
  }

  // did toIntFactor shift the decimal off the display?
  if (toIntFactor < 1) {
    printError();
  } else {
    // otherwise, display the number
    int8_t displayPos = 4;

    for (uint8_t i = 0; displayNumber || i <= fracDigits; ++i) {
      bool displayDecimal = (fracDigits != 0 && i == fracDigits);
      writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
      if (displayPos == 2)
        writeDigitRaw(displayPos--, 0x00);
      displayNumber /= base;
    }

    // display negative sign if negative
    if (isNegative)
      writeDigitRaw(displayPos--, 0x40);

    // clear remaining display positions
    while (displayPos >= 0)
      writeDigitRaw(displayPos--, 0x00);
  }
}

void Adafruit_7segment::printError(void) {
  for (uint8_t i = 0; i < SEVENSEG_DIGITS; ++i) {
    writeDigitRaw(i, (i == 2 ? 0x00 : 0x40));
  }
}


void AFRT7seg::create(I2CAddress i2cAddress) {
    new AFRT7seg(i2cAddress);
  }
 
    
  // Constructor
    AFRT7seg::AFRT7seg(I2CAddress i2cAddress){
     _I2CAddress = i2cAddress;
     addDevice(this);
    }

void AFRT7seg::_begin()  {
  // Initialise  device and sync loop() to zero seconds
  Adafruit_7segment matrix = Adafruit_7segment();
  matrix.begin(_I2CAddress);
    _display(); 
}

// Processing loop to obtain clock time.
// This self-synchronizes to the next minute tickover
void AFRT7seg::_loop(unsigned long currentMicros) { 
  uint16_t curTime = CommandDistributor::retClockTime();
  uint16_t curMin = curTime % 60;
  uint16_t curHrs = curTime / 60;

    matrix.writeDigitNum(0, (curHrs / 10), true);
    matrix.writeDigitNum(1, curHrs % 10, true);
    matrix.drawColon(true);
    matrix.writeDigitNum(3, curMin / 10, true);
    matrix.writeDigitNum(4, curMin % 10, true);
  delayUntil(currentMicros + (100000));  
}


  // Display device driver info.
  void AFRT7seg::_display()  {
    auto tstamp=getTime();
    byte ss=tstamp%60;
    tstamp/=60;
    byte mm=tstamp%60;
    byte hh=tstamp/60;
    DIAG(F("Adafruit 7-segment backpack on I2C:%s %S"), 
      _I2CAddress.toString(),
     (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
