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

#ifndef IO_AFRT7SEG_h
#define IO_AFRT7SEG_h


#include "IODevice.h"


/*!
    @brief  Class encapsulating the raw HT16K33 controller device.
*/
class Adafruit_LEDBackpack {
public:
  uint8_t _address;
static const uint8_t LED_ON = 1;  ///< GFX color of lit LED segments (single-color displays)
static const uint8_t LED_OFF = 0; ///< GFX color of unlit LED segments (single-color displays)

static const uint8_t HT16K33_BLINK_CMD = 0x80;       ///< I2C register for BLINK setting
static const uint8_t HT16K33_BLINK_DISPLAYON = 0x01; ///< I2C value for steady on
static const uint8_t HT16K33_BLINK_OFF = 0;          ///< I2C value for steady off
static const uint8_t HT16K33_BLINK_2HZ = 1;          ///< I2C value for 2 Hz blink
static const uint8_t HT16K33_BLINK_1HZ = 2;          ///< I2C value for 1 Hz blink
static const uint8_t HT16K33_BLINK_HALFHZ = 3;       ///< I2C value for 0.5 Hz blink

static const uint8_t HT16K33_CMD_BRIGHTNESS = 0xE0; ///< I2C register for BRIGHTNESS setting

static const uint8_t SEVENSEG_DIGITS = 5; ///< # Digits in 7-seg displays, plus NUL end
  /*!
    @brief  Constructor for HT16K33 devices.
  */
  Adafruit_LEDBackpack(void);

  /*!
    @brief  Start I2C and initialize display state (blink off, full
            brightness).
    @param  _addr  I2C address.
    @param  theWire  TwoWire bus reference to use.
    @return  true if successful, otherwise false

  */
  bool begin(uint8_t _addr = 0x70);

  /*!
    @brief  Turn display on or off
    @param  state  State: true = on, false = off
  */
  void setDisplayState(bool state);

  /*!
    @brief  Set display brightness.
    @param  b  Brightness: 0 (min) to 15 (max).
  */
  void setBrightness(uint8_t b);

  /*!
    @brief  Set display blink rate.
    @param  b  One of:
               HT16K33_BLINK_OFF       = no blinking
               HT16K33_BLINK_2HZ       = 2 Hz blink
               HT16K33_BLINK_1HZ       = 1 Hz blink
               HT16K33_BLINK_HALFHZ    = 0.5 Hz blink
  */
  void blinkRate(uint8_t b);

  /*!
    @brief  Issue buffered data in RAM to display.
  */
  void writeDisplay(void);

  /*!
    @brief  Clear display.
  */
  void clear(void);

  uint16_t displaybuffer[8]; ///< Raw display data

};


#define RAW_BITS 0 ///< Issue 7-segment data as raw bits

/*!
    @brief  Class for 7-segment numeric displays.
*/
class Adafruit_7segment : public Adafruit_LEDBackpack {
public:
  /*!
    @brief  Constructor for 7-segment numeric displays.
  */
  Adafruit_7segment(void);

  /*!
    @brief   Issue single character to display.
    @param   c Character to write (ASCII character, not numeric).
    @return  1 if character written, else 0 (non-ASCII characters).
  */
  size_t write(char c);

  /*!
    @brief   Write characters from buffer to display.
    @param   buffer Character array to write
    @param   size   Number of characters to write
    @return  Number of characters written
  */
  size_t write(const char *buffer, size_t size);

  /*!
    @brief  Print byte-size numeric value to 7-segment display.
    @param  c     Numeric value.
  */
  void print(char c);

  /*!
    @brief  Print unsigned byte-size numeric value to 7-segment display.
    @param  b     Numeric value.
    @param  base  Number base (default = RAW_BITS = raw bits)
  */
  void print(unsigned char b, int base = RAW_BITS);

  /*!
    @brief  Print integer value to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void print(int n, int base = DEC);

  /*!
    @brief  Print unsigned integer value to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void print(unsigned int n, int base = DEC);

  /*!
    @brief  Print long integer value to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void print(long n, int base = DEC);

  /*!
    @brief  Print unsigned long integer value to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void print(unsigned long n, int base = DEC);

  /*!
    @brief  Print double-precision float value to 7-segment display.
    @param  n       Numeric value.
    @param  digits  Fractional-part digits.
  */
  void print(double n, int digits = 2);

  /*!
    @brief  Print from a String object to 7-segment display.
    @param  c  String object, passed by reference.
  */
  void print(const String &c);

  /*!
    @brief  Print from a C-style string array to 7-segment display.
    @param  c  Array of characters.
  */
  void print(const char c[]);

  /*!
    @brief  Print byte-size numeric value w/newline to 7-segment display.
    @param  c     Numeric value.
  */
  void println(char c);

  /*!
    @brief  Print unsigned byte-size numeric value w/newline to 7-segment
            display.
    @param  b     Numeric value.
    @param  base  Number base (default = RAW_BITS = raw bits)
  */
  void println(unsigned char b, int base = RAW_BITS);

  /*!
    @brief  Print integer value with newline to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void println(int n, int base = DEC);

  /*!
    @brief  Print unsigned integer value with newline to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void println(unsigned int n, int base = DEC);

  /*!
    @brief  Print long integer value with newline to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void println(long n, int base = DEC);

  /*!
    @brief  Print unsigned long integer value w/newline to 7-segment display.
    @param  n     Numeric value.
    @param  base  Number base (default = DEC = base 10)
  */
  void println(unsigned long n, int base = DEC);

  /*!
    @brief  Print double-precision float value to 7-segment display.
    @param  n       Numeric value.
    @param  digits  Fractional-part digits.
  */
  void println(double n, int digits = 2);

  /*!
    @brief  Print from a String object w/newline to 7-segment display.
    @param  c  String object, passed by reference.
  */
  void println(const String &c);

  /*!
    @brief  Print from a C-style string array w/newline to 7-segment display.
    @param  c  Array of characters.
  */
  void println(const char c[]);

  /*!
    @brief  Print newline to 7-segment display (rewind position to start).
  */
  void println(void);

  /*!
    @brief  Write raw segment bits into display buffer.
    @param  x        Character position (0-3).
    @param  bitmask  Segment bits.
  */
  void writeDigitRaw(uint8_t x, uint8_t bitmask);

  /*!
    @brief  Set specific digit # to a numeric value.
    @param  x    Character position.
    @param  num  Numeric (not ASCII) value.
    @param  dot  If true, light corresponding decimal.
  */
  void writeDigitNum(uint8_t x, uint8_t num, bool dot = false);

  /*!
    @brief  Set specific digit # to a character value.
    @param  x    Character position.
    @param  c    Character (ASCII).
    @param  dot  If true, light corresponding decimal.
  */
  void writeDigitAscii(uint8_t x, uint8_t c, bool dot = false);

  /*!
    @brief  Set or unset colon segment.
    @param  state  'true' to enable colon, 'false' for off.
  */
  void drawColon(bool state);

  /*!
    @brief  General integer-printing function used by some of the print()
            variants.
    @param  n     Numeric value.
    @param  base  Base (2 = binary).
  */
  void printNumber(long n, uint8_t base = 2);

  /*!
    @brief  General float-printing function used by some of the print()
            variants.
    @param  n           Numeric value.
    @param  fracDigits  Fractional-part digits.
    @param  base        Base (default DEC = base 10).
  */
  void printFloat(double n, uint8_t fracDigits = 2, uint8_t base = DEC);

  /*!
    @brief  Light display segments in an error-indicating configuration.
  */
  void printError(void);

  /*!
    @brief  Issue colon-on directly to display (bypass buffer).
  */
  void writeColon(void);

private:
  uint8_t position; ///< Current print position, 0-3
};

/*!
    @brief  Class for four-digit alphanumeric displays.
*/
class Adafruit_AlphaNum4 : public Adafruit_LEDBackpack {
public:
  /*!
    @brief  Constructor for four-digit alphanumeric displays.
  */
  Adafruit_AlphaNum4(void);

  /*!
    @brief  Write single character of alphanumeric display as raw bits
            (not a general print function).
    @param  n        Character index (0-3).
    @param  bitmask  Segment bitmask.
  */
  void writeDigitRaw(uint8_t n, uint16_t bitmask);

  /*!
    @brief  Write single ASCII character to alphanumeric display.
    @param  n      Character index (0-3).
    @param  ascii  ASCII character.
    @param  dot    If true, also light corresponding dot segment.
  */
  void writeDigitAscii(uint8_t n, uint8_t ascii, bool dot = false);
};

class AFRT7seg : public IODevice {
public: 
  static const bool debug=false; 
  static void create(I2CAddress i2cAddress);
 
    
private:
  
  // Constructor
    AFRT7seg(I2CAddress i2cAddress);
    Adafruit_7segment matrix;
    void _begin() override;
    void _display() override;
    void _loop(unsigned long currentMicros) override;
};
 
#endif
