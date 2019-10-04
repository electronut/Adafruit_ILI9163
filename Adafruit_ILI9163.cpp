/*!
* @file Adafruit_ILI9163.cpp
*
* @mainpage Adafruit ILI9163 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for Adafruit's ILI9163 driver for the
* Arduino platform.
*
* These displays use SPI to communicate, 4 or 5 pins are required
* to interface (RST is optional).
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include "Adafruit_ILI9163.h"
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #ifndef RASPI
    #include "wiring_private.h"
  #endif
#endif
#include <limits.h>

#if defined (ARDUINO_ARCH_ARC32) || defined (ARDUINO_MAXIM)
  #define SPI_DEFAULT_FREQ  16000000
// Teensy 3.0, 3.1/3.2, 3.5, 3.6
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined (__AVR__) || defined(TEENSYDUINO)
  #define SPI_DEFAULT_FREQ  8000000
#elif defined(ESP8266) || defined(ESP32)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined(RASPI)
  #define SPI_DEFAULT_FREQ  80000000
#elif defined(ARDUINO_ARCH_STM32F1)
  #define SPI_DEFAULT_FREQ  36000000
#else
  #define SPI_DEFAULT_FREQ  24000000  ///< Default SPI data clock frequency
#endif

#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9163 driver with software SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ILI9163::Adafruit_ILI9163(int8_t cs, int8_t dc, int8_t mosi,
        int8_t sclk, int8_t rst, int8_t miso) : Adafruit_SPITFT(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT, cs, dc, mosi, sclk, rst, miso) {
}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9163 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_ILI9163::Adafruit_ILI9163(int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_SPITFT(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT, cs, dc, rst) {
}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9163 driver with hardware SPI using
            a specific SPI peripheral (not necessarily default).
    @param  spiClass  Pointer to SPI peripheral (e.g. &SPI or &SPI1).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and
                      CS is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_ILI9163::Adafruit_ILI9163(
  SPIClass *spiClass, int8_t dc, int8_t cs, int8_t rst) :
  Adafruit_SPITFT(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT, spiClass, cs, dc, rst) {
}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9163 driver using parallel interface.
    @param  busWidth  If tft16 (enumeration in Adafruit_SPITFT.h), is a
                      16-bit interface, else 8-bit.
    @param  d0        Data pin 0 (MUST be a byte- or word-aligned LSB of a
                      PORT register -- pins 1-n are extrapolated from this).
    @param  wr        Write strobe pin # (required).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and CS
                      is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
    @param  rd        Read strobe pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_ILI9163::Adafruit_ILI9163(tftBusWidth busWidth,
  int8_t d0, int8_t wr, int8_t dc, int8_t cs, int8_t rst, int8_t rd) :
  Adafruit_SPITFT(ILI9163_TFTWIDTH, ILI9163_TFTHEIGHT, busWidth,
    d0, wr, dc, cs, rst, rd) {
}


// static const uint8_t PROGMEM initcmd[] = {
//   0x01, 0x80, 0x80,
//   0x11, 0x80, 0x05,
//   0x3a, 0x01, 0x05,
//   0x26, 0x01, 0x04,
//   0xf2, 0x01, 0x01,
//   0xe0, 0x0f, 0x3f, 0x25, 0x1c, 0x1e, 0x20, 0x12, 0x2a, 0x90, 0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
//   0xe1, 0x0f, 0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xa7, 0x3d, 0x18, 0x25, 0x2a, 0x2b, 0x2b, 0x3a,
//   0xb1, 0x02, 0x08, 0x08,
//   0xb4, 0x01, 0x07,
//   0xc0, 0x02, 0x0a, 0x02,
//   0xc1, 0x01, 0x02,
//   0xc5, 0x02, 0x50, 0x5b,
//   0xc7, 0x01, 0x40,
//   0x2a, 0x04, 0x00, 0x00, 0x00, 0x7f,
//   0x2b, 0x04, 0x00, 0x00, 0x00, 0x7f,
//   0x36, 0x01, 0x68,      // rotation
//   0x29, 0x80, 0x78,
//   0x2c, 0x80, 0x78,
//   0x00                                   // End of list
// };


/**************************************************************************/
/*!
    @brief   Initialize ILI9163 chip
    Connects to the ILI9163 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_ILI9163::begin(uint32_t freq) {

  uint8_t data[12];

  if(!freq) freq = SPI_DEFAULT_FREQ;
  initSPI(freq);

  if(_rst < 0) {                     // If no hardware reset pin...
      sendCommand(ILI9163_SWRESET); // Engage software reset
      delay(150);
  }

  sendCommand(ILI9163_SLPOUT);
  delay(5);

  data[0] = (0x05); // 16bpp
  sendCommand(ILI9163_PIXFMT, data, 1);

  data[0] = (0x04); // Select gamma curve 3
  sendCommand(ILI9163_GAMMASET, data, 1);

  // data[0] = (0x01); // Gamma adjustment enabled
  // sendCommand(CMD_GAM_R_SEL, data, 1);

  data[0] = (0x3f); // 1st Parameter
  data[1] = (0x25); // 2nd Parameter
  data[2] = (0x1c); // 3rd Parameter
  data[3] = (0x1e); // 4th Parameter
  data[4] = (0x20); // 5th Parameter
  data[5] = (0x12); // 6th Parameter
  data[6] = (0x2a); // 7th Parameter
  data[7] = (0x90); // 8th Parameter
  data[8] = (0x24); // 9th Parameter
  data[9] = (0x11); // 10th Parameter
  data[10] = (0x00); // 11th Parameter
  data[11] = (0x00); // 12th Parameter
  data[12] = (0x00); // 13th Parameter
  data[13] = (0x00); // 14th Parameter
  data[14] = (0x00); // 15th Parameter
  sendCommand(ILI9163_GMCTRP1, data, 15);
  

  data[0] = (0x20); // 1st Parameter
  data[1] = (0x20); // 2nd Parameter
  data[2] = (0x20); // 3rd Parameter
  data[3] = (0x20); // 4th Parameter
  data[4] = (0x05); // 5th Parameter
  data[5] = (0x00); // 6th Parameter
  data[6] = (0x15); // 7th Parameter
  data[7] = (0xa7); // 8th Parameter
  data[8] = (0x3d); // 9th Parameter
  data[9] = (0x18); // 10th Parameter
  data[10] = (0x25); // 11th Parameter
  data[11] = (0x2a); // 12th Parameter
  data[12] = (0x2b); // 13th Parameter
  data[13] = (0x2b); // 14th Parameter
  data[14] = (0x3a); // 15th Parameter
  sendCommand(ILI9163_GMCTRN1, data, 15);

  data[0] = (0x08); // DIVA = 8
  data[1] = (0x08); // VPA = 8
  sendCommand(ILI9163_FRMCTR1, data, 2);

  data[0] = (0x07); // NLA = 1, NLB = 1, NLC = 1 (all on Frame Inversion)
  sendCommand(ILI9163_INVCTR, data, 1);

  data[0] = (0x0a); // VRH = 10:  GVDD = 4.30
  data[1] = (0x02); // VC = 2: VCI1 = 2.65
  sendCommand(ILI9163_PWCTR1, data, 2);

  data[0] = (0x02); // BT = 2: AVDD = 2xVCI1, VCL = -1xVCI1, VGH = 5xVCI1, VGL = -2xVCI1
  sendCommand(ILI9163_PWCTR2, data, 1);

  data[0] = (0x50); // VMH = 80: VCOMH voltage = 4.5
  data[1] = (0x5b); // VML = 91: VCOML voltage = -0.225
  sendCommand(ILI9163_VMCTR1, data, 2);

  data[0] = (0x40); // nVM = 0, VMF = 64: VCOMH output = VMH, VCOML output = VML
  sendCommand(ILI9163_VMCTR2, data, 1);

  data[0] = (0x00); // XSH
  data[1] = (0x00); // XSL
  data[2] = (0x00); // XEH
  data[3] = (0x7f); // XEL (128 pixels x)
  sendCommand(ILI9163_CASET, data, 4);

  data[0] = (0x00);
  data[1] = (0x00);
  data[2] = (0x00);
  data[3] = (0x7f); // 128 pixels y
  sendCommand(ILI9163_PASET, data, 4);

  // TODO set rotation
  data[0] = (0xA0 | 0x08);
  sendCommand(ILI9163_MADCTL, data, 1);

  // Set the display to on
  sendCommand(ILI9163_DISPON);
  sendCommand(ILI9163_RAMWR);

  _width  = ILI9163_TFTWIDTH;
  _height = ILI9163_TFTHEIGHT;
}


/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ILI9163::setRotation(uint8_t m) {
    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            m = (MADCTL_MX | MADCTL_BGR);
            _width  = ILI9163_TFTWIDTH;
            _height = ILI9163_TFTHEIGHT;
            break;
        case 1:
            m = (MADCTL_MV | MADCTL_BGR);
            _width  = ILI9163_TFTHEIGHT;
            _height = ILI9163_TFTWIDTH;
            break;
        case 2:
            m = (MADCTL_MY | MADCTL_BGR);
            _width  = ILI9163_TFTWIDTH;
            _height = ILI9163_TFTHEIGHT;
            break;
        case 3:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            _width  = ILI9163_TFTHEIGHT;
            _height = ILI9163_TFTWIDTH;
            break;
    }

    sendCommand(ILI9163_MADCTL, &m, 1);
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_ILI9163::invertDisplay(bool invert) {
    sendCommand(invert ? ILI9163_INVON : ILI9163_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void Adafruit_ILI9163::scrollTo(uint16_t y) {
    uint8_t data[2];
    data[0] = y >> 8;
    data[1] = y & 0xff;
    sendCommand(ILI9163_VSCRSADD, (uint8_t*) data, 2);
}

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void Adafruit_ILI9163::setScrollMargins(uint16_t top, uint16_t bottom) {
  // TFA+VSA+BFA must equal 320
  if (top + bottom <= ILI9163_TFTHEIGHT) {
    uint16_t middle = ILI9163_TFTHEIGHT - top + bottom;
    uint8_t data[6];
    data[0] = top >> 8;
    data[1] = top & 0xff;
    data[2] = middle >> 8;
    data[3] = middle & 0xff;
    data[4] = bottom >> 8;
    data[5] = bottom & 0xff;
    sendCommand(ILI9163_VSCRDEF, (uint8_t*) data, 6);
  }
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with the next chunk of      SPI data writes. The ILI9163 will automatically wrap the data as each row is filled
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void Adafruit_ILI9163::setAddrWindow(
  uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = (x1 + w - 1),
             y2 = (y1 + h - 1);
    writeCommand(ILI9163_CASET); // Column address set
    SPI_WRITE16(x1);
    SPI_WRITE16(x2);
    writeCommand(ILI9163_PASET); // Row address set
    SPI_WRITE16(y1);
    SPI_WRITE16(y2);
    writeCommand(ILI9163_RAMWR); // Write to RAM
}

/**************************************************************************/
/*!
    @brief  Read 8 bits of data from ILI9163 configuration memory. NOT from RAM!
            This is highly undocumented/supported, it's really a hack but kinda works?
    @param    commandByte  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ILI9163 register
 */
/**************************************************************************/
uint8_t Adafruit_ILI9163::readcommand8(uint8_t commandByte, uint8_t index) {
  uint8_t data = 0x10 + index;
  sendCommand(0xD9, &data, 1); // Set Index Register
  return Adafruit_SPITFT::readcommand8(commandByte);
}
