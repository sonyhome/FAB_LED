////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This is an example use of the FAB_LED library using the predefined pixel
/// structures rgb. It will light up the LEDs randomly with a color.
///
/// This example works for an Arduino Uno board connected to your PC via the
/// USB port to the Arduino IDE (integrated development environment used to
/// compile and load an arduino sketch program to your arduino board). For
/// other boards, you may have to change the port you are using.
///
/// Wiring:
///
/// The LED strips DI (data input) line should be on ports D0 to D7 (Digital
/// pins 0 to 7 on Arduino Uno).
/// The LED power (GND) and (+5V) should be connected on the Arduino Uno's GND
/// and +5V.
///
/// LED strip type: By default WS2812B LED code is uncommented in loop(). If
/// your LED strip is another type, comment that and uncomment the function
/// name that matches your LED strip.
///
/// Visual results:
///
/// If you use the proper LED type, your LED strip will light up in sequence
/// red, green, blue then white before turning off.
/// If you picked the wrong one, the order will be different, or may be
/// jumbled random colors. If so try another.
///
/// Details of this test:
///
/// This test is going to send each type of pixel array to your LED strip.
/// If the array format is the same as the LED strip (native order), the array
/// is send as-is and will display correctly.
/// If the array format is not the same as the LED strip, the library will
/// convert the array data to be sent in the right order (it send each color
/// separately for each pixel). This is slower to do, and in some cases the LED
/// strip display might become buggy, if the time it takes to do the conversion
/// takes too long for the LED strip. This should not happen but it could!
/// In other words, even in this case the array will display correctly. :)
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#pragma GCC optimize ("-O2")
#include <FAB_LED.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief This parameter says how many LEDs will be lit up in your strip.
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs at
/// once. Maybe no more than 8 at full power (max is 60mA at 5V is 0.3Watt/LED).
const uint16_t numPixels = 8*8;
const uint16_t maxBrightness = 16;

// Custom LED declaration
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
class custom : public avrBitbangLedStrip<
  6, // 1 high, CPU cycles
  2,  // 1 low
  2,  // 0 high
  2, // 0 low
  50, // refresh msec
  dataPortId, // data port
  dataPortBit,
  A, // clock port for SPI protocol (unused if 3-wire LED)
  0,
  GRB, // byte order for each pixel
  ONE_WIRE_BITBANG> // protocol
{
  public:
  custom() : avrBitbangLedStrip<6,2,2,2,50,dataPortId,dataPortBit,A,0,GRB,ONE_WIRE_BITBANG>() {};
  ~custom() {};
};
////////////////////////////////////////////////////////////////////////////////
// Declares the LED strip you'll write to. You can change this to the right
// LED strip model you own.

#define myLEDtype custom
//#define myLEDtype ws2812b
// #define myLEDtype apa102
myLEDtype<D,1> strip0;
myLEDtype<D,1> strip1;
myLEDtype<D,2> strip2;
myLEDtype<D,3> strip3;
myLEDtype<D,4> strip4;
myLEDtype<D,5> strip5;
myLEDtype<D,6> strip6;
myLEDtype<D,1> strip7;

////////////////////////////////////////////////////////////////////////////////
// We define the pixel array we'll store the actual red, green, blue values in.
// Note this library supports many pixel structures, and it is best to use the
// structure that holds the colors in the same order as your LED strip model.
// If you change the LED type make sure you use the proper byte order here.
grb  grbPixels[numPixels] = {};

////////////////////////////////////////////////////////////////////////////////
// Sends an array at the same time to all 8 channels of a given port. Note this
// method does not use the LED protocol classes.
void parallelPixel(avrLedStripPort dataPortId, const uint16_t arraySize, const uint8_t * array)
{
  const uint8_t oldSREG = SREG;
  __builtin_avr_cli();
  for(uint16_t j = 0; j < arraySize; j++) {
    const uint8_t val = array[j];
    for(int8_t b=7; b>=0; b--) {
      const bool bit = (val>>b) & 0x1;
 
      if (bit) {
        SET_PORT_HIGH(dataPortId, 0);
        SET_PORT_HIGH(dataPortId, 1);
        SET_PORT_HIGH(dataPortId, 2);
        SET_PORT_HIGH(dataPortId, 3);
        SET_PORT_HIGH(dataPortId, 4);
        SET_PORT_HIGH(dataPortId, 5);
        SET_PORT_HIGH(dataPortId, 6);
        SET_PORT_HIGH(dataPortId, 7);
        SET_PORT_LOW(dataPortId, 0);
        SET_PORT_LOW(dataPortId, 1);
        SET_PORT_LOW(dataPortId, 2);
        SET_PORT_LOW(dataPortId, 3);
        SET_PORT_LOW(dataPortId, 4);
        SET_PORT_LOW(dataPortId, 5);
        SET_PORT_LOW(dataPortId, 6);
        SET_PORT_LOW(dataPortId, 7);
      } else {
        SET_PORT_HIGH(dataPortId, 0);
        SET_PORT_LOW(dataPortId, 0);
        SET_PORT_HIGH(dataPortId, 1);
        SET_PORT_LOW(dataPortId, 1);
        SET_PORT_HIGH(dataPortId, 2);
        SET_PORT_LOW(dataPortId, 2);
        SET_PORT_HIGH(dataPortId, 3);
        SET_PORT_LOW(dataPortId, 3);
        SET_PORT_HIGH(dataPortId, 4);
        SET_PORT_LOW(dataPortId, 4);
        SET_PORT_HIGH(dataPortId, 5);
        SET_PORT_LOW(dataPortId, 5);
        SET_PORT_HIGH(dataPortId, 6);
        SET_PORT_LOW(dataPortId, 6);
        SET_PORT_HIGH(dataPortId, 7);
        SET_PORT_LOW(dataPortId, 7);
      }
    }
  }
  SREG = oldSREG;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Turn off up to 1000 LEDs.
  strip1.clear(1000);
  strip2.clear(1000);
  strip3.clear(1000);
  strip4.clear(1000);
  strip5.clear(1000);
  strip6.clear(1000);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Update the LED strips one after the other
  for(uint8_t i = 0; i < 2*numPixels; i++) {
    grbPixels[i%numPixels].r = random(maxBrightness);
    grbPixels[i%numPixels].g = random(maxBrightness);
    grbPixels[i%numPixels].b = random(maxBrightness);

    strip0.sendPixels(numPixels, grbPixels);
    strip1.sendPixels(numPixels, grbPixels);
    strip2.sendPixels(numPixels, grbPixels);
    strip3.sendPixels(numPixels, grbPixels);
    strip4.sendPixels(numPixels, grbPixels);
    strip5.sendPixels(numPixels, grbPixels);
    strip6.sendPixels(numPixels, grbPixels);
    strip7.sendPixels(numPixels, grbPixels);
  }
  delay(1000);

  // Turn off up to 1000 LEDs.
  strip0.clear(1000);
  strip1.clear(1000);
  strip2.clear(1000);
  strip3.clear(1000);
  strip4.clear(1000);
  strip5.clear(1000);
  strip6.clear(1000);
  strip7.clear(1000);
  delay(250);

  // update LEDs all at the same time, but using low level code
  // to set ports directly. This works as we interlace at bit level.
  for(uint8_t i = 0; i < 2*numPixels; i++) {
    grbPixels[i%numPixels].r = random(maxBrightness);
    grbPixels[i%numPixels].g = random(maxBrightness);
    grbPixels[i%numPixels].b = random(maxBrightness);

    parallelPixel(D, 3*numPixels, (uint8_t*) grbPixels);
  }
  delay(1000);

  // Turn off up to 1000 LEDs.
  strip0.clear(1000);
  strip1.clear(1000);
  strip2.clear(1000);
  strip3.clear(1000);
  strip4.clear(1000);
  strip5.clear(1000);
  strip6.clear(1000);
  strip7.clear(1000);
  delay(250);
  
  // Update the LED strips at the same time.
  // This will start failing if your CPU is not fast enough when
  // you update too many LED strips at the same time.
  // Comment and uncomment the LED strips until you find how many
  // you can drive in parallel.
  // For ws2812b, it takes too many cycles to update a pixel byte
  // so the LED strip resets. A byte takes 8 * ( 6+2+?) > 64 cycles (4 usec)
  for(uint8_t i = 0; i < 2*numPixels; i++) {
    grbPixels[i%numPixels].r = random(maxBrightness);
    grbPixels[i%numPixels].g = random(maxBrightness);
    grbPixels[i%numPixels].b = random(maxBrightness);

   const uint8_t * array = (uint8_t*) grbPixels;
    const uint8_t oldSREG = SREG;
    __builtin_avr_cli();
    for(uint16_t j = 0; j < 3*numPixels; j++) {
//      strip0.sendBytes(1, &array[j]);
//      strip1.sendBytes(1, &array[j]);
//      strip2.sendBytes(1, &array[j]);
//      strip3.sendBytes(1, &array[j]);
//      strip4.sendBytes(1, &array[j]);
//      strip5.sendBytes(1, &array[j]);
      strip6.sendBytes(1, &array[j]);
//      strip1.sendBytes(7, &array[j]);
    }
    SREG = oldSREG;

    // Wait to have display lit for a while
  }
  delay(1000);

  // Turn off up to 1000 LEDs.
  strip0.clear(1000);
  strip1.clear(1000);
  strip2.clear(1000);
  strip3.clear(1000);
  strip4.clear(1000);
  strip5.clear(1000);
  strip6.clear(1000);
  strip7.clear(1000);
  delay(500);
}
