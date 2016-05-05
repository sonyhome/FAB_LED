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
const uint16_t numPixels = 16*8;
const uint16_t maxBrightness = 8;

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
  ONE_PORT_BITBANG> // protocol
{
  public:
  custom() : avrBitbangLedStrip<6,2,2,2,50,dataPortId,dataPortBit,A,0,GRB,ONE_PORT_BITBANG>() {};
  ~custom() {};
};
////////////////////////////////////////////////////////////////////////////////
// Declares the LED strip you'll write to. You can change this to the right
// LED strip model you own.

// #define myLEDtype custom
#define myLEDtype ws2812b
// #define myLEDtype apa102
myLEDtype<D,0> strip0;
myLEDtype<D,1> strip1;
myLEDtype<D,2> strip2;
myLEDtype<D,3> strip3;
myLEDtype<D,4> strip4;
myLEDtype<D,5> strip5;
myLEDtype<D,6> strip6;
myLEDtype<D,7> strip7;

ws2812b8s<D,4,7> strip_split8;
ws2812bs<D,6,D,7> strip_split2;
ws2812bi<D,6,D,7> strip_intlv2;

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
  cli();
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
/// @brief Turns off all LED strips on pins 0 to 7, in constant time.
////////////////////////////////////////////////////////////////////////////////
void off(uint16_t npix) {
  const bool blink = (npix == numPixels);

  if (blink) digitalWrite(13, LOW);
  else digitalWrite(13, LOW);

  // Turn off up to 1000 LEDs.
  strip0.clear(npix);
  strip1.clear(npix);
  strip2.clear(npix);
  strip3.clear(npix);
  strip4.clear(npix);
  strip5.clear(npix);
  strip6.clear(npix);
  strip7.clear(npix);
  delay(10);
  if (npix != numPixels) {
    strip0.grey(2,8);
    strip1.grey(2,8);
    strip2.grey(2,8);
    strip3.grey(2,8);
    strip4.grey(2,8);
    strip5.grey(2,8);
    strip6.grey(2,8);
    strip7.grey(2,8);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  // Turn off up to 1000 LEDs.
  off(1000);

  // Create a repeating RGB display
  for(uint8_t i = 0; i < numPixels; i++) {
    grbPixels[i].r = maxBrightness * ( i   %3 == 0);
    grbPixels[i].g = maxBrightness * ((i+2)%3 == 0);
    grbPixels[i].b = maxBrightness * ((i+1)%3 == 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  const uint8_t iters = 16;
  const uint8_t repeats = 8;

  // Default way a user would use.
  Serial.println("Update LED strips one after the other.");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      strip0.sendPixels(numPixels/8, grbPixels);
      strip1.sendPixels(numPixels/8, &grbPixels[numPixels/8]);
      strip2.sendPixels(numPixels/8, &grbPixels[2*numPixels/8]);
      strip3.sendPixels(numPixels/8, &grbPixels[3*numPixels/8]);
      strip4.sendPixels(numPixels/8, &grbPixels[4*numPixels/8]);
      strip5.sendPixels(numPixels/8, &grbPixels[5*numPixels/8]);
      strip6.sendPixels(numPixels/8, &grbPixels[6*numPixels/8]);
      strip7.sendPixels(numPixels/8, &grbPixels[7*numPixels/8]);
    }
  }
  off(1000);
  delay(250);

  // Send pixels to 8 ports in parallel. Will not work on Arduino.
  Serial.println("Update LED strips 0 to 7 in parallel, split: sending 1/8 the array to each one");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      strip_split8.sendPixels(numPixels, grbPixels);
    }
  }
  off(1000);
  delay(250);

  // send first half the pixel array to strip D6, the 2n half to D7,
  // notice colors will be rgbrgb etc. and strip D7 will pick up the
  // sequence where strip 6 stopped.
  // Notice also that one strip will show 1/2 the pixels of the other
  // examples.
  Serial.println("Update LED strips 6 & 7 in parallel, split: sending 1/2 the array to each one");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      strip_split2.sendPixels(numPixels, grbPixels);
      strip_split2.sendPixels(numPixels, grbPixels);
      strip_split2.sendPixels(numPixels, grbPixels);
      strip_split2.sendPixels(numPixels, grbPixels);
    }
  }
  off(1000);
  delay(250);

  // send interleaved pixels to strip D7 and D7. pix1 to D6, pix2 to
  // D7, pix3 to D6, pix4 to D7 etc.
  // notice colors will skip: D6 array will show rbgrbg... while
  // D7 will show grbgrb... as pix1 is red, pix2 is green, etc.
  // Notice also that one strip will show 1/2 the pixels of the other
  // examples.
  Serial.println("Update LED strips 6 & 7 in parallel, interleave: sending every other pixel to the other strip.");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      strip_intlv2.sendPixels(numPixels, grbPixels);
      strip_intlv2.sendPixels(numPixels, grbPixels);
      strip_intlv2.sendPixels(numPixels, grbPixels);
      strip_intlv2.sendPixels(numPixels, grbPixels);
    }
  }
  off(1000);
  delay(250);

  // update LEDs all at the same time, but using low level code
  // to set ports directly. This works as we interlace at bit level.
  Serial.println("Update LED strips at the same time, but copy SAME pattern to all.");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      parallelPixel(D, 3*numPixels, (uint8_t*) grbPixels);
    }
  }
  off(1000);
  delay(250);

  // Update the LED strips at the same time.
  // This will start failing if your CPU is not fast enough when
  // you update too many LED strips at the same time.
  // Comment and uncomment the LED strips until you find how many
  // you can drive in parallel.
  // For ws2812b, it takes too many cycles to update a pixel byte
  // so the LED strip resets. A byte takes 8 * ( 6+2+?) > 64 cycles (4 usec)
  Serial.println("Update LED strips at the same time, interleaving each pixel sent.");
  for(uint8_t n = 0; n < iters; n++) {
    off(numPixels);
    for(uint8_t i = 0; i < repeats; i++) {
      const uint8_t * array = (uint8_t*) grbPixels;
      const uint8_t oldSREG = SREG;
      cli();
      for(uint16_t j = 0; j < 3*numPixels; j++) {
        strip0.sendBytes(1, &array[j]);
        strip1.sendBytes(1, &array[j]);
        strip2.sendBytes(1, &array[j]);
        strip3.sendBytes(1, &array[j]);
        strip4.sendBytes(1, &array[j]);
        strip5.sendBytes(1, &array[j]);
        strip6.sendBytes(1, &array[j]);
        strip7.sendBytes(1, &array[j]);
      }
      SREG = oldSREG;
      // Wait to have display lit for a while
    }
  }
  off(1000);
  delay(750);
}
