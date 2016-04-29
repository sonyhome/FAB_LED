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
/// The LED strip DI (data input) line should be on port D6 (Digital pin 6 on
/// Arduino Uno). If you need to change the port, change all declarations below
/// from, for example from "ws2812b<D,6> myWs2812" to "ws2812b<C,7> myWs2812"
/// if you wanted to use port C7.
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

#include <FAB_LED.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief This parameter says how many LEDs will be lit up in your strip.
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs at
/// once. Maybe no more than 8 at full power (max is 60mA at 5V is 0.3Watt/LED).
const uint16_t numPixels = 8;
const uint16_t maxBrightness = 16;

////////////////////////////////////////////////////////////////////////////////
// Declares the LED strip you'll write to. You can change this to the right
// LED strip model you own.
//ws2812<D,6> myLedStrip;
//sk6812<D,6>  myLedStrip;
//sk6812b<D,6>  myLedStrip;
//apa102<D,6,D,5>  myLedStrip;
//apa104<D,6>  myLedStrip;
//apa106<D,6>  myLedStrip;
ws2812b<D,6> myLedStrip;

////////////////////////////////////////////////////////////////////////////////
// We define the pixel array we'll store the actual red, green, blue values in.
// Note this library supports many pixel structures, and it is best to use the
// structure that holds the colors in the same order as your LED strip model.
grb  grbPixels[numPixels] = {};


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Turn off up to 1000 LEDs.
  myLedStrip.clear(1000);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // We pick ONE pixel and change its color.
  const uint16_t pos = random(numPixels);
  grbPixels[pos].r = random(maxBrightness);
  grbPixels[pos].g = random(maxBrightness);
  grbPixels[pos].b = random(maxBrightness);

  // Display the pixels.
  myLedStrip.sendPixels(numPixels,grbPixels);

  // Wait to have display lit for a while
  delay(100);
}
