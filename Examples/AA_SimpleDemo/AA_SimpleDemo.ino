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
/// from, for example from "ws2812b<D,6> myWs2812" to "ws2812b<B,4> myWs2812"
/// if you wanted to use port B4.
/// The LED power (GND) and (+5V) should be connected on the Arduino Uno's GND
/// and +5V.
/// For APA-102 LED strips, the LED strip CK clock should be on port D5.
///
/// LED strip type: By default WS2812B LED code is uncommented in loop(). If
/// your LED strip is another type, comment that and uncomment the function
/// name that matches your LED strip.
///
/// Visual results:
///
/// If you use the proper LED type, your LED strip have pixels lighting
/// randombly with a new color.
///
/// Details of this test:
///
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs at
/// once. Maybe no more than 8 at full power (max is 60mA at 5V is 0.3Watt/LED).
///
/// Unlike Adafruit's or FastLED library, the class defined by the FAB_LED
/// library does not hold a pixel buffer. It is only a class that defines the
/// LED protocol used to communicate with a port.
/// FAB_LED provides basic pixel types of different kinds for users to store
/// their pixel data. It is best for performance to match the pixel type to
/// the LED strip protocol (byte order), but FAB_LED will handle transparently
/// conversion from the byte array format to the native pixel order otherwise,
/// so that the patterns are shown with the correct colors.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <FAB_LED.h>

/// @brief This parameter says how many LEDs will be lit up in your strip.
const uint8_t numPixels = 2;

/// @brief This says how bright LEDs will be (max is 255)
const uint8_t maxBrightness = 16;

/// @brief Declare the protocol for your LED strip. Data is wired to port D6,
/// and the clock to port D5, for APA102 only.
ws2812b<D,6>  LEDstrip;
//ws2812<D,6>  LEDstrip;
//sk6812<D,6>   LEDstrip;
//sk6812b<D,6>  LEDstrip;
//apa104<D,6>   LEDstrip;
//apa106<D,6>   LEDstrip;
//apa102<D,6,D,5>  LEDstrip;

/// @brief Array holding the pixel data.
/// Note you have multiple formats available, to support any LED type. If you
/// use the wrong type, FAB_LED will do the conversion transparently for you.
hbgr  pixels[numPixels] = {};
//grb  pixels[numPixels] = {};
//grbw  pixels[numPixels] = {};


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // RGB initializes to zero, but for APA102, h is set to full brightness.
  // Depending on which class you use for your pixels, comment the h or w field
  // accordingly.
  for (uint8_t pos = 0; pos < numPixels; pos++) {
    pixels[pos].h = 0xFF; // hgrb has h field
    pixels[pos].g = 0;
    pixels[pos].b = 0;
    pixels[pos].r = 0;
    //pixels[pos].w = 0; // grbw has w field
  }

  LEDstrip.refresh(); // Hack: needed for apa102 to display last pixels

  // Clear display
  LEDstrip.sendPixels(numPixels,pixels);
  LEDstrip.refresh(); // Hack: needed for apa102 to display last pixels
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // We pick ONE pixel and change its color.
  const uint8_t pos = random(numPixels);
  pixels[pos].r = random(maxBrightness);
  pixels[pos].g = random(maxBrightness);
  pixels[pos].b = random(maxBrightness);

  // Display the pixels on the LED strip.
  LEDstrip.sendPixels(numPixels, pixels);
  LEDstrip.refresh(); // Hack: needed for apa102 to display last pixels
  delay(100);
}
