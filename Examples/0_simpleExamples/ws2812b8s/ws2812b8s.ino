////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2017 Dan Truong
///
/// This is the simplest exmple to use the library.
///
/// This example is for an Arduino Uno board with 4 LED strips connected to
/// port D3 to D6. Targetting any other board requires you to change something.
/// The program sends an array of pixels to display on the strip.
/// "strip" represents the hardware: LED types and port configuration,
/// "pixels" represents the data sent to the LEDs: a series of colors.
///
/// Wiring:
///
/// The 1st LED strip DI (data input) line should be on port D3 (Digital pin 3 on
/// Arduino Uno), 2nd to D4 (4), 3rd to D5 (5).
/// The 4st LED strip DI (data input) line should be on port D6 (Digital pin 6 on
/// Arduino Uno).
/// The LED power (GND) and (+5V) should be connected on the Arduino Uno's GND
/// and +5V.
///
/// Visual results:
///
/// If the hardware you use matches this program you will see the LEDs blink
/// repeatedly red, green, blue, white, in that order.
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <FAB_LED.h>

// Declare the LED protocol and the port
ws2812b8s<D,3,6>  strip;

// How many pixels to control
const uint8_t numPixels = 8;

// How bright the LEDs will be (max 255)
const uint8_t maxBrightness = 16;

// The pixel array to display
grb  pixels[numPixels] = {};


////////////////////////////////////////////////////////////////////////////////
// Sets the array to specified color
////////////////////////////////////////////////////////////////////////////////
void updateColors(char r, char g, char b)
{
  for(int i = 0; i < numPixels; i++)
  {
    pixels[i].r = r;
    pixels[i].g = g;
    pixels[i].b = b;
  }
}

////////////////////////////////////////////////////////////////////////////////
// This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Turn off the LEDs
  strip.clear(2 * numPixels);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{

  // Write the pixel array red
  updateColors(maxBrightness, 0 , 0);
  // Display the pixels on the LED strip
  strip.sendPixels(numPixels, pixels);
  // Wait 0.1 seconds
  delay(100);

  // Write the pixel array green
  updateColors(0, maxBrightness, 0);
  // Display the pixels on the LED strip
  strip.sendPixels(numPixels, pixels);
  // Wait 0.1 seconds
  delay(100);

  // Write the pixel array blue
  updateColors(0, 0, maxBrightness);
  // Display the pixels on the LED strip
  strip.sendPixels(numPixels, pixels);
  // Wait 0.1 seconds
  delay(100);

  // Write the pixel array white
  updateColors( maxBrightness, maxBrightness, maxBrightness);
  // Display the pixels on the LED strip
  strip.sendPixels(numPixels, pixels);
  // Wait 0.1 seconds
  delay(100);

  // Turn off the LEDs
  strip.clear(numPixels);
  delay(600);
}
