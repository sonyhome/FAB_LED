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

/// This parameter says how many LEDs will be lit up in your strip.
const uint8_t numPixels = 16;

/// This says how bright LEDs will be (max is 255)
const uint8_t maxBrightness = 20;

/// This class declare the protocol of your LED strip and which port it is on.
// Data output is sent to port D6 (pin 6 on Uno), and for APA102 LEDs, the clock
// is sent to port D5. Uncomment the LED strip that matches the model ou own.
//ws2812b<D,6>  strip;
//ws2812<D,6>   strip;
//sk6812<D,6>   strip;
//sk6812b<D,6>  strip;
//apa104<D,6>   strip;
//apa106<D,6>   strip;
apa102<D,6,D,5>  strip;

/// Array holding the pixel data.
/// Note you have multiple formats available, to support any LED type. If you
/// use the wrong type, FAB_LED will do the conversion transparently for you,
/// but the code will be a tiny bit bigger and slower.
//uint32_t  pixels[numPixels] = {};
//grb  pixels[numPixels] = {}; // Native type for ws2812
//grbw  pixels[numPixels] = {}; // Native type for sk6812
hbgr  pixels[numPixels] = {}; // Native type for apa102


////////////////////////////////////////////////////////////////////////////////
/// Setup() is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // All fields for a pixel initialize to zero, except field h for APA-102
  // which must be 0xE0 or greater, so we set it to 0xFF, full brightness.
  // Depending on which pixel class you uncomment above, comment the h or w
  // field below accordingly, or the code won't compile.
  for (uint8_t index = 0; index < numPixels; index++) {
    pixels[index].h = 0xFF; // hgrb has h field
    pixels[index].g = 0;
    pixels[index].b = 0;
    pixels[index].r = 0;
    //pixels[index].w = 0; // grbw has w field
  }

  // Let's just erase 1K pixels to really clear the LED strip of rogue data
  strip.clear(1000);
}

////////////////////////////////////////////////////////////////////////////////
/// On Arduino, loop() is automatically called repeatedly after setup() has run.
/// It is the main loop.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // We pick ONE pixel and change its color to whatever.
  const uint8_t index = random(numPixels);
  pixels[index].r = random(maxBrightness);
  pixels[index].g = random(maxBrightness);
  pixels[index].b = random(maxBrightness);

  // Display the pixels on the LED strip.
  strip.draw(numPixels, pixels);

  delay(100);
}
