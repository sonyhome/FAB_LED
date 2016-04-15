////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This is an example use of the FAB_LED library using the predefined pixel
/// structures rgb, grb, bgr, rgbw. It is a good first test to validate the
/// library works with your LED strip, and if needed to figure out what type
/// of LED strip you actually have.
/// On success the LEDs will cycle red, green, blue, pause.
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
/// R, G, B.
/// If you picked the wrong one, the order will be different, or may be
/// jumbled random colors.
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
#define NUM_PIXELS 4

////////////////////////////////////////////////////////////////////////////////
// Classes to write to a GRB WS2812B LED strip and a RGBW SK6812 LED strip.
// All are controlling port D6 by default, so attach your LED strip data line
// to pin 6 on an Arduino Uno. It may be another pin on other boards.
ws2812b<D,6> myWs2812;
sk6812<D,6>  mySk6812;
apa104<D,6>  myApa104;
apa106<D,6>  myApa106;

////////////////////////////////////////////////////////////////////////////////
// We define 3 arrays as example, each storing pixels in a different order.
// The rgbw order is native to sk6812
// The grb is native to ws2812b
// the rgb is native to apa106
// Whichever array we use, should on any LED strip display a single solid
// color as defined in setup. This is because the library will convert the pixel
// on display if it is not native (that is slower to run so avoid it if you can).
// If there's a bug, some of the displays will be random colors.
rgbw rgbwPixels[NUM_PIXELS] = {};
rgb  rgbPixels[NUM_PIXELS] = {};
grb  grbPixels[NUM_PIXELS] = {};


// Demo display on WS2812B LED strip
void showWs2812b(void)
{
  // Show rgb array (red)
  myWs2812.sendPixels(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (green)
  myWs2812.sendPixels(NUM_PIXELS,grbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (blue)
  myWs2812.sendPixels(NUM_PIXELS,rgbwPixels);
  // Wait to have display lit for a while
  delay(1000);
}

// Demo display on APA104 LED strip
void showApa104(void)
{
  // Show rgb array (red)
  myApa104.sendPixels(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (green)
  myApa104.sendPixels(NUM_PIXELS,grbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (blue)
  myApa104.sendPixels(NUM_PIXELS,rgbwPixels);
  // Wait to have display lit for a while
  delay(1000);
}

// Demo display on APA106 LED strip
void showApa106(void)
{
  // Show rgb array (red)
  myApa106.sendPixels(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (green)
  myApa106.sendPixels(NUM_PIXELS,grbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (blue)
  myApa106.sendPixels(NUM_PIXELS,rgbwPixels);
  // Wait to have display lit for a while
  delay(1000);
}

// Demo display on SK6812 LED strip
void showSk6812(void)
{
  // Show rgb array (red)
  mySk6812.sendPixels(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (green)
  mySk6812.sendPixels(NUM_PIXELS,grbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show rgb array (blue)
  mySk6812.sendPixels(NUM_PIXELS,rgbwPixels);
  // Wait to have display lit for a while
  delay(1000);
}

// This is for Lukas who wants to mix LED strips
// with different display formats:
// Demo display of one WS2812B LED strip soldered
// before a SK6812 LED strip, both displaying the
// same colors, so the end result should be to
// see R then G then B. If the LED configuration
// does not match the code, the pixels will have
// multiple colors and more or fewer LEDs will
// light up.
void lukas(void)
{
  // Disable interupts
  const uint8_t oldSREG = SREG;
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS/2,&rgbPixels[0]);
  mySk6812.sendPixels(NUM_PIXELS/2,&rgbPixels[NUM_PIXELS/2]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
  
  // Disable interupts
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS/2,&grbPixels[0]);
  mySk6812.sendPixels(NUM_PIXELS/2,&grbPixels[NUM_PIXELS/2]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
  
  // Disable interupts
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS/2,&rgbwPixels[0]);
  mySk6812.sendPixels(NUM_PIXELS/2,&rgbwPixels[NUM_PIXELS/2]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
}

// This is for mBlade who wants to mix WS2812B
// with APA106:
// The last LED is an APA106, then the rest are WS2812B.
void mBlade(void)
{
  // Disable interupts
  const uint8_t oldSREG = SREG;
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS-1,&rgbPixels[0]);
  myApa106.sendPixels(1,&rgbPixels[NUM_PIXELS-1]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
  
  // Disable interupts
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS-1,&grbPixels[0]);
  myApa106.sendPixels(1,&grbPixels[NUM_PIXELS-1]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
  
  // Disable interupts
  __builtin_avr_cli();
  // Draw the pixels, sk strip is after ws strip hence pushed first.
  myWs2812.sendPixels(NUM_PIXELS-1,&rgbwPixels[0]);
  myApa106.sendPixels(1,&rgbwPixels[NUM_PIXELS-1]);
  // Enable interupts
  SREG = oldSREG;
  // Wait to have display lit for a while
  delay(1000);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  for (int i=0; i < NUM_PIXELS; i++) {
    // Initialize rgb array to show as red strip
    rgbPixels[i].r = 16 + 16 * (i%2);
    // Initialise grb array to show as green
    grbPixels[i].g = 16 + 16 * (i%2);
    // Initialize rgbw array to show as blue
    rgbwPixels[i].b = 16 + 16 * (i%2);
    rgbwPixels[i].w = 16 + 16 * (i%2);
  }
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Init the LEDs, all off, up to 1000 LEDs.
  mySk6812.clear(1000);
  delay(500);

  // Comment all function calls here EXCEPT for the one that matches your LED
  // strip if you want to see the right color sequence of red, green, blue:
  showWs2812b();
//  showApa104();
//  showApa106();
//  showSk6812();
//  lukas();
//  mBlade();
}
