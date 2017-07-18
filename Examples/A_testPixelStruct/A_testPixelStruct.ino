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
#define NUM_PIXELS 40

////////////////////////////////////////////////////////////////////////////////
// Classes to write to a GRB WS2812B LED strip and a RGBW SK6812 LED strip.
// All are controlling port D6 by default, so attach your LED strip data line
// to pin 6 on an Arduino Uno. It may be another pin on other boards.
ws2812b<D,6> myWs2812;
sk6812<D,6>  mySk6812;
sk6812b<D,6> mySk6812b;
apa104<D,6>  myApa104;
apa106<D,6>  myApa106;
apa102<D,6,D,5> myApa102;

////////////////////////////////////////////////////////////////////////////////
// We define 3 arrays as example, each storing pixels in a different order.
// The rgbw order is native to sk6812
// The grbw order is native to sk6812b
// The grb is native to ws2812b
// the rgb is native to apa106
// Whichever array we use, should on any LED strip display a single solid
// color as defined in setup. This is because the library will convert the pixel
// on display if it is not native (that is slower to run so avoid it if you can).
// If there's a bug, some of the displays will be random colors.
rgbw rgbwPixels[NUM_PIXELS] = {};
grbw grbwPixels[NUM_PIXELS] = {};
rgb  rgbPixels[NUM_PIXELS] = {};
grb  grbPixels[NUM_PIXELS] = {};


// Demo display on WS2812B LED strip
void showWs2812b(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myWs2812.clear(1000);
  delay(500);

  // Show rgb array (red)
  myWs2812.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  myWs2812.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  myWs2812.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (white)
  myWs2812.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}


// Demo display on APA104 LED strip
void showApa102(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myApa102.clear(1000);
  delay(500);

  // Show rgb array (red)
  myApa102.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  myApa102.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  myApa102.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (white)
  myApa102.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// Demo display on APA104 LED strip
void showApa104(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myApa104.clear(1000);
  delay(500);

  // Show rgb array (red)
  myApa104.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  myApa104.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  myApa104.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (white)
  myApa104.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// Demo display on APA106 LED strip
void showApa106(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myApa106.clear(1000);
  delay(500);

  // Show rgb array (red)
  myApa106.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  myApa106.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  myApa106.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (white)
  myApa106.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// Demo display on RGB SK6812 LED strip
void showSk6812(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  mySk6812.clear(1000);
  delay(500);

  // Show rgb array (red)
  mySk6812.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  mySk6812.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  mySk6812.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (blue)
  mySk6812.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// Demo display on GRB SK6812 LED strip
void showSk6812b(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  mySk6812b.clear(1000);
  delay(500);

  // Show rgb array (red)
  mySk6812b.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  mySk6812b.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  mySk6812b.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (blue)
  mySk6812b.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// This method shows that all pixel types work seamlessly
// for all LED strip types. It is better to use the pixel
// type that matches your LED strip as it will be faster
// and more compact code, but otherwise, it should not
// change the outcome of your display.
template <class myStripClass>
void showSequence(void)
{
  myStripClass myStrip;

  // Init the LEDs, all off, up to 1000 LEDs.
  myStrip.clear(1000);
  delay(500);

  // Show rgb array (red)
  myStrip.draw(NUM_PIXELS,rgbPixels);
  // Wait to have display lit for a while
  delay(1000);

  // Show grb array (green)
  myStrip.draw(NUM_PIXELS,grbPixels);
  delay(1000);

  // Show rgbw array (blue)
  myStrip.draw(NUM_PIXELS,rgbwPixels);
  delay(1000);

  // Show grbw array (blue)
  myStrip.draw(NUM_PIXELS,grbwPixels);
  delay(1000);
}

// This is for Lukas who wants to mix LED strips
// with different display formats:
// Demo display of one WS2812B LED strip soldered
// before a SK6812 LED strip, both displaying the
// same colors, so the end result should be to
// see R then G then B then W. For fun, ws2812b
// LED white is done with R=G=B=16, and white on
// sk6812 with R=G=B=16 and W=48.
// There will be pixCut WS2812B pixels and
// (NUM_PIXELS - pixCut) Sk6812 pixels.
// If the LED configuration does not match the code,
// the pixels will have multiple colors and more or
// fewer LEDs will light up. This is because ws812b
// uses 3 bytes per pixels, and sk6812b uses 4 bytes
// per pixel.

const uint16_t pixCut = 2;

template<class T> void lukasDraw(uint16_t count, T * array)
{
  // Use begin/send/end instead of draw if you need to send multiple
  // arrays to update the LED strip. On most LEDs timing is critical
  // so restrict the amount of work done between begin/end to just
  // sending data to the LED strip.


  myWs2812.begin();

  myWs2812.send(pixCut, array);
  mySk6812.send(count - pixCut, &array[pixCut]);

  myWs2812.end(count);
}

void lukas(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myWs2812.clear(1000);
  delay(500);

  lukasDraw<rgb>(NUM_PIXELS, rgbPixels);
  delay(1000);

  lukasDraw<grb>(NUM_PIXELS, grbPixels);
  delay(1000);

  lukasDraw<rgbw>(NUM_PIXELS, rgbwPixels);
  delay(1000);

  lukasDraw<grbw>(NUM_PIXELS, grbwPixels);
  delay(1000);
}


// This example is for mBlade who wants to mix WS2812B with APA106:
// The last LED is an APA106, the rest are WS2812B.
template<class T> void mBladeDraw(uint16_t count, T * array)
{
  // Use begin/send/end instead of draw if you need to send multiple
  // arrays to update the LED strip. On most LEDs timing is critical
  // so restrict the amount of work done between begin/end to just
  // sending data to the LED strip.

  myWs2812.begin();

  myWs2812.send(pixCut,array);
  myApa106.send(count - pixCut,&array[pixCut]);

  myWs2812.end(count);
}

void mBlade(void)
{
  // Init the LEDs, all off, up to 1000 LEDs.
  myWs2812.clear(1000);
  delay(500);

  // Draw the pixels, sk strip is after ws strip hence pushed first.
  mBladeDraw<rgb>(NUM_PIXELS,rgbPixels);
  delay(1000);
  
  mBladeDraw<grb>(NUM_PIXELS,grbPixels);
  delay(1000);
  
  mBladeDraw<rgbw>(NUM_PIXELS,rgbwPixels);
  delay(1000);

  mBladeDraw<grbw>(NUM_PIXELS, grbwPixels);
  delay(1000);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  for (int i=0; i < NUM_PIXELS; i++) {
    // Initialize rgb array to show as red strip
    rgbPixels[i].r = 16;
    // Initialise grb array to show as green
    grbPixels[i].g = 16;
    // Initialize grbw array to show as blue
    rgbwPixels[i].b = 16;
    // Initialize rgbw array to show as white
    grbwPixels[i].r = 16;
    grbwPixels[i].g = 16;
    grbwPixels[i].b = 16;
    grbwPixels[i].w = 16*3;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Comment all function calls here EXCEPT for the one that matches your LED
  // strip if you want to see the right color sequence of red, green, blue:
  showSequence< ws2812b<D,6> >();
  mBlade();
  lukas();

  showWs2812b();
  showApa102();
  showApa104();
  showApa106();
  showSk6812();
  showSk6812b();
}
