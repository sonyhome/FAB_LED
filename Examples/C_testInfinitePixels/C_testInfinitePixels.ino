////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This example demonstrates control of up to 64K LEDs using 10 bytes of RAM.
/// By default it will light up 64 pixels. the downside of using more pixels
/// is an increase of the display refresh lag.
///
/// WARNING: If you drive many LEDs, make sure you power them separately and
/// not from your Arduino board. I set the value to 64 LEDs because I can
/// drive them with my Arduino Uno, provided I do not change the LED colors
/// or brightness from the original code.
///
/// Visual display:
///
/// 1 pixel will go red, green, blue, full brightness.
/// All pixels will go red green blue low brightness.
/// All pixels will show rainbows.
/// All pixels will fade through all colors of the rainbow.
///
/// Hardware configuration:
///
/// This example works for a regular Arduino board connected to your PC via the
/// USB port to the Arduino IDE (integrated development environment used to
/// compile and load an arduino sketch program to your arduino board).
/// By default it is expected the LED strip is on port D6.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#include <FAB_LED.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief This parameter says how many LEDs are in your strip.
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs.
const uint16_t numPixels = 1000; // Max: 64K pixels

////////////////////////////////////////////////////////////////////////////////
/// @brief this definition allows you to select which digital port the LED strip
/// is attached to: Valid are ports A through D, 0 to 7. For AtTiny, you may
/// want to use port B2 instead of D6.
/// Uncomment the LED strip model you are using and comment the others. By
/// default the ws2812b LED strip is used.
ws2812b<D,6> myLeds;
//sk6812<B,2> myLeds;
//apa102<D,6,D,5> myLeds;
//apa104<B,2> myLeds;
//apa106<B,2> myLeds;


////////////////////////////////////////////////////////////////////////////////
/// @brief Waits then clears the LED strip.
////////////////////////////////////////////////////////////////////////////////
void holdAndClear(uint16_t on_time, uint16_t off_time)
{
	// Wait 1sec, turn off LEDs, wait 200msec
	delay(on_time);
	//PORTB ^= 1U << 5; // On
	myLeds.clear(1000);
	delay(off_time);
	//PORTB ^= 1U << 5; // Off
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Display one pixel with one solid color for 1 second.
/// Each value can be from 0 to 255.
////////////////////////////////////////////////////////////////////////////////
void color1(uint8_t red, uint8_t green, uint8_t blue)
{
	rgb pix[1];

	pix[0].r = red;
	pix[0].g = green;
	pix[0].b = blue;

	// Display the LEDs
	myLeds.draw(1, pix);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with one solid color.
/// We only use a 3 byte array, storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void color1N(uint8_t red, uint8_t green, uint8_t blue)
{
	rgb pix[1];

	pix[0].r = red;
	pix[0].g = green;
	pix[0].b = blue;

	// Display the LEDs
	myLeds.begin();
	for (uint16_t i = 0; i < numPixels; i++) {
		myLeds.send(1, pix);
	}
	myLeds.end();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper routine to calculate the next color for a rainbow
/// Note: we force gcc to inline to meet the timing constraints of the LED strip
////////////////////////////////////////////////////////////////////////////////
static void colorWheel(uint8_t incStep, uint8_t & R, uint8_t & G, uint8_t & B)
__attribute__ ((always_inline));

static void colorWheel(uint8_t incStep, uint8_t & R, uint8_t & G, uint8_t & B)
{
	if (B == 0 && R != 0) {
		R = (R <= incStep) ? 0 : R - incStep;
		G = (G >= 255-incStep) ? 255 : G + incStep;
		return;
	}
	if (R == 0 && G != 0) {
		G = (G <= incStep) ? 0 : G - incStep;
		B = (B >= 255-incStep) ? 255 : B + incStep;
		return;
	}
	if (G == 0 && B != 0) {
		B = (B <= incStep) ? 0 : B - incStep;
		R = (R >= 255-incStep) ? 255 : R + incStep;
		return;
	}
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels in a single color changing to all rainbow color.
/// We use a 3 byte array storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void fade1N(uint8_t brightness, uint8_t incLevel)
{
	rgb pix[1];

	// Initialize the colors on the array
	pix[0].r = brightness;
	pix[0].g = 0;
	pix[0].b = 0;

	// Display the LEDs
	for (uint16_t iter = 0; iter < 20 ; iter++) {
		myLeds.begin();
		for (uint16_t i = 0; i < numPixels ; i++) {
			myLeds.send(1, pix);
		}
		myLeds.end();

		// Rotate the colors based on the pixel's previous color.
		colorWheel(incLevel, pix[0].r, pix[0].g, pix[0].b);
	}
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with a rotating rainbow color.
/// We use a 3 byte array storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void rainbow1N(uint8_t brightness)
{
  rgb pix[1];

  // Initialize the colors on the array
  pix[0].r = brightness;
  pix[0].g = 0;
  pix[0].b = 0;

  // Display the LEDs
  for (uint16_t iter = 0; iter < 20 ; iter++) {
    myLeds.begin();
    for (uint16_t i = 0; i < numPixels ; i++) {
      myLeds.send(1, pix);
      // Rotate the colors based on the pixel's previous color.
      colorWheel(1, pix[0].r, pix[0].g, pix[0].b);
    }
    myLeds.end();
    delay(100);
  }
}



////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
	// Turn off first 1000 LEDs
	myLeds.clear(1000);

	// Configure a strobe signal to Port B5 for people who
	// use oscilloscopes to look at the signal sent to the LEDs
	// Port B5 corresponds to the Arduino Uno pin13 (LED).
	//DDRB |= 1U << 5;
	//PORTB &= ~(1U << 5);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{

	// Show 1 bright pixel, R, G then B.
	color1(255,0,0);
	holdAndClear(1000,200);
	color1(0,255,0);
	holdAndClear(1000,200);
	color1(0,0,255);
	holdAndClear(1000,200);

	// Show numPixels dim pixels, R, G then B 
	color1N(2,0,0);
	holdAndClear(1000,200);
	color1N(0,2,0);
	holdAndClear(1000,200);
	color1N(0,0,2);
	holdAndClear(1000,200);

  // Rainbows
  rainbow1N(4);
  holdAndClear(1000,200);
  fade1N(4, 1);
  holdAndClear(1000,200);
}
