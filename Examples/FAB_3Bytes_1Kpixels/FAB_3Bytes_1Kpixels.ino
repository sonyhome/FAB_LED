////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This is an example use of the FAB_LED libbrary using 24 bits per pixel,
/// AKA 3 bytes, one byte per primary color (Green, Red, Blue).
///
/// This example works for a regular Arduino board connected to your PC via the
/// USB port to the Arduino IDE (integrated development environment used to
/// compile and load an arduino sketch program to your arduino board).A
///
/// If the program doesn't run when you specify too many LEDs, comment out the
/// calls to routines that allocate arrays of 3*numPixels, and keep those that
/// allocate a fixed size (their names end witn N). It should work and you will
/// be able to light up up to MAXINT LEDs, assuming they are powered properly.
/// When it fails LEDs won't turn on. The program works on an Uno with 255 LEDs.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <FAB_LED.h>

////////////////////////////////////////////////////////////////////////////////
/// Configurable parameters
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// @brief This parameter says how many LEDs are in your strip.
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs.
const uint16_t numPixels = 1000; // Max: 64K pixels

////////////////////////////////////////////////////////////////////////////////
/// @brief this definition allows you to select which digital port the LED strip
/// is attached to: Valid are ports A through D, 0 to 7. For AtTiny, you may
/// want to use port D2 instead of D6.
//ws2812b<D,2> WS2812B_STRIP;

ws2812b<D,6> WS2812B_STRIP;

////////////////////////////////////////////////////////////////////////////////
/// @brief Declare a pixel as 3 bytes, each holding a color of [0..255]. The
/// WS2812B LED strips colors are in Green, Red, Blue order. Some other devices
/// may order Red, Green, Blue. You can redefine this structure to handle those.
//typedef struct {
//	uint8_t r;
//	uint8_t g;
//	uint8_t b;
//} pixel_t;

typedef struct {
	uint8_t g;
	uint8_t r;
	uint8_t b;
} pixel_t;



////////////////////////////////////////////////////////////////////////////////
/// @brief Waits then clears the LED strip.
////////////////////////////////////////////////////////////////////////////////
void
holdAndClear(uint16_t on_time, uint16_t off_time)
{
	// Wait 1sec, turn off LEDs, wait 200msec
	delay(on_time);
	WS2812B_STRIP.clear(numPixels);
	delay(off_time);
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Display one pixel with one solid color for 1 second.
/// We use a 3 byte array storing one pixel.
/// Each value can be from 0 to 255.
////////////////////////////////////////////////////////////////////////////////
void
color1(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t dim = pos+1;
	uint8_t array[3*dim];
	memset(array,0,sizeof(array));

	// We cast "array" to "pix" so we can write the colors using
	// the pixel_t structure. It's easier to read.
	pixel_t * pix = (pixel_t *) array;

	pix[pos].r = red;
	pix[pos].g = green;
	pix[pos].b = blue;

	// Display the LEDs
	WS2812B_STRIP.sendPixels(dim, array);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with one solid color for 1 second.
/// We only use a 3 byte array, storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void
color1N(uint8_t red, uint8_t green, uint8_t blue)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3] = {};

	// We cast "array" to "pix" so we can write the colors using
	// the pixel_t structure. It's easier to read.
	pixel_t * pix = (pixel_t *) array;

	pix[0].r = red;
	pix[0].g = green;
	pix[0].b = blue;

	// Disable interupts, save the old interupt state
	const uint8_t oldSREG = SREG;
	__builtin_avr_cli();

	// Display the LEDs
	for (uint16_t i = 0; i < numPixels; i++) {
		WS2812B_STRIP.sendPixels(1, array);
	}

	// Restore the old interrupt state
	SREG = oldSREG;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper routine to calculate the next color for a rainbow
////////////////////////////////////////////////////////////////////////////////
void
colorWheel(uint8_t incStep, uint8_t & R, uint8_t & G, uint8_t & B)
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
/// @brief Display numPixels with a solid rainbow color.
/// We use a 3 byte array storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void
rainbow1N(uint8_t brightness, uint8_t incLevel)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3] = {};
	pixel_t * pix = (pixel_t *) array;

	// Initialize the colors on the array
	pix[0].r = brightness;
	pix[0].g = 0;
	pix[0].b = 0;

	// Display the LEDs
	for (uint16_t iter = 0; iter < 100 ; iter++) {
		const uint8_t oldSREG = SREG;
		__builtin_avr_cli();
		for (uint16_t i = 0; i < numPixels ; i++) {
			WS2812B_STRIP.sendPixels(1, array);
		}
		SREG = oldSREG;
		delay(100);

		// Rotate the colors based on the pixel's previous color.
		colorWheel(incLevel, pix[0].r, pix[0].g, pix[0].b);
	}
}



////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup() {
	// Turn off first 1000 LEDs
	WS2812B_STRIP.clear(1000);

	// Configure a strobe signal to Port D5 for people who
	// use oscilloscopes to look at the signal sent to the LEDs
	DDRD |= 1U << 5;
	PORTD &= ~(1U << 5);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{

	// Show 1 bright pixel, R, G then B.
	color1(2,255,0,0);
	holdAndClear(1000,200);
	color1(1,0,255,0);
	holdAndClear(1000,200);
	color1(0,0,0,255);
	holdAndClear(1000,200);

	// Show numPixels dim pixels, R, G then B 
	color1N(8,0,0);
	holdAndClear(1000,200);
	color1N(0,8,0);
	holdAndClear(1000,200);
	color1N(0,0,8);
	holdAndClear(1000,200);

	// Show solid color rotating in a rainbow
	rainbow1N(16, 1);
	holdAndClear(1000,200);
}
