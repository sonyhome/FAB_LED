////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This is an example use of the FAB_LED libbrary using 24 bits per pixel,
/// AKA 3 bytes, one byte per primary color. This uses the raw 24 bit interface
/// to the FAB_LED interface, passing uint8_t * arrays.
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
///
/// Note: The display will not work with sk6812 LED strips that use 4 bytes per
/// pixel.
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
const uint16_t numPixels = 16;

////////////////////////////////////////////////////////////////////////////////
/// @brief this definition allows you to select which digital port the LED strip
/// is attached to: Valid are ports A through D, 0 to 7. For AtTiny, you may
/// want to use port D2 instead of D6.
/// For different LED strip models you may want to match them to APA104 or WS2812
/// LED timings by using the matching implementation class, for example:
//ws2812<D,6> myLeds;
//apa104<D,6> myLeds;
//apa106<D,6> myLeds;
ws2812b<D,6> myLeds;

#define   RED(x, i) x[i]
#define GREEN(x, i) x[i+1]
#define  BLUE(x, i) x[i+2]

////////////////////////////////////////////////////////////////////////////////
/// @brief Waits then clears the LED strip.
////////////////////////////////////////////////////////////////////////////////
void holdAndClear(uint16_t on_time, uint16_t off_time)
{
	// Wait 1sec, turn off LEDs, wait 200msec
	delay(on_time);
	PORTB ^= 1U << 5; // On
	myLeds.clear(numPixels);
	delay(off_time);
	PORTB ^= 1U << 5; // Off
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with one solid color for 1 second.
/// We use a byte array storing 3 bytes per pixel, holding full 8-bit RGB values
/// each value can be from 0 to 255.
////////////////////////////////////////////////////////////////////////////////
void colorN(uint8_t red, uint8_t green, uint8_t blue)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3*numPixels] = {};

	// Set each set of 3 bytes for every pixel
	for (uint16_t i = 0; i < numPixels; i++) {
		  RED(array, i) = red;
		GREEN(array, i) = green;
		 BLUE(array, i) = blue;
	}

	// Display the LEDs
	myLeds.sendPixels(numPixels, array);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display one pixel with one solid color for 1 second.
/// We use a 3 byte array storing one pixel.
/// Each value can be from 0 to 255.
////////////////////////////////////////////////////////////////////////////////
void color1(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t dim = pos+1;
	uint8_t array[3*dim];
	memset(array,0,sizeof(array));

	  RED(array, pos) = red;
	GREEN(array, pos) = green;
	 BLUE(array, pos) = blue;

	// Display the LEDs
	myLeds.sendPixels(dim, array);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with one solid color for 1 second.
/// We only use a 3 byte array, storing one pixel.
/// We then send the same pixel repeatedly to the LED strip very fast.
/// This demonstrates the ability tocall the library repeatedly.
/// If this feature fails, you may see only 1 pixel lit, or a varying number
/// of pixels glitching on and off.
////////////////////////////////////////////////////////////////////////////////
void color1N(uint8_t red, uint8_t green, uint8_t blue)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3];

	  RED(array, 0) = red;
	GREEN(array, 0) = green;
	 BLUE(array, 0) = blue;

	// Disable interupts, save the old interupt state
	const uint8_t oldSREG = SREG;
	__builtin_avr_cli();

	// Display the LEDs
	for (uint16_t i = 0; i < numPixels; i++) {
		myLeds.sendPixels(1, array);
	}

	// Restore the old interrupt state
	SREG = oldSREG;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper routine to calculate the next color for a rainbow
////////////////////////////////////////////////////////////////////////////////
void colorWheel(uint8_t incStep, uint8_t & R, uint8_t & G, uint8_t & B)
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
/// @brief Display numPixels with a rainbow.
/// The rainbow is re-painted after the arryay is displayed.
////////////////////////////////////////////////////////////////////////////////
void rainbow(uint8_t brightness, uint8_t incLevel)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3*numPixels] = {};

	// Initialize the colors on the array
	  RED(array, 0) = brightness;
	GREEN(array, 0) = 0;
	 BLUE(array, 0) = 0;
	for (uint16_t i = 1; i < numPixels; i++) {
		// Set pix to previous pix color
		  RED(array, i) = RED(array, i-1);
		GREEN(array, i) = GREEN(array, i-1);
		 BLUE(array, i) = BLUE(array, i-1);
		// Then rotate it to its final color.
		colorWheel(incLevel, RED(array, i), GREEN(array, i), BLUE(array, i));
	}

	// Display the LEDs
	for (uint16_t iter = 0; iter < 100 ; iter++) {
		myLeds.sendPixels(numPixels, array);
		delay(100);

		// Rotate the colors based on the pixel's previous color.
		for (uint16_t i = 0; i < numPixels ; i++) {
			colorWheel(incLevel, RED(array, i), GREEN(array, i), BLUE(array, i));
		}
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
void rainbow1N(uint8_t brightness, uint8_t incLevel)
{
	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	uint8_t array[3];

	// Initialize the colors on the array
	  RED(array, 0) = brightness;
	GREEN(array, 0) = 0;
	 BLUE(array, 0) = 0;

	// Display the LEDs
	for (uint16_t iter = 0; iter < 100 ; iter++) {
		const uint8_t oldSREG = SREG;
		__builtin_avr_cli();
		for (uint16_t i = 0; i < numPixels ; i++) {
			myLeds.sendPixels(1, array);
		}
		SREG = oldSREG;
		delay(100);

		// Rotate the colors based on the pixel's previous color.
		colorWheel(incLevel, RED(array, 0), GREEN(array, 0), BLUE(array, 0));
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Demo: Use a byte array storing full 8-bit RGB values.
/// We set the R, G, B values randomly and jitter them by one to
/// animate them.
////////////////////////////////////////////////////////////////////////////////
void jitter()
{
	// Allocate array with half the pixels because we past it twice.
	// Add maxJitter+1 pixels because we change the offset we start displaying.
	const uint8_t maxJitter = 5;
	const uint8_t arrayDim = numPixels/2 + maxJitter+1;

	// A pixel is 3 bytes
	uint8_t array[3*arrayDim] = {};

	// Set each set of 3 bytes for every pixel
	for (uint16_t i = 0; i < arrayDim; i++) {
		if (i % 2 == 0) {
			  RED(array, 0) = 8;
		}
		if (i % 3 == 0) {
			GREEN(array, 0) = 8;
		}
		if (i % 5 == 0) {
			 BLUE(array, 0) = 8;
		}
	}
	// Mark one bright red pixel marker to show how LEDs are displayed
	  RED(array, 0) = 64;
	GREEN(array, 0) = 0;
	 BLUE(array, 0) = 0;
	// From here on, we do not change the pixels colors, we just move the pointers to animate the LED strip.

	// Repeat 10 times
	for (uint8_t repeat = 0; repeat < 10; repeat++) {
		// Scroll display
		for (uint8_t position = 0; position <= maxJitter; position++) {
			// We multiply by 3 because a pixel is 3 bytes.
			const uint8_t * displayPt = &array[3 * position];

			const uint8_t oldSREG = SREG;
			__builtin_avr_cli();
			// Display same pattern twice, separated by a fixed pixel in the middle.
			// This just demonstrates how to do more complex animations.
			myLeds.sendPixels(numPixels/2, displayPt);
			myLeds.sendPixels(1, &array[3*maxJitter]);
			myLeds.sendPixels(numPixels/2-1, displayPt);
			SREG = oldSREG;

			delay(300);
		}
	}
	myLeds.clear(numPixels);
	delay(1000);
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
	// Note that B5 correspons to the pin13 LED on Arduino Uno.
	DDRB |= 1U << 5;
	PORTB &= ~(1U << 5);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
/// It is the main loop, and it calls all the other demo methods declared below.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
	// Show numPixels dim red pixels
	colorN(8,0,0);
	holdAndClear(1000,200);
	// Show numPixels dim green pixels
	colorN(0,8,0);
	holdAndClear(1000,200);
	// Show numPixels dim blue pixels
	colorN(0,0,8);
	holdAndClear(1000,200);

	// Show 1 bright blue pixel
	color1(0,0,0,255);
	holdAndClear(1000,200);
	// Show 1 bright green pixel
	color1(1,0,255,0);
	holdAndClear(1000,200);
	// Show 1 bright red pixel
	color1(2,255,0,0);
	holdAndClear(1000,200);

	// Show numPixels dim white pixels
	color1N(8,8,8);
	holdAndClear(1000,200);

	// Show a smooth rainbow, not all colors at once
	rainbow(16,1);
	holdAndClear(1000,200);

	// Show whole rainbow spectrum on strip at once, hence coarser
	// If numPixels >= 16, the rainbow degenerates to RGB
	rainbow(16, numPixels/3);
	holdAndClear(1000,200);

	// Show solid color rotating in a rainbow
	rainbow1N(16, 1);
	holdAndClear(1000,200);

	jitter();
	holdAndClear(1000,200);
}
