////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This example will use the debug interface of the FAB_LED library to write
/// to the console debug information.
///
/// Visual display:
///
/// In the IDE, open the Serial console to see the debug information printed.
/// You will see the properties of the LED strips.
/// you will then see some LED demos, and on the console the memory used.
///
/// Hardware configuration:
///
/// This example works for a regular Arduino board connected to your PC via the
/// USB port to the Arduino IDE (integrated development environment used to
/// compile and load an arduino sketch program to your arduino board). It will
/// by default use the Serial.print() routines.
///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <FAB_LED.h>

// Make the IDE compile at -O2 (fast code) instead of -Os (small size).
#pragma GCC optimize ("-O2")

////////////////////////////////////////////////////////////////////////////////
/// @brief By default the test drives a WS2812B LED strip on port D6 (pin 6 on
/// Arduino uno). Change the model and ports here to match your configuration.
////////////////////////////////////////////////////////////////////////////////
//ws2812b<D,6> myLeds;
apa102<D,6,D,5> myLeds;
//apa104<D,6> myLeds;
//apa106<D,6> myLeds;
//sk6812<D,6> myLeds;

////////////////////////////////////////////////////////////////////////////////
/// @brief This parameter says how many LEDs you will be controlling.
/// If you power the LED strip through your Arduino USB power supply, and not
/// through a separate power supply, make sure to not turn on too many LEDs.
////////////////////////////////////////////////////////////////////////////////
const uint16_t numPixels = 16;

////////////////////////////////////////////////////////////////////////////////
// Methods defined in order to use the debug() call, which will display
// the LED strip properties on your console. The Serial class is usually built 
// in and available with standard arduino boards.
// To see this output, in the IDE, click Tools/Serial Monitor (Ctl-Shift-M)
////////////////////////////////////////////////////////////////////////////////
void FAB_Print(const char * p) {Serial.print(p);}
void FAB_Print(uint32_t u) {Serial.print(u);}



////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called once when the board boots.
////////////////////////////////////////////////////////////////////////////////
void setup()
{
	// Configure Serial for printing.
	Serial.begin(9600);

	// Display LED strip parameters through  Serial, using the two functions
	// defined above.
	myLeds.debug<&FAB_Print, &FAB_Print>();

	// Turn off first 1K LEDs
	myLeds.clear(1000);

	// Configure a strobe signal to Port D5 for people who
	// use oscilloscopes to look at the signal sent to the LEDs
	DDRD |= 1U << 5;
	PORTD &= ~(1U << 5);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
	// Show numPixels dim red pixels
	colorN(8,0,0);
	holdAndClear(1000,200);
	// Show numPixels dim red pixels
	colorN(0,8,0);
	holdAndClear(1000,200);
	// Show numPixels dim red pixels
	colorN(0,0,8);
	holdAndClear(1000,200);

	// Show 1 bright blue pixel
	color1(0,0,0,255);
	holdAndClear(1000,200);
	color1(1,0,255,0);
	holdAndClear(1000,200);
	color1(2,255,0,0);
	holdAndClear(1000,200);

	// Show numPixels dim greed pixels
	color1N(4,4,0);
	holdAndClear(1000,200);

	// Show a smooth raibow, not all colors at once
	rainbow(16,1);
	holdAndClear(1000,200);

	// Show rough rainbow, show whole rainbow on strip at once
	// If numPixels >= 16, the rainbow degenerates to RGB
	rainbow(16, numPixels);
	holdAndClear(1000,200);

	// Show solid color rotating in a rainbow
	rainbow1N(16, 1);
	holdAndClear(1000,200);

	jitter();
	holdAndClear(1000,200);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Waits then clears the LED strip.
////////////////////////////////////////////////////////////////////////////////
void holdAndClear(uint16_t on_time, uint16_t off_time)
{
	// Wait 1sec, turn off LEDs, wait 200msec
	delay(on_time);
	myLeds.clear(numPixels);
	delay(off_time);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display numPixels with one solid color for 1 second.
/// We use a byte array storing 3 bytes per pixel, holding full 8-bit RGB values
/// each value can be from 0 to 255.
////////////////////////////////////////////////////////////////////////////////
void colorN(uint8_t red, uint8_t green, uint8_t blue)
{
	// Display the function call information on the console.
	Serial.print("\ncolorN(");
	Serial.print(red);
	Serial.print(",");
	Serial.print(green);
	Serial.print(",");
	Serial.print(blue);
	Serial.println(")");

	grb array[numPixels] = {};
	// Set each set of 3 bytes for every pixel
	for (uint8_t i = 0; i < numPixels; i++) {
		array[i].r = red;
		array[i].g = green;
		array[i].b = blue;
	}

	// Prints the memory used to sore pixels on the console.
	Serial.print("Pixels array size=");
	Serial.println(sizeof(array));

	// Display the LEDs
	myLeds.draw(numPixels, array);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Display one pixel at a chosen position in the LED strip, with one
/// solid color for 1 second.
////////////////////////////////////////////////////////////////////////////////
void color1(uint8_t pos, uint8_t red, uint8_t green, uint8_t blue)
{
	// Display the function call information on the console.
	Serial.print("\ncolor1(");
	Serial.print(pos);
	Serial.print(", ");
	Serial.print(red);
	Serial.print(",");
	Serial.print(green);
	Serial.print(",");
	Serial.print(blue);
	Serial.println(")");

	uint8_t dim = pos+1;
	grb array[dim];
 
  memset(array,0,sizeof(array));
	array[pos].r = red;
	array[pos].g = green;
	array[pos].b = blue;

	// Prints the memory used to sore pixels on the console.
	Serial.print("Pixels array size=");
	Serial.println(sizeof(array));

	// Display the LEDs
	myLeds.draw(dim, array);
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
	// Display the function call information on the console.
	Serial.print("\ncolor1N(");
	Serial.print(red);
	Serial.print(",");
	Serial.print(green);
	Serial.print(",");
	Serial.print(blue);
	Serial.println(")");

	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	grb pixel = {};

	pixel.r = red;
	pixel.g = green;
	pixel.b = blue;

	// Prints the memory used to sore pixels on the console.
	Serial.print("Pixels array size=");
	Serial.println(sizeof(pixel));

  // Display the LEDs
	myLeds.begin();
	for (uint8_t i = 0; i < numPixels; i++) {
		myLeds.draw(1, &pixel);
	}
  myLeds.end();
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
	// Display the function call information on the console.
	Serial.print("\nrainbow(");
	Serial.print(brightness);
	Serial.println(")");

	// We multiply by 3 because A pixel is 3 bytes, {G,R,B}
	grb array[3*numPixels] = {};

	// Initialize the colors on the array
	array[0].r = brightness;
	array[0].g = 0;
	array[0].b = 0;
	for (uint8_t i = 1; i < numPixels; i++) {
		// Set pix to previous pix color
		array[i].r = array[i-1].r;
		array[i].g = array[i-1].g;
		array[i].b = array[i-1].b;
		// Then change the color
		colorWheel(incLevel, array[i].r, array[i].g, array[i].b);
	}

	// Prints the memory used to sore pixels on the console.
	Serial.print("Pixels array size=");
	Serial.println(sizeof(array));

	// Display the LEDs
	for (uint16_t iter = 0; iter < 100 ; iter++) {
		myLeds.draw(numPixels, array);
		delay(100);

		// Rotate the colors based on the pixel's previous color.
		for (uint16_t i = 0; i < numPixels ; i++) {
			colorWheel(incLevel, array[i].r, array[i].g, array[i].b);
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
	// Display the function call information on the console.
	Serial.print("\nrainbow1N(");
	Serial.print(brightness);
	Serial.println(")");

	grb array[1] = {};

	// Initialize the colors on the array
	array[0].r = brightness;
	array[0].g = 0;
	array[0].b = 0;

	// Prints the memory used to sore pixels on the console.
	Serial.print("Pixels array size=");
	Serial.println(sizeof(array));

	// Display the LEDs
	for (uint16_t iter = 0; iter < 100 ; iter++) {
		myLeds.begin();
		for (uint16_t i = 0; i < numPixels ; i++) {
			myLeds.send(1, array);
		}
		myLeds.end();

		// Rotate the colors based on the pixel's previous color.
		colorWheel(incLevel, array[0].r, array[0].g, array[0].b);
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Demo: Use a byte array storing full 8-bit RGB values.
/// We set the R, G, B values randomly and jitter them by one to
/// animate them.
////////////////////////////////////////////////////////////////////////////////
void jitter()
{
	Serial.println("\njitter()");
	// Allocate array with half the pixels because we past it twice.
	// Add maxJitter+1 pixels because we change the offset we start displaying.
	const uint8_t maxJitter = 2;
	const uint8_t arrayDim = numPixels/2 + maxJitter+1;

	// A pixel is 3 bytes
	grb array[3*arrayDim] = {};

	// Set each set of 3 bytes for every pixel
	for (uint8_t i = 0; i < arrayDim; i++) {
		if (i % 2 == 0) {
			array[i].r = 8;
		}
		if (i % 3 == 0) {
			array[i].g = 8;
		}
		if (i % 5 == 0) {
			array[i].b = 8;
		}
	}
	// Mark one bright red pixel marker to show how LEDs are displayed
	array[maxJitter].g = 0;
	array[maxJitter].b = 0;
	array[maxJitter].r = 64;

	Serial.print("array size=");
	Serial.println(sizeof(array));

	for (uint8_t t = 0; t < 10; t++) {
		for (uint8_t i = 0; i <= maxJitter; i++) {
			const grb * displayPt = &array[3*i];

 			// Display same pattern twice, separated by a fixed pixel.
			myLeds.begin();
			myLeds.send(numPixels/2, displayPt);
			myLeds.send(1, &array[3*maxJitter]);
			myLeds.send(numPixels/2-1, displayPt);
			myLeds.end();

			delay(300);
		}
	}
	myLeds.clear(numPixels);
	delay(1000);
}
