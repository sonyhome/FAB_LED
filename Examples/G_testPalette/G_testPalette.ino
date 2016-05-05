////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Fast Adressable Bitbang LED Library
/// Copyright (c)2015, 2016 Dan Truong
///
/// This is an demo of the FAB_LED library palettes functionality, best seen on
/// an 8x8 pixel array.
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
///
/// The LED power (GND) and (+5V) should be connected on the Arduino Uno's GND
/// and +5V.
///
///
/// Visual results:
///
/// If you use an 8x8 pixel array, you will see geometric diagonal patterns
/// changing colors constantly, which get deconstructed and reconsructed,
/// at verying speeds.
///
/// Details of this test:
///
/// The test defines a gradient monochrome palette.
/// It then uses that palette to set red, green and blue colors.
/// To do so the main loop picks a position in the palette that will be
/// index 0 for the color. It picks different indices for red, green and
/// blue.
///
/// Example:
///
/// gradient  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9
/// red             0 1 2 3 4 5 6 7 8 9
/// green                       0 1 2 3 4 5 6 7 8 9
/// blue        0 1 2 3 4 5 6 7 8 9
///
/// color 0 would be red=3, green=9, blue=1.
///
/// Furthermore, the pixel array has a pattern, which is a diagonal gradient.
/// there are 2 different ones that are used to update the main pattern by
/// putting random bits.
///
/// Example:
///
///   0 1 2 3 4
///   1 2 3 4 0
///   2 3 4 0 1
///   3 4 0 1 2
///
/// Note 4 pixels use color 0.
///
/// Once all the pixels are displayed, the red, green, blue slide in the
/// gradient table so color 0 will be different.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <FAB_LED.h>

ws2812b<D,6> myLeds;

const uint16_t numPixels = 8*8;
const uint8_t bitsPerPixel = 4;
const uint8_t numColors = 1<<bitsPerPixel; // 2^bitsPerPixel colors supported, aka 16.

// Palettes
uint8_t   gradient[2*numColors] = {};

// Pixel array
uint8_t pixels1[ARRAY_SIZE(numPixels, bitsPerPixel)];
uint8_t pixels2[ARRAY_SIZE(numPixels, bitsPerPixel)];
uint8_t pixels[ARRAY_SIZE(numPixels, bitsPerPixel)];

void setup()
{
	myLeds.clear(1000);

	const uint8_t brightness = 1; // 1 << (8-bitsPerPixel);
 
	// Set a saw shape color palette
	for(uint8_t i = 0; i < numColors; i++) {
		gradient[i] = i * brightness;
		gradient[i+numColors] = i * brightness;
	}

	// Set a gradient of colors that is diagonal on a 8x8 pixel board
	for(uint8_t i = 0; i < 8; i++) {
		for(uint8_t j = 0; j < numPixels/8; j++) {
      SET_PIXEL(pixels1, i+8*j, bitsPerPixel, (i+j) % numColors);
      SET_PIXEL(pixels2, (8-i)+8*j, bitsPerPixel, (i+j) % numColors);
      SET_PIXEL(pixels, (8-i)+numPixels-8*j, bitsPerPixel, (i+j) % numColors);
		}
	}
}

void loop()
{
  // offset for each color in gradient table
	static uint8_t r = 0;
	static uint8_t g = numColors/2;
  static uint8_t b = 0;
  // refresh delay
  static uint8_t dl = 20;
  static uint16_t count = random(numPixels*4);
  static uint8_t * pt = pixels1;
  
  uint16_t pos = random(numPixels);

	myLeds.sendPixels<bitsPerPixel,grb>(
		numPixels, pixels,
		&gradient[r], &gradient[g], &gradient[b]);

  if (count-- < 2*numPixels) {
    if (count == 0) {
      count = random(numPixels*8);
      dl = 5*random(100);
      if (pt == pixels1) {
        pt = pixels2;
      } else {
        pt = pixels1;
      }
    }
    pixels[pos] = pt[pos];
  } else if (count < 4*numPixels) {
  } else if (count < 6*numPixels) {
    pixels[pos] = pt[pos];
  } 

	// rotate the colors
	r = (r+1) % (2*numColors);
	g = (g+3) % (2*numColors);
	//b = (b-1) % (2*numColors);

	delay(dl);
}
