FAB_LED
=======

Welcome to the Fast Arduino Bitbang (FAB) LED library.

This library is meant to be a very compact, fast library to drive your addressable LEDs.

Feedback!
---------

Important: I would really appreciate feedback from people using this LED library, here's how.

* Watch/star/comment/help contribute FAB_LED in [GIT](https://github.com/sonyhome/FAB_LED)
* Comment to my blog entry for FAB_LED on [Wordpress](https://dntruong.wordpress.com/2016/03/23/my-ws2812b-library-with-palettes)
* Comment to my forum entry for FAB_LED on [Arduino.cc](http://forum.arduino.cc/index.php?topic=392074.0)

Installation
------------

The FAB_LED library is installed like any other Arduino library:

* Download the library from the [master repository](https://github.com/sonyhome/FAB_LED/archive/master.zip).
* Unzip the FAB_LED_main directory and files.
* Move the directory and its content to your Arduino library directory.
  On most platforms it will be under your documents folder:
  `Documents/Arduino/libraries`
* Launch the Arduino IDE program
* Try some examples
  * Connect your Arduino board with a LED strip wired to pin D6 of the board
  * Load one of the examples menu File/Examples/FAB_LED_master/examples/...
  * Compile and load the example (Ctrl-U or Command-U)


Why FAB_LED is better
---------------------

There are a few well establised LED libraries that are very popular:
* [Adafruit Neopixel library](https://github.com/sonyhome/Adafruit)
* [FastLED linrary](https://github.com/sonyhome/FastLED)

So why FAB_LED? WHY?

Because I'm not happy with these libraries. I want something simple, super small, super fast, and super flexible.
I think I've achieved more of that than the above libraries.

Here's some properties that may or may not be apparent benefits to you until you start using the libraries:

* The FAB_LED bit-banging code is not coded in assembly, but in C.
That makes it portable to Arduino boards of any frequency 8MHz and above (8MHz is untested).
  * It should work on AVR and ARM Arduino boards (untested on ARM).
  * It supports a wide range of wire protocols for the LEDs.
  * The library supports WS2812, WS2812B, APA104, APA106, SK6812 and other similar addressable color LEDs, with RGB, GRB, BGR, RGBW LED formats and timings, 3 or 4 byte per pixels.
    * It is easy to add new LEDs with different timings and color mappings.
    (APA102 and SPI-communication LEDs are in plan)
  * Porting for platforms other than Arduino is possible. It will just require implementing a replacement for I/O ports manipulation and for interrupt disabling. (Raspberry Pi port might happen) 

In terms of usability, and memory footprint I believe FAB_LED is supperior to the libraries mentioned above.

* FAB_LED compiles smaller, and runs faster than the other libraries, demonstrations below.
* You manipulate the LED pixel array directly, it is NOT embedded into the LED class. This allows you much easier control of your patterns.
  * In fact I plan to provide some primitives to manipulate pixel arrays to support 2D displays and sprites.
* The LED library implements direct display routines, to which you pass the pixel array.
  * FAB_LED supports many pixel representations to facilitate importing patterns from other programs like Gimp.
  * FAB_LED supports palettes natively.
    * FAB_LED palettes allow very large pixel arrays in a small memory footprint. For example 2KB of RAM can fit up to 2042 two-tone pixels, 1280 pixels with 256 colors, 682 24-bit pixels or 512 32-bit pixels.
      Note: if you have 256K of RAM or more, FAB_LED is limited to 64K pixels currently, because it uses a uint16_t to control the pixel offset for efficiency, but can be converted to uint32_t, however one LED strip of 64K pixels would have a very slow refresh rate.
      Note: On an Arduino Uno, you have 2K of RAM, which also needs to hold other data AND your program stack. Therefore you cannot use all 2kB to allocate the LED strip. In practice I believe you have about 750 bytes available for that.
    * FAB_LED palettes allow you to do palette based special effects. For example you can rotate colors without having to redraw your patterns.
* FAB_LED display routines can be called back to back, which opens many opportunities.
  * FAB_LED allows you to control an infinite number of LEDs even with a small 16MHz Arduino.
  * Ability to draw patterns repeatedly to support large displays. Just disable interrupts, and call sendPixels() repeatedly.
  * Ability to display separate pixel arrays for new visual effects. Just call sendPixels() repeatedly with different input pixel arrays.
  * Ability to display on the same port using multiple LED formats, to allow mix-n-match of otherwise signal incompatible LEDs, for example to embbed RGB APA106 LEDs with GRB WS2812B or RGWB SK6812 LEDs. This is useful to use LEDs that come with different physical properties and formats, for art projects. Just declare multiple LED strip objects on the same port, and use the one matching your LED strip model at the right pixel offset.

To demonstrate the benefits of FAB_LED, here are apples-to-apples comparison code snippets to do the same thing with different LED libraires, with compilation results for an Arduino Uno target, compiled on Mac, with Arduino 1.6.7:

### Adafruit NeoPixel library

```
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(
  8, 6, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
}

void loop() {
    pixels.setPixelColor(7, pixels.Color(16,0,0));
    pixels.show();
    delay(1000);
}
```

`Sketch uses 2,846 bytes (8%) of program storage space. Maximum is 32,256 bytes.
Global variables use 40 bytes (1%) of dynamic memory, leaving 2,008 bytes for local variables. Maximum is 2,048 bytes.`

### FAB_LED library

```
#include <FAB_LED.h>
ws2812b<D,6> myLeds;
grb pix[8] = {};

void setup() { }

void loop()
{
	pix[7].r = 16;
	myLeds.sendPixels(8, pix);
	delay(1000);
}
```

`Sketch uses 736 bytes (2%) of program storage space. Maximum is 32,256 bytes.
Global variables use 34 bytes (1%) of dynamic memory, leaving 2,014 bytes for local variables. Maximum is 2,048 bytes.`

FAB_LED memory footprint is almost <B>4X smaller than Adafruit's library</B>.

If you even look at the Arduino basic examples, the FAB_LED example uses even less memory than the
01.Basics/Blink example (1039 bytes of program storage space and 9 bytes of DRAM)!

Furthermore, from my calculations, the overhead of FAB_LED for the above code should be 146 bytes, with the rest
of the memory usage being for the Arduino infrastructure code. This means FAB_LED memory footprint is close to what
it would be if the program was coded in assembly.

Examples
========

Here you will find photos of how the example code shipped with the FAB_LED library. Please refer to these
videos to verify the output on your LEDs is as expected. If it is not exactly as expected, you may be using
the wrong LED model definition.

Demos
=====

Here you will find examples of uses of the FAB_LED library that are not shipped with the library. These can be personal projects, extensions and features not implemented yet.

170+ LED reel
-------------

A validation that FAB_LED can address very large LED strips, using very little DRAM.
A small array holds a pattern in memory, and the `SendPixels()` method is called in a loop
to repeate the pattern on the LED strip.

![A 170+ LED reel controlled with FAB_LED](Documentation/Readme/Demo/LEDStrip.jpg)  

First 2D use
------------

This displays an 8x8 pixels picture on an 8x8 pixel array, addressed left to right, top to bottom.
The drawing was exported to C from Gimp, and the colors retouched: You need less bright, pure colors.  
The display layout matches the 2D array in memory.
I plan to support more complex mappings.

![2D LED display with diffuser in front](Documentation/Readme/Demo/2D.jpg)  
![2D LED board connected on Arduino Uno](Documentation/Readme/Demo/2Dboard.jpg)  


Details
=======

Pixel formats
-------------
* Supports multiple full-color pixel sizes:
  * 24 bit pixels (3 bytes),
  * 32 bit pixels (4 bytes, one unused)
  * 16 bit pixels (5 bit per pixel, plus 3 bit brightness).
* Supports palettes for 1, 2, 4 and 8-bit per pixels:
  Define a palette array of 2, 4, 16 or 256 colors.
  Define a pixel array which stores the index of the color in the palette array.
  * Palette management is done on the fly with the bit-banging so the library
    It does not waste memory allocating a temporary pixel unlike other libraries.
  * With 1 bit per pixels, you can draw patterns on a strip with over
    6,000 pixels (756 bytes) with a Uno, before running out of memory.
* LED sendPixels() calls can be chained.
  * To repeat a pattern or write different smaller pixel arrays, it is
    repeat calls to the `sendPixels()` method.
* Convenient pixel types allow you to access pixel colors without display errors.
  * Types: rgb, grb, bgr, rgbw...
  * Use the type native to your LED strip to have the most efficient code.
  * LED library will automatically transform the pixel color order on the fly if
    you do not use the native type for the LED strip you are using. The LEDs will
    therefore show the right colors no matter which format you use, at the price
    of a slight loss of performance and extra code. This will be transparent to you.
  * If you use the native pixel type for your LED strip, you can also cast the
    pixel array back and forth to an untyped pixel array (uint8_t * or uint32_t *)
    to do manipulations. The untyped pixel arrays will be a bit more efficient to
    display with `sendPixels()`.

Bio
===

I just recently started playing with Arduino in 2015, for my own art projects.
I have been using Adafruit's library which is great, but I've outgrown it:

In my professional life I have decades of C and performance engineering for OS and compilers, and I strive to use
the most efficient solution with the smallest weakest controller that can fullfill my results requirements.
Among other things I really like the AtTiny85, and wiring it directly to make very compact projects. However I want
them to really shine. With the Adafruit library I've run out of RAM to make cool stuff, so I decided to build my
own library from scratch. Reading <i>Josh.com</i> blog, I got to understand what makes ws2812b LEDs tick, and it
opened the doors to this. It is an opportunity for me to contribute to the community something I think is cool,
instead of just coding for my corporate master.

I do not believe performance, memory footprint and functionality are antinomic and that one has to make compromises
and sacrifices. With FAB_LED, I try to provide very fast code (so you can chain the `sendPixels()` calls for example),
super tight memeory management (so you can have more value add code and control more pixels), and leverage the simplified
interfaces to actually provide more artistic possibilities.

This exercise is also to prove to myself I can use C++ to write good, efficient code. Up to now, I have been
very skeptical of C++ and OO programming in general looking at thee resulting code programmers usually generate.
I was also very skeptical as a CS architect sensitized to memory behavior when I first saw STL templates and the notion of containers.
I have now learned that template constants are actually very useful, and more powerful than the C `#define` macro alternatives,
and that C++ syntax with strict coding style can actually be beneficial to generate beautiful code.

Dan- Apr 2016.

Planned Features
================

* Improved Palette support. I need to make palettes support pixel types, and provide very intuitive manipulation routines.
* Pixel remapping: I want to implement a map of all the pixels in space, to remap a 2D array ilogical layout in memory to the physical layout of the LEDs. Example of use cases:
  * For practical reasons, multiple LEDs have data wired differently than the order they are supposed to trigger, because of the geometry of the art project.
* 2D size support:
  pictures may be bigger or smaller than the LED display. There is a need to do an alignment on display knowing the dimension of the original picture, and the offset where to display it.
* 2D/3D LED display remapping support:
   For multi-dimensional projects, the physical layout may not be a rectangle, or may have holes. Even if the logical display is kept as a 2D rectangle to simplify animations, the mapping will allow skipping display of non-existing pixels, or as above, light up pixels in an order different than the logical order if the data lines are routed to minimize wire length.
* 2D sprite support:
  I'd like to be able to move an array into a display array, possibly as an overlay, to provide basic animation possibilities.

Validation Tests
================

This table shows the hardware FAB_LED has been tried on successfully.

Hardware          | Test    | Tester
---               |:---:    |-----:
`Controllers`     |         |
Arduino Uno       | Pass    | sonyhome
AtTiny85 16MHz    | Pass    | sonyhome
                  |         |
`LEDs`            |         |
ws2812b           | Pass    | sonyhome
Apa-104           | Pass    | sonyhome
Apa-106           | Pass    | sonyhome
sk6812            | Maybe   | trash (tested as 3 byte, not 4 byte)


Releases
========

There is no official 1.0 release yet: This is still beta code, and I am requesting help from
3rd party to try the code and report issues, and engage with me to discuss features, interfaces and
design style. However I do my best to make sure the code is stable and bug free.

* April 15, 2016 Added support for sk6812, added pixel types, rewrote the readme documentation.  
* April 01, 2016 Beta with 3 examples, one for infinite LED strips, one printing stats to Serial console,
                 hearbeat on pin13/PortB5, the LED on the Arduino Uno board
                 Added apa104, ws2812b and ws2812 classes. The ws2812b class by default will have the most
                 aggressive timings to push data 2X faster to the strips.
                 Bug fixes: example function declarations reformated to compile on Mac, slowed 1H timing
                 to work with more LED strip types.  
* March 23, 2016 Beta with one example for 24bit pixels.  
