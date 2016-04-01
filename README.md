# FAB_LED
Fast Arduino Bitbang (FAB) LED library.

Installation:

Unzip the FAB_LED_main directory and move it into your Arduino library directory
(user/Documents/Arduino/libraries) and rename the directory FAB_LED. Launch the
Arduino IDE, and you should see in the examples a new FAB_LED entry. Try them.


Why FAB_LED:

FAB_LED supports WS2812B and similar LEDs, on Arduino AVR and ARM platform,
8MHz and above.

Support for other platforms may require implementing a replacement for
accessing the I/O ports.

Benefits of the FAB_LIB over other Arduino Addressable LED libraries:

* Support any LED strip size (limited to 64K pixels only because it
  uses a uint16_t to declare the LED strip size, ut easy to convert
  to uint32_t)
* Supports multiple pixel formats:
  * 24 bit pixels (3 bytes),
  * 32 bit pixels (4 bytes, one unused)
  * 16 bit pixels (5 bit per pixel, plus 3 bit brightness).
  The library can allocate at least 255 pixels at 24 bits on Arduino Uno.
* Supports 1, 2, 4 and 8-bit pixels:
  Define an associated palette array of 2, 4, 16 or 256 colors.
  Palette management is done on the fly with bit-banging so the library
  does not waste memory allocating a temporary pixel buffer to implement
  bit-banging, aka writting the pixels to the LED strip.
  With 1 bit per pixels, you can draw patterns on a strip with over
  6,000 pixels with a Uno, before running out of memory.
* LED strip writes can be chained:
  to repeat a pattern or write different smaller pixel arrays, it is
  possible to repeat calls to the write method.
  This allows support for an infinite number of pixels, repeating the
  same pattern over and over. You can even combine calls applied to
  pixel arrays defined with different bit-resolutions.

Unlike Adafruit NeoPixel and other LED libraries, this library's sendPixels()
writes to the LED strip directly, not to a pixel buffer. There is no need to
call a draw routine afterwards.

The library's bit-banging core code should work on most CPUs, for all
frequencies over 8MHz. It is written in C, and the compiler adjusts the timings
at compile time. In contrast, other libraries implement bit-banging in assembly
and only support CPU frequencies that they have been hardcoded.

# Releases

April 01, 2016 - Beta with 3 examples, one for infinite LED strips, one printing stats to Serial console,
                 hearbeat on pin13/PortB5, the LED on the Arduino Uno board
                 Added apa104, ws2812b and ws2812 classes. The ws2812b class by default will have the most
                 aggressive timings to push data 2X faster to the strips.
                 Bug fixes: example function declarations reformated to compile on Mac, slowed 1H timing
                 to work with more LED strip types.
March 23, 2016 - Beta with one example for 24bit pixels.
