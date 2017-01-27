These examples all do the same boring thing:
blink LEDs red, green, blue and white.

Each example represents a different a different LED protocol.


Special multi-port protocols:

ws2812bs an ws2812bi are used to connect 2 LED strips and displaying half of the pixels on one strip and the other half on the second strip. The ws2812bs mode splits the first half of the array to the first strip and the second half to the other strip. The ws2812bi interleaved mode sends every other pixel to the one strip then the other.

ws2812b8s is a mode that allows up to 8 ports (same letter port) to be displayed in parallel for faster LED strip refreshes. At 16MHz, this mode can support up to 6 ports before glitches appear.


The predefined LED strip types are:

* ws2812b
* apa104
* apa106
* sk6812 /sk6812b
* apa102
