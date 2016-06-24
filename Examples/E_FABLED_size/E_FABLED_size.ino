#pragma GCC optimize ("-O2")

#include <FAB_LED.h>
ws2812b<D,6> myLeds;

////////////////////////////////////////////////////////////////////////////////
// This is an experiment to see how much memory FAB_LED uses.
// To verify, compile, then comment out all FAB_LED related code
// and recompile.
//
// 2454 - 209 All code
// 2350 - 209 clear off     -> 104 B
// 2344 - 209 sendPixel off -> 110 B
// 2220 - 209 both off      ->  20 B (sendBytes?)
// 2204 - 208 both off, object commented -> 16 B (globals)
// 2204 - 208 no fab_led
//
// FAB_LED flash memory footprint: 250B, RAM: 1B when using clear and sendPixels
// FAB_LED flash memory footprint: 146B, RAM: 1B when using sendPixels alone
////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Configure Serial for printing.
  Serial.begin(9600);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief This method is automatically called repeatedly after setup() has run.
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  uint8_t pix[3*8] = {};
  pix[7] = 16;
  Serial.print("pix[7]=");
  Serial.print(pix[7]);
  Serial.print("\n");

  myLeds.clear(1000);
  delay(1000);
  myLeds.draw(8, pix);
  delay(1000);
}
