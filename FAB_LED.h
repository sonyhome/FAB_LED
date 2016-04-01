////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Fast Addressable LED Library
// Copyright (c)2015 Dan Truong
//
// Licence: limited use is authorized for the purpose of beta testing
//
// The goal of this library is to be very fast, very small and take advantage
// of the relaxed LED timings to allow other work besides bit banging while the
// LEDs are being updated.
// The library allows calling multiple times the same function to repeat
// patterns, and allows inline color palette management to save on memory space
// usage. The palette conversion is done inline as the pixels are being painted.
//
// The bit-banging is being done using GCC built-ins to avoid contrived ASM
// language code blocks support. It should work for any arduino platform of
// 16MHz or more to function.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifndef FALL_H
#define FALL_H

#include <stdint.h>
#include <Arduino.h>

// This code is sensitive to optimizations if you want to cascade function calls,
// so make the IDE compile at -O2 instead of -Os (size).
#pragma GCC optimize ("-O2")

/// @brief Helper macro for palette index encoding into a char * array
// For example, for encoding colors on 2 bits (4 colors, 0,1,2,3, in the palette),
// Encoding color #3, using 2 bits per pixels, in the buffer for pixel i:
// SET_PIXEL(buffer, i, 2, 3);
#define SET_PIXEL(array, index, bitsPerPixel, color) \
	_SET_PIXEL((array), (index), (bitsPerPixel), (color))

#define _SET_PIXEL(array, index, bitsPerPixel, color)                          \
		array[index/(8/bitsPerPixel)] = (                              \
			array[index/(8/bitsPerPixel)] &                        \
			~(((1<<bitsPerPixel)-1) << ((index * bitsPerPixel)%8)) \
		) | color << ((index * bitsPerPixel)%8)

#define GET_PIXEL(array, index, bitsPerPixel) \
	_GET_PIXEL((array), (index), (bitsPerPixel))

#define _GET_PIXEL(array, index, bitsPerPixel) (                                 \
			array[index/(8/bitsPerPixel)] >> (index *bitsPerPixel)%8 \
		) & ((1<<bitsPerPixel)-1)

/// @brief Helper routine to compute the size of a char * array using a palette
/// with a resolution of N bits per pixels.
#define ARRAY_SIZE(numPixels, bitsPerPixel) (((numPixels)+7) / 8 * (bitsPerPixel))


/// @brief Unused: Attempt to implement assertions
#define CONCAT_TOKENS( TokenA, TokenB )       TokenA ## TokenB
#define EXPAND_THEN_CONCAT( TokenA, TokenB )  CONCAT_TOKENS( TokenA, TokenB )
#define ASSERT( Expression )                  enum{ EXPAND_THEN_CONCAT( ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) }
#define ASSERTM( Expression, Message )        enum{ EXPAND_THEN_CONCAT( Message ## _ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) } 

/// @brief Conversion between cycles and nano seconds
#define NS_PER_SEC  1000000000ULL
#define CYCLES_PER_SEC ((uint64_t) (F_CPU))
#define CYCLES(time_ns) (((CYCLES_PER_SEC * (time_ns)) + NS_PER_SEC - 1ULL) / NS_PER_SEC)
#define NANOSECONDS(cycles) (((cycles) * NS_PER_SEC + CYCLES_PER_SEC-1) / CYCLES_PER_SEC)

////////////////////////////////////////////////////////////////////////////////
// AVR (Arduino) bitBang LED support class, implements bitBang based sentBytes()
////////////////////////////////////////////////////////////////////////////////

// AVR ports are referenced A through D by our template...
enum avrLedStripPort {
	A = 1,
	B = 2,
	C = 3,
	D = 4,
	E = 5,
	F = 6
};

////////////////////////////////////////////////////////////////////////////////
/// Macros hack:
/// avoid compilation errors for arduinos that are missing some of the 4 ports

// Define a DUMMY_PORT_ID that will be used to patch unknown ports to map to
// the first existing port we find. Undefined ports won't be used but this
// lets the compiler work without complaining.
#if defined(PORTD)
#define DUMMY_PORT PORTD
#define DUMMY_DDR   DDRD
#elif defined(PORTB)
#define DUMMY_PORT PORTB
#define DUMMY_DDR   DDRB
#elif defined(PORTA)
#define DUMMY_PORT PORTA
#define DUMMY_DDR   DDRA
#elif defined(PORTC)
#define DUMMY_PORT PORTC
#define DUMMY_DDR   DDRC
#endif // PORT

// Now if any of the ports we support does not exist, re-map it to the dummy port.
#ifdef DUMMY_PORT
#ifndef PORTA
#define PORTA DUMMY_PORT
#define  DDRA DUMMY_DDR
#endif // PORTA
#ifndef PORTB
#define PORTB DUMMY_PORT
#define  DDRB DUMMY_DDR
#endif // PORTB
#ifndef PORTC
#define PORTC DUMMY_PORT
#define  DDRC DUMMY_DDR
#endif // PORTC
#ifndef PORTD
#define PORTD DUMMY_PORT
#define  DDRD DUMMY_DDR
#endif // PORTD
#ifndef PORTE
#define PORTE DUMMY_PORT
#define  DDRE DUMMY_DDR
#endif // PORTE
#ifndef PORTF
#define PORTF DUMMY_PORT
#define  DDRF DUMMY_DDR
#endif // PORTF
#endif // DUMMY_PORT

// These macros access DDRn and PORTn I/O registers
#ifdef PORTB
// AVR/Arm processors on Arduino platform - They always have a port B
#define AVR_DDR(id) _AVR_DDR((id))
#define _AVR_DDR(id) ((id==A) ? DDRA : (id==B) ? DDRB : (id==C) ? DDRC : \
		(id==D) ? DDRD : (id==E) ? DDRE : DDRF)
#define AVR_PORT(id) _AVR_PORT((id))
#define _AVR_PORT(id) ((id==A) ? PORTA : (id==B) ? PORTB : (id==C) ? PORTC : \
		(id==D) ? PORTD : (id==E) ? PORTE : PORTF)

#else // not(PORTB)
// Non Arduino architecture - I dunno if I can configure the I/O ports
// End-user must redefine AVR_DDR and AVR_PORT
#error "Unsupported Architecture"
#endif // PORTB



////////////////////////////////////////////////////////////////////////////////
// Base class defining LED strip operations allowed.
////////////////////////////////////////////////////////////////////////////////
static const uint8_t blank[3] = {128,128,128};

/// @brief Class to drive LED strips. Relies on custom sendBytes() method to push data to LEDs
template<
	int16_t high1,          // Number of cycles high for logical one
	int16_t low1,           // Number of cycles  low for logical one
	int16_t high0,          // Number of cycles high for logical zero
	int16_t low0,           // Number of cycles  low for logical zero
	uint32_t minMsRefresh,  // Minimum milliseconds to wait to reset LED strip data stream
	avrLedStripPort portId, // AVR port the LED strip is attached to
	uint8_t portPin         // AVR port bit the LED strip is attached to
> class avrBitbangLedStrip
{
	public:

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends N bytes to the LED using bit-banging.
	///
	/// @param[in] count Number of bytes sent
	/// @param[in] array Array of bytes to send
	///
	/// Caller must handle interupts!
	/// If interupts are not off, the timer interupt will happen, and it takes
	/// too long and will cause a LED strip reset.
	///
	/// __attribute__ ((always_inline)) forces gcc to inline this function in
	/// the caller, to optimize there and avoid function call overhead. Our
	/// timing is critical at 16MHz.
	///
	/// __builtin_avr_delay_cycles() is a gcc built-in that will generate ASM
	/// to implement the exact number of cycle delays specified using the most
	/// compact instruction loop.
	///
	/// The AVR port bit-set math will be changed by gcc to a sbi/cbi instruction
	///
	/// @note 74B for the call itself
	//void
	//sendBytes(const uint16_t count, const uint8_t * array)
	static inline void
	sendBytes(const uint16_t count, const uint8_t * array) __attribute__ ((always_inline))
	{
		const int sbiCycles = 2;
		const int cbiCycles = 2;

		// Debug: Verify values make sense.
		ASSERT(high1 >= sbiCycles);
		ASSERT(low1  >= cbiCycles);
		ASSERT(high0 >= sbiCycles);
		ASSERT(low0  >= cbiCycles);

		for(uint16_t c=0; c < count; c++) {
			const uint8_t val = array[c];
			for(int8_t b=7; b>=0; b--) {
				const bool bit = (val>>b) & 0x1;
 
 				if (bit) {
					// Send a ONE

					// HIGH with ASM sbi (2 words, 2 cycles)
					AVR_PORT(portId) |= 1U << portPin;
					// Wait exact number of cycles specified
					__builtin_avr_delay_cycles(high1 - sbiCycles);
					//  LOW with ASM cbi (2 words, 2 cycles)
					AVR_PORT(portId) &= ~(1U << portPin);
					// Wait exact number of cycles specified
					__builtin_avr_delay_cycles(low1 - cbiCycles);
				} else {
					// Send a ZERO

					// HIGH with ASM sbi (2 words, 2 cycles)
					AVR_PORT(portId) |= 1U << portPin;
					// Wait exact number of cycles specified
					__builtin_avr_delay_cycles(high0 - sbiCycles);
					//  LOW with ASM cbi (2 words, 2 cycles)
					AVR_PORT(portId) &= ~(1U << portPin);
					// Wait exact number of cycles specified
					__builtin_avr_delay_cycles(low0 - cbiCycles);
				}
			}
		}
	}

	////////////////////////////////////////////////////////////////////////
	/// @brief Constructor: Set selected portId.portPin to digital output
	////////////////////////////////////////////////////////////////////////
	avrBitbangLedStrip()
	{
		// Digital out pin mode
		// bitSet(portDDR, portPin);
		// DDR? |= 1U << this->portPin;
		AVR_DDR(portId) |= 1U << portPin;

		AVR_PORT(portId) &= ~(1U << portPin);
	};

	////////////////////////////////////////////////////////////////////////
	/// @brief Destructor.
	////////////////////////////////////////////////////////////////////////
	~avrBitbangLedStrip()
	{ };

	////////////////////////////////////////////////////////////////////////
	/// @brief Prints to console the configuration
	/// @note You must implement the print routines (see example)
	////////////////////////////////////////////////////////////////////////
	template <void printChar(const char *),void printInt(uint32_t)>
	static inline void
	debug(void)
	{
		printChar("\nclass avrBitbangLedStrip<...>\n");

		printInt(CYCLES_PER_SEC/1000000);
		printChar("MHz CPU, ");
		printInt(NANOSECONDS(1000));
		printChar(" picoseconds per cycle\n");

		printChar("ONE  HIGH=");
		printInt(high1);
		printChar(" LOW=");
		printInt(low1);
		printChar(" cycles\n");

		printChar("ZERO HIGH=");
		printInt(high0);
		printChar(" LOW=");
		printInt(low0);
		printChar(" cycles\n");

		printChar("REFRESH MSEC=");
		printInt(minMsRefresh);
		printChar("\n");

		switch(portId) {
			case A:
				printChar("PORT A.");
				break;
			case B:
				printChar("PORT B.");
				break;
			case C:
				printChar("PORT C.");
				break;
			case D:
				printChar("PORT D.");
				break;
			case E:
				printChar("PORT E.");
				break;
			case F:
				printChar("PORT F.");
				break;
			default:
				printChar("PORT UNKNOWN.");
		}
		printInt(portPin);
		printChar("\n");
	}


	////////////////////////////////////////////////////////////////////////
	/// @brief Clears the LED strip
	/// @param[in] numPixels  Number of pixels to erase
	////////////////////////////////////////////////////////////////////////
	static inline void clear( const uint16_t numPixels); __attribute__ ((always_inline))

	////////////////////////////////////////////////////////////////////////
	/// @brief Sets the LED strip to a grey value
	/// @param[in] numPixels  Number of pixels to erase
	/// @param[in] value      Brightness of the grey
	////////////////////////////////////////////////////////////////////////
	static inline void grey( const uint16_t numPixels, const uint8_t value) __attribute__ ((always_inline))
	{
		// Disable interupts
		uint8_t oldSREG = SREG;
 		__builtin_avr_cli();

		const uint8_t array[3] = {value, value, value};
		for( uint16_t i = 0; i < numPixels; i++) {
			sendBytes(3, array);
		}

		// Restore interupts
		SREG = oldSREG;
	}

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends a raw 3-byte pixels to the LED strip.
	/// This function handles the most compact form of 24bit/pixel arrays.
	///
	/// @param[in] numPixels Number of pixels to write
	/// @param[in] array     Array of 3-bytes per pixels in native order
	///                      (for example GBR for WS2812B LED strips)
	////////////////////////////////////////////////////////////////////////
	///
	/// @note 100B - tested, CAN chain.
	static inline void sendPixels(
			const uint16_t numPixels,
			const uint8_t * array) __attribute__ ((always_inline))
	{
		// Disable interupts
		uint8_t oldSREG = SREG;
		// cli();
 		__builtin_avr_cli();

		sendBytes(numPixels * 3, array);

		// Restore interupts
		SREG = oldSREG;
	}


	////////////////////////////////////////////////////////////////////////
	/// @brief Sends an array of 32bit words encoding 0x00bbrrgg to the LEDs
	/// This is the standard encoding for most libraries and wastes 25% of
	/// the SRAM.
	///
	/// @param[in] numPixels Number of pixels to write
	/// @param[in] array     Array of 1 word per pixels in native order
	///                      (most significant byte is ignored)
	////////////////////////////////////////////////////////////////////////
	///
	/// - tested, can chain.
	static inline void sendPixels(
			const uint16_t numPixels,
			const uint32_t * pixelArray)
	{
		// Disable interupts
		uint8_t oldSREG = SREG;
		//cli();
 		__builtin_avr_cli();

		for (int i=0; i< numPixels; i++) {
			uint8_t * bytes = (uint8_t *) & pixelArray[i];
			// Send lower 3 bytes, drop most significant byte
			sendBytes(3, bytes);
		}

		// Restore interupts
		SREG = oldSREG;
	}

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends an array of bits encoded with a palette to the LEDs.
	/// This is a compact method to store patterns, compressed 3x to 24x,
	/// but with the overhead of a palette array.
	///
	/// @param[in] count   Size of the array in bytes
	/// @param[in] array   Array of bitsPerPixel bits per pixels, where the
	///                    number of bits is 1,2, 4 or 8.
	/// @param[in] palette Palette array with one entry per color used.
	///
	/// 1bit  ->   6B palette
	/// 2bits ->  12B palette
	/// 4bits ->  48B palette
	/// 8bits -> 768B palette
	///
	/// @note bitsPerPixel is a template constant to allow the compiler to
	/// optimize the bit-masking code.
	////////////////////////////////////////////////////////////////////////
	///
	/// 342B - tested, can't chain.


	template <const uint8_t bitsPerPixel>
	static inline void sendPixels (
			const uint16_t count,
			const uint8_t * pixelArray,
			const uint8_t * palette)
	{
		// Debug: Support simple palettes 2, 4, 16 or 256 colors
		ASSERT( bitsPerPixel == 1 || bitsPerPixel == 2 ||
			bitsPerPixel == 4 || bitsPerPixel == 8);

		// 1,2 4 and 8 bit bitmasks. Note 3,5,6 don't make sense
		// and 5bits is handled separately with uint16_t type.
		const uint8_t andMask = (bitsPerPixel ==1) ? 0x01 :
				(bitsPerPixel == 2) ? 0x03 :
				(bitsPerPixel == 4) ? 0x0F :
				(bitsPerPixel == 8) ? 0xFF :
				0x00;

		// Disable interupts
		//const uint8_t oldSREG = SREG;
 		//__builtin_avr_cli();
		// Send each byte as 1 to 4 pixels
		uint16_t offset;
		offset = 0;
		uint16_t index;
		index = 0;
		while (1) {
			uint8_t elem; 
			elem = pixelArray[offset++]; 
			for (uint8_t j = 0; j < 8/bitsPerPixel; j++) {
				if (index++ >= count) {
					goto end;
				}
				const uint8_t colorIndex = elem & andMask;
				sendBytes(3, &palette[3*colorIndex]);
				elem >>= bitsPerPixel;
			}
		}
end:
		// Restore interupts
		//SREG = oldSREG;
		return;
	}
	////////////////////////////////////////////////////////////////////////
	/// @brief Sends an array of 3 pixels per 16bit words to the LEDs
	/// This yelds 32K colors, and saves 33% RAM without using a palette. 
	/// 
	/// the SRAM.
	/// @brief Sends an array of 16-bit words to the LEDs. Each word encodes
	/// one pixel with 64 levels (5 bits)
	//
	/// @note brightness is a template constant that controls the unused 3
	/// bits of the color. Its value is from zero to 2. Often set to 0 when
	/// LED strip is too bright.
	////////////////////////////////////////////////////////////////////////
	template <uint8_t brightness>
	static inline void sendPixels(int count, uint16_t * pixelArray) {
		const uint8_t mask5 = ((1 << (5 - brightness))-1) || ((1 << brightness) - 1);
		const uint8_t mask10 = ((1 << (10 - brightness))-1) || ((1 << brightness) - 1);
		uint8_t bytes[3];

		// Debug: Support brightness 0..3
		ASSERT(brightness < 3);

		// Disable interupts
		uint8_t oldSREG = SREG;
		cli();

		for (int i = 0; i < count; i++) {
			const uint16_t elem = pixelArray[i]; 
			bytes[0] = (uint8_t) elem << brightness;
			bytes[1] = (elem >> (5 - brightness)) & mask5;
			bytes[2] = (elem >> (10 - brightness)) & mask10;
			sendBytes(3, bytes);
		}

		// Restore interupts
		SREG = oldSREG;
	}


	void refresh() {
		delay(minMsRefresh);
	}

	////////////////////////////////////////////////////////////////////////
	/// @brief Convert RGB pixels to native WS2812B GBR format
	////////////////////////////////////////////////////////////////////////
	static inline void RGBtoGBR(uint8_t * pixel) {
		uint8_t t = pixel[2];
		pixel[2]=pixel[0];
		pixel[0]=pixel[1];
		pixel[1]=t;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	static inline void RGBtoGBR(uint32_t * pixel) {
		RGBtoGBR(&((uint8_t*)pixel)[1]);
	}

	////////////////////////////////////////////////////////////////////////
	/// @brief Returns minimum value to delay() between frame refrreshes so
	/// that pixels are fully lit, and the next write starts a new display
	/// cycle.
	////////////////////////////////////////////////////////////////////////
	uint32_t minRefreshDelay() {
		return minMsRefresh;
	}
};


////////////////////////////////////////////////////////////////////////////////
/// Class methods definition
////////////////////////////////////////////////////////////////////////////////

#define FAB_TDEF template<      \
	int16_t high1,          \
	int16_t low1,           \
	int16_t high0,          \
	int16_t low0,           \
	uint32_t minMsRefresh,  \
	avrLedStripPort portId, \
	uint8_t portPin>

#define FAB_TVAR high1, low1, high0, low0, minMsRefresh, portId, portPin


FAB_TDEF
inline void
avrBitbangLedStrip<FAB_TVAR>::clear( const uint16_t numPixels)
{
	// Disable interupts
	uint8_t oldSREG = SREG;
 	__builtin_avr_cli();

	const uint8_t array[3] = {0,0,0};
	for( uint16_t i = 0; i < numPixels; i++) {
		sendBytes(3, array);
	}

	// Restore interupts
	SREG = oldSREG;
}


////////////////////////////////////////////////////////////////////////////////
// Implementation classes for LED strip
// Defines the actual LED timings
// WS2811 2811B 2812B 2812S 2801 LEDs use similar protocols and timings
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// WS2812B (default, mainstream)
////////////////////////////////////////////////////////////////////////////////
#if 0
// These are the less agressive bitbanging timings
#define WS2812B_1H_CY CYCLES(650)  // 650ns-950ns _----------__
#define WS2812B_1L_CY CYCLES(125)  // 250ns-550ns .    .    .
#define WS2812B_0H_CY CYCLES(125)  // 250ns-550ns _-----_______
#define WS2812B_0L_CY CYCLES(650)  // 650ns-950ns .    .    .
#define WS2812B_MS_REFRESH 50      // 50,000ns Minimum sleep time to reset LED strip
#define WS2812B_NS_RF 2000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#else
// These are the more agressive bitbanging timings, note 0 and 1 have different durations
#define WS2812B_1H_CY CYCLES(500)  // 6  7 10 _----------__
#define WS2812B_1L_CY CYCLES(125)  // 2  2  4 .    .    .
#define WS2812B_0H_CY CYCLES(125)  // 2  2  5 _-----_______
#define WS2812B_0L_CY CYCLES(188)  // 2  2  7 .    .    .
#define WS2812B_MS_REFRESH 20      // Minimum sleep time (low) to reset LED strip
#define WS2812B_NS_RF 2000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#endif

#define FAB_TVAR_WS2812B WS2812B_1H_CY, WS2812B_1L_CY, WS2812B_0H_CY, \
	WS2812B_0L_CY, WS2812B_MS_REFRESH, portId, portBit
template<avrLedStripPort portId, uint8_t portBit>
class ws2812b : public avrBitbangLedStrip<FAB_TVAR_WS2812B>
{
	public:
	ws2812b() : avrBitbangLedStrip<FAB_TVAR_WS2812B>() {};
	~ws2812b() {};
};
#undef FAB_TVAR_WS2812B


////////////////////////////////////////////////////////////////////////////////
// WS2812 (1st generation of LEDs)
////////////////////////////////////////////////////////////////////////////////
#define WS2812_1H_CY CYCLES(550)  // 500ns 550ns-850ns  _----------__
#define WS2812_1L_CY CYCLES(200)  // 125ns 200ns-500ns  .    .    .
#define WS2812_0H_CY CYCLES(200)  // 125ns 200ns-500ns  _-----_______
#define WS2812_0L_CY CYCLES(550)  // 500ns 550ns-850ns  .    .    .
#define WS2812_MS_REFRESH 50      //  50,000ns Minimum wait time to reset LED strip
#define WS2812_NS_RF 5000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_WS2812 WS2812_1H_CY, WS2812_1L_CY, WS2812_0H_CY, \
	WS2812_0L_CY, WS2812_MS_REFRESH, portId, portBit
template<avrLedStripPort portId, uint8_t portBit>
class ws2812 : public avrBitbangLedStrip<FAB_TVAR_WS2812>
{
	public:
	ws2812() : avrBitbangLedStrip<FAB_TVAR_WS2812>() {};
	~ws2812() {};
};
#undef FAB_TVAR_WS2812


////////////////////////////////////////////////////////////////////////////////
// APA104 (newer better defined timings)
////////////////////////////////////////////////////////////////////////////////
#define APA104_1H_CY CYCLES(1210) // 500ns 1210ns-1510ns _----------__
#define APA104_1L_CY CYCLES(200)  // 125ns  200ns-500ns  .    .    .
#define APA104_0H_CY CYCLES(200)  // 125ns  200ns-500ns  _-----_______
#define APA104_0L_CY CYCLES(1210) // 500ns 1210ns-1510ns .    .    .
#define APA104_MS_REFRESH 50      //  50,000ns Minimum wait time to reset LED strip
#define APA104_NS_RF 5000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_APA104 APA104_1H_CY, APA104_1L_CY, APA104_0H_CY, \
	APA104_0L_CY, APA104_MS_REFRESH, portId, portBit
template<avrLedStripPort portId, uint8_t portBit>
class apa104 : public avrBitbangLedStrip<FAB_TVAR_APA104>
{
	public:
	apa104() : avrBitbangLedStrip<FAB_TVAR_APA104>() {};
	~apa104() {};
};
#undef FAB_TVAR_APA104
#define pl9823 apa104; 

////////////////////////////////////////////////////////////////////////////////
/// @brief SK6812 LED strips - Same as WS2812 except faster PWM refresh rate
/// @note I need to use microsleep to not round up sleep to 1msec.
////////////////////////////////////////////////////////////////////////////////
#define SK6812_NS_RF 833333 // Minimum refresh rate for all pixels to light up (LED PWM is 1200Hz)
#define SK6812_MS_REFRESH (((WS2812B_NS_RS>SK6812_NS_RF) ? WS2812B_NS_RS : SK6812_NS_RF) +999999)/1000000

#endif // FALL_H
