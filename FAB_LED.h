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
#ifndef FAB_LED_H
#define FAB_LED_H

#include <stdint.h>
#include <Arduino.h>

// This code is sensitive to optimizations if you want to cascade function calls,
// so make the IDE compile at -O2 instead of -Os (size).
#pragma GCC optimize ("-O2")

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper macro for palette index encoding into a char * array
// For example, for encoding colors on 2 bits (4 colors, 0,1,2,3, in the palette),
// Encoding color #3, using 2 bits per pixels, in the buffer for pixel i:
// SET_PIXEL(buffer, i, 2, 3);
////////////////////////////////////////////////////////////////////////////////
#define SET_PIXEL(array, index, bitsPerPixel, color)          \
	_SET_PIXEL((array), (index), (bitsPerPixel), (color))

#define _SET_PIXEL(array, index, bitsPerPixel, color)                          \
		array[index/(8/bitsPerPixel)] = (                              \
			array[index/(8/bitsPerPixel)] &                        \
			~(((1<<bitsPerPixel)-1) << ((index * bitsPerPixel)%8)) \
		) | color << ((index * bitsPerPixel)%8)

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper macro for palette manipulation, extracts a pixel from a pixel
/// array
////////////////////////////////////////////////////////////////////////////////
#define GET_PIXEL(array, index, bitsPerPixel)        \
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
#define NS_PER_SEC          1000000000ULL
#define CYCLES_PER_SEC      ((uint64_t) (F_CPU))
#define CYCLES(time_ns)     (((CYCLES_PER_SEC * (time_ns)) + NS_PER_SEC - 1ULL) / NS_PER_SEC)
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

// Declares the byte order for every color of the LED strip
enum pixelFormat {
	NONE = 0, // Defaults to 3 bytes per pixel of unspecified order
	RGB = 1,
	GRB = 2,
	BGR = 3,
	RGBW = 4,
	GRBW = 5,
	BGRW = 6
};

// Declares the type of hardware protocol for the LED strip
enum ledProtocol {
	ONE_WIRE_BITBANG = 1, // ws2812 and any LED with single data line
	ONE_WIRE_PWM = 2,     // Not implemented
	ONE_WIRE_UART = 3,    // Not implemented
	SPI_BITBANG = 4,      // APA-102 and any LED with data and clock line
	SPI_HARDWARE = 5      // Not implemented
};

#define PIXEL_FORMAT_4B RGBW
#define PROTOCOL_SPI SPI_BITBANG

// Defines pixel structures for every format supported. It can be used
// to simplify access to a pixel byte array, or even to send typed pixels
// to the LED strip.
typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
} rgbw;

typedef struct {
	uint8_t g;
	uint8_t r;
	uint8_t b;
	uint8_t w;
} grbw;

typedef struct {
	uint8_t b;
	uint8_t g;
	uint8_t r;
	uint8_t w;
} bgrw;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb;

typedef struct {
	uint8_t g;
	uint8_t r;
	uint8_t b;
} grb;

typedef struct {
	uint8_t b;
	uint8_t g;
	uint8_t r;
} bgr;

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

#define DELAY_CYCLES(count) __builtin_avr_delay_cycles(count);
#define SET_DDR_HIGH( portId, portPin) AVR_DDR(portId)  |= 1U << portPin
#define SET_PORT_HIGH(portId, portPin) AVR_PORT(portId) |= 1U << portPin
#define SET_PORT_LOW( portId, portPin) AVR_PORT(portId) |= 1U << portPin
#define DISABLE_INTERRUPTS __builtin_avr_cli()

#else // not(PORTB)
// Non Arduino architecture - I dunno if I can configure the I/O ports
// End-user must redefine AVR_DDR and AVR_PORT
//#error "Unsupported Architecture"
//#define DELAY_CYCLES(count) __builtin_arm_delay_cycles(count);
//#define DELAY_CYCLES(count) SysTick_Wait(count)
#define DELAY_CYCLES(count) delay(count)

#define SET_DDR_HIGH( portId, portPin)
#define SET_PORT_HIGH(portId, pinId)   digitalWriteFast(pinId, 1)
#define SET_PORT_LOW( portId, pinId)   digitalWriteFast(pinId, 0)
#define DISABLE_INTERRUPTS cli()

//mov r0, #COUNT
//L:
//subs r0, r0, #1
//bnz L
#define MACRO_CMB( A , B)           A##B
#define M_RPT(__N, __macro)         MACRO_CMB(M_RPT, __N)(__macro)
#define M_RPT0(__macro)
#define M_RPT1(__macro)             M_RPT0(__macro)   __macro(0)
#define M_RPT2(__macro)             M_RPT1(__macro)   __macro(1)
//...
#define MY_NOP(__N)                 __asm ("nop");
#define delay150cycles M_RPT(150, MY_NOP);
#endif // PORTB



////////////////////////////////////////////////////////////////////////////////
// Base class defining LED strip operations allowed.
////////////////////////////////////////////////////////////////////////////////
static const uint8_t blank[3] = {128,128,128};

#define FAB_TDEF int16_t high1,             \
		int16_t low1,               \
		int16_t high0,              \
		int16_t low0,               \
		uint32_t minMsRefresh,      \
		avrLedStripPort dataPortId, \
		uint8_t dataPortPin,        \
		avrLedStripPort clockPortId,\
		uint8_t clockPortPin,       \
		pixelFormat colors,         \
		ledProtocol protocol

#define FAB_TVAR high1, low1, high0, low0, minMsRefresh, dataPortId, dataPortPin, clockPortId, clockPortPin, colors, protocol

/// @brief Class to drive LED strips. Relies on custom sendBytes() method to push data to LEDs
//template<
//	int16_t high1,          // Number of cycles high for logical one
//	int16_t low1,           // Number of cycles  low for logical one
//	int16_t high0,          // Number of cycles high for logical zero
//	int16_t low0,           // Number of cycles  low for logical zero
//	uint32_t minMsRefresh,  // Minimum milliseconds to wait to reset LED strip data stream
//	avrLedStripPort dataPortId, // AVR port the LED strip is attached to
//	uint8_t dataPortPin         // AVR port bit the LED strip is attached to
//>
template <FAB_TDEF>
class avrBitbangLedStrip
{
	static const uint8_t bytesPerPixel = (colors < PIXEL_FORMAT_4B) ? 3 : 4;

	public:
	////////////////////////////////////////////////////////////////////////
	/// @brief Constructor: Set selected dataPortId.dataPortPin to digital output
	////////////////////////////////////////////////////////////////////////
	avrBitbangLedStrip();
	////////////////////////////////////////////////////////////////////////
	~avrBitbangLedStrip() { };

	////////////////////////////////////////////////////////////////////////
	/// @brief Prints to console the configuration
	/// @note You must implement the print routines (see example)
	////////////////////////////////////////////////////////////////////////
	template <void printChar(const char *),void printInt(uint32_t)>
	static inline void debug(void);

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends N bytes to the LED using bit-banging.
	///
	/// @param[in] count Number of bytes sent
	/// @param[in] array Array of bytes to send
	///
	/// @warning
	/// The caller must handle interupts!
	/// If interupts are not off, the timer interupt will happen, and it takes
	/// too long and will cause a LED strip reset.
	///
	/// @note
	/// __attribute__ ((always_inline)) forces gcc to inline this function in
	/// the caller, to optimize there and avoid function call overhead. The
	/// timing is critical at 16MHz.
	/// @note
	/// __builtin_avr_delay_cycles() is a gcc built-in that will generate ASM
	/// to implement the exact number of cycle delays specified using the most
	/// compact instruction loop, and is portable.
	/// @note
	/// The AVR port bit-set math will be changed by gcc to a sbi/cbi in ASM
	////////////////////////////////////////////////////////////////////////
	static inline void
	sendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends N uint32_t in a row set to zero or 0xFFFFFFFF, to build
	/// a frame for each pixel, and for a whole strip, SPI protocol only
	////////////////////////////////////////////////////////////////////////
	static inline void
	spiSoftwareSendFrame(const uint16_t count, bool high)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @bried Implements sendBytes for the SPI protocol only
	////////////////////////////////////////////////////////////////////////
	static inline void
	spiSoftwareSendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Implements sendBytes for the 1-wire protocol (WS2812B)
	////////////////////////////////////////////////////////////////////////
	static inline void
	oneWireSoftwareSendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Clears the LED strip
	/// @param[in] numPixels  Number of pixels to erase
	////////////////////////////////////////////////////////////////////////
	static inline void clear( const uint16_t numPixels)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Sets the LED strip to a grey value
	/// @param[in] numPixels  Number of pixels to erase
	/// @param[in] value      Brightness of the grey [0..255]
	////////////////////////////////////////////////////////////////////////
	static inline void grey(
			const uint16_t numPixels,
			const uint8_t value)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends an array of 32bit words encoding 0x00bbrrgg to the LEDs
	/// This is the standard encoding for most libraries and wastes 25% of
	/// the SRAM.
	///
	/// @param[in] numPixels Number of pixels to write
	/// @param[in] array     Array of 1 word per pixels in native order
	///                      (most significant byte is ignored)
	////////////////////////////////////////////////////////////////////////
	static inline void sendPixels(
			const uint16_t numPixels,
			const uint32_t * pixelArray)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Sends 3-byte pixels to the LED strip.
	/// This function should be the most common used method.
	///
	/// @note for SK6812, which uses 4 bytes per pixels, this method will send
	/// 4 bytes per pixel and expect an array containing 4 bytes per pixel.
	/// @param[in] numPixels Number of pixels to write
	/// @param[in] array     Array of 3-bytes per pixels in native order
	///                      (for example GBR for WS2812B LED strips)
	////////////////////////////////////////////////////////////////////////
	static inline void sendPixels(
			const uint16_t numPixels,
			const uint8_t * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const rgbw * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const grbw * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const bgrw * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const rgb * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const grb * array) __attribute__ ((always_inline));

	static inline void sendPixels(
			const uint16_t numPixels,
			const bgr * array) __attribute__ ((always_inline));

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
	template <const uint8_t bitsPerPixel>
	static inline void sendPixels (
			const uint16_t count,
			const uint8_t * pixelArray,
			const uint8_t * palette) __attribute__ ((always_inline));

	template <const uint8_t bitsPerPixel, class paletteColors>
	static inline void sendPixels (
			const uint16_t count,
			const uint8_t * pixelArray,
			const paletteColors * palette) __attribute__ ((always_inline));

/*	template <pixelType> 
	static inline void sendPixels2D(
			const uint16_t numPixels,
			const pixelType * array,
			const uint16_t X,
			const uint16_t Y) __attribute__ ((always_inline));
*/

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
	static inline void sendPixels(int count, uint16_t * pixelArray) __attribute__ ((always_inline));



	////////////////////////////////////////////////////////////////////////
	/// @brief Waits long enough to trigger a LED strip reset
	////////////////////////////////////////////////////////////////////////
	static inline void refresh() {
		if (protocol == SPI_BITBANG) {
			// SPI: Reset LED strip to accept a refresh
			spiSoftwareSendFrame(1, 0);
		} else {
			// 1-wire: Delay next pixels to cause a refresh
			delay(minMsRefresh);
		}
	}

#if 0
	////////////////////////////////////////////////////////////////////////
	/// @brief Convert RGB pixels to native WS2812B GBR format
	////////////////////////////////////////////////////////////////////////
	static inline void RGBtoGRB(uint8_t * pixel) {
		uint8_t t = pixel[0];
		pixel[0]=pixel[1];
		pixel[1]=t;
	}

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	static inline void RGBtoGRB(uint32_t * pixel) {
		RGBtoGRB(&((uint8_t*)pixel)[1]);
	}
#endif
};


////////////////////////////////////////////////////////////////////////////////
/// Class methods definition
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
template<FAB_TDEF>
avrBitbangLedStrip<FAB_TVAR>::avrBitbangLedStrip()
{
	// Digital out pin mode
	// bitSet(portDDR, dataPortPin);
	// DDR? |= 1U << dataPortPin;
	SET_DDR_HIGH(dataPortId, dataPortPin);

	// Set port to LOW state
	SET_PORT_LOW(dataPortId, dataPortPin);

//// DEBUG - For some reason this calls delay() and hangs the CPU! Also can't Serial.print()
//// Most likely called BEFORE setup() loop.
////	refresh();
};

template<FAB_TDEF>
template <void printChar(const char *),void printInt(uint32_t)>
inline void
avrBitbangLedStrip<FAB_TVAR>::debug(void)
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

	switch (colors) {
		case NONE: printChar("NONE"); break;
		case RGB: printChar("RGB"); break;
		case GRB: printChar("GRB"); break;
		case BGR: printChar("BGR"); break;
		case RGBW: printChar("RGBW"); break;
		case GRBW: printChar("GRBW"); break;
		case BGRW: printChar("BGRW"); break;
		default: printChar("ERROR!"); break;
	}

	printChar(" REFRESH MSEC=");
	printInt(minMsRefresh);

	printChar("\nDATA_PORT ");
	switch(dataPortId) {
		case A:
			printChar("A.");
			break;
		case B:
			printChar("B.");
			break;
		case C:
			printChar("C.");
			break;
		case D:
			printChar("D.");
			break;
		case E:
			printChar("E.");
			break;
		case F:
			printChar("F.");
			break;
		default:
			printChar("UNKNOWN.");
	}
	printInt(dataPortPin);
	printChar(", ");

	if (protocol >= PROTOCOL_SPI) {
		printChar("CLOCK_PORT ");
		switch(clockPortId) {
			case A:
				printChar("A.");
				break;
			case B:
				printChar("B.");
				break;
			case C:
				printChar("C.");
				break;
			case D:
				printChar("D.");
				break;
			case E:
				printChar("E.");
				break;
			case F:
				printChar("F.");
				break;
			default:
				printChar("UNKNOWN.");
		}
		printInt(clockPortPin);
		printChar(", ");
	}

	switch(protocol) {
		case ONE_WIRE_BITBANG:
			printChar("ONE-WIRE (bitbang)");
			break;
		case ONE_WIRE_PWM:
			printChar("ONE-WIRE (PWM)");
			break;
		case ONE_WIRE_UART:
			printChar("ONE-WIRE (UART)");
			break;
		case SPI_BITBANG:
			printChar("TWO-WIRE SPI (bitbang)");
			break;
		case SPI_HARDWARE:
			printChar("TWO-WIRE SPI (hardware)");
			break;
		default:
			printChar("PROTOCOL UNKNOWN");
	}
	printChar("\n");
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendBytes(const uint16_t count, const uint8_t * array)
{
	switch (protocol) {
		case ONE_WIRE_BITBANG:
			oneWireSoftwareSendBytes(count, array);
			break;
		case SPI_BITBANG:
			spiSoftwareSendBytes(count, array);
			break;
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::spiSoftwareSendFrame(const uint16_t count, bool high)
{
	if (high) {
		SET_PORT_HIGH(dataPortId, dataPortPin);
	} else {
		SET_PORT_LOW(dataPortId, dataPortPin);
	}
	for(uint32_t c = 0; c < 32 * count; c++) {
		SET_PORT_LOW(clockPortId, clockPortPin);
		SET_PORT_HIGH(clockPortId, clockPortPin);
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::spiSoftwareSendBytes(const uint16_t count, const uint8_t * array)
{
	for(uint16_t c = 0; c < count; c++) {
		const uint8_t val = array[c];
		// If LED strip is defined as 3 byte type (default) then hard code the first
		// byte to 0xFFFF, aka max brightness.
		// This hard-codes the APA-102 protocol here so it is a bit hacky.
		if (colors >= PIXEL_FORMAT_4B && c % 3 == 0) {
			spiSoftwareSendFrame(1, true);
		}
		// To send a bit to SPI, set its value, then transtion clock low-high
		for(int8_t b=7; b>=0; b--) {
			const bool bit = (val>>b) & 0x1;
			SET_PORT_LOW(clockPortId, clockPortPin);
 			if (bit) {
				SET_PORT_HIGH(dataPortId, dataPortPin);
			} else {
				SET_PORT_LOW(dataPortId, dataPortPin);
			}
			SET_PORT_HIGH(clockPortId, clockPortPin);
		}
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::oneWireSoftwareSendBytes(const uint16_t count, const uint8_t * array)
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
				SET_PORT_HIGH(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(high1 - sbiCycles);
				//  LOW with ASM cbi (2 words, 2 cycles)
				SET_PORT_LOW(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(low1 - cbiCycles);
			} else {
				// Send a ZERO

				// HIGH with ASM sbi (2 words, 2 cycles)
				SET_PORT_HIGH(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(high0 - sbiCycles);
				//  LOW with ASM cbi (2 words, 2 cycles)
				SET_PORT_LOW(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(low0 - cbiCycles);
			}
		}
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::clear(const uint16_t numPixels)
{
	if (protocol == SPI_BITBANG) {
		// SPI: Send start frame, Clean numPixels, and Reset LED strip
		spiSoftwareSendFrame(1 + numPixels + (numPixels+1)/2, 0);
	} else {
		// 1-wire: Delay next pixels to cause a refresh

		// Disable interupts
		uint8_t oldSREG = SREG;
 		DISABLE_INTERRUPTS;

		const uint8_t array[4] = {0,0,0,0};
		for( uint16_t i = 0; i < numPixels; i++) {
			sendBytes(bytesPerPixel, array);
		}

		// Restore interupts
		SREG = oldSREG;
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::grey(const uint16_t numPixels, const uint8_t value)
{
	// Disable interupts
	uint8_t oldSREG = SREG;
	DISABLE_INTERRUPTS;

	const uint8_t array[4] = {value, value, value, value};
	for( uint16_t i = 0; i < numPixels; i++) {
		sendBytes(bytesPerPixel, array);
	}

	// Restore interupts
	SREG = oldSREG;
}

// 3B raw input array
template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const uint8_t * array)
{
	// Disable interupts
	uint8_t oldSREG = SREG;
	// cli();
 	DISABLE_INTERRUPTS;

	sendBytes(numPixels * bytesPerPixel, array);

	// Restore interupts
	SREG = oldSREG;
}

// Since colors is a constant, the switch case will convert to 4 sendBytes max.
#define SEND_REMAPPED_PIXELS(numPixels, array, sendWhite)          \
		uint8_t oldSREG = SREG;                            \
 		DISABLE_INTERRUPTS;                               \
		for (uint16_t i = 0; i < numPixels; i++) {         \
			switch (colors) {                          \
				case RGB:                          \
					sendBytes(1, &array[i].r); \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].b); \
					break;                     \
				case GRB:                          \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].r); \
					sendBytes(1, &array[i].b); \
					break;                     \
				case BGR:                          \
					sendBytes(1, &array[i].b); \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].r); \
					break;                     \
				case RGBW:                         \
					sendBytes(1, &array[i].r); \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].b); \
					sendWhite;                 \
					break;                     \
				case GRBW:                         \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].r); \
					sendBytes(1, &array[i].b); \
					sendWhite;                 \
					break;                     \
				case BGRW:                         \
					sendBytes(1, &array[i].b); \
					sendBytes(1, &array[i].g); \
					sendBytes(1, &array[i].r); \
					sendWhite;                 \
					break;                     \
				default:                           \
					break;                     \
			}                                          \
		}                                                  \
		SREG = oldSREG;                                    \

#define sendWhiteMacro sendBytes(1, &array[i].w)
#define SEND_REMAPPED_PIXELS_4B(numPixels, array) SEND_REMAPPED_PIXELS(numPixels, array, sendWhiteMacro)
#define SEND_REMAPPED_PIXELS_3B(numPixels, array) SEND_REMAPPED_PIXELS(numPixels, array, )

// 4B struct input arrays
template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const rgbw * array)
{
	if (colors == RGBW || colors == NONE) {
		sendPixels(numPixels, (const uint8_t *) array);
	} else if (colors == RGB) {
		// Output array is same order but 3B, send as 32bit, which will be converted
		sendPixels(numPixels, (const uint32_t *) array);
	} else {
		// Handle input array of different format than LED strip
		SEND_REMAPPED_PIXELS_4B(numPixels, array);
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const grbw * array)
{
	if (colors == GRBW || colors == NONE) {
		sendPixels(numPixels, (const uint8_t *) array);
	} else if (colors == GRB) {
		sendPixels(numPixels, (const uint32_t *) array);
	} else {
		// Handle input array of different format than LED strip
		SEND_REMAPPED_PIXELS_4B(numPixels, array);
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const bgrw * array)
{
	if (colors == BGRW || colors == NONE) {
		sendPixels(numPixels, (const uint8_t *) array);
	} else if (colors == BGR) {
		sendPixels(numPixels, (const uint32_t *) array);
	} else {
		// Handle input array of different format than LED strip
		SEND_REMAPPED_PIXELS_4B(numPixels, array);
	}
}

// 3B struct input arrays
template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const rgb * array)
{
	if (colors == RGB || colors == NONE) {
		// Input array is native format. No conversion.
		sendPixels(numPixels, (const uint8_t *) array);
	} else {
		// 4B, or 3B pixel array with different byte order, must be converted.
		SEND_REMAPPED_PIXELS_3B(numPixels, array);
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const grb * array)
{
	if (colors == GRB || colors == NONE) {
		sendPixels(numPixels, (const uint8_t *) array);
	} else {
		SEND_REMAPPED_PIXELS_3B(numPixels, array);
	}
}

template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const bgr * array)
{
	if (colors == BGR || colors == NONE) {
		sendPixels(numPixels, (const uint8_t *) array);
	} else {
		SEND_REMAPPED_PIXELS_3B(numPixels, array);
	}
}

// 4B raw input array
template<FAB_TDEF>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		const uint16_t numPixels,
		const uint32_t * pixelArray)
{
	// Disable interupts
	uint8_t oldSREG = SREG;
	//cli();
 	DISABLE_INTERRUPTS;

	if (colors >= PIXEL_FORMAT_4B) {
		// 4 byte per pixel array, send all bytes.
		sendBytes((const uint16_t) numPixels * bytesPerPixel, (const uint8_t *) pixelArray);
	} else {
		// 3 byte per pixel array, send 3 out of 4 bytes.
		for (int i=0; i< numPixels; i++) {
			uint8_t * bytes = (uint8_t *) & pixelArray[i];
			// For LED strips using 3 bytes per color, drop a byte.
			sendBytes(bytesPerPixel, bytes);
		}
	}

	// Restore interupts
	SREG = oldSREG;
}

// Palette input arrays
template<FAB_TDEF>
template <const uint8_t bitsPerPixel>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels (
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
	const uint8_t oldSREG = SREG;
 	DISABLE_INTERRUPTS;

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
			sendBytes(bytesPerPixel, &palette[bytesPerPixel*colorIndex]);
			elem >>= bitsPerPixel;
		}
	}
end:
	// Restore interupts
	SREG = oldSREG;
	return;
}

template<FAB_TDEF>
template <const uint8_t bitsPerPixel, class T>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels (
		const uint16_t count,
		const uint8_t * pixelArray,
		const T * palette)
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
	const uint8_t oldSREG = SREG;
 	DISABLE_INTERRUPTS;

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
			sendBytes(bytesPerPixel, &palette[bytesPerPixel*colorIndex]);
			elem >>= bitsPerPixel;
		}
	}
end:
	// Restore interupts
	SREG = oldSREG;
	return;
}

/// @todo Rewrite this to support R5G6B5, R4G4B4W4 and a variable brightness.
template<FAB_TDEF>
template <uint8_t brightness>
inline void
avrBitbangLedStrip<FAB_TVAR>::sendPixels(
		int count,
		uint16_t * pixelArray)
{
	const uint8_t mask5 = ((1 << (5 - brightness))-1) || ((1 << brightness) - 1);
	const uint8_t mask10 = ((1 << (10 - brightness))-1) || ((1 << brightness) - 1);
	uint8_t bytes[4];

	// Debug: Support brightness 0..3
	ASSERT(brightness < 3);

	// Disable interupts
	uint8_t oldSREG = SREG;
	cli();

	bytes[3] = 0;
	for (int i = 0; i < count; i++) {
		const uint16_t elem = pixelArray[i]; 
		bytes[0] = (uint8_t) elem << brightness;
		bytes[1] = (elem >> (5 - brightness)) & mask5;
		bytes[2] = (elem >> (10 - brightness)) & mask10;
		sendBytes(bytesPerPixel, bytes);
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
	WS2812B_0L_CY, WS2812B_MS_REFRESH, dataPortId, dataPortBit, A, 0, GRB, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
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
	WS2812_0L_CY, WS2812_MS_REFRESH, dataPortId, dataPortBit, A, 0, GRB, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
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
	APA104_0L_CY, APA104_MS_REFRESH, dataPortId, dataPortBit, A, 0, GRB, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
class apa104 : public avrBitbangLedStrip<FAB_TVAR_APA104>
{
	public:
	apa104() : avrBitbangLedStrip<FAB_TVAR_APA104>() {};
	~apa104() {};
};
#undef FAB_TVAR_APA104
#define pl9823 apa104; 


////////////////////////////////////////////////////////////////////////////////
// APA106 (like APA104, but with LEDs ordered as RGB instead of GRB)
////////////////////////////////////////////////////////////////////////////////
#define APA106_1H_CY CYCLES(1210) // 500ns 1210ns-1510ns _----------__
#define APA106_1L_CY CYCLES(200)  // 125ns  200ns-500ns  .    .    .
#define APA106_0H_CY CYCLES(200)  // 125ns  200ns-500ns  _-----_______
#define APA106_0L_CY CYCLES(1210) // 500ns 1210ns-1510ns .    .    .
#define APA106_MS_REFRESH 50      //  50,000ns Minimum wait time to reset LED strip
#define APA106_NS_RF 5000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_APA106 APA106_1H_CY, APA106_1L_CY, APA106_0H_CY, \
	APA106_0L_CY, APA106_MS_REFRESH, dataPortId, dataPortBit, A, 0, RGB, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
class apa106 : public avrBitbangLedStrip<FAB_TVAR_APA106>
{
	public:
	apa106() : avrBitbangLedStrip<FAB_TVAR_APA106>() {};
	~apa106() {};
};
#undef FAB_TVAR_APA106



////////////////////////////////////////////////////////////////////////////////
// SK6812 (4 color RGBW LEDs, faster PWM frequency, updated timings)
/// @note I need to use microsleep to not round up sleep to 1msec.
////////////////////////////////////////////////////////////////////////////////
#define SK6812_1H_CY CYCLES(1210) // 500ns 1210ns-1510ns _----------__
#define SK6812_1L_CY CYCLES(200)  // 125ns  200ns-500ns  .    .    .
#define SK6812_0H_CY CYCLES(200)  // 125ns  200ns-500ns  _-----_______
#define SK6812_0L_CY CYCLES(1210) // 500ns 1210ns-1510ns .    .    .
#define SK6812_MS_REFRESH 84      //  84,000ns Minimum wait time to reset LED strip
#define SK6812_NS_RF  833333      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_SK6812 SK6812_1H_CY, SK6812_1L_CY, SK6812_0H_CY, \
	SK6812_0L_CY, SK6812_MS_REFRESH, dataPortId, dataPortBit, A, 0, RGBW, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
class sk6812 : public avrBitbangLedStrip<FAB_TVAR_SK6812>
{
	public:
	sk6812() : avrBitbangLedStrip<FAB_TVAR_SK6812>() {};
	~sk6812() {};
};
#undef FAB_TVAR_SK6812




////////////////////////////////////////////////////////////////////////////////
// SK6812B (4 color GRBW LEDs, faster PWM frequency, updated timings)
/// @note I need to use microsleep to not round up sleep to 1msec.
////////////////////////////////////////////////////////////////////////////////
#define SK6812B_1H_CY CYCLES(1210) // 500ns 1210ns-1510ns _----------__
#define SK6812B_1L_CY CYCLES(200)  // 125ns  200ns-500ns  .    .    .
#define SK6812B_0H_CY CYCLES(200)  // 125ns  200ns-500ns  _-----_______
#define SK6812B_0L_CY CYCLES(1210) // 500ns 1210ns-1510ns .    .    .
#define SK6812B_MS_REFRESH 84      //  84,000ns Minimum wait time to reset LED strip
#define SK6812B_NS_RF  833333      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_SK6812B SK6812B_1H_CY, SK6812B_1L_CY, SK6812B_0H_CY, \
	SK6812B_0L_CY, SK6812B_MS_REFRESH, dataPortId, dataPortBit, A, 0, GRBW, ONE_WIRE_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit>
class sk6812b : public avrBitbangLedStrip<FAB_TVAR_SK6812B>
{
	public:
	sk6812b() : avrBitbangLedStrip<FAB_TVAR_SK6812B>() {};
	~sk6812b() {};
};
#undef FAB_TVAR_SK6812B


////////////////////////////////////////////////////////////////////////////////
// APA-102 (3 color RGB LEDs, SPI protocol)
////////////////////////////////////////////////////////////////////////////////
#define APA102_1H_CY CYCLES(0) // Unused
#define APA102_1L_CY CYCLES(0)  // Unused
#define APA102_0H_CY CYCLES(0)  // Unused
#define APA102_0L_CY CYCLES(0) // Unused
#define APA102_MS_REFRESH 84      //  84,000ns Minimum wait time to reset LED strip
#define APA102_NS_RF  833333      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#define FAB_TVAR_APA102 APA102_1H_CY, APA102_1L_CY, APA102_0H_CY, \
	APA102_0L_CY, APA102_MS_REFRESH, dataPortId, dataPortBit, clockPortId, clockPortBit, GRBW, SPI_BITBANG
template<avrLedStripPort dataPortId, uint8_t dataPortBit, avrLedStripPort clockPortId, uint8_t clockPortBit>
class apa102 : public avrBitbangLedStrip<FAB_TVAR_APA102>
{
	public:
	apa102() : avrBitbangLedStrip<FAB_TVAR_APA102>() {};
	~apa102() {};
};
#undef FAB_TVAR_APA102

#endif // FAB_LED_H
