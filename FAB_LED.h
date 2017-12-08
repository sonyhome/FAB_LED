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

////////////////////////////////////////////////////////////////////////////////
/// General definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// This code is sensitive to optimizations if you want to cascade function calls,
// so make the IDE compile at -O2 (fast code) instead of -Os (size).
////////////////////////////////////////////////////////////////////////////////
#pragma GCC optimize ("-O2")


////////////////////////////////////////////////////////////////////////////////
// static_assert is a built-in C++ 11 assert that Arduino g++ does not implement
// so we work around this by triggering an invalid char array size to make the
// compiler error out at the given line
////////////////////////////////////////////////////////////////////////////////
#ifndef static_assert
#define _STATIC_ASSERT(a,b) a##b
#define static_assert(cond, key)	enum { _STATIC_ASSERT(key, __LINE__) = \
					sizeof( char[(cond) ? +1 : -1] ) }
#endif


////////////////////////////////////////////////////////////////////////////////
/// @brief LED PORT COMMUNICATION PROTOCOL list defines the protocol used to
/// send bits on the port and the way this driver controls the port to be
/// compatible with the protocol (the main method is bitbanging)
/// ONE WIRE PROTOCOL: Data port, each bit is H->L, but 1 has long H, 0 short H
/// and the LED samples at a given clock rate.
/// SPI PROTOCOL: Data + clock ports, data bit latched when clock goes H->L
////////////////////////////////////////////////////////////////////////////////
#define LED_PORT_COMMUNICATION_PROTOCOLS_LIST                                 \
	LED_PORT_COMMUNICATION_PROTOCOL( 1, BITBANG_1WI_1P,                   \
		       	"1-wire 1 port (bitbang)")                            \
	LED_PORT_COMMUNICATION_PROTOCOL( 2, BITBANG_1WI_S2P,                  \
			"1-wire 2 ports, split half (bitbang)")               \
	LED_PORT_COMMUNICATION_PROTOCOL( 3, BITBANG_1WI_I2P,                  \
			"1-wire 2 ports, split interleaved (bitbang)")        \
	LED_PORT_COMMUNICATION_PROTOCOL( 4, BITBANG_1WI_8P,                   \
		       	"1-wire 8 ports max split evenly (bitbang)")          \
	LED_PORT_COMMUNICATION_PROTOCOL( 5, PWM_1P,                           \
	       		"1-wire 1 port (hardware PWM)")                       \
	LED_PORT_COMMUNICATION_PROTOCOL( 6, UART_1P,                          \
			"1-wire 1 port (hardware serial UART)")               \
                                                                              \
	LED_PORT_COMMUNICATION_PROTOCOL( 7, BITBANG_SPI,                      \
		    	"SPI data+clock port (bitbang)")                      \
	LED_PORT_COMMUNICATION_PROTOCOL( 8, HARDWARE_SPI,                     \
		     	"SPI data+clock port (hardware SPI)") 

// Minimum index of SPI protocol entries, below are 1WI
#define PROTOCOL_SPI BITBANG_SPI

/// @brief Type for low-level SendBytes method to send data to the LED port
#define LED_PORT_COMMUNICATION_PROTOCOL(index, label, desc) label = index,
enum letPortComm_t {
	LED_PORT_COMMUNICATION_PROTOCOLS_LIST
	UNSUPPORTED = 9
};
#undef LED_PORT_COMMUNICATION_PROTOCOL


////////////////////////////////////////////////////////////////////////////////
/// @brief LED type defines the physical properties of the LED/LED strip
/// IT is in LED_TYPES_LIST macro because it helps define the classes and enum
///      (     NAME  ,     TYPE  ,PIXEL,H1ns,L1ns,H0ns,L0ns,RSTus, PROTOCOL        )
////////////////////////////////////////////////////////////////////////////////
#define LED_TYPES_LIST                                                               \
LED_TYPE2(  apa102   ,  APA102   , hbgr,   0,   0,   0,   0,   84, BITBANG_SPI     ) \
LED_TYPE2(  ws2801   ,  WS2801   ,  bgr,   0,   0,   0,   0,   84, BITBANG_SPI     ) \
LED_TYPE2(  sk9813   ,  SK9813   ,  bgr,   0,   0,   0,   0,   84, BITBANG_SPI     ) \
LED_TYPE1(  sk6812   ,  SK6812   , rgbw,1210, 200, 200,1210,   0, BITBANG_1WI_1P  ) \
LED_TYPE1(  sk6812b  ,  SK6812B  , grbw,1210, 200, 200,1210,   0, BITBANG_1WI_1P  ) \
LED_TYPE1(  apa104   ,  APA104   , grb ,1210, 200, 200,1210,   50, BITBANG_1WI_1P  ) \
LED_TYPE1(  apa106   ,  APA106   , rgb ,1210, 200, 200,1210,   50, BITBANG_1WI_1P  ) \
LED_TYPE1(  ws2811   ,  WS2811   , rgb , 500, 125, 125, 188,   20, BITBANG_1WI_1P  ) \
LED_TYPE1(  ws2812b  ,  WS2812B  , grb , 320,   0,   0, 128,    0, BITBANG_1WI_1P  ) \
LED_TYPE1(  ws2812   ,  WS2812   , grb , 550, 200, 200, 550,   20, BITBANG_1WI_1P  ) \
LED_TYPE1(  ws2812std,  WS2812STD, grb , 650, 125, 125, 650,   50, BITBANG_1WI_1P  ) \
LED_TYPE2(  ws2812bi ,  WS2812BI , grb , 550, 200, 200, 550,    0, BITBANG_1WI_I2P ) \
LED_TYPE2(  ws2812bs ,  WS2812BS , grb , 550, 200, 200, 550,    0, BITBANG_1WI_S2P ) \
LED_TYPER(  ws2812b8 ,  WS2812B8 , grb , 500, 125, 125, 188,    0, BITBANG_1WI_8P  )

// Enum specifying the LED type.
#define LED_TYPE1(name, typeName, pixel, h1, l1, h0, l0,reset, protocol) typeName,
#define LED_TYPE2(name, typeName, pixel, h1, l1, h0, l0,reset, protocol) typeName,
#define LED_TYPER(name, typeName, pixel, h1, l1, h0, l0,reset, protocol) typeName,
enum ledType_t
{
	LED_TYPES_LIST
	UNKNOWN = 3
};
#undef LED_TYPE1
#undef LED_TYPE2
#undef LED_TYPER


////////////////////////////////////////////////////////////////////////////////
/// @brief pixel structure types for every LED protocol supported.
/// The user manipulates arrays of these types for convenience. Each LED type
/// has a native pixel type. The user can reference a color (r, red, etc.)
/// without worrying what is the color order, or the raw bytes.
/// FAB_LED will detect and allow pixel types mismatching the native LED pixel
/// type, and transparently perform the conversion to display colors properly.
///
/// Each pixel type has a static "type" field used to process the pixel properly
/// by the fab_led class when it is passed as a template type. The values of
/// "type" is built with the following definitions.
////////////////////////////////////////////////////////////////////////////////
// Pixel Type: Colors byte order
#define PT_RGB  0b00100000
#define PT_GRB  0b01000000
#define PT_BGR  0b10000000
#define PT_RBG  0b11000000 // Unused yet
#define PT_GBR  0b10100000 // Unused yet
#define PT_BRG  0b01100000 // Unused yet
/////// /////   ..XXXxxxxx
#define PT_COL  0b11100000 // Mask for the 3-byte colors defined below
#define PT_IS_SAME_COLOR_ORDER(a, b) ((a) & PT_COL == (b) & PT_COL)
/// Pixel Type: Extra Properties, 0 none, 2 invalid.
#define PT_XXXW 0b00000001 // Postfix 8bit white brightness (4-colors)
#define PT_WXXX 0b00000010 // Prefix  8bit white pixel brightness (4-colors)
#define PT_BXXX 0b00000011 // Prefix  5bit brightness level (APA102 0b111bbbbb)
#define PT_XCHK 0b00000100 // Prefix  8bit checksum (SK9814)
/////// /////   ..xxxxxXXX
#define PT_4BYT 0b00000111
#define PT_BYTES_PER_PIXEL(v) (3 + (((v) & PT_4BYT) != 0))
#define PT_HAS_WHITE(v)		((((v) & PT_XTRA) == PT_XXXW) || \
				 (((v) & PT_XTRA) == PT_WXXX))
#define PT_HAS_BRIGHT(v)	 (((v) & PT_XTRA) == PT_BXXX)
#define PT_X16b 0b00001000 // uint16_t with 5r6g5b bits per color
/////// /////   ..xxxXXXXX
#define PT_XTRA 0b00011111 // Mask for extra byte(s) defined below


typedef struct _rgb {
	static const uint8_t type = PT_RGB;
	union {
		struct {
			union { uint8_t r; uint8_t red;};
			union { uint8_t g; uint8_t green;};
			union { uint8_t b; uint8_t blue;};
		};
		uint8_t raw[3];
	};
} rgb;

typedef struct _grb {
	static const uint8_t type = PT_GRB;
	union {
		struct {
			union { uint8_t g; uint8_t green;};
			union { uint8_t r; uint8_t red;};
			union { uint8_t b; uint8_t blue;};
		};
		uint8_t raw[3];
	};
} grb;

typedef struct _bgr {
	static const uint8_t type = PT_BGR;
	union {
		struct {
			union { uint8_t b; uint8_t blue;};
			union { uint8_t g; uint8_t green;};
			union { uint8_t r; uint8_t red;};
		};
		uint8_t raw[3];
	};
} bgr;

typedef struct _rgbw {
	static const uint8_t type = PT_RGB | PT_XXXW;
	union {
		struct {
			union { uint8_t r; uint8_t red;};
			union { uint8_t g; uint8_t green;};
			union { uint8_t b; uint8_t blue;};
			union { uint8_t w; uint8_t white;};
		};
		uint8_t raw[4];
	};
} rgbw;

typedef struct _wrgb {
	static const uint8_t type = PT_RGB | PT_WXXX;
	union {
		struct {
			union { uint8_t w; uint8_t white;};
			union { uint8_t r; uint8_t red;};
			union { uint8_t g; uint8_t green;};
			union { uint8_t b; uint8_t blue;};
		};
		uint8_t raw[4];
	};
} wrgb;

typedef struct _grbw {
	static const uint8_t type = PT_GRB | PT_XXXW;
	union {
		struct {
			union { uint8_t g; uint8_t green;};
			union { uint8_t r; uint8_t red;};
			union { uint8_t b; uint8_t blue;};
			union { uint8_t w; uint8_t white;};
		};
		uint8_t raw[4];
	};
} grbw;

typedef struct _hbgr {
	static const uint8_t type = PT_BGR | PT_BXXX;
	union {
		struct {
			uint8_t H:3; // Hight 3 bits must be set to 1.
			union { uint8_t w:5; uint8_t white:5;
				uint8_t B:5; uint8_t brightness:5; };
			union { uint8_t b; uint8_t blue;};
			union { uint8_t g; uint8_t green;};
			union { uint8_t r; uint8_t red;};
		};
		uint8_t raw[4];
	};
} hbgr;

/// @brief compressed 16 bit pixel with 32/64/32 levels per color.
/// Actual brightness is multiplied by 4 or 8.
typedef struct _r5g6b5 {
	static const uint8_t type = PT_X16b;
	union {
		struct {
			uint16_t r : 5;
			uint16_t g : 6;
			uint16_t b : 5;
		};
		uint8_t raw[2];
		uint16_t raw16;
	};
} r5g6b5;

/// @brief type to declare a pixel array where each pixel is an index to
/// a color in a color palette. It supports 1, 2, 4 and 8 bits per pixel,
/// for a max of 2, 4, 16 or 256 colors.
typedef union _paletteColor {
       struct {
	       uint8_t p0:1, p1:1, p2:1, p3:1, p4:1, p5:1, p6:1, p7:1;
       } bpp1;
       struct {
	       uint8_t p0:2, p1:2, p2:2, p3:2;
       } bpp2;
       struct {
	       uint8_t p0:4, p1:4;
       } bpp4;
       uint8_t bpp8;
       uint8_t raw;
} paletteColor;


////////////////////////////////////////////////////////////////////////////////
// Define native LED pixel types. A user can use it to have a pixel type native
// to the LED they use. For example:
// ws2818bPixel array[numPixels];
////////////////////////////////////////////////////////////////////////////////
#define TYPE_GEN(a, b) a ## b
#define LED_TYPE1(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) typedef PIXEL TYPE_GEN(TYPE, Pixel);
#define LED_TYPE2(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) typedef PIXEL TYPE_GEN(TYPE, Pixel);
#define LED_TYPER(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) typedef PIXEL TYPE_GEN(TYPE, Pixel);
LED_TYPES_LIST
#undef LED_TYPE1
#undef LED_TYPE2
#undef LED_TYPER

/// @brief Lists all the pixel types suported by the API
/// The template parameters can use fab_led internal class types
/// and constants.
///
/// API_ENTRY(_countOut, _typeIn, _typeOut)
/// _countOut: Number of elements (bytes or pixels) to pass to the underlying
///            implementation, usually count (aka number of pixels in array)
///            unless the array is re-cast for palette support
/// _typeIn:   The type of the user's array. There should be one entry for each
///            pixel struct declared in this section.
/// _typeOut:  The type to cast the user's array to. Usually same as _typeIn
///            except for generic types which can map to palettes or other
///            implicit types we assume. uint32_t is mapped to wrgb by default
///            and uint16_t to r5g6b5 which is kind of a standard. For palette
///            support the uint8_t input is parsed knowing defined stripBPP
///            and the colors are expected to match pixelClass.
///            This cast will allow the compiler to instantiate the templates
///
/// if _typeOut doesn't match the native pixel type of the LED strip, the
/// functions will do automatically a conversion at a cost of extra code.
///
/// @todo uint32_t type to rgbw likely screws up rgbw support!!!
#define SEND_AND_DRAW_API_LIST                                                 \
	API_ENTRY(count/stripBPP, paletteColor, pixelClass)                    \
	API_ENTRY(count/stripBPP, uint8_t, pixelClass)                         \
	API_ENTRY(count, r5g6b5, r5g6b5)                                       \
	API_ENTRY(count, uint16_t, r5g6b5)                                     \
	API_ENTRY(count, wrgb, wrgb)                                           \
	API_ENTRY(count, uint32_t, wrgb)                                       \
	API_ENTRY(count, rgb, rgb)                                             \
	API_ENTRY(count, grb, grb)                                             \
	API_ENTRY(count, bgr, bgr)                                             \
	API_ENTRY(count, rgbw, rgbw)                                           \
	API_ENTRY(count, grbw, grbw)                                           \
	API_ENTRY(count, hbgr, hbgr)



////////////////////////////////////////////////////////////////////////////////
/// @brief Color Palette helper routines
////////////////////////////////////////////////////////////////////////////////

#if __cplusplus <= 201103L
//  #error This library needs at least a C++11 compliant compiler
#define constexpr inline
#endif

/// @brief converts number of bits per pixel to number of colors possible 2^bpp
/// Example: 
///   const uint8_t myBpp = 2;
///   const uint8_t myNumColors = bpp2nc(myBpp);
///   grb   myPalette[myNumColors] = myPalette();
/// For 2 bits per pixel, this allocates a palette of 4 colors
constexpr uint8_t bpp2nc(const uint8_t bpp)
{
	return 1 << bpp;
};

/// @brief converts number of colors supported to bits per pixels log2(nc)
constexpr uint8_t nc2bpp(const uint8_t nc)
{
	return nc <= 2 ? 1 : ( 1 + nc2bpp(nc/2));
};

/// @brief Computes the size of the array holding pixel that use a color palette
///   Example:
///   paletteColor buffer[PIarraySize<2>(128)];
/// Declares an array of 128 pixels encoded with 2 bits per pixel (4 colors).
///
/// template   bitsPerPixel Number of bits used to declare one pixel. One bit is
///                         two colors, 2bits is 4, 4bits is 16, 8bits is 256.
/// @param[in] numPixels    Number of pixels to display
/// return     size in bytes of the array that can hold said number of pixels
template <uint8_t bitsPerPixel>
constexpr uint16_t PIarraySize(const uint16_t numPixels)
{
	return (((numPixels)+7) / 8 * (bitsPerPixel));
};
#define ARRAY_SIZE(numPixels, bitsPerPixel) (((numPixels)+7) / 8 * (bitsPerPixel))


/// @brief Encode a pixel's color by setting the index of the color in the
/// color palette.
///   Example:
///   grb myPalette[bpp2nc(2)] = myPalette(); // 2 bit pixels = 2^2 = 4 colors
///   myPalette[1] = {g:128, r:0, b:0};
///   paletteColor buffer<2>[PIarraySize(128)];
///   for(int i=0; i < 128, i++) {
///     PIsetPixel<2>(buffer, i, 1);
///   }
///   buffer[0].bpp2.p3 = 3;
/// Declares a 2-bit color palette, sets color #1 to green, and then allocates a
/// buffer of 128 pixels all set to use color #1. Lastly, set pixel 3 to color 3
///
/// template   bitsPerPixel Number of bits used to declare one pixel. One bit is
///                         two colors, 2bits is 4, 4bits is 16, 8bits is 256.
/// @param[in] array        Array containing the pixels
/// @param[in] pixelIndex   Index of pixels to set
/// @param[in] colorIndex   Index of color to set
template <uint8_t bitsPerPixel>
constexpr void PIsetPixel(uint8_t *array,
			  const uint16_t pixelIndex,
			  const uint8_t colorIndex)
{
	const uint8_t  pixelsPerByte = 8/bitsPerPixel;
	const uint16_t arrayIndex = pixelIndex / pixelsPerByte;
	const uint8_t  pixelBitOffset = (pixelIndex * pixelsPerByte) % 8;
	const uint8_t  numColors = 1 << bitsPerPixel;
	const uint8_t  colorMask = numColors - 1;
	// Mask the color in case user screws up
	const uint8_t  color = (colorIndex < numColors) ? colorIndex :
				colorIndex & colorMask;

	// Clear the pixel's bits and set them to the chosen color.
	array[arrayIndex] =
		(array[arrayIndex] & ~(colorMask << pixelBitOffset)) |
		(color << pixelBitOffset);
}
//#define SET_PIXEL(array, index, bitsPerPixel, color) PIsetPixel<bitsPerPixel>(array, index, color)


/// @brief extract a pixel's palette color index from a pixel array
///   Example:
///   colorIndex = PIgetPixel<2>(buffer, i, 2);
///
/// template   bitsPerPixel Number of bits used to declare one pixel. One bit is
///                         two colors, 2bits is 4, 4bits is 16, 8bits is 256.
/// @param[in] array        Array containing the pixels
/// @param[in] pixelIndex   Index of pixels to set
/// return     Index of color of chosen pixel
template <uint8_t bitsPerPixel>
constexpr uint8_t PIgetPixel(const uint8_t *array,
			     const uint16_t pixelIndex)
{
	const uint8_t  pixelsPerByte = 8/bitsPerPixel;
	const uint16_t arrayIndex = pixelIndex / pixelsPerByte;
	const uint8_t  pixelBitOffset = (pixelIndex * pixelsPerByte) % 8;
	const uint8_t  numColors = 1 << bitsPerPixel;
	const uint8_t  colorMask = numColors - 1;
	return (array[arrayIndex] >> pixelBitOffset) % colorMask;
};
//#define GET_PIXEL(array, index, bitsPerPixel) PIgetPixel<bitsPerPixel>(array, index)


////////////////////////////////////////////////////////////////////////////////
/// @brief Conversion between cycles and nano seconds
////////////////////////////////////////////////////////////////////////////////

#define NSEC_PER_SEC    (1000ULL * 1000ULL * 1000ULL)
#define CYCLES_PER_SEC  ((uint64_t) (F_CPU))
#define CYCLES(time_ns) (((CYCLES_PER_SEC * (time_ns)) + NSEC_PER_SEC - 1ULL)  \
			/ NSEC_PER_SEC)
#define NSEC_PER_CYCLE(cycles) (((cycles) * NSEC_PER_SEC                       \
				+ CYCLES_PER_SEC - 1ULL) / CYCLES_PER_SEC)


////////////////////////////////////////////////////////////////////////////////
/// @brief definitions for class template specializations
////////////////////////////////////////////////////////////////////////////////

/// @brief AVR port registers are referenced A through F. Example ws2812b<D,6>
/// references bit 6 of port D (pin D6).
/// Reference:
/// PIN: toggles the pin or reads the pin
/// DDR: Data direction register (0: input, 1: output)
/// PORT: (1 + input = pull up resistor, 1 + output = high, 0+ output = low)
/// 0x20 PINA 0x21 DDRA 0x22 PORTA
/// 0x23 PINB 0x24 DDRB 0x25 PORTB
/// 0x26 PINC 0x27 DDRC 0x28 PORTC
/// 0x29 PIND 0x2A DDRD 0x2B PORTD
/// 0x2C PINE 0x2D DDRE 0x2E PORTE
/// 0x2F PINF 0x30 DDRF 0x31 PORTF
enum avrLedStripPort {
	A = 1,
	B = 2,
	C = 3,
	D = 4,
	E = 5,
	F = 6
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Hack to survive undefined port on Arduino
/// avoid compilation errors for arduinos that are missing some of the 4 ports
/// by defining all the unknown ports to be any of the known ones. This quiets
/// the compiler (useless code optimizes away).
////////////////////////////////////////////////////////////////////////////////

// Define a DUMMY_PORT/DUMMY_DDR used to patch unknown ports
// This lets the compiler work without complaining.
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

// If any of the ports we support does not exist, re-map it to the dummy port.
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


////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_AVR
////////////////////////////////////////////////////////////////////////////////
/// @brief Arduino AVR low level macros
////////////////////////////////////////////////////////////////////////////////

/// Port Data Direction control Register address
#define AVR_DDR(id) _AVR_DDR((id))
#define _AVR_DDR(id) ((id==A) ? DDRA : (id==B) ? DDRB : (id==C) ? DDRC : \
		(id==D) ? DDRD : (id==E) ? DDRE : DDRF)

#define SET_DDR_PIN_HIGH(portId, portPin) AVR_DDR(portId) |=   1U << portPin
#define SET_DDR_PIN_LOW( portId, portPin) AVR_DDR(portId) &= ~(1U << portPin)
#define SET_DDR_BYTE(portId, val) AVR_DDR(portId) = val

/// Port address & pin level manipulation
#define AVR_PORT(id) _AVR_PORT((id))
#define _AVR_PORT(id) ((id==A) ? PORTA : (id==B) ? PORTB : (id==C) ? PORTC : \
		(id==D) ? PORTD : (id==E) ? PORTE : PORTF)

#define SET_PORT_BYTE(portId, val) AVR_PORT(portId) = val
// Note: gcc converts these bit manipulations to sbi and cbi instructions
#define SET_PORT_PIN_HIGH(portId, portPin) AVR_PORT(portId) |=   1U << portPin
#define SET_PORT_PIN_LOW( portId, portPin) AVR_PORT(portId) &= ~(1U << portPin)

#define CLEAR_INTERRUPTS   oldSREG = SREG; \
			   __builtin_avr_cli()
#define RESTORE_INTERRUPTS SREG = oldSREG

/// Method to optimally delay N cycles with nops for bitBang.
#define DELAY_CYCLES(count) if (count > 0) __builtin_avr_delay_cycles(count)

// Number of cycles sbi and cbi instructions take when using SET macros
const int sbiCycles = 2;
const int cbiCycles = 2;


////////////////////////////////////////////////////////////////////////////////
#elif defined(__arm__)
////////////////////////////////////////////////////////////////////////////////
/// @brief ARM0 - ARM3 low level macros
/// @note: Ports have no letter, just port pins. ws2812b<D,6> will ignore D and
/// just route to pin 6.
///
/// Register information:
/// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337h/BABJFFGJ.html
/// DWT_CYCCNT: Cycle count register
////////////////////////////////////////////////////////////////////////////////

/// @todo THE debug() function template resolution does not work on non optimized teensy
//#define DISABLE_DEBUG_METHOD

#define SET_DDR_PIN_HIGH(portId, portPin)
#define SET_DDR_PIN_LOW( portId, portPin)
#define SET_DDR_BYTE( portId, val)

#define SET_PORT_BYTE(portId, val) /* 8-port mode not supported */
#define SET_PORT_PIN_HIGH(portId, pinId)   digitalWriteFast(pinId, 1)
#define SET_PORT_PIN_LOW( portId, pinId)   digitalWriteFast(pinId, 0)

#define CLEAR_INTERRUPTS   oldSREG = SREG
#define RESTORE_INTERRUPTS SREG = oldSREG

/// Delay N cycles using cycles register
inline void spinDelay(const uint16_t count)
{
	const uint16_t till = count + ARM_DWT_CYCCNT;
	while (ARM_DWT_CYCCNT < till);
}
#define DELAY_CYCLES(count) spinDelay(count)

// Number of cycles sbi and cbi instructions take
const int sbiCycles = 2;
const int cbiCycles = 2;

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

////////////////////////////////////////////////////////////////////////////////
#elif defined(ESP8266)

#error "Unsupported Tensilica (ARC)Architecture"

#else
////////////////////////////////////////////////////////////////////////////////
/// @brief unknown processor architecture
////////////////////////////////////////////////////////////////////////////////

#error "Unsupported Architecture"

////////////////////////////////////////////////////////////////////////////////
#endif // CPU ARCHITECTURE
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Base class defining LED strip operations allowed.
////////////////////////////////////////////////////////////////////////////////
//static const uint8_t blank[3] = {128,128,128};

/// @brief template parameters
///	int16_t high1               Number of cycles high for logical one
///	int16_t low1                Number of cycles  low for logical one
///	int16_t high0               Number of cycles high for logical zero
///	int16_t low0                Number of cycles  low for logical zero
///	uint32_t minMsRefresh       Minimum msec delay to reset LED strip
///	avrLedStripPort dataPortId  Port the LED strip DI is attached to
///	uint8_t dataPortPin         Port bit the LED strip DI is attached to
///	avrLedStripPort clockPortId Port the LED strip CI is attached to
///	uint8_t clockPortPin        Port bit the LED strip CI is attached to
///     class pixelClass            Native LED strip pixel type (grbw, etc.)
///     letPortComm_t protocol      LED port communication protocol (spi, etc.)

/// @brief FAB_TDEF is the body of the class template declaration
/// example: template <FAB_TDEF> class foo;
#define FAB_TDEF                      \
	int16_t         high1,        \
	int16_t         low1,         \
	int16_t         high0,        \
	int16_t         low0,         \
	uint32_t        minMsRefresh, \
	avrLedStripPort dataPortId,   \
	uint8_t         dataPortPin,  \
	avrLedStripPort clockPortId,  \
	uint8_t         clockPortPin, \
	class           pixelClass,   \
	ledType_t       ledType,      \
	letPortComm_t   protocol

/// @brief FAB_TVAR is the instantiation of class the template declaration
/// example: template <FAB_TDEF> foo<FAB_TVAR>::function(...){...}
#define FAB_TVAR                      \
	                high1,        \
	                low1,         \
	                high0,        \
	                low0,         \
	                minMsRefresh, \
	                dataPortId,   \
	                dataPortPin,  \
	                clockPortId,  \
	                clockPortPin, \
	                pixelClass,   \
	                ledType,      \
	                protocol

/// @brief Class to drive LED strips. Relies on custom sendBytes() method to push data to LEDs
template <FAB_TDEF>
class fab_led
{
	/// @brief Constant declaring if LED strip uses 3 or 4 bytes per pixel
	static const uint8_t stripBPP = PT_BYTES_PER_PIXEL(pixelClass::type);
	/// @brief Counts how many pixels were sent on APA102 strip before refresh
	static uint16_t pixelsDisplayed;
	static uint8_t oldSREG;

public:
	////////////////////////////////////////////////////////////////////////
	/// @brief Constructor: Set selected dataPortId.dataPortPin to digital output
	////////////////////////////////////////////////////////////////////////
	fab_led();
	////////////////////////////////////////////////////////////////////////
	~fab_led() { };

	////////////////////////////////////////////////////////////////////////
	/// @brief Prints to console the configuration
	/// You must implement the template print routines
	///   Example:
	///   void myC(const char * s) { Serial.print(s); }
	///   void myI(const uint32_t i) { Serial.print(i); }
	///   foo.debug<myC, myS>();
	/// The method can be disabled with:
	///   #define DISABLE_DEBUG_METHOD
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

private:
	////////////////////////////////////////////////////////////////////////
	/// @brief Sends N bits in a row set to zero or one, to build
	/// a frame for each pixel, and for a whole strip, SPI protocol only
	////////////////////////////////////////////////////////////////////////
	static inline void
	bitbangSpi_Flatline(const uint16_t count, bool high)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Implements sendBytes for the 1-port SPI protocol
	////////////////////////////////////////////////////////////////////////
	static inline void
	bitbangSpi_SendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Implements sendBytes for the 1-ports protocol
	////////////////////////////////////////////////////////////////////////
	static inline void
	bitbang1PortWs_SendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Implements sendBytes for the 2-ports protocol
	////////////////////////////////////////////////////////////////////////
	static inline void
	bitbang2portWs_SendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Implements sendBytes for the 8-ports protocol
	////////////////////////////////////////////////////////////////////////
	static inline void
	bitbang8PortWs_SendBytes(const uint16_t count, const uint8_t * array)
	__attribute__ ((always_inline));


public:
	////////////////////////////////////////////////////////////////////////
	/// @brief Clears the LED strip
	/// @param[in] count  Number of pixels to erase
	////////////////////////////////////////////////////////////////////////
	static inline void clear(const uint16_t count) __attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Sets the LED strip to a grey value
	/// @param[in] count  Number of pixels to erase
	/// @param[in] value  Brightness of the grey [0..255]
	////////////////////////////////////////////////////////////////////////
	static inline void grey(
			const uint16_t count,
			const uint8_t value)
	__attribute__ ((always_inline));


	////////////////////////////////////////////////////////////////////////
	/// @brief Starts a write sequence to the LED strip for send
	/// @note: Call end() as soon as possible
	////////////////////////////////////////////////////////////////////////
	static inline void
	begin(void) __attribute__ ((always_inline));

	////////////////////////////////////////////////////////////////////////
	/// @brief Ends a write sequence to the LED strip for send
	/// @param[in] count  Total number of pixels that were sent. In rare
	///                   cases, the user may send pixels to a port using 2
	///                   or more protocols. In that case the count can't
	///                   be tracked and the user must provide it.
	/// @note: Call end() as soon as possible after begin()
	////////////////////////////////////////////////////////////////////////
	static inline void
	end(uint16_t count = 0) __attribute__ ((always_inline));


private:
	////////////////////////////////////////////////////////////////////////
	/// @brief Display the array of pixels at the current position in the
	/// LED strip (does not reset the display).
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of size equal or greater than count.
	////////////////////////////////////////////////////////////////////////

	/// @brief
	/// send<bpp, pixel, palette, map>(numPix, array, palette, map)
	/// Palette + Pixel Mapping implementation
	template <const uint8_t bitsPerPixel,
		 class arrayClassF,
		 class paletteClassF,
		 class mapIntF>
	static inline void
	send(	const uint16_t count,
		const arrayClassF * array,
		const paletteClassF * palette,
		const mapIntF * map)
	__attribute__ ((always_inline));

	template <const uint8_t bitsPerPixel,
		 class arrayClassF,
		 class paletteClassF,
		 class mapIntF>
	static inline void
	send(	const uint16_t count,
		const arrayClassF * array,
		const uint8_t * redPalette,
		const uint8_t * greenPalette,
		const uint8_t * bluePalette,
		const mapIntF * map)
	__attribute__ ((always_inline));

	/// @brief
	/// send<paletteType>(numPix, palette, array)
	// Palette implementation
	template <class paletteClassF>
	static inline void
	send(	const uint16_t count,
		const paletteClassF * palette,
		const uint8_t * array)
	__attribute__ ((always_inline));

	/// @brief
	/// send(numPix, array)
	// Normal pixel implementation
	template <class pixelClassF>
	static inline void
	send(const uint16_t count, const pixelClassF * array)
	__attribute__ ((always_inline));


public:
	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// Display the array of pixels at the current position in the LED strip
	///
	/// send(uint16_t count, pixelType * array);
	///
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of any supported type
	///
	/// @note
	/// uint16_t array (assume r:5,g:6,b:5)
	/// uint8_t array  (assume bytes are in native LED strip format)
	/// uint32_t array (assume wrgb. The first byte must be > 0xE0 if the
	///                 strip is APA-102!)
	////////////////////////////////////////////////////////////////////////
#define API_ENTRY(_countOut, _typeIn, _typeOut)                                \
	static inline void send(                                               \
			const uint16_t count,                                  \
			const _typeIn * array                                  \
	) {                                                                    \
		send<_typeOut>(_countOut, (_typeOut *) array);                 \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST;
#undef API_ENTRY


	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// Display with remapping.
	/// Display the array of pixels at the current position in the LED strip
	/// The pixels are remapped. The map[] tells the offset in the array[]
	/// to read the color from. map[] entries correspond to the physical LED
	/// position in the strip. The array[] entries correspond to the logical
	/// pixels in the picture.
	/// map[ledPos] = ArrayPixelIndex
	///
	/// send(uint16_t count, pixelType * array, uint8_t * map);
	/// send(uint16_t count, pixelType * array, uint16_t * map);
	///
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of 16bit/pixel {r:5, g:6, b:5}
	/// @param[in] map    Array to remap pixels,
	/// @param[in] brightness (optional) value 0..7 increases LED power
	///
	/// @note For remapping use (uint8_t*) arrays for less than 256 pixels,
	///       else use (uint16_t*) arrays, but this consumes 2X the memory.
	////////////////////////////////////////////////////////////////////////

	// 8bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut)                                \
	static inline void send(                                               \
			const uint16_t count,                                  \
			const _typeIn * array,                                 \
			uint8_t * map                                          \
	) {                                                                    \
		send<255, _typeOut, _typeOut, uint8_t>(                        \
				_countOut,                                     \
				(_typeOut *) array,                            \
				NULL,                                          \
				map                                            \
		);                                                             \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	// 16bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	static inline void send( \
			const uint16_t count, \
			const _typeIn * array, \
			uint16_t * map \
	) { \
		send<0,_typeOut,_typeOut,uint16_t>( \
				_countOut, \
				(_typeOut *) array, \
				NULL, map \
		); \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// Display with color palette and optional remapping.
	///
	/// Color Palette:
	/// Display the array of pixels at the current position in the LED strip
	/// The pixels use a color palette. The array[] holds bpp-bits for each
	/// pixel, the value represents a color index in the palette table.
	/// array[arrayPixelIndex] = colorPaletteIndex
	/// palette[colorPaletteIndex] = rgbColor
	///
	/// Pixel Remapping:
	/// To read the color from. map[] entries correspond to the physical LED
	/// position in the strip. The array[] entries correspond to the logical
	/// pixels in the picture.
	/// map[ledPos] = ArrayPixelIndex
	/// 
	/// @note The method will scan the map linearly, to extract the next
	/// ArrayPixelIndex to read, get the colorPaletteIndex, and display the
	/// color from palette[colorPaletteIndex]. This is done on the fly, so
	/// it is slower than displaying a simple pixel array, but does not use
	/// any more memory.
	///
	/// Using a palette saves memory.
	/// Memory use is sizeof(palette) + sizeof(map) + sizeof(array)
	/// For N pixels, and bpp:
	/// Memory use is 2^bpp * sizeof(pixel) + N + N * bpp/8
	/// Example: 256 ws2812b pixels and 4 colors (2 bits per pixel)
	/// 1 bit per pixel 2 * 3 + 256 + 256 / 8 =  6 + 256 + 32 = 294 bytes
	/// 2 bit per pixel 4 * 3 + 256 + 256 / 4 = 12 + 256 + 64 = 332 bytes
	/// 4 bit per pixel 16* 3 + 256 + 256 / 2 = 48 + 256 +128 = 432 bytes
	/// 8 bit per pixel 256*3 + 256 + 256 / 1 =768 + 256 +256 =1280 bytes
	/// A simple pixel array is                       3 * 256 = 768 bytes
	/// A simple pixel array with remapping is  256 + 3 * 256 =1024 bytes
	///
	/// Example: 1024 ws2812b pixels palette only
	/// 1 bpp   2 * 3 + 1024 * 1 / 8 =   6 +  128 =  134 bytes
	/// 2 bpp   4 * 3 + 1024 * 2 / 8 =  12 +  256 =  268 bytes
	/// 4 bpp  16 * 3 + 1024 * 4 / 8 =  48 +  512 =  560 bytes
	/// 8 bpp 256 * 3 + 1024 * 8 / 8 = 768 + 1024 = 1792 bytes
	/// A simple pixel array is          3 * 1024 = 3072 bytes
	///
	/// send<bpp>(count, uint8_t  array, pixelType * palette);
	/// send<bpp>(count, uint16_t array, pixelType * palette);
	/// send<bpp>(count, uint16_t array, uint8_t * redPalette,
	///        uint8_t * greenPalette, uint8_t * bluePalette);
	/// send<bpp>(count, uint8_t  array, pixelType * palette, uint8_t  map);
	/// send<bpp>(count, uint16_t array, pixelType * palette, uint16_t map);
	/// send<bpp>(count, uint16_t array, uint8_t * redPalette,
	///        uint8_t * greenPalette, uint8_t * bluePalette, uint16_t map);
	///
	/// @param[in] bpp    Template parameter, number of bits per pixel.
	///                   Valid values: 1, 2, 4 or 8 bits per pixel, which
	///                   correspond to 2, 4, 16 and 256 color palettes
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of bpp bits per pixel
	/// @param[in] map    (optional) Remapping. map[LEDindex] = pixelOffset
	///
	/// @note A palette array does not need to be fully populated. It just
	///         needs to have a pixel color defined for the colors in use.
	////////////////////////////////////////////////////////////////////////

	// 8bit map (max 256 pixels)
#define API_ENTRY(_countOut, _typeIn, _typeOut)                                \
	template <const uint8_t bitsPerPixel>                                  \
	static inline void send(                                               \
			const uint16_t count,                                  \
			const uint8_t * array,                                 \
			const _typeIn * palette,                               \
			uint8_t * map = NULL                                   \
	) {                                                                    \
		send<bitsPerPixel, uint8_t, _typeOut, uint8_t>(                \
				_countOut,                                     \
				array,                                         \
				(_typeOut *) palette,                          \
				map                                            \
		);                                                             \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	/*
#define API_ENTRY(_countOut, _typeIn, _typeOut)                                \
	template <const uint8_t bitsPerPixel>                                  \
	static inline void sendX(                                               \
			const uint16_t count,                                  \
			const uint8_t * array,                                 \
			const uint8_t * redPalette,                            \
			const uint8_t * greenPalette,                          \
			const uint8_t * bluePalette,                           \
			uint8_t * map = NULL                                   \
	) {                                                                    \
		send<bitsPerPixel, uint8_t, _typeOut, uint8_t>(                \
				_countOut,                                     \
				array,                                         \
				redPalette,                                    \
				greenPalette,                                  \
				bluePalette,                                   \
				map                                            \
		);                                                             \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY
*/

	// 16 bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut)                                \
	template <const uint8_t bitsPerPixel>                                  \
	static inline void send(                                               \
			const uint16_t count,                                  \
			const uint16_t * array,                                \
			const _typeIn * palette,                               \
			uint16_t * map = NULL                                  \
	) {                                                                    \
		send<bitsPerPixel, uint16_t, _typeOut, uint16_t>(              \
				_countOut,                                     \
				array,                                         \
				(_typeOut *) palette,                          \
				map                                            \
		);                                                             \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY


	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// draw(uint16_t count, pixelType * array);
	///
	/// Display the array of pixels at the current position in the LED strip
	///
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of any supported type
	///
	/// @note
	/// uint16_t array (assume r:5,g:6,b:5)
	/// uint8_t array  (assume bytes are in native LED strip format)
	/// uint32_t array (assume wrgb. The first byte must be > 0xE0 if the
	///                 strip is APA-102!)
	////////////////////////////////////////////////////////////////////////
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	static inline void draw(const uint16_t count, const _typeIn * array) { \
		begin(); \
		send(count, array); \
		end(); \
	} __attribute__ ((always_inline)); 
	SEND_AND_DRAW_API_LIST;
#undef API_ENTRY


	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// draw(uint16_t count, pixelType * array, uint8_t * map);
	/// draw(uint16_t count, pixelType * array, uint16_t * map);
	///
	/// Pixel Remapping:
	/// Display the array of pixels at the current position in the LED strip
	/// The pixels are remapped. The map[] tells the offset in the array[]
	/// to read the color from. map[] entries correspond to the physical LED
	/// position in the strip. The array[] entries correspond to the logical
	/// pixels in the picture.
	///
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of 16bit/pixel {r:5, g:6, b:5}
	/// @param[in] map    Array to remap pixels,
	/// @param[in] brightness (optional) value 0..7 increases LED power
	///
	/// @note For remapping use (uint8_t*) arrays for less than 256 pixels,
	///       else use (uint16_t*) arrays, but this consumes 2X the memory.
	////////////////////////////////////////////////////////////////////////

	// 8bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	static inline void draw(const uint16_t count, const _typeIn * array, uint8_t * map) { \
		begin(); \
		send(count, array, NULL, map); \
		end(); \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	// 16bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	static inline void draw(const uint16_t count, const _typeIn * array, uint16_t * map) { \
		begin(); \
		send(count, array, NULL, map); \
		end(); \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	////////////////////////////////////////////////////////////////////////
	/// @brief
	/// draw<N>(uint16_t count, uint8_t array, pixelType * palette);
	/// draw<N>(uint16_t count, uint16_t array, pixelType * palette);
	/// draw<N>(uint16_t count, uint8_t array, pixelType * palette, uint8_t map);
	/// draw<N>(uint16_t count, uint16_t array, pixelType * palette, uint16_t map);
	///
	/// Color Palette:
	/// Display the array of pixels at the current position in the LED strip
	/// The pixels use a color palette. The array[] holds N-bits for each
	/// pixel, the value represents a color index in the palette table.
	///
	/// Pixel Remapping:
	/// To read the color from. map[] entries correspond to the physical LED
	/// position in the strip. The array[] entries correspond to the logical
	/// pixels in the picture.
	///
	/// @param[in] N      Template parameter, number of bits per pixel.
	///                   Valid values: 1, 2, 4 or 8 bits per pixel, which
	///                   correspond to 2, 4, 16 and 256 color palettes
	/// @param[in] count  Number of pixels to send
	/// @param[in] array  Pixel array of N bits per pixel
	/// @param[in] map    (optional) Remapping. map[LEDindex] = pixelOffset
	///
	/// @note A palette array does not need to be fully populated. It just
	///         needs to have a pixel color defined for the colors in use.
	////////////////////////////////////////////////////////////////////////

	// 8bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	template <const uint8_t bitsPerPixel> \
	static inline void draw(const uint16_t count, const uint8_t * array, const _typeIn * palette, uint8_t * map = NULL) { \
		begin(); \
		send<bitsPerPixel>(count, array, palette, map); \
		end(); \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY

	// 16 bit map
#define API_ENTRY(_countOut, _typeIn, _typeOut) \
	template <const uint8_t bitsPerPixel> \
	static inline void draw(const uint16_t count, const uint16_t * array, const _typeIn * palette, uint16_t * map = NULL) { \
		begin(); \
		send<bitsPerPixel>(count, array, palette, map); \
		end(); \
	} __attribute__ ((always_inline));
	SEND_AND_DRAW_API_LIST
#undef API_ENTRY
};


////////////////////////////////////////////////////////////////////////////////
/// Class definition
////////////////////////////////////////////////////////////////////////////////

/// @brief Counts number of pixels sent (for apa-102 refresh)
/// Static because must count all pixels sent from multiple class instances
/// on a given port (ok to overestimate if we write mulitple ports)
template<FAB_TDEF>
uint16_t fab_led<FAB_TVAR>::pixelsDisplayed = 0;

/// @brief Save interrupt vector
template<FAB_TDEF>
uint8_t fab_led<FAB_TVAR>::oldSREG = SREG;

////////////////////////////////////////////////////////////////////////////////
/// @brief Default constructor
////////////////////////////////////////////////////////////////////////////////
template<FAB_TDEF>
fab_led<FAB_TVAR>::fab_led()
{

	// Cofigure the port as output, grounded to low state
	// Digital out pin mode
	// bitSet(portDDR, dataPortPin);
	// DDR? |= 1U << dataPortPin;
	switch (protocol) {
		case BITBANG_1WI_1P:
			SET_DDR_PIN_HIGH(dataPortId, dataPortPin);
			SET_PORT_PIN_LOW(dataPortId, dataPortPin);
			break;
		case BITBANG_1WI_S2P:
		case BITBANG_1WI_I2P:
			// Data and Clock are both used as data ports
			SET_DDR_PIN_HIGH(dataPortId,  dataPortPin);
			SET_DDR_PIN_HIGH(clockPortId, clockPortPin);
			SET_PORT_PIN_LOW(dataPortId,  dataPortPin);
			SET_PORT_PIN_LOW(clockPortId, clockPortPin);
			break;
		case BITBANG_1WI_8P:
			// Take over all 8 bits of the port
			SET_DDR_BYTE(dataPortId, 0xFF);  // all pins out
			SET_PORT_BYTE(dataPortId, 0x00); // all pins low
			break;
		case BITBANG_SPI:
		case HARDWARE_SPI:
			// Use Data and Clock pins
			SET_DDR_PIN_HIGH(dataPortId, dataPortPin);
			SET_DDR_PIN_HIGH(clockPortId, clockPortPin);
			// SPI: Reset LED strip to accept a refresh
			bitbangSpi_Flatline(32, false);
			break;
		default:
			static_assert(protocol < UNSUPPORTED, LED_protocol_not_supported);
			break;
	}
};


////////////////////////////////////////////////////////////////////////////////
// Debug routine
////////////////////////////////////////////////////////////////////////////////
template<FAB_TDEF>
template <void printChar(const char *),void printInt(uint32_t)>
inline void
fab_led<FAB_TVAR>::debug(void)
{
#ifndef DISABLE_DEBUG_METHOD
	printChar("\nclass fab_led<...>\n");

	printInt(CYCLES_PER_SEC/1000000);
	printChar("MHz CPU, ");
	printInt(NSEC_PER_CYCLE(1000));
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

	if (pixelClass::type & PT_BXXX) {
		printChar("H");
	}
	if (pixelClass::type & PT_WXXX) {
		printChar("W");
	}
	switch (pixelClass::type & PT_COL) {
		case PT_RGB: printChar("RGB"); break;
		case PT_GRB: printChar("GRB"); break;
		case PT_BGR: printChar("BGR"); break;
		default: printChar("[ERROR]"); break;
	}
	if (pixelClass::type & PT_XXXW) {
		printChar("W");
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

	switch(ledType) {
#define LED_TYPE1(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) case TYPE: printChar( #TYPE); break;
#define LED_TYPE2(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) case TYPE: printChar( #TYPE); break;
#define LED_TYPER(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) case TYPE: printChar( #TYPE); break;
		LED_TYPES_LIST                                                               \
		default:
			printChar("UNKNOWN.");
#undef LED_TYPE1
#undef LED_TYPE2
#undef LED_TYPER
	}

	switch(protocol) {
#define LED_PORT_COMMUNICATION_PROTOCOL(index, label, desc) case label: printChar(desc); break;
	LED_PORT_COMMUNICATION_PROTOCOLS_LIST
#undef LED_PORT_COMMUNICATION_PROTOCOL
		default:
			printChar("Unsupported LED port communication protocol");
	}
	printChar("\n");
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Low level sendBytes routines
////////////////////////////////////////////////////////////////////////////////
template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::sendBytes(const uint16_t count, const uint8_t * array)
{
	switch (protocol) {
		case BITBANG_1WI_1P:
			bitbang1PortWs_SendBytes(count, array);
			break;
		case BITBANG_1WI_S2P:
		case BITBANG_1WI_I2P:
			// Note: the function will detect and handle modes I and S
			bitbang2portWs_SendBytes(count, array);
			break;
		case BITBANG_1WI_8P:
			bitbang8PortWs_SendBytes(count, array);
			break;
		case BITBANG_SPI:
			bitbangSpi_SendBytes(count, array);
			break;
		case HARDWARE_SPI:
		default:
			static_assert(protocol < UNSUPPORTED, LED_protocol_not_supported);
			break;
	}
}

template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbangSpi_Flatline(const uint16_t numBits, bool high)
{
	if (high) {
		SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
	} else {
		SET_PORT_PIN_LOW(dataPortId, dataPortPin);
	}
	for(uint16_t c = 0; c < numBits; c++) {
		SET_PORT_PIN_LOW(clockPortId, clockPortPin);
		SET_PORT_PIN_HIGH(clockPortId, clockPortPin);
	}
}

template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbangSpi_SendBytes(const uint16_t count, const uint8_t * array)
{
	for(uint16_t cnt = 0; cnt < count; ++cnt) {
		const uint8_t val = array[cnt];
		// Send byte msbit first
		for(int8_t b=7; b>=0; b--) {
			const bool bit = (val>>b) & 0x1;
 			if (bit) {
				SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
			} else {
				SET_PORT_PIN_LOW(dataPortId, dataPortPin);
			}
			// Latch the value in the LED
			SET_PORT_PIN_LOW(clockPortId, clockPortPin);
			SET_PORT_PIN_HIGH(clockPortId, clockPortPin);
		}

	}
}

/// @brief sends the array split across two ports each having half the LED strip to illuminate.
/// To achieve this, we repurpose the clock port used for SPI as a second data port.
/// We support two protocols:
/// BITBANG_1WI_S2P: The array is split into 2 halves sent each sent to one of the ports
/// BITBANG_1WI_I2P: The array is interleaved and each pixel of 3 byte is sent to the next port
template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbang2portWs_SendBytes(const uint16_t count, const uint8_t * array)
{
	// If split mode, we send a block of half size
	const uint16_t blockSize = (protocol == BITBANG_1WI_S2P) ? count/2 : count;

	// Stride is two for interlacing to jump pixels going to the other port
	const uint8_t stride = (protocol == BITBANG_1WI_S2P) ? 1 : 2;
	const uint8_t increment = stride * stripBPP;

	// Loop to scan all pixels, potentially skipping every other pixel, or scanning 1/2 the pixels
	// based on the display protocol used.
	for(uint16_t pix = 0; pix < blockSize; pix += increment) {
		// Loop to send 3 or 4 bytes of a pixel to the same port
		for(uint16_t pos = pix; pos < pix+stripBPP; pos++) {
			for(int8_t bit = 7; bit >= 0; bit--) {
				const uint8_t mask = 1 << bit;

				volatile bool isbitDhigh = array[pos] & mask;
	
				volatile bool isbitChigh = (protocol == BITBANG_1WI_S2P) ?
					array[pos + blockSize] & mask : // split: pixel is blockSize away.
					array[pos + stripBPP] & mask;        // interleaved: pixel is next one.

				if (isbitDhigh) SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
				if (isbitChigh) SET_PORT_PIN_HIGH(clockPortId, clockPortPin);

				if (!isbitDhigh) {
					SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
					DELAY_CYCLES(high0 - sbiCycles);
					SET_PORT_PIN_LOW(dataPortId, dataPortPin);
				}
				if (!isbitChigh) {
					SET_PORT_PIN_HIGH(clockPortId, clockPortPin);
					DELAY_CYCLES(high0 - sbiCycles);
					SET_PORT_PIN_LOW(clockPortId, clockPortPin);
				}
				DELAY_CYCLES(high1 - 2*sbiCycles);
				SET_PORT_PIN_LOW(dataPortId, dataPortPin);
				SET_PORT_PIN_LOW(clockPortId, clockPortPin);
				DELAY_CYCLES(low1 - 2*cbiCycles);
			}
		}
	}
}

#if 0
template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbang2portWs_SendBytes(const uint16_t count, const uint8_t * array)
{
	const uint16_t blockSize = count/2;

	for(uint16_t pos = 0; pos < blockSize; pos++) {
		for(int8_t bit = 7; bit >= 0; bit--) {
			const uint8_t mask = 1 << bit;
			const bool isbitDhigh = array[0*blockSize + pos] & mask;
			const bool isbitChigh = array[1*blockSize + pos] & mask;

			if (isbitDhigh) SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
			if (isbitChigh) SET_PORT_PIN_HIGH(clockPortId, clockPortPin);

			if (!isbitDhigh) {
				SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
				//DELAY_CYCLES(high0 - sbiCycles);
				DELAY_CYCLES(0);
				SET_PORT_PIN_LOW(dataPortId, dataPortPin);
			}
			if (!isbitChigh) {
				SET_PORT_PIN_HIGH(clockPortId, clockPortPin);
				//DELAY_CYCLES(high0 - sbiCycles);
				DELAY_CYCLES(0);
				SET_PORT_PIN_LOW(clockPortId, clockPortPin);
			}
			DELAY_CYCLES(0);
			SET_PORT_PIN_LOW(dataPortId, dataPortPin);
			SET_PORT_PIN_LOW(clockPortId, clockPortPin);
			DELAY_CYCLES(0);
		}
	}
}
#endif

/// @brief
/// Bitbang up to 8 port pins to control up to 8 LED strips in parallel.
/// Each LED strip receives data from a separate part of the pixel array which
/// is split in N portions if there are N LED strips (max 8)
///
/// Timing is critical so the bits sent out are pipelines with the array byte reads.
/// Each iteration we load one byte from one of the N array locations into a specific
/// register and fetch one bit for each port from N registers, and write all N bits
/// to the port. This must be done before the LED bit resets (~ 1200ns).
/// 16MHz=6.25ns would be 200 cycles?!
template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbang8PortWs_SendBytes(const uint16_t count, const uint8_t * array)
{
	const uint16_t blockSize __asm__("r14") = count / (clockPortPin - dataPortPin + 1) / stripBPP * stripBPP;

	// Buffer one byte of each port in register
	uint8_t r0 = 0; // __asm__("r2") = 0;
	uint8_t r1 = 0; // __asm__("r3") = 0;
	uint8_t r2 = 0; // __asm__("r4") = 0;
	uint8_t r3 = 0; // __asm__("r5") = 0;
	uint8_t r4 = 0; // __asm__("r6") = 0;
	uint8_t r5 = 0; // __asm__("r7") = 0;
	uint8_t r6 = 0; // __asm__("r8") = 0;
	uint8_t r7 = 0; // __asm__("r9") = 0;

	// Since bits for each port are sent delayed by one cycle to
	// implement the memory access pipeline, we must leave the
	// ports not yet in use set to LOW level. Once the pipeline
	// is initialized fully onMask == 0xFF, and we always set it
	// to high to send a zero or a one.
	uint8_t onMask __asm__("r10") = 0;

	for(uint16_t c __asm__("r12") = 0; c < blockSize ; ++c) {
		for(int8_t b __asm__("r13") = 7; b >= 0; --b) {
			uint8_t bitmask __asm__("r10") = 0;

			// Load ONE byte per iteration
			// Skip end condition check to reduce CPU cycles.
			// It is expected that the LED strip won't have extra pixels.
			switch (b) {
				case 0:
					// By checking statically if rX is read from memory, we
					// avoid changing it, and the compiler will optimize out all
					// the code including bitmask setting. On 16MHz Uno, we can
					// only set bitmask for 6 ports before the strip resets.
					if (0 == dataPortPin) {
						onMask |= 1;
						r0 = array[c];
					}

					if (r0 & 0b10000000) bitmask |= 1;
					if (r1 & 0b01000000) bitmask |= 2;
					if (r2 & 0b00100000) bitmask |= 4;
					if (r3 & 0b00010000) bitmask |= 8;
					if (r4 & 0b00001000) bitmask |= 16;
					if (r5 & 0b00000100) bitmask |= 32;
					if (r6 & 0b00000010) bitmask |= 64;
					if (r7 & 0b00000001) bitmask |= 128;
					break;
				case 1:
					if ((1 >= dataPortPin) && (1 <= clockPortPin)) {
						onMask |= 2;
						r1 = array[c + (1-dataPortPin) * blockSize];
					}

					if (r0 & 0b00000001) bitmask |= 1;
					if (r1 & 0b10000000) bitmask |= 2;
					if (r2 & 0b01000000) bitmask |= 4;
					if (r3 & 0b00100000) bitmask |= 8;
					if (r4 & 0b00010000) bitmask |= 16;
					if (r5 & 0b00001000) bitmask |= 32;
					if (r6 & 0b00000100) bitmask |= 64;
					if (r7 & 0b00000010) bitmask |= 128;
					break;
				case 2:
					if ((2 >= dataPortPin) && (2 <= clockPortPin)) {
						onMask |= 4;
						r2 = array[c + (2-dataPortPin) * blockSize];
					}

					if (r0 & 0b00000010) bitmask |= 1;
					if (r1 & 0b00000001) bitmask |= 2;
					if (r2 & 0b10000000) bitmask |= 4;
					if (r3 & 0b01000000) bitmask |= 8;
					if (r4 & 0b00100000) bitmask |= 16;
					if (r5 & 0b00010000) bitmask |= 32;
					if (r6 & 0b00001000) bitmask |= 64;
					if (r7 & 0b00000100) bitmask |= 128;
					break;
				case 3:
					if ((3 >= dataPortPin) && (3 <= clockPortPin)) {
						onMask |= 8;
						r3 = array[c + (3-dataPortPin) * blockSize];
					}

					if (r0 & 0b00000100) bitmask |= 1;
					if (r1 & 0b00000010) bitmask |= 2;
					if (r2 & 0b00000001) bitmask |= 4;
					if (r3 & 0b10000000) bitmask |= 8;
					if (r4 & 0b01000000) bitmask |= 16;
					if (r5 & 0b00100000) bitmask |= 32;
					if (r6 & 0b00010000) bitmask |= 64;
					if (r7 & 0b00001000) bitmask |= 128;
					break;
				case 4:
					if ((4 >= dataPortPin) && (4 <= clockPortPin)) {
						onMask |= 16;
						r4 = array[c + (4-dataPortPin) * blockSize];
					}

					if (r0 & 0b00001000) bitmask |= 1;
					if (r1 & 0b00000100) bitmask |= 2;
					if (r2 & 0b00000010) bitmask |= 4;
					if (r3 & 0b00000001) bitmask |= 8;
					if (r4 & 0b10000000) bitmask |= 16;
					if (r5 & 0b01000000) bitmask |= 32;
					if (r6 & 0b00100000) bitmask |= 64;
					if (r7 & 0b00010000) bitmask |= 128;
					break;
				case 5:
					if ((5 >= dataPortPin) && (5 <= clockPortPin)) {
						onMask |= 32;
						r5 = array[c + (5-dataPortPin) * blockSize];
					}

					if (r0 & 0b00010000) bitmask |= 1;
					if (r1 & 0b00001000) bitmask |= 2;
					if (r2 & 0b00000100) bitmask |= 4;
					if (r3 & 0b00000010) bitmask |= 8;
					if (r4 & 0b00000001) bitmask |= 16;
					if (r5 & 0b10000000) bitmask |= 32;
					if (r6 & 0b01000000) bitmask |= 64;
					if (r7 & 0b00100000) bitmask |= 128;
					break;
				case 6:
					if ((6 >= dataPortPin) && (6 <= clockPortPin)) {
						onMask |= 64;
						r6 = array[c + (6-dataPortPin) * blockSize];
					}

					if (r0 & 0b00100000) bitmask |= 1;
					if (r1 & 0b00010000) bitmask |= 2;
					if (r2 & 0b00001000) bitmask |= 4;
					if (r3 & 0b00000100) bitmask |= 8;
					if (r4 & 0b00000010) bitmask |= 16;
					if (r5 & 0b00000001) bitmask |= 32;
					if (r6 & 0b10000000) bitmask |= 64;
					if (r7 & 0b01000000) bitmask |= 128;
					break;
				case 7:
					if ((7 >= dataPortPin) && (7 <= clockPortPin)) {
						onMask |= 128;
						r7 = array[c + (7-dataPortPin) * blockSize];
					}

					if (r0 & 0b01000000) bitmask |= 1;
					if (r1 & 0b00100000) bitmask |= 2;
					if (r2 & 0b00010000) bitmask |= 4;
					if (r3 & 0b00001000) bitmask |= 8;
					if (r4 & 0b00000100) bitmask |= 16;
					if (r5 & 0b00000010) bitmask |= 32;
					if (r6 & 0b00000001) bitmask |= 64;
					if (r7 & 0b10000000) bitmask |= 128;
					break;
				}

			SET_PORT_BYTE(dataPortId, onMask); // Set high all lines
			DELAY_CYCLES(high0 - sbiCycles);

			SET_PORT_BYTE(dataPortId, bitmask); // Set 0 lines low 
			DELAY_CYCLES( high1 - high0 + sbiCycles);

			// We should only clear the bits we control but that implies p &= offMask operation
			SET_PORT_BYTE(dataPortId, 0x00); // Set 1 lines low
			// Estimate overhead of math before port IO to ~ 20 cycles
			DELAY_CYCLES(low0 - cbiCycles - 20);
		}
	}
}


template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::bitbang1PortWs_SendBytes(const uint16_t count, const uint8_t * array)
{
	uint16_t c; // __asm__("r2");
	int8_t b; // __asm__("r4");

	for(c=0; c < count; ++c) {
		const uint8_t val = array[c];
		for(b=7; b>=0; --b) {
			const bool bit = (val>>b) & 0x1;
 
 			if (bit) {
				// Send a ONE

				// HIGH with ASM sbi (2 words, 2 cycles)
				SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(high1 - sbiCycles);
				//  LOW with ASM cbi (2 words, 2 cycles)
				SET_PORT_PIN_LOW(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(low1 - cbiCycles);
			} else {
				// Send a ZERO

				// HIGH with ASM sbi (2 words, 2 cycles)
				SET_PORT_PIN_HIGH(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(high0 - sbiCycles);
				//  LOW with ASM cbi (2 words, 2 cycles)
				SET_PORT_PIN_LOW(dataPortId, dataPortPin);
				// Wait exact number of cycles specified
				DELAY_CYCLES(low0 - cbiCycles);
			}
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
// Helper routines
////////////////////////////////////////////////////////////////////////////////
template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::clear(const uint16_t numPixels)
{
	grey(numPixels, 0);
}

template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::grey(const uint16_t numPixels, const uint8_t value)
{
	pixelClass p;

	// For APA102 set first byte to trigger LED protocol
       	*(char*) & p = 0xFF;
	p.r = p.g = p.b = value;

 	begin();
	for(uint16_t i = 0; i < numPixels; i++) {
		send(1, &p);
	}
	end();
}


////////////////////////////////////////////////////////////////////////////////
// Begin/End
////////////////////////////////////////////////////////////////////////////////

template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::begin(void)
{
	switch (protocol) {
		case BITBANG_1WI_1P:
		case BITBANG_1WI_S2P:
		case BITBANG_1WI_I2P:
		case BITBANG_1WI_8P:
			// 1-wire: Delay next pixels to cause a refresh
			//DELAY_CYCLES(10);
			if (minMsRefresh)
				delay(minMsRefresh);
			// Disable interrupts.
			CLEAR_INTERRUPTS;
			break;
		case BITBANG_SPI:
			// SPI: Send start frame of 32 bits set to zero to force refresh
			bitbangSpi_Flatline(32, false);
			break;
		case HARDWARE_SPI:
		default:
			static_assert(protocol < UNSUPPORTED, LED_protocol_not_supported);
			break;
	}
}

template<FAB_TDEF>
inline void
fab_led<FAB_TVAR>::end(uint16_t count)
{
	switch (protocol) {
		case BITBANG_1WI_1P:
		case BITBANG_1WI_S2P:
		case BITBANG_1WI_I2P:
		case BITBANG_1WI_8P:
			// Restore interrupts.
			RESTORE_INTERRUPTS;
			break;
		case BITBANG_SPI:
			if (ledType == APA102) {
				// User overrides number of pixels tracked (rare)
				if (count) {
					pixelsDisplayed = count;
				}
				// Send end frame equal to 1/2 bit per pixel sent.
				bitbangSpi_Flatline((1+pixelsDisplayed)/2, true);
				pixelsDisplayed = 0;
			}
			break;
		case HARDWARE_SPI:
		default:
			static_assert(protocol < UNSUPPORTED, LED_protocol_not_supported);
			break;
	}
}

template<FAB_TDEF>
template <class pixelClassF>
inline void
fab_led<FAB_TVAR>::send(
		const uint16_t numPixels,
		const pixelClassF * array)
{
	if (pixelClassF::type == pixelClass::type) {
		// Native type, send as-is
		sendBytes(numPixels * stripBPP, (uint8_t *) array);
	} else {
		// Create a native pixel for buffer
		pixelClass p = pixelClass();
		uint8_t * pRaw = (uint8_t *) & p;
		// initialize special non-zero byte
		if (pixelClass::type & PT_BXXX) {
			pRaw[0] = 0xFF;
		}
		for (uint16_t i = 0; i < numPixels; i++) {
			const pixelClassF & a = array[i];
			const uint8_t * aRaw = (const uint8_t *) & a;

			// If statement is resolved at compile time
			if (pixelClass::type & pixelClassF::type & PT_BXXX) {
				pRaw[0] = aRaw[0];
			}
			// If statement is resolved at compile time
			if (pixelClass::type & pixelClassF::type & PT_XXXW) {
				pRaw[3] = aRaw[3];
			} else if (pixelClass::type & pixelClassF::type & PT_WXXX) {
				pRaw[0] = aRaw[0];
			} else if ((pixelClass::type & PT_XXXW) && (pixelClassF::type & PT_WXXX)) {
				pRaw[3] = aRaw[0];
			} else if ((pixelClass::type & PT_WXXX) && (pixelClassF::type & PT_XXXW)) {
				pRaw[0] = aRaw[3];
			}
			p.r = a.r;
			p.g = a.g;
			p.b = a.b;

			// Pixel p is a native type, send as-is
			sendBytes(stripBPP, (uint8_t *) &p);
		}
	}
	switch (protocol) {
		case BITBANG_1WI_1P:
		case BITBANG_1WI_S2P:
		case BITBANG_1WI_I2P:
		case BITBANG_1WI_8P:
			break;
		case BITBANG_SPI:
		case HARDWARE_SPI:
			if (ledType == APA102) {
				pixelsDisplayed += numPixels;
			}
			break;
		default:
			break;
	}
}


// The remap is optional. When it is used the remap array represents the physical
// layout of the LED strip, and holds the index in array[] of the color.
template<FAB_TDEF>
template <const uint8_t bitsPerPixel, class arrayClassF, class paletteClassF, class mapIntF>
inline void
fab_led<FAB_TVAR>::send(
		const uint16_t count,
		const arrayClassF * array,   // uint8_t if palette, else pixelClassF
		const paletteClassF * palette, // any custom pixelClassF, but not uint8_t
		const mapIntF * map)         // use uint8_t for <256 pixels, else uint16_t
{
	// Support only palettes with 2, 4, 16 or 256 colors
	static_assert( bitsPerPixel == 1 || bitsPerPixel == 2 ||
		bitsPerPixel == 4 || bitsPerPixel == 8,
		Unsupported_bits_per_pixel_palette);

	const mapIntF iMax = (const mapIntF) count;

	for (mapIntF i = 0; i < iMax; i++) {
		// Remapped index in array is index of the next pixel to push.
		// Pixels are pushed in order of the LED strip.
		const mapIntF ri = (map != NULL) ? map[i] : i;
		uint8_t ci = 0;

		// If palette is null, then it's only a remap. ArrayClassF must be a color
		if (palette == NULL) {
			send(1, (paletteClassF *) &array[ri]);
			continue;
		}

		// Extract the N-bit color index in order of LED strip using remap
		ci = PIgetPixel<bitsPerPixel>(array, ri);

		// Get the color/pixel to send. Without palette, the array is the pixel
		const paletteClassF * pixel =  (paletteClassF *) &palette[ci];

		// Draw pixel by pixel
		send(1, pixel);
	}
}


// The remap is optional. When it is used the remap array represents the physical
// layout of the LED strip, and holds the index in array[] of the color.
template<FAB_TDEF>
template <const uint8_t bitsPerPixel, class arrayClassF, class paletteClassF, class mapIntF>
inline void
fab_led<FAB_TVAR>::send(
		const uint16_t count,
		const arrayClassF * array,   // uint8_t if palette, else pixelClassF
		const uint8_t * redPalette, // any custom pixelClassF, but not uint8_t
		const uint8_t * greenPalette, // any custom pixelClassF, but not uint8_t
		const uint8_t * bluePalette, // any custom pixelClassF, but not uint8_t
		const mapIntF * map)         // use uint8_t for <256 pixels, else uint16_t
{
	// Support only palettes with 2, 4, 16 or 256 colors
	static_assert( bitsPerPixel == 1 || bitsPerPixel == 2 ||
		bitsPerPixel == 4 || bitsPerPixel == 8,
		Unsupported_bits_per_pixel_palette);

	const mapIntF iMax = (const mapIntF) count;

	for (mapIntF i = 0; i < iMax; i++) {
		// Remapped index in array is index of the next pixel to push.
		// Pixels are pushed in order of the LED strip.
		const mapIntF ri = (map != NULL) ? map[i] : i;
		uint8_t ci = 0;

		// Extract the N-bit color index via bitmasks
		ci = PIgetPixel<bitsPerPixel>(array, ri);

		// Get the color/pixel to send. Without palette, the array is the pixel
		const paletteClassF pixel;
	       	pixel.r = (redPalette != NULL) ?  redPalette[ci] : 0;
	       	pixel.g = (greenPalette != NULL) ?  greenPalette[ci] : 0;
	       	pixel.b = (bluePalette != NULL) ?  bluePalette[ci] : 0;

		// Draw pixel by pixel
		send(1, &pixel);
	}
}



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

// These are the more agressive bitbanging timings, note 0 and 1 have different durations
#define WS2812B_1H_CY CYCLES(500)  // 6  7 10 _----------__
#define WS2812B_1L_CY CYCLES(125)  // 2  2  4 .    .    .
#define WS2812B_0H_CY CYCLES(125)  // 2  2  5 _-----_______
#define WS2812B_0L_CY CYCLES(188)  // 2  2  7 .    .    .
#define WS2812B_MS_REFRESH 20      // Minimum sleep time (low) to reset LED strip
#define WS2812B_NS_RF 2000000      // Max refresh rate for all pixels to light up 2msec (LED PWM is 500Hz)
#endif


///////////////////////////////////////////////////////////////////////////////
// Implementation classes for LED strip
// Defines the actual LED timings
// WS2811 2811B 2812B 2812S 2801 LEDs use similar protocols and timings
////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Define all the LED protocol classes for each LED_TYPE using macro magic.
/// LED_TYPE1: One port definition, like ws2812b<D,6>
/// LED_TYPE2: Two ports definition, like apa102<D,5,D,6>
/// LED_TYPER: One port range defintion, like ws2812b8<D,0,6>
///
/// The macro declares the class setting all the template constants.
/// For example when NAME is ws2812b, that creates a class you can instantiate
/// in your code as: ws2812b<D,6>.
///
/// @param
/// NAME: Name of class
/// TYPE: Flags defining custom properties for that LED type
/// PIXEL: Native pixel type for that LED type (compared against the type
///        used for your pixel array
/// H1, L1, H0, L0: Timings in nano-seconds for bit banging (0 if unused)
/// RESET: Minimum time in msec to idle the bus between display refreshes
/// PROTOCOL: Wire communication protocol, to send the data across at low level
/// DPID: LED data line port register ID (a letter from A to I)
/// DPBIT: LED data line port bit (0 to 7)
/// CPID: LED clock line port register ID (a letter from A to I)
/// CPBIT: LED clock line port bit (0 to 7)
///////////////////////////////////////////////////////////////////////////////
#define LED_TYPE1(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) \
	template<avrLedStripPort  dataPortId, uint8_t  dataPortBit>           \
	class NAME : public fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET,dataPortId, dataPortBit, A, 0, PIXEL, TYPE, PROTOCOL>                      \
	{                                                                     \
		public:                                                       \
		NAME() : fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET, dataPortId, dataPortBit, A, 0, PIXEL, TYPE, PROTOCOL>() {};                   \
		~NAME() {};                                                   \
	};

#define LED_TYPE2(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL) \
	template<avrLedStripPort  dataPortId, uint8_t  dataPortBit,           \
	         avrLedStripPort clockPortId, uint8_t clockPortBit>           \
	class NAME : public fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET,dataPortId, dataPortBit, clockPortId, clockPortBit, PIXEL, TYPE, PROTOCOL>                      \
	{                                                                     \
		public:                                                       \
		NAME() : fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET, dataPortId, dataPortBit, clockPortId, clockPortBit, PIXEL, TYPE, PROTOCOL>() {};                   \
		~NAME() {};                                                   \
	};

#define LED_TYPER(NAME, TYPE, PIXEL, H1, L1, H0, L0,RESET, PROTOCOL)                                \
	template<avrLedStripPort  dataPortId, uint8_t  dataPortBit1, uint8_t dataPortBit2>           \
	class NAME : public fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET,dataPortId, dataPortBit1, dataPortId, dataPortBit2, PIXEL, TYPE, PROTOCOL>                      \
	{                                                                     \
		public:                                                       \
		NAME() : fab_led<CYCLES(H1), CYCLES(L1), CYCLES(H0), CYCLES(L0), RESET, dataPortId, dataPortBit1, dataPortId, dataPortBit2, PIXEL, TYPE, PROTOCOL>() {};                   \
		~NAME() {};                                                   \
	};

LED_TYPES_LIST

#undef LED_TYPE1
#undef LED_TYPE2
#undef LED_TYPER
#undef LED_TEMPLATE_PARAMS

#endif // FAB_LED_H
