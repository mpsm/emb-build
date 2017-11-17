#pragma once
#include "MK64F12.h"
#include "FreeRTOSConfig.h"
#include <stdbool.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
// Piny PTA:																//
//////////////////////////////////////////////////////////////////////////////

/// Przycisk SW3
#define PTA_BUTTON3 (1 << 4)

//////////////////////////////////////////////////////////////////////////////
// Piny PTB:																//
//////////////////////////////////////////////////////////////////////////////

/// Czerwony żarnik diody
#define PTB_RGB_RED (1 << 22)

/// Niebieski żarnik diody
#define PTB_RGB_BLUE (1 << 21)

//////////////////////////////////////////////////////////////////////////////
// Piny PTC:																//
//////////////////////////////////////////////////////////////////////////////

/// Przycisk SW2
#define PTC_BUTTON2 (1 << 6)

//////////////////////////////////////////////////////////////////////////////
// Piny PTD:																//
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Piny PTE:																//	
//////////////////////////////////////////////////////////////////////////////

/// Zielony żarnik diody
#define PTE_RGB_GREEN (1 << 26)

//////////////////////////////////////////////////////////////////////////////
// PORT																		//	
//////////////////////////////////////////////////////////////////////////////

/// Wartość pola multiplexera, aby ustawić pin jako przerwanie na wznoszącej wartości, czyli jako deaktywacja wejścia
#define PORT_PCR_MUX_INTERRUPT_RISING 0x00090000
/// Wartość pola multiplexera, aby ustawić pin jako przerwanie na spadającej wartości, czyli jako aktywacja wejścia
#define PORT_PCR_MUX_INTERRUPT_FALLING 0x000A0000

/// Baud rate dla UART (13-bitowe)
#define TARGET_BAUD_RATE 115200
#define UART_CLOCK_DIVISOR ((int16_t)(DEFAULT_SYSTEM_CLOCK / (TARGET_BAUD_RATE * 16)))

#ifdef DEBUG
	#define PRINT_DEBUG_STRING(string) sendUART0String(string "\n")
#else
	#define PRINT_DEBUG_STRING(string)
#endif

//////////////////////////////////////////////////////////////////////////////
// INNE																		//	
//////////////////////////////////////////////////////////////////////////////

/// Kolor żarnika diody
typedef enum SubLedColor
{
	SUBRED,
	SUBGREEN,
	SUBBLUE
} SubLedColor;

/// Kolor całej diody
/// Kolejno: 
/// - 3 kolory podstawowe 
/// - 3 złożone 
/// - biały
/// - wyłączony
typedef enum LedColor
{
	RED,
	GREEN,
	BLUE,
	YELLOW,
	CYAN,
	MAGENTA,
	WHITE,
	BLACK
} LedColor;

/// Przycisk
typedef enum Button
{
	BUTTON2,
	BUTTON3
} Button;

/// Typ initializacji
typedef enum InitType
{
	INIT_LED,
	INIT_BUTTON3,
	INIT_BUTTON2,
	INIT_BUTTON2_INTERRUPT,
	INIT_UART0_POLLING,
	INIT_UART0_INTERRUPT,
	INIT_FTM0,
	INIT_RTC
} InitType;

/// Co wywołało przerwanie
typedef enum InterruptSource
{
	INTERRUPT_SOURCE_BUTTON2,
	INTERRUPT_SOURCE_FTM0
} InterruptSource;

/// Tryb działania urządzenia, czy jako odpytywanie, czy jako przerwania, używane aby nieobsługiwać niezainicjalizowanych urządzeń
typedef enum DeviceMode
{
	DEVICE_MODE_DISABLED,
	DEVICE_MODE_POLLING,
	DEVICE_MODE_INTERRUPT
} DeviceMode;

/// Stan działania systemu
typedef struct GlobalState
{
	bool ledEnabled;
	DeviceMode ftm0;
	DeviceMode button2;
	DeviceMode button3;
	DeviceMode uart0;
	DeviceMode rtc;
} GlobalState;

/// Globalny stan działania programu
extern GlobalState globalState;

/// Inicjalizuj piny, które mają być używane w programie 
void initPins(InitType type);

/// Ustaw jedną z żarówek diody
void setSubRgbLed(SubLedColor color, bool state);

/// Ustaw ogólny kolor diody
void setRgbLed(LedColor color);

/// Czy przycisk jest naciśnięty?
bool isButtonPressed(Button button);

/// Usuń, zaakceptuj przerwanie
void clearInterrupt(InterruptSource source);

/// Aktywuj periodyczne przerwania o określonym okresie
void setClock0(uint16_t clockValue);

/// Ustaw tryb paniki, urządzenie pulsuje określonym światłem
void pulsePanic(LedColor color);

/// Ustaw tryb paniki, udządzenie pulsuje nieskończonej pętli światłem będącym sumą dwóch kolorów
void pulsePanicDouble(LedColor color1, LedColor color2);

/// Jakiś wewnętrzny błąd systemu się stał, ta funkcja blokuje się w nieskończonej pętli
void panicGeneralCriticalError();

/// Jakiś błąd urządzenia się stał
void panicDeviceError();

/// Wyślij bajt na UART0 przez odpytywanie w pętli
void sendUART0BytePolling(uint8_t byte);

/// Odczytaj bajt z UART0 przez odpytywanie w pętli
uint8_t receiveUART0BytePolling();

/// Wyślij string na UART0 przez odpytywanie w pętli
void sendUART0String(const char* string);
