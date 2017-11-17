#include "fredem.h"

/// Wartość multipleksera, aby był sterowany z gpio
#define MUX_GPIO 1

GlobalState globalState;

/// Typ zegara na urządzenia
typedef enum ClockType
{
	CLOCK_PORTA,
	CLOCK_PORTB,
	CLOCK_PORTC,
	CLOCK_PORTE,
	CLOCK_UART0,
	CLOCK_FTM0,
	CLOCK_RTC
} ClockType;

/// Aktywuj zegar na odpowiednie urządzenie
static void clockEnable(ClockType type);

/// Ustaw odpowiednią wartość multipleksera do sterowania źródłem sygnału
#define setPinMux(pinControl, value)								\
{																	\
	(pinControl) &= ~PORT_PCR_MUX_MASK;								\
	(pinControl) |= PORT_PCR_MUX(value);							\
}

static void clockEnable(ClockType type)
{
	switch(type)
	{
		case CLOCK_PORTA:
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
			break;
		case CLOCK_PORTB:
			SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
			break;
		case CLOCK_PORTC:
			SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
			break;
		case CLOCK_PORTE:
			SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
			break;
		case CLOCK_UART0:
			SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
			break;
		case CLOCK_FTM0:
			SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
			break;
		case CLOCK_RTC:
			SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;
			break;
	}
}

void setSubRgbLed(SubLedColor color, bool state)
{
	if(state)
	{
		switch(color)
		{
			case SUBRED:
				GPIOB_PDOR &= ~PTB_RGB_RED;
				break;
			case SUBGREEN:
				GPIOE_PDOR &= ~PTE_RGB_GREEN;
				break;
			case SUBBLUE:
				GPIOB_PDOR &= ~PTB_RGB_BLUE;
				break;
		}
	}
	else
	{
		switch(color)
		{
			case SUBRED:
				GPIOB_PDOR |= PTB_RGB_RED;
				break;
			case SUBGREEN:
				GPIOE_PDOR |= PTE_RGB_GREEN;
				break;
			case SUBBLUE:
				GPIOB_PDOR |= PTB_RGB_BLUE;
				break;
		}
	}
}

void setRgbLed(LedColor color)
{	
	//ustaw odpowiednie żarniki
	switch(color)
	{
		case RED:
			setSubRgbLed(SUBRED, true);
			setSubRgbLed(SUBGREEN, false);
			setSubRgbLed(SUBBLUE, false);
			break;
		case GREEN:
			setSubRgbLed(SUBRED, false);
			setSubRgbLed(SUBGREEN, true);
			setSubRgbLed(SUBBLUE, false);
			break;
		case BLUE:
			setSubRgbLed(SUBRED, false);
			setSubRgbLed(SUBGREEN, false);
			setSubRgbLed(SUBBLUE, true);
			break;
		case CYAN:
			setSubRgbLed(SUBRED, false);
			setSubRgbLed(SUBGREEN, true);
			setSubRgbLed(SUBBLUE, true);
			break;
		case YELLOW:
			setSubRgbLed(SUBRED, true);
			setSubRgbLed(SUBGREEN, true);
			setSubRgbLed(SUBBLUE, false);
			break;
		case MAGENTA:
			setSubRgbLed(SUBRED, true);
			setSubRgbLed(SUBGREEN, false);
			setSubRgbLed(SUBBLUE, true);
			break;
		case WHITE:
			setSubRgbLed(SUBRED, true);
			setSubRgbLed(SUBGREEN, true);
			setSubRgbLed(SUBBLUE, true);
			break;
		case BLACK:
			setSubRgbLed(SUBRED, false);
			setSubRgbLed(SUBGREEN, false);
			setSubRgbLed(SUBBLUE, false);
			break;
	}
}

bool isButtonPressed(Button button)
{
	switch(button)
	{
		case BUTTON2:
			if(globalState.button2 != DEVICE_MODE_POLLING)
			{
				return false;
			}
			return (GPIOC_PDIR & PTC_BUTTON2) == 0;
			break;
		case BUTTON3:
			if(globalState.button3 != DEVICE_MODE_POLLING)
			{
				return false;
			}
			return (GPIOA_PDIR & PTA_BUTTON3) == 0;
			break;
	}
	return false;
}

void initPins(InitType type)
{
	switch(type)
	{
		case INIT_LED:
			clockEnable(CLOCK_PORTB);
			clockEnable(CLOCK_PORTE);
			// ustaw piny, żeby były sterowane za pomocą GPIO
			setPinMux(PORTB_PCR21, MUX_GPIO);
			setPinMux(PORTB_PCR22, MUX_GPIO);
			setPinMux(PORTE_PCR26, MUX_GPIO);
			// ustaw piny LED jako wyjścia
			GPIOB_PDDR |= PTB_RGB_RED;
			GPIOB_PDDR |= PTB_RGB_BLUE;
			GPIOE_PDDR |= PTE_RGB_GREEN;
			globalState.ledEnabled = true;
			break;
		case INIT_BUTTON2:
			clockEnable(CLOCK_PORTC);
			// ustaw piny, aby były sterowalne za pomocą GPIO
			setPinMux(PORTC_PCR6, MUX_GPIO);
			// ustaw piny przycisku 2 jako wejścia
			GPIOC_PDDR &= ~PTC_BUTTON2;
			globalState.button2 = DEVICE_MODE_POLLING;
			break;
		case INIT_BUTTON3:
			clockEnable(CLOCK_PORTA);
			// ustaw piny, aby były sterowalne za pomocą GPIO
			setPinMux(PORTA_PCR4, MUX_GPIO);
			// ustaw piny przycisku 3 jako wejścia
			GPIOA_PDDR &= ~PTA_BUTTON3;
			globalState.button3 = DEVICE_MODE_POLLING;
			break;
		case INIT_BUTTON2_INTERRUPT:
			// ustaw piny, jak do wyjścia GPIO
			initPins(INIT_BUTTON2);
			// ustaw piny, jako przerwanie
			PORTC_PCR6 &= ~PORT_PCR_IRQC_MASK;
			PORTC_PCR6 |= PORT_PCR_MUX_INTERRUPT_FALLING;
			// zaakceptuj przerwanie
			clearInterrupt(INTERRUPT_SOURCE_BUTTON2);
			globalState.button2 = DEVICE_MODE_INTERRUPT;
			// aktywuj funcję, do której ma skoczyć na przerwanie
			NVIC_EnableIRQ(PORTC_IRQn);
			break;
		case INIT_UART0_POLLING:
			// 8 Bitów, brak parzystości, jeden bit stopu, 9600 baud rate na odbiorniku (9600 8N1)
			// SDA jest podłączone do UART0 i UART1, używamy UART0
			clockEnable(CLOCK_UART0);
			clockEnable(CLOCK_PORTB);
			// ustaw piny, aby przekazywały do UART0
			setPinMux(PORTB_PCR16, 3);
			setPinMux(PORTB_PCR17, 3);
			// wyłącz nadajnik i odbiornik, WARNING może niekoniecznie jest to potrzebne
			UART0_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
			// ustaw tryb domyślny (8 bitów, brak parzystości)
			UART0_C1 = 0x00;
			// ustaw 1 bit stopu
			UART0_BDH &= ~UART_BDH_SBNS_MASK;
			// zapisz baud rate, wartość 13-bitowa zapisywana do dwóch 8-bitowych rejestrów, zapisanie dolnego dopiero aktywuje górny
			UART0_BDH &= 0xE0;
			UART0_BDH |= 0x1F & (UART_CLOCK_DIVISOR >> 8);
			UART0_BDL = (uint8_t)(UART_CLOCK_DIVISOR & 0xFF);
			// włącz nadajnik i odbiornik
			UART0_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
			globalState.uart0 = DEVICE_MODE_POLLING;
			break;
		case INIT_UART0_INTERRUPT:
			// początek w taki sam sposób, jak odpytywanie
			initPins(INIT_UART0_POLLING);
			// aktywuj przerwanie na wolny bufor
			UART0_C2 |= UART_C2_TIE_MASK;
			// aktywuj przerwanie na zajęty bufor
			UART0_C2 |= UART_C2_RIE_MASK;
			globalState.uart0 = DEVICE_MODE_INTERRUPT;
			// włącz przerwania
			NVIC_EnableIRQ(UART0_RX_TX_IRQn);
			break;
		case INIT_FTM0:
			clockEnable(CLOCK_FTM0);
			// zegar dzieli się na 128
			FTM0_SC |= FTM_SC_PS(7);
			// źródło zegara "System clock"
			FTM0_SC |= FTM_SC_CLKS(1);
			// aktywacja przerwań na zmodulowanie zegara
			FTM0_SC |= FTM_SC_TOIE_MASK;
			globalState.ftm0 = DEVICE_MODE_INTERRUPT;
			// inicjalizujemy funcję do której przerywa
			NVIC_EnableIRQ(FTM0_IRQn);
			break;
		case INIT_RTC:
			clockEnable(CLOCK_RTC);
			// wypuść oscylator, WARNING powinno się czekać, aż ustabilizuje się
			if((RTC_CR & RTC_CR_OSCE_MASK) == 0)
			{
				RTC_CR |= RTC_CR_OSCE_MASK;
			}
			// reset
			RTC_CR &= ~RTC_CR_SWR_MASK;
			// sprawdź, czy czas jest poprawny
			if((RTC_SR & RTC_SR_TIF_MASK) != 0)
			{
				// niepoprawny czas, więc reset liczników
				// wyłącz licznik
				RTC_SR &= ~RTC_SR_TCE_MASK;
				// zapisz alarm na następny wiek
				RTC_TAR = 0xFFFFFFFF;
				// zapisz czas 0
				RTC_TPR = 0x0000;
				RTC_TSR = 0x00000000;
			}
			// włącz licznik
			RTC_SR |= RTC_SR_TCE_MASK;
			if((RTC_SR & RTC_SR_TIF_MASK) != 0)
			{
				// czas nadal jest niepoprawny
				panicDeviceError();
			}
			// włącz przerwania co sekundę
			RTC_IER |= RTC_IER_TSIE_MASK;
			globalState.rtc = DEVICE_MODE_INTERRUPT;
			// pozwól na przerwania co sekundę
			NVIC_EnableIRQ(RTC_Seconds_IRQn);
			// pozwól na inne przerwania
			NVIC_EnableIRQ(RTC_IRQn);
			break;
	}
}

void clearInterrupt(InterruptSource source)
{
	switch(source)
	{
		case INTERRUPT_SOURCE_BUTTON2:
			PORTC_ISFR |= PTC_BUTTON2;
			break;
		case INTERRUPT_SOURCE_FTM0:
			FTM0_SC &= ~FTM_SC_TOF_MASK;
			break;
	}
}

void setClock0(uint16_t clockValue)
{
	// ustaw ilość cykli powodujących zmodulowanie
	FTM0_MOD = clockValue;
}

void pulsePanic(LedColor color)
{
	pulsePanicDouble(color, color);
}

void panicGeneralCriticalError()
{
	PRINT_DEBUG_STRING("Ogólny błąd krytyczny systemu!");
	pulsePanic(RED);
}

void sendUART0BytePolling(uint8_t byte)
{
	if(globalState.uart0 != DEVICE_MODE_POLLING)
	{
		return;
	}
	// czekaj na bit wolnego bufora, gdy będzie 1
	while((UART0_S1 & UART_S1_TDRE_MASK) == 0);
	// zapisz do bufora pamięci
	UART0_D = byte;
}

uint8_t receiveUART0BytePolling()
{
	if(globalState.uart0 != DEVICE_MODE_POLLING)
	{
		return 0;
	}
	//czekaj na bit zajętego bufora
	while((UART0_S1 & UART_S1_RDRF_MASK) == 0);
	// odczytaj z bufora pamięci
	return UART0_D;
}

void pulsePanicDouble(LedColor color1, LedColor color2)
{
	PRINT_DEBUG_STRING("Panika Systemu!");
	// licznik jednego okresu, przepełnia się, dioda zapalona dla części wartości, 255 stopni jasności
	uint8_t counter = 0;
	// duży licznik od współczynnika zapalenia w czasie, przepełnia się rzadko i może brzydko mrugnąć diodą
	uint32_t bigCounter = 0;
	// zamieniamy kolor co drugi cykl
	bool isColor1 = true;
	while(true)
	{
		if(!globalState.ledEnabled)
		{
			continue;
		}
		// licznik się przepełnia
		counter++;
		// ponieważ ludzkie oko nie widzi koloru liniowo, zastosowanie liniowej zmiany jasności powoduje bardzo mało zauważalne przygasania
		// rozwiązaniem jest użycie nieliniowej pulsacji, aby w oku wyglądała bardziej liniowo, wykres jest jak fale oceanu w kształcie kolców _/\_/\_/\_/\_
		if(counter < (-fabs(sinf(bigCounter * 0.02)) + 1.0) * UINT8_MAX)
		{
			if(isColor1)
			{
				setRgbLed(color1);
			}
			else
			{
				setRgbLed(color2);
			}
		}
		else
		{
			setRgbLed(BLACK);
		}
		if(counter == UINT8_MAX)
		{
			bigCounter++;
		}
		isColor1 = !isColor1;
		//dzielimy tak, aby nie popsuł cyklu
		bigCounter %= (uint32_t)(3.1415 / 0.02);
	}
}

void sendUART0String(const char* string)
{
	if(globalState.uart0 != DEVICE_MODE_POLLING)
	{
		return;
	}
	int i = 0;
	while(string[i] != 0)
	{
		sendUART0BytePolling(string[i]);
		i++;
	}
}

void panicDeviceError()
{
	PRINT_DEBUG_STRING("Błąd urządzenia!");
#ifdef DEBUG
	pulsePanicDouble(CYAN, GREEN);
#else
	panicGeneralCriticalError();
#endif
}
