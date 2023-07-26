/*
 * main.c
 *
 * Created: 1/25/2022 2:34:40 PM
 *  Author: igor.abramov
 */ 

#define F_CPU   16000000L

#define Check(REG,BIT) (REG &  _BV(BIT))
#define Inv(REG,BIT)   (REG ^= _BV(BIT)) 
#define High(REG,BIT)  (REG |= _BV(BIT))
#define Low(REG,BIT)   (REG &= ~_BV(BIT))  

#define Led     Check(PORTB, PORTB5)
#define LedOn   High(PORTB, PORTB5)
#define LedOff  Low(PORTB, PORTB5)
#define LedInv  Inv(PORTB, PORTB5)

#define True	1
#define False	0

#define Forward  0
#define Backyard 1

#define Left  0
#define Right 1

#define Init 2
#define On	 1
#define Off  0

#define NextLine 0x0A
#define FillCell 0xFF
#define Terminator '$'

#define RxBufferSize 100
#define TxBufferSize 100

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

//#define DDSOut	 (Check(PORTD, 7))
//#define DDSOutInv Inv(PORTD, 7)

//#define ImpOn  Low(PORTD, 7)
//#define ImpOff High(PORTD, 7)
#define ImpOn  High(PORTB, PORTB3)
#define ImpOff Low(PORTB, PORTB3)
#define ImpInv Inv(PORTB, PORTB3)

#define ServoUp		 High(PORTB, PORTB1)
#define	ServoDown 	 Low(PORTB, PORTB1)
#define ServoCommand (Check(PINC, PINC0))

#define Active		(!Check(PIND, PIND2))

#define Counter		0
#define Oscillator  1

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <avr/eeprom.h>
#include "lcd/lcdpcf8574/lcdpcf8574.h"
#include "dht/dht.h"

const unsigned long int		ACCUM_MAXIMUM = 1875000000;
//const unsigned int		FREQUENCY_MAXIMUM = 7812; // timer2 divider 1024
const unsigned int	    FREQUENCY_MAXIMUM = 31250; // timer2 divider 256
//const unsigned long int   FREQUENCY_MAXIMUM = 62500; // timer2 divider 128

bool handleAfterSecond = false;
unsigned int timer2_overflowCount = 0;

struct
{
	unsigned int ms40, ms200, ms1000;
	unsigned int ms16, ms992, sec;
	bool isr;
} MainTimer;

struct
{
	unsigned short forward;
	unsigned short backward;
	unsigned short button;
	float multiplier;
	float addendumValues[4];
	enum Addendums
	{
		one,
		ten,
		hundred,
		thousand
	} addendum;
} Encoder;

struct
{
	unsigned short sec, min, hour, day, week, month, year;
} Watch;

struct
{
	unsigned char byte;
	unsigned short byteReceived;
} Rx;

struct
{
	unsigned long int ticksCurrent,ticksPrevious,ticks;
	unsigned long int overflows,ticksBuffer;
	unsigned short action, periodicMeasure, ovfFlag;
	unsigned short done, index, valuesFull, method, zero;
	unsigned int average;
	float values[100];
	float period, frequency, previousFrequency, bufFrequency, pulseCount;
} Measure;

struct
{
	float setting;
	unsigned long int increment, accum, frequency;
} DDS;

struct
{
	float Kpid;
	float Kp;
	float Ki;
	float Kd;
	
} Factors;

struct
{
	float tension;
	int value;
	unsigned short done;
} Convert;

struct
{
	float temperature;
	float humidity;
} Env;

struct
{
	char ip[16];
	unsigned int port;
} Server;

ISR(TIMER0_OVF_vect)
{
	MainTimer.ms16++;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT0 = 5;
}

ISR(TIMER1_OVF_vect)
{
	if (MainTimer.ms40 % 5 == 0) MainTimer.ms200++;
	
	if (MainTimer.ms40 >= 25)
	{
		MainTimer.ms1000++;
		MainTimer.ms40 = 0;
	}
	
	MainTimer.isr = true;
	TCNT1 = 64911;
}

//ISR(TIMER2_OVF_vect)
//{
	//TCNT2 = 255;
	//
	//DDS.accum += DDS.increment;
	//
	//if (DDS.accum >= ACCUM_MAXIMUM)
	//{
		////DDSOutInv;
		//DDS.accum -= ACCUM_MAXIMUM;
	//}
//}

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;	
}

ISR(ADC_vect)
{
	ADCSRA |= (0<<ADSC);
	Convert.value = ADCW;
	Convert.done++;
}

ISR(ANALOG_COMP_vect)
{
	LedInv;
}

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(0 << CS01)|(1 << CS00);
		TIMSK0 = (1 << TOIE0);
		TCNT0 = 0;
		return;
	}
	
	TCCR0B = (0 << CS02)|(0 << CS01)|(0 << CS00);
	TIMSK0 = (0 << TOIE0);
	TCNT0 = 0;
}

void Timer1(unsigned short mode)
{
	switch(mode)
	{
		case Counter:
			TCCR1B = (1 << CS12)|(0 << CS11)|(1 << CS10);
			TIMSK1 = (1 << TOIE1);
			TCNT1 = 62411;
			break;
		case Oscillator:
			TCCR1A|=(1<<COM1A1)|(1<<WGM11);        //NON Inverted PWM
			TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
			ICR1=4999;  //fPWM=50Hz	
			break;
		default:
			TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
			TIMSK1 = (0 << TOIE1);
			TCNT1 = 62411;
			break;
	}
}

//void Timer2(bool enable)
//{
	//if (enable)
	//{
		//TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20); // 128 bit scaler
		//TIMSK2 = (1<<TOIE2);
		//return;
	//}
	//
	//TCCR2B = (0<<CS22) | (0<<CS21) | (0<<CS20);
	//TIMSK2 = (0<<TOIE2);
	//TCNT2 = 0;
//}

void Timer2(bool enable)
{
	TCNT2 = 0; 	   // reset count register
	
	if (enable)
	{
		TCCR2A = (1 << WGM21)|(1 << WGM20);				// configuration hardware PWM
		TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);	// scaler 1024
		High(TIMSK2, TOIE2);							// overflow interrupt enabled, every 8 ms.
		return;
	}
	
	TCCR2B = 0x00;										// stop count
	Low(TIMSK2, TOIE2);									// overflow interrup disabled
}

ISR(TIMER2_OVF_vect)
{
	timer2_overflowCount++;					 // increase overflow variable

	if (timer2_overflowCount >= 125)		 // 8*125 = 1000 ms
	{
		handleAfterSecond = true;			 // set flag to handle data
		timer2_overflowCount = 0;			 // reset overflow counter
	}
	// load 131 to count register, 256-131 = 125 ticks of 64us, 125*64 = 8ms
	TCNT2 = 131;
}
 
void Comparator()
{
	Low(ADCSRA, ADEN);
	Low(ADCSRB, ACME);
	Low(ACSR, ACI);
	High(ACSR, ACBG);
	High(ACSR, ACIE);
	High(ACSR, ACIS1);
}
 
void EraseUnits(int x, int y, int offset, float count)
{
	static unsigned char eraser = 32;
	
	if (count<1000000000 || count < 0)
	{
		lcd_gotoxy(x+offset+9,y);
		lcd_putc(eraser);
	}
	
	if (count<100000000 || count < 0)
	{
		lcd_gotoxy(x+offset+8,y);
		lcd_putc(eraser);
	}
	
	if (count<10000000 || count < 0)
	{
		lcd_gotoxy(x+offset+7,y);
		lcd_putc(eraser);
	}
	
	if (count<1000000 || count < 0)
	{
		lcd_gotoxy(x+offset+6,y);
		lcd_putc(eraser);
	}
	
	if (count<100000 || count < 0)
	{
		lcd_gotoxy(x+offset+5,y);
		lcd_putc(eraser);
	}
	
	if (count<10000)
	{
		lcd_gotoxy(x+offset+4,y);
		lcd_putc(eraser);
	}
	
	if (count<1000)
	{
		lcd_gotoxy(x+offset+3,y);
		lcd_putc(eraser);
	}
	
	if (count<100)
	{
		lcd_gotoxy(x+offset+2,y);
		lcd_putc(eraser);
	}
	
	if (count<10)
	{
		lcd_gotoxy(x+offset+1,y);
		lcd_putc(eraser);
	}
}

void DisplayPrint()
{
	static char seconds[20];
	
	EraseUnits(0, 0, 0, MainTimer.sec);
	sprintf(seconds, "%d", MainTimer.sec);
	lcd_gotoxy(0, 0);
	lcd_puts(seconds);
}

void Converter(unsigned short option)
{
	switch (option)
	{
		case Off:
			ADCSRA |= (0<<ADSC);
			break;
		case On:
			ADCSRA |= (1<<ADSC);
			break;
		default:
			ADCSRA = 0x8F;
			ADMUX = 0x41;
			ADCSRA |= (0<<ADSC);
			break;
	}
}

void USART(unsigned short option)
{
	switch (option)
	{
		case 0:
			UCSR0B |= (0 << TXEN0);
			break;
		case 1:
			UCSR0B |= (1 << TXEN0);
			break;
		default:
			UCSR0B = (1 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
			UBRR0L = 3;	 // 250000 b/s
			break;
	}
}

void SPI_Write(unsigned int word)
{
	unsigned char MSB = ((word >> 8) & 0x0f) | 0x70;  	//filter out MS
	unsigned char LSB = word & 0xff;			//filter out LS
	//
	//StartSPI;

	SPDR = MSB;							// 	send First 8 MS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy
	
	SPDR = LSB;							// 	send Last 8 LS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy

	//EndSPI;
}

void TxChar(unsigned char c)
{
	while (!Check(UCSR0A, UDRE0));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

void Transmit()
{
	static char a[8] = { 0 }, b[8] = { 0 }, d[8] = { 0 };
	static char buffer[32] = { 0 };
	
	sprintf(a, "A%d$ ", 379);
	sprintf(b, "P%d$ ", 384);
	sprintf(d, "D%d", 2);
	
	strcat(buffer, a);
	strcat(buffer, b);
	strcat(buffer, d);
	
	TxString(buffer);
	
	buffer[0] = '\0';
}

void Receive()
 {
	 static char RxBuffer[RxBufferSize];
	 static char TxBuffer[TxBufferSize] = "error: ";
	 static char RxCharBuffer[2];
	 
	 RxCharBuffer[0] = Rx.byte;
	 
	 if (RxCharBuffer[0] != Terminator)
	 { 
		 strcat(RxBuffer, RxCharBuffer);
		 return;
	 }
	 
	 if (!(strcasecmp(RxBuffer, "led")))
	 {
		 LedInv;
	 }
	 else
	 {
		 strcat(TxBuffer, RxBuffer);
		 TxString(TxBuffer);
		 memset(TxBuffer, 9, TxBufferSize);
	 }
	 
	 memset(RxBuffer, 0, RxBufferSize);
 }

float GetAddendum(void)
{
	static unsigned int divider = 0;
	divider = DDS.setting < 11000 ? 10000 : 100000;
	return (((ACCUM_MAXIMUM/divider)*DDS.setting)/FREQUENCY_MAXIMUM)*divider;
}

void SetOptionDDS(short direction)
{
	if (!direction)
	{
		DDS.setting = Measure.frequency;// * Encoder.multiplier;
		DDS.increment = GetAddendum();
		return;
	}
	
	if (direction > 0)
	{
		Encoder.multiplier += DDS.setting >= FREQUENCY_MAXIMUM ? 0 : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
	
	if (direction < 0)
	{
		Encoder.multiplier -= Encoder.multiplier <= Encoder.addendumValues[Encoder.addendum] ? Encoder.multiplier : Encoder.addendumValues[Encoder.addendum];
		eeprom_update_float((float*)1, Encoder.multiplier);
	}
}

void RegulatorInit(float Kpid, float Kp, float Ki, float Kd)
{
	Factors.Kpid = Kpid;
	Factors.Kp = Kp;
	Factors.Ki = Ki;
	Factors.Kd = Kd;
}

void StepperStep()
{
	static unsigned short direction = Forward;

	if (DDS.setting <= 0 && direction == Backyard) direction = Forward;
	if (DDS.setting >= 20000 && direction == Forward) direction = Backyard;
	if (direction == Forward) Measure.frequency++;// else Measure.frequency--;
	SetOptionDDS(0);
}

void Regulator(void)
{
	static float I,previousError;
	float P,D,regulationError,factor;

	DDS.setting = (Measure.frequency*Encoder.multiplier)*Factors.Kpid;
	regulationError = DDS.setting - DDS.frequency;
	
	P = regulationError*Factors.Kp;
	I = (I+(regulationError*0.08))*Factors.Ki;
	D = ((regulationError-previousError)/0.08)*Factors.Kd;
	
	factor = P+I+D;
	previousError = regulationError;
	
	DDS.frequency = factor < 0 ? factor*(-1) : factor;
	
	DDS.increment = GetAddendum();
}

void ServoStep(unsigned int direction)
{
	static unsigned short action = 0;
	
	if (ServoCommand) action = 0;
	{
		if (!ServoCommand) action++;
		{
			if (action == 1)
			{	
				ServoUp;
				_delay_ms(1);
				ServoDown;
				action = 0;
			}
		}
	}
}

void EnvRequest()
{	
	static char bufferT[20], bufferH[20];
	
	if(dht_gettemperaturehumidity(&Env.temperature, &Env.humidity) != -1) 
	{
		sprintf(bufferT, " T%.1f ", Env.temperature);
		TxString(bufferT);
		sprintf(bufferH, " H%.1f ", Env.humidity);
		TxString(bufferH);
	} 
	else 
	{
		TxString(" error ");
	}
}

unsigned short GetDataSize(float value, unsigned short literalSize)
{
	if (value < 10)	return literalSize + 3 + 1; // literal size (F, Tn...) + figures quantity + Terminator
	if (value < 100) return literalSize + 4 + 1;
	if (value < 1000) return literalSize + 5 + 1;
	if (value < 10000) return literalSize + 6 + 1;
	if (value < 100000) return literalSize + 7 + 1;
	return 0;
}

bool ConnectToServer()
{
	static char connectString[60] = "AT+CIPSTART=\"TCP\",\"192\".\"168\".\"252\".\"69\",11000";
	TxString(connectString);	
	return true;
}

void SendToServer()
{ 
	
	static unsigned short size = 0;
	static char frequency[20], sizeBuffer[10], buffer[100];
	
	
	size = GetDataSize(DDS.setting, 1);
	sprintf(frequency, "F%.1f$", DDS.setting);
	sprintf(sizeBuffer, "%.d", size);
	strcat(buffer, "AT+CIPSEND=");
	strcat(buffer, sizeBuffer);
	TxString(buffer);
	TxString(frequency);
}

void Step(short direction)
{
	ImpOn;
	
	if (direction) 
	{
		_delay_ms(12);  // invert - 800 us   // direct 1 ms
		ImpOff;
		_delay_ms(1);
		return;
	}
	
	_delay_ms(1);
	ImpOff;
	_delay_ms(12);
}

void Step2(short direction)
{
	if (direction)
	{
		if (OCR2A == 132) return;
		OCR2A = 132;
		return;
	}
	
	if (OCR2A == 250) return; // 250
	OCR2A = 250;
} 

void Control()
{
	if (Active) { Step2(Right);	return; }
	
	Step2(Left);
}

int main(void)
{
	DDRB = 0b00111111;					// ports init
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000100;
	PORTD = 0b11111011;
	
	//Timer1(Counter);
	Timer2(On);
	//Comparator();
	USART(Init);
	USART(On);
	sei();
	
	//for (int i = 0; i < 1024; i++) eeprom_update_word((uint16_t*)i, 0);
	
	while(1)
	{	
		if (handleAfterSecond)
		{
			LedInv;

			Transmit();

			handleAfterSecond = false;
		}

		//Control();

		//if (MainTimer.isr)
		//{
			//MainTimer.ms40++;
			//MainTimer.isr = false;
		//}

		//if (MainTimer.ms200)
		//{
			//MainTimer.ms200 = 0;
		//}

		//if (MainTimer.ms1000)
		//{
			//MainTimer.sec++;
			//if (MainTimer.sec >= 59) MainTimer.sec = 0;
			//MainTimer.ms1000 = 0;
		//}
	}								  
}