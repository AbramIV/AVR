/*
 * Unwinder.c
 *
 * Created: 17.02.2023 20:05:24
 * Author : prote
 */ 

#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define EchoPin	        Check(PINB, PINB0)

#define Ultrasonic      Check(PORTB, PORTB1)
#define UltrasonicOn    High(PORTB, PORTB1)
#define UltrasonicOff   Low(PORTB, PORTB1)
#define UltrasonicInv	Inv(PORTB, PORTB1)

#define Triac			Check(PORTB, PORTB2)
#define TriacOn			High(PORTB, PORTB2)
#define TriacOff		Low(PORTB, PORTB2)
#define TriacInv		Inv(PORTB, PORTB2)

#define Led				Check(PORTB, PORTB5)
#define LedOn			High(PORTB, PORTB5)
#define LedOff			Low(PORTB, PORTB5)
#define LedInv			Inv(PORTB, PORTB5)

#define Off		0
#define On		1
#define Init	2

#define ArraySize 16

#define WaveDuration	25

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "lcd/lcdpcf8574/lcdpcf8574.h"

unsigned long int StartTicks = 0;
unsigned long int LastTicks = 0;
bool PulseCaptured = false;

unsigned short WaveDurationCount = 0;
unsigned short Timer0_OverflowCount = 0;
unsigned long int Timer1_OverflowCount = 0;
unsigned short Timer2_OverflowCount = 0;
bool HandleAfterSecond = false;
bool HandleAfter200ms = false;

void Timer0(bool enable)
{
	if (enable)
	{
		TCNT0 = 251;
		High(TCCR0B, CS02);
		High(TIMSK0, TOIE0);
		return;
	}
	
	Timer0_OverflowCount = 0;
	Low(TIMSK0, TOIE0);
	Low(TCCR0B, CS02);
	TCCR0B = 0x00;
	TCNT0 = 0;
}

ISR(TIMER0_OVF_vect)
{
	if (WaveDurationCount) WaveDurationCount--;
	TCNT0 = 251;
}

void Timer1(unsigned short option)
{
	switch(option)
	{
		case Init:
			High(TCCR1B, ICNC1);
			TIMSK1 = (1 << TOIE1)|(1 << ICIE1);
			TCNT1 = 0;
			break;
		case On:
			High(TCCR1B, ICES1);
			High(TCCR1B, CS10);
			High(TIMSK1, TOIE1);
			High(TIMSK1, ICIE1);
			break;
		default:
			Low(TCCR1B, CS10);
			Low(TIMSK1, TOIE1);
			Low(TIMSK1, ICIE1);
			break;
	}
}

ISR(TIMER1_OVF_vect)
{
	Timer1_OverflowCount++;
}

ISR(TIMER1_CAPT_vect)
{	
	if (EchoPin) 
	{
		Timer1_OverflowCount = 0;
		StartTicks = ICR1;
		Low(TCCR1B, ICES1);
		return;
	}
	
	LastTicks = ICR1;
	Timer1(Off);
	PulseCaptured = true;
}

void Timer2(bool enable)
{
	TCNT2 = 0;
	
	if (enable)
	{
		TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);
		High(TIMSK2, TOIE2);
		return;
	}
	
	TCCR2B = 0x00;
	Low(TIMSK2, TOIE2);
}

ISR(TIMER2_OVF_vect)
{
	Timer2_OverflowCount++;
	
	if (Timer2_OverflowCount % 25 == 0) HandleAfter200ms = true;
	
	if (Timer2_OverflowCount >= 125)
	{
		HandleAfterSecond = true;
		Timer2_OverflowCount = 0;
	}

	TCNT2 = 131;
}

void ExternalInterrupt(bool enable)
{
	if (enable)
	{
		High(EICRA, ISC01);
		High(EIMSK, INT0);
		return;
	}
	
	Low(EICRA, ISC01);
	Low(EIMSK, INT0);
}

ISR(INT0_vect)
{
	WaveDurationCount = WaveDuration;
	Timer0(true);
	TriacOn;
}

void USART(unsigned short option)
{
	switch (option)
	{
		case On:
		UCSR0B |= (1 << TXEN0);
		break;
		case Off:
		UCSR0B |= (0 << TXEN0);
		break;
		default:
		UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
		UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
		UBRR0  =  3;
		break;
	}
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

void Transmit(unsigned short *value)
{
	static char d[4] = { 0 };
		
	sprintf(d, "D%d$", *value);
	TxString(d);	
}

void EraseUnits(int x, int y, int offset, unsigned short count)
{
	char eraser = 32;

	if (count<100000)
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
	
	
	lcd_gotoxy(x, y);
}

void DisplayPrint(unsigned short *distance, float *frequency)
{
	static char d[8] = { 0 }, f[8] = { 0 };
	
	EraseUnits(0, 0, 3, *distance);
	sprintf(d, "%d mm", *distance);
	lcd_gotoxy(0, 0);
	lcd_puts(d);
	
	EraseUnits(0, 1, 3, *frequency);
	sprintf(f, "%.1f Hz", *frequency);
	lcd_gotoxy(0, 1);
	lcd_puts(f);
}

void Initialization()
{
	DDRB = 0b00111110;
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000010;
	PORTD = 0b11111101;
	
	lcd_init(LCD_DISP_ON);
	lcd_led(false);
	lcd_clrscr();
	lcd_home();
	
	//Timer0(true);
	Timer1(Init);
	Timer2(true);
	//ExternalInterrupt(true);
	USART(Init);
	USART(On);
	sei();
}

unsigned short Average(unsigned short value)
{
	static unsigned short index = 0;
	static float array[ArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % ArraySize;
	
	return result/ArraySize;
}

int main(void)
{	
	unsigned short distance = 0;
	unsigned long int ticks = 0;
	float frequency = 0;
	
	Initialization();
	
    while (1) 
    {
		if (PulseCaptured)
		{
			ticks = (Timer1_OverflowCount*65536L+LastTicks) - StartTicks;
			distance = Average((unsigned short)(ticks * 0.01046875));
			StartTicks = 0;
			LastTicks = 0;
			Timer1_OverflowCount = 0;
			TCNT1 = 0;
			PulseCaptured = false;
		}
		
		if (!WaveDurationCount && Triac)
		{
			TriacOff;
			Timer0(false);
		}
		
		if (HandleAfter200ms)
		{
			Timer1(On);
			
			UltrasonicOn;
			_delay_us(11);
			UltrasonicOff;
			
			HandleAfter200ms = false;
		}
		
		if (HandleAfterSecond)
		{
			LedInv;
			
			DisplayPrint(&distance, &frequency);
			Transmit(&distance);
			
			HandleAfterSecond = false;
		}
    }
}