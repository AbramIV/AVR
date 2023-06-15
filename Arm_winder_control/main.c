/*
 * Arm_winder_control.c
 *
 * Created: 05.06.2023 22:36:52
 * Author : igor.abramov
 */ 

#define Check(REG,BIT) (REG & (1<<BIT))	   	  // check bit
#define Inv(REG,BIT)   (REG ^= (1<<BIT))	  // invert bit
#define High(REG,BIT)  (REG |= (1<<BIT))	  // set bit
#define Low(REG,BIT)   (REG &= ~(1<<BIT))	  // clear bit

#define Led			Check(PORTB, PORTB5)	  // operational led
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)

#define PulseA		Check(PORTD, PORTD6)
#define PulseAOn	High(PORTD, PORTD6)
#define PulseAOff	Low(PORTD, PORTD6)

#define Button		Check(PIND, PIND7)     	  // buttons control input pins    

#define PulsesOn  High(TCCR0B, CS02)
#define PulsesOff Low(TCCR0B, CS02)
	
/* Direct */
#define Close 100 // 1472 us 90
#define Open  50  // 256 us	 30

/* Inverted */
//#define Closed 165
//#define Opened 225 

#define TransitionInterval	5

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

unsigned short Direction = Close;

unsigned short Timer2_OverflowCount = 0;
bool HandleAfterSecond = false;
bool HandleAfter200ms = false;
bool HandleAfter40ms = false;

unsigned short LockCounter = TransitionInterval;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0A = (1 << COM0A1)|(1 << WGM01)|(1 << WGM00);
		TCCR0B = (1 << CS02)|(0 << CS01)|(0 << CS00); // divider 256, 1 tick = 16 us
		OCR0A = Close;
		return;
	}
	
	TCCR0B = 0x00;										  
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
	
	if (Timer2_OverflowCount % 5 == 0) HandleAfter40ms = true;
	
	if (Timer2_OverflowCount % 25 == 0) HandleAfter200ms = true;
	
	if (Timer2_OverflowCount >= 125)
	{
		HandleAfterSecond = true;
		Timer2_OverflowCount = 0;
	}

	TCNT2 = 131;
}

void Initialization()
{
	DDRB = 0b00100000;					
	PORTB = 0b11011111;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b01100000;
	PORTD = 0b10011111;

	Timer0(true);
	Timer2(true);	
	sei();
			
	LedOn;
}

void ButtonHandle()
{
	static unsigned short switched = 0;
	
	if (!Button) switched++;
	{
		if (switched == 1)
		{
			if (OCR0A == Close) Direction = Open;
			else Direction = Close;
			
			PulsesOn;
			LedOn;
			
			LockCounter = TransitionInterval;
			switched = 0;
		}
	}
}

int main(void)
{	
	Initialization();

	while(1)
	{
		if (!LockCounter) ButtonHandle();
		
		if (HandleAfter40ms)
		{
			if (Direction == Close && OCR0A < Close) OCR0A++;
			if (Direction == Open && OCR0A > Open) OCR0A--;
			
			HandleAfter40ms = false;
		}
		
		if (HandleAfterSecond)
		{
			if (LockCounter) LockCounter--;
			
			if (!LockCounter && OCR0A == Close)
			{
				PulsesOff;
				LedOff;
			}
			
			HandleAfterSecond = false;
		}
	}
}