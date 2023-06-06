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

#define Button		Check(PIND, PIND6)     	  // buttons control input pins    

#define ChannelA	Check(TCCR0A, COM0A1)  	  // on/off pwm on PulsePin
#define ChannelAOn	High(TCCR0A, COM0A1)
#define ChannelAOff	Low(TCCR0A, COM0A1)

#define ChannelB	Check(TCCR0A, COM0B1)  	  // on/off pwm on PulsePin
#define ChannelBOn	High(TCCR0A, COM0B1)
#define ChannelBOff	Low(TCCR0A, COM0B1)

#define Off				  0				  	  // internal parameters enumeration
#define On				  1
#define Init			  2			

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

unsigned short Timer0_OverflowCount = 0;
unsigned short Timer1_OverflowCount = 0;
unsigned short Timer2_OverflowCount = 0;

bool HandleAfterSecond = false;
bool HandleAfter200ms = false;
bool HandleAfter8ms = false;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0A = (1 << WGM01)|(1 << WGM00);
		TCCR0B = (1 << CS02)|(0 << CS01)|(1 << CS00);							
		return;
	}
	
	TCCR0B = 0x00;										  
}

ISR(TIMER0_OVF_vect)
{
	Timer0_OverflowCount++;	  
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);   
		High(TIMSK1, TOIE1);
		TCNT1 = 0;
		return;
	}
	
	Low(TIMSK1, TOIE1);
	TCCR1B = 0x00;
}

ISR(TIMER1_OVF_vect)
{
	Timer1_OverflowCount++;
}

void Timer2(bool enable)
{
	TCNT2 = 0; 	   
	
	if (enable)
	{
		TCCR2A = (1 << WGM21)|(1 << WGM20);				
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
	HandleAfter8ms = true;
	
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
}

int main(void)
{
	unsigned short counter = 0;
	
	Initialization();

	ChannelAOn;

	while(1)
	{
		if (HandleAfter8ms)
		{
			
			HandleAfter8ms = false;
		}
		
		if (HandleAfter200ms)
		{	
			
			 HandleAfter200ms = false;
		}
		
		if (HandleAfterSecond)	 
		{
			LedInv;
			counter++;
			
			if (counter > 2)
			{
				if (OCR0A > 3) OCR0A = 3;
				else OCR0A = 14;
				
				counter = 0;
			}
			
			HandleAfterSecond = false;
		}
	}
}

