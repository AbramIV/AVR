/*
 * A328Ps_Debugger.c
 *
 * Created: 06.12.2022 21:14:24
 * Author : prote
 */ 

#define Check(REG,BIT) (REG & (1<<BIT))	    // check bit
#define Inv(REG,BIT)   (REG ^= (1<<BIT))	// invert bit
#define High(REG,BIT)  (REG |= (1<<BIT))	// set bit
#define Low(REG,BIT)   (REG &= ~(1<<BIT))	// clear bit

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

struct Time
{
	unsigned short ms, ms200, handle;
} MainTimer = { 0, 0 };

void Timer2()
{
	TCCR2B |= (1 << CS22)|(0 << CS21)|(1 << CS20);
	TIMSK2 |= (1 << TOIE2);
}

ISR(TIMER2_OVF_vect)
{
	MainTimer.ms++;

	if (MainTimer.ms % 50 == 0) MainTimer.ms200++;  

	if (MainTimer.ms >= 1000)
	{
		MainTimer.handle++;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

void SetDigit(short overfeed)
{
	static unsigned short dozens = 0, units = 0;
	
	dozens = overfeed / 10;
	units = overfeed % 10;
	
	if (Check(PORTC, PORTC5)) PORTC = 0xD0 | units;
	else PORTC = 0xE0 | dozens;
}

int main(void)
{
	static short overfeed = -34;
	
	DDRD = 0x00;
	PORTD = 0xFF;
	
	DDRB = 0xFF;
	PORTB = 0x00;
	
	DDRC = 0b00111111;
	PORTC = 0b01100000;
	
	Timer2();
	
	sei();
	
    while (1) 
    {	
		if (MainTimer.ms200)
		{
			SetDigit(abs(overfeed));
			
			MainTimer.ms200--;
		}
		
		if (MainTimer.handle)
		{
			PORTB ^= (1<<PORTB5);
			MainTimer.handle = 0;	
		}
    }			  
}