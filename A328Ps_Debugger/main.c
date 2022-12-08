/*
 * A328Ps_Debugger.c
 *
 * Created: 06.12.2022 21:14:24
 * Author : prote
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

struct Time
{
	unsigned short ms, handle;
} MainTimer = { 0, 0 };

ISR(TIMER2_OVF_vect)
{
	MainTimer.ms++;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.handle++;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

int main(void)
{
	wdt_enable(WDTO_8S);
	TCCR2B |= (1 << CS22)|(0 << CS21)|(1 << CS20);
	TIMSK2 |= (1 << TOIE2);
	sei();
	
    while (1) 
    {
		if (MainTimer.handle)
		{
			PORTB ^= (1<<PORTB5);
			MainTimer.handle = 0;	
		}
		
		wdt_reset();
    }			  
}