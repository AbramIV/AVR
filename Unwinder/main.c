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

#define Led			Check(PORTD, PORTD7)
#define LedOn		High(PORTD, PORTD7)
#define LedOff		Low(PORTD, PORTD7)
#define LedInv		Inv(PORTD, PORTD7)

#define EchoPin	    Check(PORTB, PORTB0)

#define Off		0
#define On		1
#define Init	2

#define UltrasonicOn  High(PORTB, PORTB0)
#define UltrasonicOff Low(PORTB, PORTB0)

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

struct Measures
{
	unsigned int ticks, current, last;
	float period;
	bool done;
} Ruler = { 0, 0, 0, 0, false };

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
		High(TIMSK0, TOIE0);
		TCNT0 = 0;
		return;
	}
	
	Low(TIMSK0, TOIE0);
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
		TCCR1B = (1 << ICNC1)|(1 << ICES1)|(0 << CS12)|(0 << CS11)|(1 << CS10);
		TIMSK1 = (1 << TOIE1)|(1 << ICIE1);
		return;
	}
	
	Low(TCCR1B, CS10);
	Low(TIMSK1, TOIE1);
	Low(TIMSK1, ICIE1);
}

ISR(TIMER1_OVF_vect)
{
	Timer1_OverflowCount++;
}

ISR(TIMER1_CAPT_vect)
{	
	if (EchoPin) 
	{
		TCNT1 = 0;
		Timer1_OverflowCount = 0;
		Low(TCCR1B, ICES1);
		return;
	}
	
	Timer1(false);
	Ruler.ticks = ICR1;
	Ruler.done = true;
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

void Transmit(float* value)
{
	static char d[8] = { 0 };
		
	sprintf(d, "D%.3f&", *value);
	TxString(d);	
}

void Initialization()
{
	DDRB = 0b00000110;
	PORTB = 0b00111001;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00001100;
	PORTD = 0b11110011;
	
	Timer2(true);
	USART(Init);
	USART(On);
	sei();
}

int main(void)
{
	Initialization();
	
    while (1) 
    {
		if (HandleAfter8ms)
		{
			if (Ruler.done)
			{
				Ruler.current = ((Timer1_OverflowCount * 65536L) + Ruler.ticks) - Ruler.last;
				Ruler.last = Ruler.ticks;
				Ruler.period = Ruler.current*0.0000000625;
				Ruler.done = false;
			}
			
			HandleAfter8ms = false;
		}
		
		if (HandleAfter200ms)
		{
			Transmit(&Ruler.period);
			
			UltrasonicOn;
			_delay_us(12);
			UltrasonicOff;
			Timer1(true);
			
			HandleAfter200ms = false;
		}
		
		if (HandleAfterSecond)
		{
			LedInv;
			
			TxString("OK");
			
			HandleAfterSecond = false;
		}
    }
}