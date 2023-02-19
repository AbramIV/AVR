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

#define WaveDuration	62

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

volatile struct Measures
{
	unsigned long int ticks, current, last;
	unsigned short distance;
	float period;
	bool done;
} Ruler = { 0, 0, 0, 0, 0, false };

unsigned short WaveDurationCount = 0;
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

void Transmit(float value)
{
	static char d[16] = { 0 };
		
	sprintf(d, "D%.3f\r\n", value);
	TxString(d);	
}

void Initialization()
{
	DDRB = 0b00111110;
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000010;
	PORTD = 0b11111101;
	
	Timer0(true);
	Timer2(true);
	ExternalInterrupt(true);
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
				Ruler.period = (float)Ruler.current*0.0000000625;
				Ruler.distance = (unsigned short)Ruler.period*335;
				Ruler.done = false;
			}
			
			HandleAfter8ms = false;
		}
		
		if (!WaveDurationCount && Triac)
		{
			TriacOff;
			Timer0(false);
		}
		
		if (HandleAfter200ms)
		{
			UltrasonicOn;
			_delay_us(12);
			UltrasonicOff;
			Timer1(true);
			
			HandleAfter200ms = false;
		}
		
		if (HandleAfterSecond)
		{
			LedInv;
			
			Transmit(Ruler.distance);
			
			HandleAfterSecond = false;
		}
    }
}