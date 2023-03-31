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

#define EncoderB    (PIND  & (1<<3))
#define EncoderL    (~PIND & (1<<4))
#define EncoderR    (~PIND & (1<<5))

#define Off		0
#define On		1
#define Init	2

#define DistanceArraySize  16
#define FrequencyArraySize 8

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

const float MAX = 2.4;

unsigned long int Ticks = 0;
bool PulseCaptured = false;

unsigned short WaveDurationCount = 0;
unsigned short WaveDuration = 85;

unsigned short Timer2_OverflowCount = 0;
bool HandleAfterSecond = false;
bool HandleAfter200ms = false;

void Timer0(bool enable)
{
	if (enable)
	{
		High(TIMSK0, TOIE0);
		High(TCCR0B, CS02);
		return;
	}
	
	Low(TCCR0B, CS02);
	Low(TIMSK0, TOIE0);
	TCNT0 = 251;
}

ISR(TIMER0_OVF_vect)
{
	if (WaveDurationCount > 0) WaveDurationCount--;
	else
	{
		TriacOn;
		Timer0(false);
	}
	TCNT0 = 251;
}

void Timer1(unsigned short option)
{
	switch(option)
	{
		case Init:
			High(TCCR1B, ICNC1);
			TIMSK1 = (1 << TOIE1)|(1 << ICIE1);
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
			TCNT1 = 0;
			break;
	}
}

ISR(TIMER1_OVF_vect)
{
	Ticks += 65536;
}

ISR(TIMER1_CAPT_vect)
{	
	if (EchoPin) 
	{
		Ticks = 0;
		Low(TCCR1B, ICES1);
		return;
	}

	Ticks += ICR1;
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
		High(EICRA, ISC00);
		High(EICRA, ISC01);
		High(EIMSK, INT0);
		return;
	}
	
	Low(EICRA, ISC00);
	Low(EICRA, ISC01);
	Low(EIMSK, INT0);
}

ISR(INT0_vect)
{
	TriacOff;
	WaveDurationCount = WaveDuration;
	Timer0(true);
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

void Transmit()
{
	static char d[2] = { 0 };
	
	sprintf(d, "%d", WaveDuration);
	TxString(d);
}

void EncoderControl(void)
{
	static unsigned short right = 0, left = 0, button = 0;
	
	if (EncoderR) right = 0;
	{
		if (!EncoderR) right++;
		{
			if (right == 1 && EncoderL)
			{
				if (WaveDuration < 124) WaveDuration++;
				return;
			}
		}
	}
	
	if (EncoderL) left = 0;
	{
		if (!EncoderL) left++;
		{
			if (left == 1 && EncoderR)
			{
				if (WaveDuration > 1) WaveDuration--;
				return;
			}
		}
	}
	
	if (!EncoderB) button++;
	{
		if (button == 1)
		{
			
			button = 0;
		}
	}
}

void Initialization()
{
	DDRB = 0b00111110;
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000010;
	PORTD = 0b11111101;
	
	//Timer1(Init);
	Timer2(true);
	ExternalInterrupt(true);
	//USART(Init);
	//USART(On);
	sei();
}

unsigned short AverageDistance(unsigned short value)
{
	static unsigned short index = 0;
	static float array[DistanceArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % DistanceArraySize;
	
	return result/DistanceArraySize;
}

float AverageFrequency(unsigned short *value)
{
	static unsigned short index = 0;
	static float array[FrequencyArraySize] = { 0 };
	static float result = 0;
	
	result += *value - array[index];
	array[index] = *value;
	index = (index + 1) % FrequencyArraySize;
	
	return result/FrequencyArraySize;
}

void GetDuration()
{
	S = (-374/2?50) * (cos(2?50*(0.005)) - 1)
}

int main(void)
{	
	unsigned short distance = 0;
	
	Initialization();

    while (1) 
    {
		if (PulseCaptured)
		{
			distance = AverageDistance((unsigned short)(Ticks * 0.01046875));
			PulseCaptured = false;
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
			
			//Transmit();
			
			HandleAfterSecond = false;
		}
		
		EncoderControl();
    }
}