/*
 * A328Ps_Debugger.c
 *
 * Created: 06.12.2022 21:14:24
 * Author : prote
 */ 

#define Check(REG,BIT) (REG &  _BV(BIT))
#define Inv(REG,BIT)   (REG ^= _BV(BIT))
#define High(REG,BIT)  (REG |= _BV(BIT))
#define Low(REG,BIT)   (REG &= ~_BV(BIT))

#define Led     Check(PORTB, PORTB5)
#define LedOn   High(PORTB, PORTB5)
#define LedOff  Low(PORTB, PORTB5)
#define LedInv  Inv(PORTB, PORTB5)

#define Right    (~PIND & (1<<2))
#define Left     (~PIND & (1<<3))
#define Enter    (PIND  & (1<<4))

#define SPI_Start	Low(PORTB, PORTB5)
#define SPI_Stop	High(PORTB, PORTB5) 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

struct Time
{
	unsigned int ms, ms5, ms200, handle;
} MainTimer = { 0, 0, 0 };
	
struct
{
	unsigned int up;
	unsigned int down;
	unsigned int button;
} Encoder = { 0, 0, 0 };

unsigned int Addendum = 1000;

void Timer1()
{	
	TCCR1A |= (1 << COM1A0);
	TCCR1B |= (1 << WGM12) | (1 << CS10);
};

void Timer2()
{
	TCCR2B |= (1 << CS22)|(0 << CS21)|(1 << CS20);
	TIMSK2 |= (1 << TOIE2);
}

ISR(TIMER2_OVF_vect)
{
	MainTimer.ms++;

	//if (IsAcceleration) OCR1A++; else OCR1A--;
	//if (OCR1A <= 1) IsAcceleration = true;
	//if (OCR1A >= 65535) IsAcceleration = false;

	if (MainTimer.ms % 5 == 0) MainTimer.ms5++;
	if (MainTimer.ms % 200 == 0) MainTimer.ms200++;  

	if (MainTimer.ms >= 1000)
	{
		MainTimer.handle++;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

void USART(void)
{
	Low(UCSR0A, U2X0);
	UCSR0B = (1 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0 = 0;
}

void TxChar(unsigned char c)
{
	while (!Check(UCSR0A, UDRE0));
	UDR0 = c;
}

void TxString(const char *s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

void Transmit()
{
	static char data[16] = { 0 }, buffer[64] = { 0 };
	
	buffer[0] = '!';
	sprintf(data, "$FnA%d$", Addendum);
	strcat(buffer, data);
	sprintf(data, "RgOCR1A%d$", OCR1A); 
	strcat(buffer, data);
	strcat(buffer, "#");
	
	TxString(buffer);
	
	memset(buffer, 0, sizeof(buffer));
}

void ControlEncoder(void)
{
	if (Right) Encoder.up = 0;
	{
		if (!Right) Encoder.up++;
		{
			if (Encoder.up == 1 && Left)
			{
				OCR1A -= Addendum;
				return;
			}
		}
	}
	
	if (Left) Encoder.down = 0;
	{
		if (!Left) Encoder.down++;
		{
			if (Encoder.down == 1 && Right)
			{
				OCR1A += Addendum;
				return;
			}
		}
	}
	
	if (Enter) Encoder.button = 0;
	{
		if (!Enter) Encoder.button++;
		{
			if (Encoder.button == 1)
			{
				switch(Addendum)
				{
					case 1:
						Addendum = 2;
						break;
					case 2:
						Addendum = 3;
						break;
					case 3:
						Addendum = 4;
						break;
					case 4:
						Addendum = 5;
						break;
					case 5:
						Addendum = 10;
						break;
					case 10:
						Addendum = 100;
						break;
					case 100:
						Addendum = 1000;
						break;
					default:
						Addendum = 1;
						break;
				}
			}
		}
	}
}

void SPI_Write(unsigned int word)
{
	unsigned char MSB = ((word >> 8) & 0x0f) | 0x70;  	//filter out MS
	unsigned char LSB = word & 0xff;			//filter out LS

	SPI_Start;

	SPDR = MSB;							// 	send First 8 MS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy
	
	SPDR = LSB;							// 	send Last 8 LS of data
	while (!(SPSR & (1<<SPIF)));			//	while busy

	SPI_Stop;
}

int main(void)
{
	DDRB = 0b11111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b01100000;
	
	DDRD = 0b00000000;
	PORTD = 0b00011111;
	
	Timer1();
	Timer2();
	USART();
	
	sei();
	
	//SPI_Write();
									
    while (1) 
    {	
		if (MainTimer.ms5) 
		{
			ControlEncoder();
			
			MainTimer.ms5 = 0;
		}
		
		if (MainTimer.ms200)
		{
			Transmit();
			
			MainTimer.ms200--;
		}
		
		if (MainTimer.handle)
		{
			//LedInv;

			MainTimer.handle = 0;	
		}
    }			  
}