/*
 * main.c
 *
 * Created: 3/14/2022 3:07:47 PM
 *  Author: igor.abramov
 */ 

/*
  Frequency comparer with counter inputs
  IR Receiver with continue receive will be down after 530 ms
*/

#define F_CPU	16000000L

#define Check(REG,BIT) (REG &  (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= (0<<BIT))

#define Led			Check(PORTB, 5)
#define LedOn		High(PORTB, 5)
#define LedOff		Low(PORTB, 5)
#define LedInv		Inv(PORTB, 5)

#define Pulse		Check(PORTD, 7)
#define PulseOn		High(PORTD, 7)
#define PulseOff	Low(PORTD, 7)
#define PulseInv	Inv(PORTD, 7)

#define RightOn		(!Check(PIND, 2)) 
#define LeftOn		(!Check(PIND, 3))
#define Active		(!Check(PIND, 6))

#define Init	 0
#define On		 1
#define Off		 2
#define TxOn	 3
#define TxOff	 4
#define RxOn	 5
#define RxOff	 6

#define Right	 		10
#define Left 			20
#define Stop			30
#define Short			40

#define Acceleration	11
#define Waiting			22
#define Process			33

#define Before			30
#define Between			31
#define Inside			32
#define After			33

#define Interval		5
#define AccelDelay		40
#define TempBufferSize  20
#define TxBufferSize	100
#define RxBufferSize    250
#define RangeUp			0.07
#define RangeDown	   -0.07
#define AArraySize		40
#define PArraySize		40
#define TArraySize		10
#define HArraySize		10

#define NextLine		0x0A
#define FillCell		0xFF
#define Terminator		'$'
#define Arrow			'>'
#define Eraser			' '
#define BufferEnd		'#'
#define StringEnd		'\0'
#define CR				'\r'
#define LF				'\n'

#include <xc.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <util/delay.h>
#include "lcd/lcd.h"
#include "dht/dht.h"

volatile struct
{
	volatile unsigned int interval, ms16, ms992;
} MainTimer;

volatile struct
{
	unsigned int ovf;
	float Fa, Fp, temperature, humidity;
} Measure;

volatile struct
{
	unsigned short mode;
	unsigned short count;
	short direction;		
} Mode;

volatile struct
{
	unsigned char byte;
	bool byteReceived;
} Rx;

volatile struct
{
	bool tryToConnect, connected;
	bool receiving, handling;
	bool building, sending;
	bool reset;
	unsigned short delay, timeout;
	
} Server;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
		TIMSK0 = (1 << TOIE0);
		TCNT0 = 0;
		return;
	}
	
	TCCR0B = (0 << CS02)|(0 << CS01)|(0 << CS00);
	TIMSK0 = (0 << TOIE0);
	TCNT0 = 0;
}

ISR(TIMER0_OVF_vect)
{
	Measure.ovf++;	
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);
		return;
	}
	
	TCCR1B = (0 << CS12)|(0 << CS11)|(0 << CS10);
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);
		TIMSK2 = (1 << TOIE2);
		TCNT2 = 0;
		return;
	}
	
	TCCR2B = (0 << CS22)|(0 << CS21)|(0 << CS20);
	TIMSK2 = (0 << TOIE2);
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{
	MainTimer.ms16++;
	if (MainTimer.interval > 0) MainTimer.interval--;
	
	if (MainTimer.ms16 >= 62)
	{
		MainTimer.ms992++;
		MainTimer.ms16 = 0;
	}
	
	TCNT2 = 5;
}

void USART(unsigned short option)
{
	switch (option)
	{
		case TxOn:
			UCSR0B |= (1 << TXEN0);
			break;
		case TxOff:
			UCSR0B &= (0 << TXEN0);
			break;
		case RxOn:
			UCSR0B |= (1 << RXEN0) | (1 << RXCIE0);
			break;
		case RxOff:
			UCSR0B &= (0 << RXEN0) | (0 << RXCIE0);
			break;
		case On:
			UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
			break;
		case Off:
			UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			break;
		default:
			UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << RXCIE0);
			UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
			UBRR0  =  3;
			break;
	}
}

ISR(USART_RX_vect)
{
	Rx.byte = UDR0;
	Rx.byteReceived++;
}

void TxChar(unsigned char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

void Transmit()
{
	static char buffer[TxBufferSize] = { 0 }, aramid[20], polyamide[20], temperature[20], humidity[20], direction[10];
		
	sprintf(aramid, "A%.2f$", Measure.Fa);
	strcat(buffer, aramid);
	
	sprintf(polyamide, "P%.2f$", Measure.Fp);
	strcat(buffer, polyamide);
	
	sprintf(temperature, "T%.1f$", Measure.temperature);
	strcat(buffer, temperature);
	
	sprintf(humidity, "H%.1f$", Measure.humidity);
	strcat(buffer, humidity);
	
	sprintf(direction, "D%d$", Mode.direction);
	strcat(buffer, direction);
	
	TxString(buffer);
	memset(buffer, 0, TxBufferSize);
}

float MovAvgAramid(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[AArraySize];
	static float result;
	
	if (reset)
	{
		result = 0;
		index = 0;
		memset(values, 0, AArraySize-1);
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % AArraySize;
	
	return result/AArraySize;
}

float MovAvgPolyamide(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[PArraySize];
	static float result;
	
	if (reset)
	{
		result = 0;
		index = 0;
		memset(values, 0, PArraySize-1);
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % PArraySize;
	
	return result/PArraySize;
}

float MovAvgTemp(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[TArraySize];
	static float result;
	
	if (reset)
	{
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % TArraySize;
	
	return result/TArraySize;
}

float MovAvgHum(float value, bool reset)
{
	static unsigned short index = 0;
	static float values[HArraySize];
	static float result;
	
	if (reset)
	{
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % HArraySize;
	
	return result/HArraySize;
}

void GetOneWireData()
{
	static float temperature, humidity;
	
	if (dht_gettemperaturehumidity(&temperature, &humidity) != -1)
	{
		Measure.temperature = MovAvgTemp(temperature, false);
		Measure.humidity = MovAvgHum(humidity, false);
		return;
	}
	
	Measure.temperature = MovAvgTemp(-1, false);
	Measure.humidity = MovAvgHum(-1, false);
}

void EraseUnits(int x, int y, int offset, float count)
{
	char eraser = 32;

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

void DisplayPrint()
{
	static char speedAramid[20], speedPolyamide[20];
	 
	EraseUnits(0, 0, 3, Measure.Fa);
	sprintf(speedAramid, "%.2f", Measure.Fa < 0 ? 0 : Measure.Fa);
	lcd_puts(speedAramid);
	
	EraseUnits(0, 1, 3, Measure.Fp);
	sprintf(speedPolyamide, "%.2f", Measure.Fp < 0 ? 0 : Measure.Fp);
	lcd_puts(speedPolyamide);
}

void Initialization()
 {
	 DDRB = 0b00000111;
	 PORTB = 0b00000111;
	 
	 DDRC = 0b00111100;
	 PORTC = 0b00000000;
	 
	 DDRD = 0b10000000;
	 PORTD = 0b01111111;
	 
	 lcd_init(LCD_DISP_ON);
	 lcd_clrscr();
	 lcd_home();
		
	 lcd_clrline(9, 0);
	 lcd_puts("OK");
	 DisplayPrint();
	 
	 PulseOff;
	 Mode.mode = Waiting;
	 Mode.count = 0;
	 
	 Measure.Fa = MovAvgAramid(0, true);
	 Measure.Fp = MovAvgPolyamide(0, true);
	 Measure.temperature = MovAvgTemp(0, true);
	 Measure.humidity = MovAvgHum(0, true);
	 
	 Server.tryToConnect = true;
	 Server.delay = -1;
	 
	 USART(Init);
	 Timer0(true);
	 Timer1(true);
	 Timer2(true);
	 sei();
 }

void Calculation()
{
	// F = (k / q) * L * t = X m/s
	// k = 1000 ms / 160 ms = 6.25 (measure during 160 ms) 
	// q = 50 imp/rev for both impellers
	// La aramid roll D = 0.027 m, L = 0.0848 m (measured)
	// Lp polyamide roll D = 0.0512 m, L = 0.161 m (calculated)	
	// t = 60 seconds
	// in the same F and original sizes asm = -20
	// Lp experimantal v1 = 0.1570 // 1.1775 // asm = +4
	// Lp experimental v4 = 0.1572 // 1.1790 // asm = -3
	// Lp experimental v3 = 0.1575 // 1,1812 // asm = -2
	// Lp experimantal v2 = 0.1580 // 1.1850 // asm = -6 0.19113
			
	// 160 ms														   
	//Measure.Fa = MovAvgAramid(((255.f*Measure.ovf)+TCNT0)*0.636, false); // (6.25/50.f * 0.0848 * 60 = 0.636 
	//Measure.Fp = MovAvgPolyamide(TCNT1*1.1790, false); // 50 imp/rev // (6.25/50.f * 0.161 * 60 = 1.2075 
	
	// 992 ms
	Measure.Fa = MovAvgAramid(((255.f*Measure.ovf)+TCNT0)*0.10258, false); // (1000/992/50.f * 0.0848 * 60 = 0.10258
	Measure.Fp = MovAvgPolyamide(TCNT1*0.19052, false); // 50 imp/rev // (1000/992/50.f * 0.1575 * 60 = 0.19052

	TCNT0 = 0;
	TCNT1 = 0;
	Measure.ovf = 0;
}

void Step(unsigned short direction)
{
	if (MainTimer.interval) return;
	
	switch (direction)
	{
		case Right:
			PulseOn;
			_delay_us(200);
			PulseOff;
			_delay_ms(70);
			break;
		case Left:
			PulseOn;
			_delay_ms(70);
			PulseOff;
			break;
		default:
			PulseOff;
			break;	
	}
	
	MainTimer.interval = Interval;	 
}

void Manual()
{
	static unsigned short key = Stop;
	
	if (!(RightOn | LeftOn)) 
	{
		if (key == Stop) return;
		
		lcd_clrline(9, 0);
		lcd_puts("OK");
		key = Stop;
		return;
	}
	
	if (RightOn & LeftOn) 
	{
		if (key == Short) return;
		
		lcd_clrline(9, 0);
		lcd_puts("Short");
		key = Short;
		return;
	}
	
	if (RightOn) 
	{ 
		if (key == Right) 
		{
			Step(Right);
			return;
		}
		
		Step(Right);
		lcd_clrline(9, 0);
		lcd_puts("Right");
		key = Right;
		return; 
	}
	
	if (LeftOn) 
	{
		if (key == Left)
		{
			Step(Left);
			return;
		}
		
		Step(Left);
		lcd_clrline(9, 0);
		lcd_puts("Left");
		key = Left;
		return;
	}
}

void ModeControl()
{
	if (Active)
	{
		if (Mode.mode == Waiting)
		{
			Mode.count = AccelDelay;
			Mode.mode = Acceleration;
			USART(TxOn);
			return;
		}
		
		if (Mode.mode == Acceleration && !Mode.count) 
		{	
			LedOn;							 
			Mode.mode = Process;		
		}
		
		return;
	}
	
	if (Mode.mode == Waiting) return;
	
	LedOff;
	PulseOff;
	Mode.mode = Waiting;
	Measure.Fa = MovAvgAramid(0, true);
	Measure.Fp = MovAvgPolyamide(0, true);
	Measure.temperature = MovAvgTemp(0, true);
	Measure.humidity = MovAvgHum(0, true);
	Measure.ovf = 0;
	DisplayPrint();
	USART(Off);		
}

void Regulator()
{
	static unsigned short key = Stop;
	static float difference = 0;
	
	if (RightOn || LeftOn) 
	{
		if (key == Left || key == Right)
		{
			Mode.direction = 0;
			key = Stop;
		}
		return; 
	}
	
	if (Mode.mode == Waiting || Mode.mode == Acceleration) 
	{
		if (key == Stop) return;
		lcd_clrline(9, 0);
		lcd_puts("OK");
		Mode.direction = 0;
		key = Stop;
		return;
	}
	
	difference = Measure.Fa - Measure.Fp;
	
	if (difference > RangeDown && difference < RangeUp) 
	{
		if (key == Stop) return;
		lcd_clrline(9, 0);
		lcd_puts("OK");
		Mode.direction = 0;
		key = Stop;
		return;
	}
	
	if (difference >= RangeUp) 
	{
		if (key == Left)
		{
			Step(Left);
			return;
		}
		
		Step(Left);
		lcd_clrline(9, 0);
		lcd_puts("Left");
		Mode.direction = 1;
		key = Left;
	}
	else 
	{
		if (key == Right)
		{
			Step(Right);
			return;
		}
		
		Step(Right);
		lcd_clrline(9, 0);
		lcd_puts("Right");
		Mode.direction = -1;
		key = Right;
	}
}

unsigned short GetDataSize(char *buffer)
{
	int count = 0;
	
	for (int i=0;i<100; i++)
	{
		if (buffer[i] == BufferEnd) break;
		count++;
	}
	
	return count;
}

void ConnectToServer(bool connect)
{
	Server.connected = false;
	Server.tryToConnect = true;
	if (connect) TxString("AT+CIPSTART=\"TCP\",\"192.168.43.108\",11000\r\n");
	else TxString("AT+CIPCLOSE\r\n");
	Server.receiving = true;
	Server.delay++;
}

void TransmitToServer()
{
	static char buffer[TxBufferSize] = { 0 };
	static char temp[TempBufferSize] = { 0 };
	
	if (Server.building)
	{
		char Command[TempBufferSize] = "AT+CIPSEND=";
		
		sprintf(temp, "A%.2f$", Measure.Fa);
		strcat(buffer, temp);
		
		sprintf(temp, "P%.2f$", Measure.Fp);
		strcat(buffer, temp);
		
		sprintf(temp, "T%.1f$", Measure.temperature);
		strcat(buffer, temp);
		
		sprintf(temp, "H%.1f$", Measure.humidity);
		strcat(buffer, temp);
		
		sprintf(temp, "D%d$", Mode.direction);
		strcat(buffer, temp);
		
		sprintf(temp, "%d\r\n", GetDataSize(buffer));
		strcat(Command, temp);
		
		TxString(Command);
		
		Server.building = false;
		return;
	}
	
	if (Server.sending)
	{
		TxString(buffer);
		Server.sending = false;
		memset(buffer, 0, TxBufferSize);
	}
}

void Receive()
{
	static unsigned int charIndex = 0, wordIndex = 0, position = Before;
	static char RxBuffer[RxBufferSize] = { 0 };
	static char Response[10][80] = { 0 };
	
	if (Server.receiving)
	{
		RxBuffer[charIndex++] = Rx.byte;
		return;
	}
	
	if (Server.connected && !Server.receiving && !Server.handling)
	{
		RxBuffer[charIndex++] = Rx.byte;
		Server.receiving = true;
		Server.delay++;
		return;
	}
	
	if (charIndex < 1) return;
	
	RxBuffer[charIndex] = StringEnd;
	wordIndex = 0;
	charIndex = 0;
	position = Before;
	
	for(int i=0; i<sizeof(RxBuffer); i++)
	{
		if (position == Before)
		{
			if (RxBuffer[i] == StringEnd) break;
			if (RxBuffer[i] == CR || RxBuffer[i] == LF) continue;
			position = Inside;
			i--;
		}
		else if (position == Inside)
		{
			if (RxBuffer[i] == CR || RxBuffer[i] == LF)
			{
				Response[wordIndex][charIndex] = StringEnd;
				position = Between;
				charIndex = 0;
				continue;
			}
			
			Response[wordIndex][charIndex++] = RxBuffer[i];
			if (RxBuffer[i] == StringEnd) position = After;
			
		}
		else if (position == Between)
		{
			if (RxBuffer[i] == StringEnd) position = After;
			if (RxBuffer[i] == CR || RxBuffer[i] == LF) continue;
			position = Inside;
			wordIndex++;
			i--;
		}
		else
		{
			
		}
	}
	
	Server.timeout = 0;
	charIndex = 0;
	
	if (!Server.connected && !strcasecmp(Response[1], "CONNECT") && !strcasecmp(Response[2], "OK"))
	{
		Server.connected = true;
		Server.tryToConnect = false;
		return;
	}
	
	if (!strcasecmp(Response[1], "CLOSED") && !strcasecmp(Response[2], "OK"))
	{
		Server.connected = false;
		return;
	}
	
	if (!strcasecmp(Response[0], "CLOSED"))
	{
		Server.connected = false;
		Server.tryToConnect = true;
		return;
	}
	
	for (int i=0; i<=wordIndex; i++)
	{
		if (!strcasecmp(Response[i], "ERROR"))
		{

			return;
		}
	}
	
	if (!strcasecmp(Response[0], "+IPD,3:Get"))
	{
		Server.building = true;
		return;
	}
	
	if (Response[2][0] == Arrow)
	{
		Server.sending = true;
		return;
	}
}

int main(void)
{
	Initialization();
	
	while(1)
	{
		Manual();
		ModeControl();
		Regulator();
		
		if (MainTimer.ms992)
		{
			if (Mode.mode == Acceleration || Mode.mode == Process)
			{
				Calculation();
				GetOneWireData();
				DisplayPrint();
				Transmit();
			}
			
			if (Mode.count) Mode.count--;
			if (Mode.mode == Acceleration) LedInv;
			MainTimer.ms992 = 0;
		}
	}
}