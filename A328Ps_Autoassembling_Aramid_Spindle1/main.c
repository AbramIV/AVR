/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 * k = Tcount*0.028*Pi*60/100
 */ 

#define F_CPU	16000000L
				
#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Imp			Check(PORTB, PORTB0)	// control pulses of motor
#define ImpOn		High(PORTB, PORTB0)
#define ImpOff		Low(PORTB, PORTB0)
#define ImpInv		Inv(PORTB, PORTB0)

#define Fault		Check(PORTB, PORTB1) // output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB1)
#define FaultOff	Low(PORTB, PORTB1)
#define FaultInv	Inv(PORTB, PORTB1)

#define Led			Check(PORTB, PORTB5) // operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define Running		Check(PIND, PIND3)  // spindle run input
#define Aramid		Check(PIND, PIND4)  // aramid speed pulses input
#define Polyamide   Check(PIND, PIND5)  // polyamide speed pulses input

#define Off				0
#define On				1
#define Init			2

#define Right	 		10
#define Left 			20
#define Locked			30
	
#define ArraySize		  32		// these parameters also should be positioned in ROM
#define StartDelay		  5			// delay to start measuring after spindle start
#define FaultDelay		  1200  	// if Mode.operation != Stop > FaultDelay then spindle stop
#define RangeUp			  0.006		// if ratio > range up then motor left
#define RangeDown		  -0.006
#define LeftStepDuration  2			// sp1
#define RightStepDuration 2			// sp1
#define PauseBetweenSteps 20		// sp1
#define Overfeed		  0			// factor to keep wrong assembling (for example if we need asm - 10)

#define Eraser ' '

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>			
#include <stdbool.h>
#include <string.h>

struct TimeControl
{
	unsigned int ms;
	bool s;
} MainTimer;

struct Data
{
	unsigned short ovf;
	float Fa,Fp;
} Measure;

struct ModeControl
{
	unsigned int startDelay,faultDelay;
	bool fault,run;
} Mode;

struct MotorControl
{
	unsigned int isDelay,isStep,operation;
} Motor; 

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);
		High(TIMSK0, TOIE0);
		return;
	}
	
	TCCR0B = 0x00;
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
	
	TCCR1B = 0x00;
}

void Timer2(bool enable)
{
	if (enable)
	{
		TCCR2B = (1 << CS22)|(0 << CS21)|(1 << CS20);
		TIMSK2 = (1 << TOIE2);
		TCNT2 = 0;
		return;
	}
	
	TCCR2B = 0x00;
	Low(TIMSK2,TOIE2);
	TCNT2 = 0;
}

ISR(TIMER2_OVF_vect)
{	
	MainTimer.ms++;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.s = true;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
}

ISR(ANALOG_COMP_vect)
{
	Measure.Fp++;
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
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

float AverageA(unsigned int aramidFrequency)
{
	static float values[ArraySize] = { 0 };
	static int index = 0;
	static float result = 0;
	
	if (++index >= ArraySize) index = 0;
		
	result -= values[index];					
	result += (float)aramidFrequency;		
	values[index] = (float)aramidFrequency;	
	
	return ((float)result / ArraySize);
}

float AverageP(unsigned int polyamideFrequency)
{
	static float values[ArraySize] = { 0 };
	static int index = 0;
	static float result = 0;
	
	if (++index >= ArraySize) index = 0;
		
	result -= values[index];					
	result += (float)polyamideFrequency;		
	values[index] = (float)polyamideFrequency;
		
	return ((float)result / ArraySize);
}

void Transmit()
{
	 static char fa[20], fp[20];
	 static char buffer[60];
	
	 sprintf(fa, "A%.1f", Measure.Fa);
	 sprintf(fp, "P%.1f", Measure.Fp);
	 strcat(buffer, fa);
	 strcat(buffer, fp);
	 TxString(buffer);
	 
	 memset(buffer, 0, 60);
}

void ResetFilters()
{
	for (int i = 0; i<ArraySize; i++)
	{
		AverageA(0);
		AverageP(0);
	}
}

void Calculation()
{	
	Measure.Fa = AverageA(TCNT0+(Measure.ovf*256));
	Measure.Fp = AverageP(TCNT1);		
}

void Initialization()
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000010;
	PORTD = 0b00000011;
	
	MainTimer.ms = 0;
	MainTimer.s = false;
	
	Measure.Fa = 0;
	Measure.Fp = 0;
	
	Mode.run = false;
	Mode.fault = false;
	Mode.faultDelay = FaultDelay;
	Mode.startDelay = 0;
	Motor.operation = Locked;
	
	Timer2(true);
	USART(Init);
	USART(On);
	sei();
}

void Step()
{
	ImpOn;
	LedOn;
	
	if (Motor.operation == Right) _delay_us(500);
	if (Motor.operation == Left) _delay_ms(5);

	LedOff;
	ImpOff;
	_delay_ms(5);
}

void Regulation()
{
	static float difference = 0, ratio = 0;
	
	ratio = 1 - ((Measure.Fa == 0 ? 1 : Measure.Fa) / (Measure.Fp == 0 ? 1 : Measure.Fp));
	difference = Overfeed - ratio;
	
	if (Motor.isStep || Motor.isDelay) return;
	
	if ((difference > RangeDown && difference < RangeUp))
	{
		Mode.faultDelay = FaultDelay;
		Motor.operation = Locked;
		return;
	}
	
	if (difference >= RangeUp) 
	{
		Motor.operation = Left;
		Motor.isStep = LeftStepDuration; 
	}
	else 
	{
		Motor.operation = Right;
		Motor.isStep = RightStepDuration;
	}
}
							   					
int main(void)
{
	Initialization();
	
    while(1)
    {	
		if (MainTimer.s)
		{	
			if (Mode.startDelay) Mode.startDelay--;
			
			if (Mode.run && !Mode.startDelay)
			{
				LedInv;
				Calculation();
				Transmit();
			
				if (Motor.isDelay > 0) Motor.isDelay--;
				
				if (Motor.isStep) 
				{
					Motor.isStep--;
					if (!Motor.isStep) Motor.isDelay = PauseBetweenSteps;
				}
				
				Regulation();

				if (Motor.operation != Locked && Mode.faultDelay && !Mode.fault) Mode.faultDelay--;
				
				if (!Mode.faultDelay && !Mode.fault)
				{
					FaultOn;
					Mode.fault = true;
				}
			}
			
			if (Mode.run)
			{
				TCNT0 = 0;
				TCNT1 = 0;
				Measure.Fp = 0;
				Measure.ovf = 0;
			}
			
			if (Running && !Mode.run)
			{
				FaultOff;
				Mode.run = true;
				Mode.startDelay = StartDelay;
				Mode.faultDelay = FaultDelay;
				Mode.fault = false;
				Timer0(true);
				Timer1(true);
			}
			
			if (!Running && Mode.run)
			{
				LedOff;
				ImpOff;
				Timer0(false);
				Timer1(false);
				ResetFilters();
				Measure.Fa = 0;
				Measure.Fp = 0;
				Mode.run = false;
				Mode.fault = false;
				Mode.faultDelay = FaultDelay;
				Mode.startDelay = 0;
				Motor.operation = Locked;
			}
			
			MainTimer.s = false;
		}
		
		if (Motor.isStep) Step();
    }
}