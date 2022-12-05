/*
 * main.c
 *
 * Created: 6/4/2022 7:34:54 PM
 *  Author: igor.abramov
 * k = Tcount*0.028*Pi*60/100
 * PWM 14 % OCR2A = 144; 
 * PWM 18 % OCR2A = 152; 
 * CC4R1, CC4R3
 */ 
			
#define Check(REG,BIT) (REG & (1<<BIT))		
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Fault		Check(PORTB, PORTB1)	// output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB1)
#define FaultOff	Low(PORTB, PORTB1)
#define FaultInv	Inv(PORTB, PORTB1)

#define Led			Check(PORTB, PORTB5)	// operating led period = 1984 ms if not something wrong
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define Running		Check(PIND, PIND3)  // spindle run input
#define Aramid		Check(PIND, PIND4)  // aramid speed pulses input
#define Polyamide   Check(PIND, PIND5)  // polyamide speed pulses input

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

#define Off				  0			// hardware features modes
#define On				  1
#define Init			  2

#define Right	 		  10	    // move directions of motor
#define Left 			  20
#define Locked			  30
	
#define StartDelay		  10		// delay to start measuring after spindle start
#define FaultDelay		  1200  	// (seconds) if Mode.operation != Stop more than FaultDelay seconds then spindle stop
#define RangeUp			  4			// if ratio > range up then motor moves left
#define RangeDown		  -4			// if ratio < range down then motor moves right
#define StepDuration	  4			// work time of PWM to one step
#define StepsInterval	  25		// interval between	steps
#define Overfeed		  0

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>			
#include <stdbool.h>
#include <string.h>
#include <avr/wdt.h>

struct TimeControl
{
	unsigned short ms;
	bool handle;
} MainTimer = { 0, false };

struct ModeControl
{
	unsigned short startDelay, faultDelay;
	bool fault, run;
} Mode = { 0, FaultDelay, false, false };

struct MotorControl
{
	unsigned short isDelay, isStep, operation, stepsInterval; 
} Motor = { 0, 0, Locked, 0 };

struct Data
{
	short overflow;
	short f1, f2, ratio;
} Measure = { 0, 0, 0, 0 };

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
	Measure.overflow++;
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
	TCNT2 = 0;
	OCR2A = 0; 
	
	if (enable)
	{
		TCCR2A = (1 << WGM21)|(1 << WGM20);
		TCCR2B = (1 << CS22)|(0 << CS21)|(1 << CS20);
		High(TIMSK2, TOIE2);
		return;
	}
	
	TCCR2B = 0x00;
	Low(TIMSK2, TOIE2);
}

ISR(TIMER2_OVF_vect)
{	
	MainTimer.ms++;

	if (MainTimer.ms >= 1000)
	{
		MainTimer.handle = true;
		MainTimer.ms = 0;
	}
	
	TCNT2 = 130;
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
	static char fa[10] = { 0 }, fp[10] = { 0 }, fr[10];
	static char buffer[32] = { 0 };
	
	sprintf(fa, "A%d$", Measure.f1);
	sprintf(fp, "P%d$", Measure.f2);
	sprintf(fr, "R%d$", Measure.ratio);
	strcat(buffer, fa);
	strcat(buffer, fp);
	strcat(buffer, fr);
	TxString(buffer);
	
	for (int i=0; i<32; i++) buffer[i] = 0;
}

short Kalman(short value, bool reset)
{
	static float measureVariation = 5, estimateVariation = 5, speedVariation = 0.001;
	static float CurrentEstimate = 0;
	static float LastEstimate = 0;
	static float Gain = 0;
	
	if (reset)
	{
		estimateVariation = 5;
		CurrentEstimate = 0;
		LastEstimate = 0;
		Gain = 0;
	}
	
	Gain = estimateVariation / (estimateVariation + measureVariation);
	CurrentEstimate = LastEstimate + Gain * (value - LastEstimate);
	estimateVariation = (1.f - Gain) * estimateVariation + fabs(LastEstimate - CurrentEstimate) * speedVariation;
	LastEstimate = CurrentEstimate;
	return (short)CurrentEstimate;
}

void Initialization()
{
	DDRB = 0b00101010;
	PORTB = 0b00010101;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000000;
	PORTD = 0b11000111;

	Timer2(true);
	USART(Init);
	USART(On);
	sei();
}

void StartOrStop()
{
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
		PulseOff;
		OCR2A = 0;
		Timer0(false);
		Timer1(false);
		Kalman(0, true);
		Measure.overflow = 0;
		Measure.f1 = 0;
		Measure.f2 = 0;
		Measure.ratio = 0;
		Mode.run = false;
		Mode.fault = false;
		Mode.faultDelay = FaultDelay;
		Mode.startDelay = 0;
		Motor.operation = Locked;
	}
}

void ClearCountRegs()
{
	TCNT0 = 0;
	TCNT1 = 0;
	Measure.overflow = 0;
}

void SetDirection()
{	
	if (Motor.isStep) return;
	
	if (Measure.ratio >= RangeDown && Measure.ratio <= RangeUp)
	{
		Mode.faultDelay = FaultDelay;
		Motor.operation = Locked;
		return;
	}
	
	if (Motor.isDelay) return;
	
	if (Measure.ratio >= RangeUp) 
	{
		Motor.operation = Left;
		OCR2A = 190;   // sp5 190 / 144
	}
	else 
	{
		Motor.operation = Right;
		OCR2A = 145;   // sp5 145 / 132
	}
	
	Motor.isStep = StepDuration;
	PulseOn;
}
						   					
int main(void)
{	
	Initialization();

    while(1)
    {	
		if (MainTimer.handle)
		{	
			StartOrStop();
			
			if (Mode.startDelay) Mode.startDelay--;
			
			if (Mode.run && !Mode.startDelay)
			{
				LedInv;
				
				Measure.f1 = Measure.overflow*256+TCNT0;
				Measure.f2 = TCNT1;
				
				if (Measure.f1 <= Measure.f2) 
					Measure.ratio = Kalman(Overfeed - (1-(float)Measure.f1/(Measure.f2 == 0 ? 1 : Measure.f2))*-1000, false);
				else 
					Measure.ratio = Kalman(Overfeed - (1-(float)Measure.f2/Measure.f1)*1000, false);
				
				Transmit();
				
				if (Motor.isDelay > 0) Motor.isDelay--;
				
				if (Motor.isStep)
				{
					Motor.isStep--;
					
					if (!Motor.isStep)
					{
						Motor.isDelay = StepsInterval;
						PulseOff;
					}
				}
				
				SetDirection();

				if (Motor.operation != Locked && Mode.faultDelay && !Mode.fault) Mode.faultDelay--;
				
				if (!Mode.faultDelay && !Mode.fault)
				{
					FaultOn;
					Mode.fault = true;
				}
			}
			
			if (Mode.run) ClearCountRegs();
			
			MainTimer.handle = false;
		}
    }
}