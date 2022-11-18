/*
 * main.c
 *
 * Created: 11/18/2022 12:57:15 PM
 *  Author: igor.abramov
 */  
			
#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define Imp			Check(PORTB, PORTB0)	// control pulses of motor
#define ImpOn		High(PORTB, PORTB0)
#define ImpOff		Low(PORTB, PORTB0)
#define ImpInv		Inv(PORTB, PORTB0)

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

#define Off				  0			// hardware features modes
#define On				  1
#define Init			  2

#define Right	 		  10	    // move directions of motor
#define Left 			  20
#define Locked			  30
	
#define ArraySize		  64		// average array length
#define StartDelay		  10		// delay to start measuring after spindle start
#define FaultDelay		  1200  	// (seconds) if Mode.operation != Stop more than FaultDelay seconds then spindle stop
#define Setpoint		  0.001		// ratio value for stop motor
#define RangeUp			  0.005		// if ratio > range up then motor moves left
#define RangeDown		  -0.005	// if ratio < range down then motor moves right
#define StepDuration	  4			// work time of PWM to one step
#define StepsInterval	  32		// interval between	steps
#define PulsesInterval	  3

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
	bool handleS, handleMS;
} MainTimer = { 0, false };

struct Data
{
	unsigned short ovf;
	float Fa, Fp, d;
} Measure = { 0, 0, 0, 0 };

struct ModeControl
{
	unsigned short startDelay, faultDelay;
	bool fault, run;
} Mode = { 0, FaultDelay, false, false };

struct MotorControl
{
	unsigned short isDelay, isStep, operation, stepsInterval, pulseCounter; 
	bool isFirstPulse;
} Motor = { 0, 0, Locked, 0, false };

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
	TCNT2 = 0; 
	
	if (enable)
	{
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
		MainTimer.handleS = true;
		MainTimer.ms = 0;
	}

	TCNT2 = 130;
}

float Average(float difference, bool isReset)
{
	static float values[ArraySize] = { 0 };
	static int index = 0;
	static float result = 0;
	
	if (isReset)
	{
		for (int i = 0; i<ArraySize; i++) values[i] = 0;
		index = 0;
		result = 0;
		return 0;
	}
	
	if (++index >= ArraySize) index = 0;
	
	result -= values[index];
	result += difference;
	values[index] = difference;
	
	return result / ArraySize;
}

void Calculation()
{	
	Measure.Fa = (float)TCNT0 + Measure.ovf*256.f;
	Measure.Fp = (float)TCNT1;
	Measure.d = Average(1 - (Measure.Fa == 0 ? 1 : Measure.Fa) / (Measure.Fp == 0 ? 1 : Measure.Fp), false);		
}

void Initialization()
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000010;
	PORTD = 0b00000011;
	
	Timer2(true);
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
		ImpOff;
		OCR2A = 0;
		Timer0(false);
		Timer1(false);
		Average(0, true);
		Measure.Fa = 0;
		Measure.Fp = 0;
		Measure.d = 0;
		Mode.run = false;
		Mode.fault = false;
		Mode.faultDelay = FaultDelay;
		Mode.startDelay = 0;
		Motor.operation = Locked;
	}
}

void Regulation()
{
	if (Motor.isStep) return;
	
	if (fabs(Measure.d) <= Setpoint)
	{
		Mode.faultDelay = FaultDelay;
		Motor.operation = Locked;
		return;
	}
	
	if (Motor.isDelay) return;
	
	if (Measure.d >= RangeUp) 
	{
		Motor.operation = Left;
		//Motor.pulseCounter = 
	}
	else 
	{
		Motor.operation = Right;
	}
	
	Motor.isStep = StepDuration;
	Motor.isFirstPulse = true;
}

void Process()
{
	if (Mode.run && !Mode.startDelay)
	{
		LedInv;
		
		Calculation();
		
		if (Motor.isDelay > 0) Motor.isDelay--;
		
		if (Motor.isStep)
		{	
			Motor.isStep--;
			
			if (!Motor.isStep) 
			{
				Motor.isDelay = StepsInterval;
			}
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
		Measure.ovf = 0;
	}
}

void StepControl()
{
	if (Motor.pulseCounter) 
	{
		Motor.pulseCounter--;
		
		if (!Motor.pulseCounter) 
		{
			if (Imp) 
			{
				ImpOff;
				//Motor.stepsInterval
			}
		}
	} 
}
							   					
int main(void)
{
	Initialization();

    while(1)
    {	
		if (MainTimer.handleMS)
		{
			StepControl();
			MainTimer.handleMS = 0;
		}
		
		if (MainTimer.handleS)
		{	
			if (Motor.pulseCounter) Motor.pulseCounter--;
			
			if (Mode.startDelay) Mode.startDelay--;

			StartOrStop();
			Process();
			
			MainTimer.handleS = false;
		}
    }
}