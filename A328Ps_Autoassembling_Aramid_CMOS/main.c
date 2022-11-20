/*
 * main.c
 *
 * Created: 11/18/2022 12:57:15 PM
 *  Author: igor.abramov
 * OCR2A min 132 - 0.13 us - 1.6 %
 * OCR2A max 254 - 8 ms - 99.2 %   
 * flip-flop T ~ 6.8 ms
 */  
			
#define Check(REG,BIT) (REG & (1<<BIT))
#define Inv(REG,BIT)   (REG ^= (1<<BIT))
#define High(REG,BIT)  (REG |= (1<<BIT))
#define Low(REG,BIT)   (REG &= ~(1<<BIT))

#define RightLed	Check(PORTB, PORTB0)	// if f1 < f2 - led on else off
#define RightLedOn	High(PORTB, PORTB0)
#define RightLedOff	Low(PORTB, PORTB0)

#define LeftLed		Check(PORTB, PORTB1)	// if f1 > f2 - led on else off
#define LeftLedOn	High(PORTB, PORTB1)
#define LeftLedOff	Low(PORTB, PORTB1)

#define FaultLed	Check(PORTB, PORTB2)	// if fault - led on before next start or reset
#define FaultLedOn	High(PORTB, PORTB2)
#define FaultLedOff	Low(PORTB, PORTB2)

#define PulsePin	Check(PORTB, PORTB3)	// PWM pin

#define Fault		Check(PORTB, PORTB4)	// output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB4)
#define FaultOff	Low(PORTB, PORTB4)
#define FaultInv	Inv(PORTB, PORTB4)

#define Led			Check(PORTB, PORTB5)	// operating led, period = 2 s during winding, if stop off
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)
 
#define Running		(!Check(PIND, PIND3))  // spindle run input
#define InputF1		Check(PIND, PIND4)	   // T0 speed pulses input
#define InputF2		Check(PIND, PIND5)     // T1 speed pulses input

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

#define Off				  0				// hardware features modes
#define On				  1
#define Init			  2

#define Right	 		  10			// move directions of motor
#define Left 			  20
#define Locked			  30
	
#define ArraySize		  32			// average array length
#define StartDelay		  10			// delay to start measuring after spindle start
#define FaultDelay		  1200  		// (seconds) if Mode.operation != Stop more than FaultDelay seconds then spindle stop
#define Setpoint		  0.003			// ratio value for stop motor
#define RangeUp			  0.006			// if ratio > range up then motor moves left
#define RangeDown		  -0.006		// if ratio < range down then motor moves right
#define StepDuration	  3				// work time of PWM to one step (roughly)
#define StepsInterval	  4				// interval between steps
#define MeasureFaultLimit 100

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>			
#include <stdbool.h>
#include <string.h>
#include <avr/wdt.h>

volatile unsigned short timer0_overflowCount = 0;
volatile unsigned short timer2_overflowCount = 0;
volatile bool handleAfterSecond = false;

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
	timer0_overflowCount++;
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
	timer2_overflowCount++;

	if (timer2_overflowCount >= 125)
	{
		handleAfterSecond = true;
		timer2_overflowCount = 0;
	}

	TCNT2 = 131;
}

float Average(float value)
{
	static unsigned short index = 0;
	static float array[ArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % ArraySize;
	
	return result/ArraySize;
}

void Initialization()
{
	wdt_disable();
	
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000000;
	PORTD = 0b11111111;
	
	for (int i = 0; i<ArraySize; i++) Average(0);

	Timer2(true);
	sei();
	
	wdt_enable(WDTO_1S);
}

void SetDirection(float ratio)
{	
	static unsigned short faultCounter = 0, motorState = Locked, stepCount = 0, stepsInterval = 0;
	
	if (fabs(ratio) <= Setpoint)
	{
		PulseOff;
		RightLedOff;
		LeftLedOff;
		if (faultCounter > 0) faultCounter = 0;
		motorState = Locked;
		stepCount = 0;
		stepsInterval = 0;
		return;
	}
	
	if (stepCount)
	{
		stepCount--;
		if (stepCount < 1)
		{
			PulseOff;
			stepsInterval = StepsInterval;
		}
			
		return;
	}
		
	if (stepsInterval)
	{
		stepsInterval--;
		return;
	}
	
	faultCounter++;
	
	if (faultCounter > FaultDelay) 
	{
		FaultOn;
		PulseOff;
		FaultLedOn;
		faultCounter = 0;
		motorState = Locked;
		return;
	}
	
	if (ratio >= RangeUp) 
	{
		if (motorState != Right) 
		{
			OCR2A = 132;
			motorState = Right;
			RightLedOn;
			LeftLedOff;
		}
	}
	else 
	{
		if (motorState != Left)
		{
			OCR2A = 254;
			motorState = Left;
			LeftLedOn;
			RightLedOff;
		}
	}
	
	PulseOn;
	stepCount = StepDuration;
}
							   					
int main(void)
{
	static float f1 = 0, f2 = 0;
	static unsigned short startDelay = StartDelay, f1_measureFaults = 0, f2_measureFaults = 0;
	static bool run = false;
	
	Initialization();

    while(1)
    {			
		if (handleAfterSecond)
		{			
			if (Running && !run)
			{
				FaultLedOff;
				RightLedOff;
				LeftLedOff;
				run = true;
				startDelay = StartDelay;
				Timer0(true);
				Timer1(true);
			}
			
			if (!Running && run)
			{
				LedOff;
				if (Fault) FaultOff;
				Timer0(false);
				Timer1(false);
				for (int i = 0; i<ArraySize; i++) Average(0);
				f1_measureFaults = 0;
				f2_measureFaults = 0;
				startDelay = 0;
				run = false;
				f1 = 0;
				f2 = 0;
			}
			
			if (run && !startDelay)
			{
				LedInv;

				f1 = TCNT0 + timer0_overflowCount*256.f;
				f2 = TCNT1;
				
				SetDirection(Average(1 - (f1 == 0 ? 1 : f1) / (f2 == 0 ? 1 : f2)));
				
				if (f1 < 10) f1_measureFaults++;
				if (f2 < 10) f2_measureFaults++;	
				
				if (f1_measureFaults >= MeasureFaultLimit || f2_measureFaults >= MeasureFaultLimit)
				{
					FaultOn;
					PulseOff;
					FaultLedOn;
					SetDirection(0);
					if (f1_measureFaults >= MeasureFaultLimit) RightLedOn; else LeftLedOn;
				}
			}
			
			if (startDelay) startDelay--;

			TCNT0 = 0;
			TCNT1 = 0;
			timer0_overflowCount = 0;
			handleAfterSecond = false;
		}
		
		wdt_reset();
    }
}