/*
 * main.c
 *
 * Created: 11/18/2022 12:57:15 PM
 *  Author: igor.abramov
 * OCR2A min 135 - 0.256 us - 3.2 %
 * OCR2A max 250 - 7.8 ms - 96 %   
 * flip-flop T ~ 6.2 ms
 */  
			
#define Check(REG,BIT) (REG & (1<<BIT))	    // check bit
#define Inv(REG,BIT)   (REG ^= (1<<BIT))	// invert bit
#define High(REG,BIT)  (REG |= (1<<BIT))	// set bit
#define Low(REG,BIT)   (REG &= ~(1<<BIT))	// clear bit

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
	
#define ArraySize		  40			// average array length
#define StartDelay		  10			// delay to start measuring after spindle start
#define FaultDelay		  1200  		// (seconds) if Mode.operation != Stop more than FaultDelay seconds then spindle stop
#define Setpoint		  0.002			// ratio value for stop motor
#define RangeUp			  0.006			// if ratio > range up then motor moves left
#define RangeDown		  -0.006		// if ratio < range down then motor moves right
#define StepDuration	  2				// work time of PWM to one step (roughly)
#define StepsInterval	  20			// interval between steps
#define MeasureFaultLimit 100			// count of measurements frequency. if f < 10 more than 100 times stop

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>			
#include <stdbool.h>
#include <string.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

struct Logger
{
	float length;
	unsigned int setpoint;
	unsigned short cell;
} Logs = { 0, 0, 1 };

volatile unsigned short timer0_overflowCount = 0;  // count of ISR timer 0 (clock from pin T0)
volatile unsigned short timer2_overflowCount = 0;  // count of ISR timer 2 (clock from 16 MHz)
volatile bool handleAfterSecond = false;		   // flag to handle data, every second

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);	 // external source clock
		High(TIMSK0, TOIE0);							 // ISR enabled
		return;
	}
														 // stop count
	TCCR0B = 0x00;
}

ISR(TIMER0_OVF_vect)
{
	timer0_overflowCount++;	  // increase ISR counter 
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);   // external source clock 
		return;
	}
	
	TCCR1B = 0x00;
}

void Timer2(bool enable)
{
	TCNT2 = 0; 	   // reset count register
	
	if (enable)
	{
		TCCR2A = (1 << WGM21)|(1 << WGM20);				// configuration hardware PWM 
		TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);	// scaler 1024
		High(TIMSK2, TOIE2);							// overflow interrupt enabled, every 8 ms.
		return;
	}
	
	TCCR2B = 0x00;										// stop count
	Low(TIMSK2, TOIE2);									// overflow interrup disabled
}

ISR(TIMER2_OVF_vect)
{	
	timer2_overflowCount++;					 // increase overflow variable

	if (timer2_overflowCount >= 125)		 // 8*125 = 1000 ms
	{
		handleAfterSecond = true;			 // set flag to handle data
		timer2_overflowCount = 0;			 // reset overflow counter
	}
											 // load 131 to count register, 256-131 = 125 ticks of 64us, 125*64 = 8ms
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
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void TxString(const char* s)
{
	for (int i=0; s[i]; i++) TxChar(s[i]);
}

void Transmit(float ratio)
{
	static char r[16] = { 0 };
	
	sprintf(r, "R%.3f$", ratio);
	TxString(r);
}

float Average(float value)		   // moving average for frequencies ratio
{
	static unsigned short index = 0;		  // initialize static vars
	static float array[ArraySize] = { 0 };
	static float result = 0;
	
	result += value - array[index];
	array[index] = value;
	index = (index + 1) % ArraySize;
	
	return result/ArraySize;
}

void Initialization()
{
	//wdt_disable();				   // disable wdt timer
	
	unsigned short length = 0;
	
	DDRB = 0b00111111;					// ports init
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000000;
	PORTD = 0b11111111;
	
	for (int i = 0; i<ArraySize; i++) Average(0);  // reset average array

	Timer2(true);	// timer 2 switch on
	USART(Init);
	USART(On);
	sei();			// enable global interrupts

	length = eeprom_read_word((unsigned int*)0);
	
	for (unsigned int i=1; i<=length; i++) Transmit(eeprom_read_float((float*)i));

	//wdt_enable(WDTO_1S);  // enable wdt with reset after 1 second hang
}

void SetDirection(float ratio)
{	
	static unsigned short faultCounter = 0, motorState = Locked, stepCount = 0, stepsInterval = 0;
	
	if (fabs(ratio) <= Setpoint)   // if ratio reached setpoint, reset fault counter, leds off, stop motor
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
	
	if (stepCount)	 // while stepCount > 0 motor is moving
	{
		stepCount--;
		
		if (stepCount < 1)	   // if stepCount reached 0 motor stopped
		{
			PulseOff;
			stepsInterval = StepsInterval;
		}
			
		return;
	}
		
	if (stepsInterval)	 // after motor moving step, interval before new step
	{
		stepsInterval--;
		return;
	}
	
	faultCounter++;		 // ratio not in ranges, increase fault count
	
	if (faultCounter > FaultDelay) 	// if fault counter reached fault limit it is not regulated, stop spindle
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
		if (motorState != Right) // if f1 < f2 pwm is on with width < 1 %
		{
			OCR2A = 135;
			motorState = Right;
			RightLedOn;
			LeftLedOff;
		}
	}
	else 
	{
		if (motorState != Left)  // if f1 < f2 pwm is on with width 99%
		{
			OCR2A = 250;
			motorState = Left;
			LeftLedOn;
			RightLedOff;
		}
	}
	
	PulseOn;
	stepCount = StepDuration;	 // set duration of pwm work
}
							   					
int main(void)
{
	float f1 = 0, f2 = 0, ratio = 0;
	unsigned short startDelayCount = StartDelay, f1_measureFaults = 0, f2_measureFaults = 0;
	bool run = false;
	
	Initialization();
	
    while(1)
    {			
		if (handleAfterSecond)	  // if second counted handle data
		{			
			if (Running && !run)  // initialize before start regulation
			{
				FaultLedOff;
				RightLedOff;
				LeftLedOff;
				run = true;
				startDelayCount = StartDelay;  // set seconds for pause before calc
				Timer0(true);
				Timer1(true);
			}
			
			if (!Running && run)   // reset after stop spindle; right, left, fault could be high before next start
			{
				LedOff;
				PulseOff;
				if (Fault) FaultOff;
				Timer0(false);
				Timer1(false);
				SetDirection(0);	// reset function counters
				for (int i = 0; i<ArraySize; i++) Average(0);
				Logs.length = 0;
				Logs.cell = 1;
				Logs.setpoint = 0;
				f1_measureFaults = 0;
				f2_measureFaults = 0;
				startDelayCount = 0;
				run = false;
				f1 = 0;
				f2 = 0;
			}
			
			if (run && !startDelayCount)	 // handle data after startDelay
			{
				LedInv;		   // operating LED	inversion

				f1 = (float)TCNT0 + (float)timer0_overflowCount*256.f;  // calculation f1	(aramid)
				f2 = (float)TCNT1;										// calculation f2	(polyamide)
				ratio = Average(1 - (f1 == 0 ? 1 : f1) / (f2 == 0 ? 1 : f2));
				SetDirection(ratio);									// calculation average ratio
				Transmit(ratio);
				
				Logs.length += (((f1+f2)/200)*0.08796);
				
				if (Logs.length >= Logs.setpoint)
				{
					eeprom_update_float((float*)Logs.cell, ratio);
					eeprom_update_word((unsigned int*)0, Logs.cell);
					Logs.setpoint += 1000;
					Logs.cell++;
				}
				
				if (f1 < 10) f1_measureFaults++;   // count measure error f1 (10 is experimental value, not tested yet)
				if (f2 < 10) f2_measureFaults++;   // count measure error f2
				
				// if error measure reached limit, stop spindle, led on accordingly wrong measure channel
				if (f1_measureFaults >= MeasureFaultLimit || f2_measureFaults >= MeasureFaultLimit)
				{
					FaultOn;
					PulseOff;
					FaultLedOn;
					if (f1_measureFaults >= MeasureFaultLimit) RightLedOn; else LeftLedOn; // set led accordingly measure fault channel
				}
			}
			
			if (startDelayCount) startDelayCount--;  // start delay counter

			TCNT0 = 0;					  // reset count registers after receiving values
			TCNT1 = 0;
			timer0_overflowCount = 0;
			handleAfterSecond = false;	  // reset handle second flag
		}
		
		//wdt_reset();					  // wdt reset
    }
}