/*
 * main.c
 *
 * Created: 11/18/2022 12:57:15 PM
 * Author: igor.abramov
 * OCR2A min 135 - 0.256 us - 3.2 %
 * OCR2A max 250 - 7.8 ms - 96 %   
 * flip-flop T ~ 6.2 ms
 * CC4R4
 */  
			
#define Check(REG,BIT) (REG & (1<<BIT))	    // check bit
#define Inv(REG,BIT)   (REG ^= (1<<BIT))	// invert bit
#define High(REG,BIT)  (REG |= (1<<BIT))	// set bit
#define Low(REG,BIT)   (REG &= ~(1<<BIT))	// clear bit


#define Fault		Check(PORTB, PORTB2)	// output for open contact of yarn brake
#define FaultOn		High(PORTB, PORTB2)
#define FaultOff	Low(PORTB, PORTB2)

#define PulsePin	Check(PORTB, PORTB3)	// PWM pin

#define Led			Check(PORTB, PORTB5)	// operating led, period = 2 s during winding, if stop off
#define LedOn		High(PORTB, PORTB5)
#define LedOff		Low(PORTB, PORTB5)
#define LedInv		Inv(PORTB, PORTB5)

#define Point		Check(PORTD, PORTD2)
#define PointOn		High(PORTD, PORTD2)
#define PointOff	Low(PORTD, PORTD2)
 
#define Running		(!Check(PIND, PIND3))  // spindle run input
#define InputF1		Check(PIND, PIND4)	   // T0 speed pulses input
#define InputF2		Check(PIND, PIND5)     // T1 speed pulses input

#define BtnUp		Check(PIND, PIND6)     // T1 speed pulses input
#define BtnDown		Check(PIND, PIND7)     // T1 speed pulses input

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

#define Off				  0				// hardware features modes
#define On				  1
#define Init			  2
#define Setting			  3
#define	Current			  4

#define Right	 		  10			// move directions of motor
#define Left 			  20
#define Locked			  30

#define OverfeedPointer	  0

#include <xc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>			
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>

const unsigned short START_DELAY = 30;
const unsigned short SETPOINT = 2;
const unsigned short HYSTERESIS = 4;
const unsigned short PULSE_DURATION = 2;
const unsigned short INTERVAL_BETWEEN_PULSES = 20;

const unsigned short ERROR_MOTOR = 0;
const unsigned short ERROR_F1 = 1;
const unsigned short ERROR_F2 = 2;
const unsigned short ERROR = 4;

unsigned short timer0_overflowCount = 0;  // count of ISR timer 0 (clock from pin T0)
unsigned short timer2_overflowCount = 0;  // count of ISR timer 2 (clock from 16 MHz)
bool handleAfterSecond = false;		   // flag to handle data, every second
bool handleAfter8ms = false;

short overfeed = 0;
bool overfeedChanged = true;

unsigned short displayMode = Off;
unsigned short displaySettingCount = 0;

bool pulseBan = false;
unsigned short pulseBanCount = 0;

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
	handleAfter8ms = true;
	
	if (timer2_overflowCount >= 125)		 // 8*125 = 1000 ms
	{
		handleAfterSecond = true;			 // set flag to handle data
		timer2_overflowCount = 0;			 // reset overflow counter
	}
											 // load 131 to count register, 256-131 = 125 ticks of 64us, 125*64 = 8ms
	TCNT2 = 131;
}

short Kalman(short value, bool reset)
{
	static float measureVariation = 5, estimateVariation = 5, speedVariation = 0.008;
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
	DDRB = 0b00111111;					// ports init
	PORTB = 0b00000000;
	
	DDRC = 0b00000000;
	PORTC = 0b11111111;
	
	DDRD = 0b00000000;
	PORTD = 0b11111111;

	overfeed = eeprom_read_word((uint16_t*)OverfeedPointer);

	Timer2(true);	// timer 2 switch on								
	sei();			// enable global interrupts
}

void SetDirection(short ratio, bool isReset)
{	
	static unsigned short motorState = Locked, stepCount = 0, stepsInterval = 0;
	
	if (isReset)
	{
		motorState = Locked;
		stepCount = 0;
		stepsInterval = 0;
	}
	
	if (stepsInterval)	 // after motor moving step, interval before new step
	{
		stepsInterval--;
		return;
	}
	
	if (fabs(ratio) <= SETPOINT)   // if ratio reached setpoint, reset fault counter, leds off, stop motor
	{
		if (motorState == Locked) return;
		PulseOff;
		motorState = Locked;
		stepCount = 0;
		stepsInterval = INTERVAL_BETWEEN_PULSES;
		return;
	}
	
	if (stepCount)	 // while stepCount > 0 motor is moving
	{
		stepCount--;
		
		if (stepCount < 1)	   // if stepCount reached 0 motor stopped
		{
			PulseOff;
			stepsInterval = INTERVAL_BETWEEN_PULSES;
		}
		
		return;
	}
	
	if (ratio >= HYSTERESIS) 
	{
		OCR2A = 135;
		motorState = Right;
		stepCount = PULSE_DURATION;
		PulseOn;
	}
	
	if (ratio <= (HYSTERESIS*(-1))) 
	{
		OCR2A = 250;
		motorState = Left;
		stepCount = PULSE_DURATION;
		PulseOn;
	}
}

void ControlButtons()
{
	static unsigned short overfeedUp = 0, overfeedDown = 0;
	
	displayMode = Current;
	displaySettingCount = 5;
	
	if (BtnUp) overfeedUp = 0;
	{
		if (!BtnUp) overfeedUp++;
		{
			if (overfeedUp == 1)
			{
				if (overfeed < 99) overfeed++;
				else return;
			}
		}
	}
	
	if (BtnDown) overfeedDown = 0;
	{
		if (!BtnDown) overfeedDown++;
		{
			if (overfeedDown == 1)
			{
				if (overfeed > 0) overfeed--;
				else return;
			}
		}
	}
	
	eeprom_update_word((uint16_t*)OverfeedPointer, overfeed);
	overfeedChanged = true;	
}

void ShowSetting()
{
	static unsigned short dozens = 0, units = 0;
	
	if (overfeedChanged)
	{
		dozens = abs(overfeed) / 10;
		units = abs(overfeed) % 10;
		overfeedChanged = false;
	}
	
	if (Check(PORTC, PORTC5)) 
	{
		PORTC = 0xD0 | units;
		if (Point) PointOff;
	}
	else 
	{
		PORTC = 0xE0 | dozens;
		if (overfeed < 0) PointOn;
	}
}

void ShowCurrent(short value)
{
	static unsigned short dozens = 0, units = 0;
	
	dozens = abs(value) / 10;
	units = abs(value) % 10;
	
	if (Check(PORTC, PORTC5))
	{
		PORTC = 0xD0 | units;
		if (Point) PointOff;
	}
	else
	{
		PORTC = 0xE0 | dozens;
		if (value < 0) PointOn;
	}
}
							   					
int main(void)
{
	unsigned short startDelayCount = 0;
	short f1 = 0, f2 = 0, ratio = 0;
	bool run = false;
	
	Initialization();
	
    while(1)
    {	
		if (handleAfter8ms)
		{
			ControlButtons();
			
			if (displayMode == Setting) ShowSetting();	
			if (displayMode == Current)	ShowCurrent(ratio);
			if (displayMode == Off && !(Check(PORTC, PORTC4) && Check(PORTC, PORTC5))) PORTC |= 0x30;
			
			handleAfter8ms = false;
		}
				
		if (handleAfterSecond)	  // if second counted handle data
		{	
			if (Running && !run)  // initialize before start regulation
			{
				run = true;
				startDelayCount = START_DELAY;  // set seconds for pause before 
				if (!displaySettingCount) displayMode = Current;
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
				SetDirection(0, true);	// reset function counters
				Kalman(0, true);
				run = false;
				if (!displaySettingCount) displayMode = Off;
			}
					
			if (run)						 // handle data after startDelay
			{
				LedInv;						 // operating LED	inversion

				f1 = (short)TCNT0 + timer0_overflowCount*256;  // calculation f1	(aramid)
				f2 = (short)TCNT1;							   // calculation f2	(polyamide)
	
				TCNT0 = 0;					  // reset count registers after receiving values
				TCNT1 = 0;
				timer0_overflowCount = 0;
	
				if (f1 <= f2) ratio = Kalman((overfeed-(float)f1/(f2 == 0 ? 1 : f2))*1000, false);	  
				else ratio = Kalman((overfeed-(float)f2/f1)*-1000, false);
				
				if (!startDelayCount) SetDirection(ratio, false);		// calculation average ratio
			}
			
			if (startDelayCount) startDelayCount--;  // start delay counter

			if (displaySettingCount)
			{
				displaySettingCount--;
				
				if (!displaySettingCount)
				{
					if (run) displayMode = Current;
					else displayMode = Off;
				}
			}

			handleAfterSecond = false;	  // reset handle second flag
		}
    }
}