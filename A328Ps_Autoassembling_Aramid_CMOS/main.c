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

#define Dot			Check(PORTD, PORTD2)
#define DotOn		High(PORTD, PORTD2)
#define DotOff		Low(PORTD, PORTD2)
 
#define Running		(!Check(PIND, PIND3))  // spindle run input
#define InputF1		Check(PIND, PIND4)	   // T0 speed pulses input
#define InputF2		Check(PIND, PIND5)     // T1 speed pulses input

#define BtnUp		Check(PIND, PIND6)     // T1 speed pulses input
#define BtnDown		Check(PIND, PIND7)     // T1 speed pulses input

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

#define Off				  0				   // hardware features modes
#define On				  1
#define Init			  2
#define Setting			  3
#define	Current			  4
#define Error			  5

#define Right	 		  10				// move directions of motor
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
const unsigned short PULSE_DURATION = 2;
const unsigned short INTERVAL_BETWEEN_PULSES = 20;

const unsigned short overfeed_limit = 200;

const unsigned short ERROR_F1 = 1;
const unsigned short ERROR_F2 = 2;
const unsigned short ERROR_MOTOR = 3;

unsigned short timer0_overflowCount = 0;  // count of ISR timer 0 (clock from pin T0)
unsigned short timer2_overflowCount = 0;  // count of ISR timer 2 (clock from 16 MHz)
bool handleAfterSecond = false;		   // flag to handle data, every second
bool handleAfter8ms = false;

short overfeed = 0;					 // in mm

unsigned short displayMode = Setting;
unsigned short displaySettingCount = 5;

bool pulseIsLocked = false;
unsigned short pulseLockCount = 0;
unsigned short currentError = 0;

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

void Transmit(short f1, short f2, short ratio)
{
	static char fa[10] = { 0 }, fp[10] = { 0 }, fr[10];
	static char buffer[32] = { 0 };
	
	sprintf(fa, "A%d$", f1);
	sprintf(fp, "P%d$", f2);
	sprintf(fr, "R%d$\r\n", ratio);
	strcat(buffer, fa);
	strcat(buffer, fp);
	strcat(buffer, fr);
	TxString(buffer);
	
	buffer[0] = '\0';
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
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000100;
	PORTD = 0b11111011;

	overfeed = eeprom_read_word((uint16_t*)OverfeedPointer);

	Timer2(true);	// timer 2 switch on								
	sei();			// enable global interrupts
}

void SetDirection(short ratio, bool isReset)
{	
	static unsigned short motorState = Locked, stepCount = 0, stepsInterval = 0;
	
	if (pulseIsLocked)
	{
		motorState = Locked;
		stepCount = 0;
		stepsInterval = 0;
		return;
	}
	
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
	
	if (ratio >= 5) 
	{
		OCR2A = 135;
		motorState = Right;
		stepCount = PULSE_DURATION;
		PulseOn;
	}
	
	if (ratio <= -5) 
	{
		OCR2A = 250;
		motorState = Left;
		stepCount = PULSE_DURATION;
		PulseOn;
	}
}

void ControlButtons()
{
	static short overfeedUp = 0, overfeedDown = 0;
	static bool isChanged = false;
	
	if (displayMode == Error) return;
	
	if (BtnUp) overfeedUp = 0;
	{
		if (!BtnUp) overfeedUp++;
		{
			if (overfeedUp == 1)
			{
				if (overfeed < 200) 
				{
					if (displayMode != Setting)
					{
						displayMode = Setting;
						return;
					}
					overfeed++;
					isChanged = true;
				}
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
				if (overfeed > -200) 
				{
					if (displayMode != Setting) 
					{
						displayMode = Setting;
						return;
					}
					overfeed--;
					isChanged = true;
				}
				else return;
			}
		}
	}	
	
	if (isChanged)
	{
		eeprom_update_word((uint16_t*)OverfeedPointer, overfeed);
		displayMode = Setting;
		displaySettingCount = 5;
		isChanged = false;
	}
}

void Print(short value)
{
	static unsigned short dozens = 0, units = 0, uvalue = 0;
	
	uvalue = abs(value);
	
	if (uvalue > 999)
	{
		dozens = 9;
		units = 9;
	}
	else if (uvalue > 100)
	{
		dozens = uvalue / 100;
		units = (uvalue / 10) % 10;
	}
	else
	{
		dozens = uvalue / 10;
		units = uvalue % 10;	
	}
	
	if (Check(PORTC, PORTC5))
	{
		PORTC = 0xD0 | units;
		
		if (Dot)
		{
			if (value >= 0) DotOff;
		}
		else
		{
			if (value < 0) DotOn;
		}
	}
	else
	{
		PORTC = 0xE0 | dozens;
		
		if (Dot)
		{
			if (uvalue >= 100) DotOff;
		}
		else
		{
			if (uvalue < 100) DotOn;
		}
	}
}
	
void PrintError()
{
	static bool blink = false;
	
	if (blink)
	{
		PORTC = 0xD0 | currentError;
		if (Dot) DotOff;
		return;
	}	
	
	PORTC |= 0x30;
}
							   					
int main(void)
{
	unsigned short startDelayCount = 0;
	short f1 = 0, f2 = 0, ratio = 0, difference = 0;
	bool run = false;
	
	Initialization();
	
	USART(Init);
	USART(On);
	
    while(1)
    {	
		if (handleAfter8ms)
		{
			ControlButtons();
			
			if (displayMode == Setting) Print(overfeed);
			if (displayMode == Current)	Print(ratio);
			if (displayMode == Error)	PrintError();
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
				handleAfterSecond = false;
				continue;
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
				TCNT0 = 0;
				TCNT1 = 0;
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
	
				if (f1 <= f2) ratio = (1-(float)f1/(f2 == 0 ? 1 : f2))*1000;
				else ratio = (1-(float)f2/f1)*-1000;
				
				if (f1 == 0 && f2 == 0) ratio = 0;
				
				difference = Kalman(overfeed - ratio, false);
				
				Transmit(f1, f2, difference);
				if (!startDelayCount) SetDirection(difference, false);		// calculation average ratio
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