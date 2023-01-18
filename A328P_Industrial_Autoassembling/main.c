/*
 * A328P_Industrial_Autoassembling.c
 *
 * Created: 27.12.2022 21:34:47
 * Author : Abramov IV
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

#define BtnPlus		Check(PIND, PIND6)     // T1 speed pulses input
#define BtnMinus	Check(PIND, PIND7)     // T1 speed pulses input

#define Pulse		Check(TCCR2A, COM2A1)  // Fast PWM output 2 of timer 2
#define PulseOn		High(TCCR2A, COM2A1)
#define PulseOff	Low(TCCR2A, COM2A1)

#define Off				  0				   // hardware features modes
#define On				  1
#define Init			  2
#define Setting			  3
#define	Current			  4
#define Error			  5
#define Common			  6
#define Settings		  7

#define Right	 		  10				// move directions of motor
#define Left 			  20
#define Locked			  30

#define ErrorLimit		  100
#define DisplayTimeout	  10

#define OverfeedPointer			0
#define SetpointPointer			2
#define PulseDurationPointer    4
#define PulsesIntervalPointer	6
#define StartDelayPointer		8
#define FactorAPointer			10
#define FactorBPointer			12
#define FactorMeasurePointer	14
#define FactorEstimatePointer	16
#define FactorSpeedPointer		18

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>

const unsigned short ERROR_F1 = 1;
const unsigned short ERROR_F2 = 2;
const unsigned short ERROR_F = 3;
const unsigned short ERROR_MOTOR = 4;
const unsigned short ERROR_OVERREG = 5;

short Pointers[] = { OverfeedPointer, SetpointPointer, PulseDurationPointer, PulsesIntervalPointer, 
				     StartDelayPointer, FactorAPointer, FactorBPointer, FactorMeasurePointer,
				     FactorEstimatePointer, FactorSpeedPointer };
short ChangableValue = 0;

short Overfeed = 0;
unsigned short Setpoint = 0;       // 1
unsigned short PulseDuration = 0;  // 2
unsigned short PulsesInterval = 0; // 4
unsigned short StartDelay = 0;     // 30
float FactorA = 0;				   // 1
float FactorB = 0;				   // 1
short FactorMeasure	= 0;		   // 5
short FactorEstimate = 0;		   // 5
float FactorSpeed = 0;             // 0.05

unsigned short Timer0_OverflowCount = 0;  // count of ISR timer 0 (clock from pin T0)
unsigned short Timer1_OverflowCount = 0;  // count of ISR timer 1 (clock from pin T1)
unsigned short Timer2_OverflowCount = 0;  // count of ISR timer 2 (clock from 16 MHz)
bool HandleAfterSecond = false;			  // flag to handle data, every second
bool HandleAfter200ms = false;
bool HandleAfter8ms = false;					 

unsigned short InterfaceMode = Common;	 
unsigned short DisplayMode = Off;
unsigned short IndexCurrentSetting = 0;
unsigned short DisplaySettingCount = 0;
unsigned short SettingExitCount = 0;
bool Blink = false;
bool SaveSetting = false;
bool ManualControl = false;

bool PlusPushed = false, MinusPushed = false;

unsigned short PulseLockCount = 0;
unsigned short CurrentError = 0;

bool IsRun = false;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);	 // external source clock
		High(TIMSK0, TOIE0);							 // ISR enabled
		TCNT0 = 0;
		return;
	}
	
	Low(TIMSK0, TOIE0);
	TCCR0B = 0x00;										  // stop count
}

ISR(TIMER0_OVF_vect)
{
	Timer0_OverflowCount++;	  // increase ISR counter
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);   // external source clock
		High(TIMSK1, TOIE1);
		TCNT1 = 0;
		return;
	}
	
	Low(TIMSK1, TOIE1);
	TCCR1B = 0x00;
}

ISR(TIMER1_OVF_vect)
{
	Timer1_OverflowCount++;
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
	Timer2_OverflowCount++;					 // increase overflow variable
	HandleAfter8ms = true;
	
	if (Timer2_OverflowCount % 25 == 0) HandleAfter200ms = true;
	
	if (Timer2_OverflowCount >= 125)		 // 8*125 = 1000 ms
	{
		HandleAfterSecond = true;			 // set flag to handle data
		Timer2_OverflowCount = 0;			 // reset overflow counter
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

void Transmit(short *f1, short *f2, short *ratio)
{
	static char fa[10] = { 0 }, fp[10] = { 0 }, fr[10] = { 0 };
	static char buffer[32] = { 0 };
	
	sprintf(fa, "A%d$ ", *f1);
	sprintf(fp, "P%d$ ", *f2);
	sprintf(fr, "D%d$", *ratio);
	
	strcat(buffer, fa);
	strcat(buffer, fp);
	strcat(buffer, fr);
	
	TxString(buffer);
	
	buffer[0] = '\0';
}

short Kalman(short value, bool reset)
{
	static float estimateVariation = 0;
	static float CurrentEstimate = 0;
	static float LastEstimate = 0;
	static float Gain = 0;
	
	if (reset)
	{
		estimateVariation = FactorEstimate;
		CurrentEstimate = 0;
		LastEstimate = 0;
		Gain = 0;
	}
	
	Gain = estimateVariation / (estimateVariation + FactorMeasure);
	CurrentEstimate = LastEstimate + Gain * (value - LastEstimate);
	estimateVariation = (1.f - Gain) * estimateVariation + fabs(LastEstimate - CurrentEstimate) * (FactorSpeed/1000);
	LastEstimate = CurrentEstimate;
	return (short)CurrentEstimate;
}

void LoadSettings()
{
	Overfeed = eeprom_read_word((uint16_t*)OverfeedPointer);
	Setpoint = eeprom_read_word((uint16_t*)SetpointPointer);
	PulseDuration = eeprom_read_word((uint16_t*)PulseDurationPointer);
	PulsesInterval = eeprom_read_word((uint16_t*)PulsesIntervalPointer);
	StartDelay = eeprom_read_word((uint16_t*)StartDelayPointer);
	FactorA = 1.-(float)eeprom_read_word((uint16_t*)FactorAPointer)/100.f;
	FactorB = 1.-(float)eeprom_read_word((uint16_t*)FactorBPointer)/100.f;
	FactorMeasure = eeprom_read_word((uint16_t*)FactorMeasurePointer);
	FactorEstimate = eeprom_read_word((uint16_t*)FactorEstimatePointer);
	FactorSpeed = (float)eeprom_read_word((uint16_t*)FactorSpeedPointer)/1000.f;
}

void Initialization()
{
	DDRB = 0b00111111;					// ports init
	PORTB = 0b00000000;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00000100;
	PORTD = 0b11111011;

	PORTC |= 0x30;	  // display off

	LoadSettings();

	Kalman(0, true);
	Timer2(true);	// timer 2 switch on
	USART(Init);
	USART(On);
	sei();			// enable global interrupts
}

short GetRatio(short *p_f1, short *p_f2)
{
	if (!*p_f1 && !*p_f2) return 0;
	
	if (*p_f1 <= *p_f2) return (1-(float)*p_f1/(*p_f2 == 0 ? 1 : *p_f2))*1000;
	else return (1-(float)*p_f2/(*p_f1))*-1000;
}

void SetDirection(short *p_ratio, bool isReset)
{
	static unsigned short motorState = Locked, stepCount = 0, stepsInterval = 0, errorCount = 0;
	
	if (isReset)
	{
		motorState = Locked;
		stepCount = 0;
		stepsInterval = 0;
		errorCount = 0;
		return;
	}
	
	if (stepsInterval)	 // after motor moving step, interval before new step
	{
		stepsInterval--;
		return;
	}
	
	if (fabs(*p_ratio) <= Setpoint)   // if ratio reached setpoint, reset fault counter, leds off, stop motor
	{
		if (motorState == Locked) return;
		if (errorCount > 0) errorCount = 0;
		
		PulseOff;
		motorState = Locked;
		stepCount = 0;
		stepsInterval = PulsesInterval;
		return;
	}
	
	if (stepCount)	 // while stepCount > 0 motor is moving
	{
		stepCount--;
		
		if (stepCount < 1)	   // if stepCount reached 0 motor stopped
		{
			PulseOff;
			stepsInterval = PulsesInterval;
		}
		
		return;
	}
	
	if (motorState != Locked) errorCount++;
	
	if (errorCount >= ErrorLimit)
	{
		DisplayMode = Error;
		CurrentError = ERROR_OVERREG;
		errorCount = 0;
		FaultOn;
		return;
	}
	
	if (*p_ratio >= 5)
	{
		OCR2A = 135;
		motorState = Right;
		stepCount = PulseDuration;
		PulseOn;
	}
	
	if (*p_ratio <= -5)
	{
		OCR2A = 250;
		motorState = Left;
		stepCount = PulseDuration;
		PulseOn;
	}
}

void Print(short *p_value)
{
	static unsigned short dozens = 0, units = 0, uvalue = 0;
	
	uvalue = abs(*p_value);
	
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
			if (*p_value >= 0) DotOff;
		}
		else
		{
			if (*p_value < 0) DotOn;
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
	if (Check(PORTC, PORTC4) && Check(PORTC, PORTC5))
	{
		PORTC = 0xE0 | CurrentError;
		if (Dot) DotOff;
		return;
	}
	
	PORTC |= 0x30;
}

void ControlButtons()
{
	static unsigned short plus = 0, minus = 0;
	
	if (!BtnPlus) plus++;
	{
		if (plus == 1)
		{
			PlusPushed = true;
			plus = 0;
		}
	}
	
	if (!BtnMinus) minus++;
	{
		if (minus == 1)
		{
			MinusPushed = true;
			minus = 0;
		}
	}
	
	if (PlusPushed && MinusPushed)
	{
		if (InterfaceMode == Common) 
		{
			PulseOff;
			InterfaceMode = Settings;
			DisplayMode = Settings;
			DisplaySettingCount = 0;
		}
		else if (InterfaceMode == Settings) 
		{
			InterfaceMode = Setting;
			DisplayMode = Setting;
			ChangableValue = eeprom_read_word((uint16_t*)Pointers[IndexCurrentSetting]);
		}
		else 
		{
			SaveSetting = true;
		}
		
		PlusPushed = false;
		MinusPushed = false;
	}
}

void InstantValuesCountrol(short *p_f1, short *p_f2)
{
	static unsigned short errorCount = 0;
	
	if (*p_f1 < 10 || *p_f2 < 10)
	{
		errorCount++;
		if (*p_f1 < 10) CurrentError = ERROR_F1;
		if (*p_f2 < 10) CurrentError = ERROR_F2;
		if (*p_f1 < 10 && *p_f2 < 10) CurrentError = ERROR_F;
		if (errorCount >= ErrorLimit)
		{
			DisplayMode = Error;
			errorCount = 0;
			FaultOn;
		}
		return;	
	}
	
	if (errorCount) errorCount = 0;
}

void CommonControl()
{
	if (IsRun && DisplayMode == Off && (PlusPushed || MinusPushed))
	{
		DisplayMode = Current;
		DisplaySettingCount = DisplayTimeout;
		PlusPushed = false;
		MinusPushed = false;
		return;	
	}
	
	if (PlusPushed)
	{
		if (OCR2A != 135 || !Pulse)
		{
			OCR2A = 135;
			PulseOn;
		}
		ManualControl = true;
		PlusPushed = false;
	}
	
	if (MinusPushed)
	{
		if (OCR2A != 250 || !Pulse)
		{
			OCR2A = 250;
			PulseOn;
		}
		ManualControl = true;
		MinusPushed = false;
	}
}

void SettingsControl()
{	
	if (PlusPushed)
	{
		if (IndexCurrentSetting < 9) IndexCurrentSetting++;
		PlusPushed = false;
		return;
	}
	
	if (MinusPushed)
	{
		if (IndexCurrentSetting > 0) IndexCurrentSetting--;
		MinusPushed = false;
	}
}

void SettingControl()
{	
	if (SaveSetting)
	{	
		eeprom_update_word((uint16_t*)Pointers[IndexCurrentSetting], ChangableValue);
		ChangableValue = 0;
		PlusPushed = false;
		MinusPushed = false;
		InterfaceMode = Settings;
		DisplayMode = Settings;
		SaveSetting = false;
		return;
	}
	
	switch (Pointers[IndexCurrentSetting])
	{
		case OverfeedPointer:
			if (PlusPushed && ChangableValue < 200) ChangableValue++;
			if (MinusPushed && ChangableValue > -200) ChangableValue--;
			break;
		case SetpointPointer:
			if (PlusPushed && ChangableValue < 5) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case PulseDurationPointer:
			if (PlusPushed && ChangableValue < 3) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case PulsesIntervalPointer:
			if (PlusPushed && ChangableValue < 60) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case StartDelayPointer:
			if (PlusPushed && ChangableValue < 300) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case FactorAPointer:
		case FactorBPointer:
		case FactorSpeedPointer:
		case FactorMeasurePointer:
		case FactorEstimatePointer:
			if (PlusPushed && ChangableValue < 99) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		default:
			InterfaceMode = Settings;
			DisplayMode = Settings;
			IndexCurrentSetting = 0;
			ChangableValue = 0;
			break;
	}
	
	PlusPushed = false;
	MinusPushed = false;
}

bool Start()
{
	Timer0(true);
	Timer1(true);
	CurrentError = Off;
	DisplayMode = Current;
	DisplaySettingCount = DisplayTimeout;
	return true;
}

bool Stop()
{
	LedOff;
	PulseOff;
	FaultOff;
	Timer0(false);
	Timer1(false);
	SetDirection(0, true);	// reset function counters
	Kalman(0, true);
	if (!DisplaySettingCount && DisplayMode != Error) DisplayMode = Off;
	return false;
}

int main(void)
{
	unsigned short startDelayCount = 0;
	short f1 = 0, f2 = 0, ratio = 0, difference = 0;

	Initialization();

	while(1)
	{
		if (HandleAfter8ms)
		{
			if (DisplayMode == Current)	 Print(&difference);
			if (DisplayMode == Settings) Print(&Pointers[IndexCurrentSetting]);
			if (DisplayMode == Setting)	 Print(&ChangableValue);
			if (DisplayMode == Off && !(Check(PORTC, PORTC4) && Check(PORTC, PORTC5))) PORTC |= 0x30;
			
			HandleAfter8ms = false;
		}
		
		if (HandleAfter200ms)
		{	
			 ControlButtons();
			 
			 if (InterfaceMode == Setting)
			 {
				 if (Blink) DisplayMode = Off;
				 else DisplayMode = Setting;
				 Blink = !Blink;
			 }
			 
			 if (InterfaceMode == Common)   CommonControl();
			 if (InterfaceMode == Settings) SettingsControl();
			 if (InterfaceMode == Setting)  SettingControl();
			 
			 if (SettingExitCount > 0 && BtnMinus) SettingExitCount = 0;
			 
			 if (ManualControl && BtnPlus && BtnMinus)
			 {
				 PulseOff;
				 ManualControl = false;
			 }
			 
			 HandleAfter200ms = false;
		}
		
		if (HandleAfterSecond)	 
		{
			if (!BtnMinus && InterfaceMode == Settings) SettingExitCount++;
			
			if (SettingExitCount >= 3) 
			{
				SettingExitCount = 0;
				IndexCurrentSetting = 0;
				InterfaceMode = Common;
				if (CurrentError) DisplayMode = Error;
				else DisplayMode = Off;
				LoadSettings();
			}
			
			if (Running && !IsRun) 
			{
				IsRun = Start();
				HandleAfterSecond = false;
				startDelayCount = StartDelay;
				continue;
			}
			
			if (!Running && IsRun) IsRun = Stop();
			
			if (IsRun)						 // handle data after startDelay
			{
				LedInv;						 // operating LED	inversion

				f1 = (short)((TCNT0 + Timer0_OverflowCount*256)*FactorA);        // calculation f1	(aramid)
				f2 = (short)(((TCNT1 + Timer1_OverflowCount*65535L)/5)*FactorB);	 // calculation f2	(polyamide)
				ratio = GetRatio(&f1, &f2);
				difference = Kalman(Overfeed - ratio, false);
				
				Transmit(&f1, &f2, &difference);
				
				if (!startDelayCount)
				{
					InstantValuesCountrol(&f1, &f2);
					SetDirection(&difference, false);		// calculation average ratio
				}
				
				TCNT0 = 0;					  // reset count registers after receiving values
				TCNT1 = 0;
				Timer0_OverflowCount = 0;
			}
			
			if (startDelayCount) startDelayCount--;  // start delay counter

			if (DisplaySettingCount)
			{
				DisplaySettingCount--;
				if (!DisplaySettingCount) DisplayMode = Off;
			}
			
			if (DisplayMode == Error)	PrintError();

			HandleAfterSecond = false;
		}
	}
}