/*
 * A328P_Industrial_Autoassembling.c
 *
 * Created: 27.12.2022 21:34:47
 * Author : Abramov IV
 */ 

#define Check(REG,BIT) (REG & (1<<BIT))	   
#define Inv(REG,BIT)   (REG ^= (1<<BIT))	
#define High(REG,BIT)  (REG |= (1<<BIT))	
#define Low(REG,BIT)   (REG &= ~(1<<BIT))	

#define Running		(!Check(PINB, PINB0))  

#define Led			Check(PORTB, PORTB1)	
#define LedOn		High(PORTB, PORTB1)
#define LedOff		Low(PORTB, PORTB1)
#define LedInv		Inv(PORTB, PORTB1)

#define Fault		Check(PORTB, PORTB2)	
#define FaultOn		High(PORTB, PORTB2)
#define FaultOff	Low(PORTB, PORTB2)

#define Units		Check(PORTC, PORTC4)
#define Dozens		Check(PORTC, PORTC5)

#define Dot			Check(PORTD, PORTD2)
#define DotOn		High(PORTD, PORTD2)
#define DotOff		Low(PORTD, PORTD2)

#define PulsePin	Check(PORTD, PORTD3)	

#define BtnPlus		Check(PIND, PIND6)     
#define BtnMinus	Check(PIND, PIND7)     

#define Pulse		Check(TCCR2A, COM2B1)  
#define PulseOn		High(TCCR2A, COM2B1)
#define PulseOff	Low(TCCR2A, COM2B1)

#define Off				  0				  
#define On				  1
#define Init			  2
#define Setting			  3
#define	Current			  4
#define Error			  5
#define Common			  6
#define Settings		  7

#define Locked			  100
#define Right	 		  135
#define Left			  250				

#define OverfeedPointer			0
#define SetpointPointer			2
#define HysteresisUpPointer		4
#define HysteresisDownPointer   6
#define PulseDurationPointer    8
#define PulsesIntervalPointer	10
#define StartDelayPointer		12
#define FactorAPointer			14
#define FactorBPointer			16
#define DividerAPointer			18
#define DividerBPointer			20
#define FactorMeasurePointer	22
#define FactorEstimatePointer	24
#define FactorSpeedPointer		26
#define DisplayTimeoutPointer   28
#define IsTransmitPointer		30
#define MeasuresLimitPointer	32
#define MoveLackLimitPointer	34
#define OvertimeLimitPointer	36

#define MemoryGetterPointer		90
#define VarsGetterPointer		92
#define DefaultSetterPointer	99

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

const unsigned short ERROR_A = 1;
const unsigned short ERROR_B = 2;
const unsigned short ERROR_C = 3;
const unsigned short ERROR_MOTOR = 4;
const unsigned short ERROR_OVERTIME_MOVING = 5;

short Pointers[] = { OverfeedPointer, SetpointPointer, HysteresisUpPointer, HysteresisDownPointer, PulseDurationPointer, 
					 PulsesIntervalPointer, StartDelayPointer, FactorAPointer, FactorBPointer, DividerAPointer, DividerBPointer,
					 FactorMeasurePointer, FactorEstimatePointer, FactorSpeedPointer, DisplayTimeoutPointer,
					 IsTransmitPointer,	MeasuresLimitPointer, MoveLackLimitPointer, OvertimeLimitPointer,
					 MemoryGetterPointer, VarsGetterPointer, DefaultSetterPointer };
					 
short Defaults[] = { 0, 1, 4, -4, 1, 5, 40, 0, 0, 1, 1, 10, 10, 45, 0, 0, 10, 10, 90 };
					 
short ChangableValue = 0;

short Overfeed = 0;
short Setpoint = 0;       
short HysteresisUp = 0;   
short HysteresisDown = 0;		  
unsigned short PulseDuration = 0;  
unsigned short PulsesInterval = 0; 
unsigned short StartDelay = 0;     
float FactorA = 0;				   
float FactorB = 0;				   
unsigned short DividerA = 0;	  
unsigned short DividerB = 0;	  
unsigned short FactorMeasure = 0; 
unsigned short FactorEstimate = 0;
float FactorSpeed = 0;             
unsigned short DisplayTimeout = 0; 
unsigned short IsTransmit = 0;	   
unsigned short MeasuresLimit = 0;  
unsigned short MoveLackLimit = 0;  
unsigned short OvertimeLimit = 0;  

unsigned short Timer0_OverflowCount = 0;  
unsigned short Timer1_OverflowCount = 0;  
unsigned short Timer2_OverflowCount = 0;  
bool HandleAfterSecond = false;			  
bool HandleAfter200ms = false;
bool HandleAfter8ms = false;					 

unsigned short InterfaceMode = Common;	 
unsigned short DisplayMode = Off;
unsigned short IndexCurrentSetting = 0;
unsigned short DisplayTimeoutCount = 0;
unsigned short SettingExitCount = 0;
bool Blink = false;
bool SaveSetting = false;
bool ManualControl = false;

bool PlusPushed = false;
bool MinusPushed = false;

unsigned short PulseLockCount = 0;
unsigned short CurrentError = 0;

bool IsReloadSettings = false;
bool IsRun = false;

void Timer0(bool enable)
{
	if (enable)
	{
		TCCR0B = (1 << CS02)|(1 << CS01)|(1 << CS00);	 
		High(TIMSK0, TOIE0);							
		TCNT0 = 0;
		return;
	}
	
	Low(TIMSK0, TOIE0);
	TCCR0B = 0x00;										  
}

ISR(TIMER0_OVF_vect)
{
	Timer0_OverflowCount++;	  
}

void Timer1(bool enable)
{
	if (enable)
	{
		TCCR1B = (1 << CS12)|(1 << CS11)|(1 << CS10);   
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
	Timer2_OverflowCount++;					 
	HandleAfter8ms = true;
	
	if (Timer2_OverflowCount % 25 == 0) HandleAfter200ms = true;
	
	if (Timer2_OverflowCount >= 125)		 
	{
		HandleAfterSecond = true;			 
		Timer2_OverflowCount = 0;			 
	}

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

void Transmit(short *p_a, short *p_b, short *p_d)
{
	static char a[8] = { 0 }, b[8] = { 0 }, d[8] = { 0 };
	static char buffer[64] = { 0 };
	
	sprintf(a, "A%d$ ", *p_a);
	sprintf(b, "P%d$ ", *p_b);
	sprintf(d, "D%d", *p_d);
	
	strcat(buffer, a);
	strcat(buffer, b);
	strcat(buffer, d);
	
	TxString(d);
	
	buffer[0] = '\0';
}

short Kalman(short value, bool reset)
{
	static float estimateVariation = 0;
	static float currentEstimate = 0;
	static float lastEstimate = 0;
	static float Gain = 0;
	
	if (reset)
	{
		estimateVariation = FactorEstimate;
		currentEstimate = 0;
		lastEstimate = 0;
		Gain = 0;
	}
	
	Gain = estimateVariation / (estimateVariation + FactorMeasure);
	currentEstimate = lastEstimate + Gain * (value - lastEstimate);
	estimateVariation = (1.f - Gain) * estimateVariation + fabs(lastEstimate - currentEstimate) * FactorSpeed;
	lastEstimate = currentEstimate;
	return (short)currentEstimate;
}

void UploadMemory()
{
	short size = sizeof(Defaults)/sizeof(short);
	char value[16] = { 0 };
	
	TxString("\r\n");
	
	for (int i = 0; i < size; i++)
	{
		sprintf(value, "%d$\r\n", eeprom_read_word((uint16_t*)Pointers[i]));
		TxString(value);
	}	
}

void UploadVariables()
{
	char value[16] = { 0 };
	
	sprintf(value, "\r\n%d$\r\n", Overfeed);
	TxString(value);
	
	sprintf(value, "%d$\r\n", Setpoint);
	TxString(value);
	
	sprintf(value, "%d$\r\n", HysteresisUp);
	TxString(value);
	
	sprintf(value, "%d$\r\n", HysteresisDown);
	TxString(value);

	sprintf(value, "%d$\r\n", PulseDuration);
	TxString(value);

	sprintf(value, "%d$\r\n", PulsesInterval);
	TxString(value);

	sprintf(value, "%d$\r\n", StartDelay);
	TxString(value);

	sprintf(value, "%.3f$\r\n", FactorA);
	TxString(value);

	sprintf(value, "%.3f$\r\n", FactorB);
	TxString(value);
	
	sprintf(value, "%d$\r\n", DividerA);
	TxString(value);

	sprintf(value, "%d$\r\n", DividerB);
	TxString(value);

	sprintf(value, "%d$\r\n", FactorMeasure);
	TxString(value);

	sprintf(value, "%d$\r\n", FactorEstimate);
	TxString(value);

	sprintf(value, "%.3f$\r\n", FactorSpeed);
	TxString(value);
	
	sprintf(value, "%d$\r\n", DisplayTimeout);
	TxString(value);
	
	sprintf(value, "%d$\r\n", IsTransmit);
	TxString(value);
	
	sprintf(value, "%d$\r\n", MeasuresLimit);
	TxString(value);
	
	sprintf(value, "%d$\r\n", MoveLackLimit);
	TxString(value);
	
	sprintf(value, "%d$\r\n", OvertimeLimit);
	TxString(value);
}

void SetDefaultSettings()
{
	short size = sizeof(Defaults)/sizeof(short);
	
	for (int i = 0; i < size; i++) 
		eeprom_update_word((uint16_t*)Pointers[i], Defaults[i]);
}

void LoadSettings()
{
	Overfeed = (eeprom_read_word((uint16_t*)OverfeedPointer))*-1;
	Setpoint = eeprom_read_word((uint16_t*)SetpointPointer);
	HysteresisUp = eeprom_read_word((uint16_t*)HysteresisUpPointer);
	HysteresisDown = eeprom_read_word((uint16_t*)HysteresisDownPointer);
	PulseDuration = eeprom_read_word((uint16_t*)PulseDurationPointer);
	PulsesInterval = eeprom_read_word((uint16_t*)PulsesIntervalPointer);
	StartDelay = eeprom_read_word((uint16_t*)StartDelayPointer);
	FactorA = 1.-(float)eeprom_read_word((uint16_t*)FactorAPointer)/1000.f;
	FactorB = 1.-(float)eeprom_read_word((uint16_t*)FactorBPointer)/1000.f;
	DividerA = eeprom_read_word((uint16_t*)DividerAPointer);
	DividerB = eeprom_read_word((uint16_t*)DividerBPointer);
	FactorMeasure = eeprom_read_word((uint16_t*)FactorMeasurePointer);
	FactorEstimate = eeprom_read_word((uint16_t*)FactorEstimatePointer);
	FactorSpeed = (float)eeprom_read_word((uint16_t*)FactorSpeedPointer)/1000.f;
	DisplayTimeout = eeprom_read_word((uint16_t*)DisplayTimeoutPointer);
	IsTransmit = eeprom_read_word((uint16_t*)IsTransmitPointer);
	MeasuresLimit = eeprom_read_word((uint16_t*)MeasuresLimitPointer);
	MoveLackLimit = eeprom_read_word((uint16_t*)MoveLackLimitPointer);   
	OvertimeLimit = eeprom_read_word((uint16_t*)OvertimeLimitPointer);  
}

void Initialization()
{
	DDRB = 0b00000110;					
	PORTB = 0b00111001;
	
	DDRC = 0b00111111;
	PORTC = 0b11000000;
	
	DDRD = 0b00001100;
	PORTD = 0b11110011;
	
	LoadSettings();

	Kalman(0, true);
	Timer2(true);	
	USART(Init);
	USART(On);
	sei();			
	
	wdt_enable(WDTO_2S);
}

short GetRatio(short *p_a, short *p_b)
{
	if (!*p_a && !*p_b) return 0;
	
	if (*p_a <= *p_b) return (1-(float)*p_a/(*p_b == 0 ? 1 : *p_b))*-1000;
	else return (1-(float)*p_b/(*p_a))*1000;
}

void SetDirection(short *p_d, bool isReset)
{
	static unsigned short motorState = Locked, stepCount = 0, stepsInterval = 0;
	static unsigned short overtimeCount = 0, moveLackCount = 0, lastDifference = 0;
	
	if (isReset)
	{
		motorState = Locked;
		stepCount = 0;
		stepsInterval = 0;
		overtimeCount = 0;
		return;
	}
	
	if (stepsInterval)	 
	{
		stepsInterval--;
		return;
	}
	
	if (abs(*p_d) <= Setpoint)   
	{
		if (motorState == Locked) return;
		if (overtimeCount) overtimeCount = 0;
		if (moveLackCount) moveLackCount = 0;
		
		PulseOff;
		motorState = Locked;
		stepCount = 0;
		stepsInterval = PulsesInterval;
		return;
	}
	
	if (stepCount)	 
	{
		stepCount--;
		
		if (!stepCount)	   
		{
			PulseOff;
			stepsInterval = PulsesInterval;
		}
		
		return;
	}
	
	if (CurrentError == ERROR_A || CurrentError == ERROR_B || CurrentError == ERROR_C) return;
	
	if ((*p_d >= HysteresisUp || *p_d <= HysteresisDown) && MoveLackLimit)
	{
		if (motorState == Locked) lastDifference = abs(*p_d);
		else
		{
			if (abs(lastDifference - abs(*p_d)) < 2) moveLackCount++;
			else 
			{
				moveLackCount = 0;
				lastDifference = abs(*p_d);
			}
		}
	}
	
	if (moveLackCount >= MoveLackLimit)
	{
		DisplayMode = Error;
		CurrentError = ERROR_MOTOR;
		moveLackCount = 0;
		FaultOn;
		return;
	}
	
	if (PulseDuration)
	{
		if (*p_d >= HysteresisUp || (*p_d > 0 && motorState != Locked))
		{
			OCR2B = Right;
			motorState = Right;
			if (OvertimeLimit) overtimeCount++;
			stepCount = PulseDuration;
			PulseOn;
			return;
		}
		
		if (*p_d <= HysteresisDown || (*p_d < 0 && motorState != Locked))
		{
			OCR2B = Left;
			motorState = Left;
			if (OvertimeLimit) overtimeCount++;
			stepCount = PulseDuration;
			PulseOn;
		}
	}
	
	if (overtimeCount >= OvertimeLimit)
	{
		DisplayMode = Error;
		CurrentError = ERROR_OVERTIME_MOVING;
		overtimeCount = 0;
		FaultOn;
		return;
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
	
	if (Dozens)
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
		
		if (InterfaceMode == Settings) { DotOff; return; }
		if (Pointers[IndexCurrentSetting] == OverfeedPointer ||
			Pointers[IndexCurrentSetting] == FactorAPointer  ||
			Pointers[IndexCurrentSetting] == FactorBPointer)
		{
			if (Dot)
			{
				if (uvalue >= 100) DotOff;
			}
			else
			{
				if (uvalue < 100) DotOn;
			}
		}
		else DotOff;
	}
}

void PrintError()
{
	if (!(Check(PORTC, PORTC4) | Check(PORTC, PORTC5)))
	{
		PORTC = 0xE0 | CurrentError;
		if (Dot) DotOff;
		return;
	}
	
	PORTC &= 0xC0;
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
}

void ModesControl()
{
	if (PlusPushed && MinusPushed)
	{
		if (InterfaceMode == Common)
		{
			PulseOff;
			InterfaceMode = Settings;
			DisplayMode = Settings;
			DisplayTimeoutCount = 0;
		}
		else if (InterfaceMode == Settings)
		{
			switch (Pointers[IndexCurrentSetting])
			{
				case DefaultSetterPointer:
				SetDefaultSettings();
				IsReloadSettings = true;
				break;
				case MemoryGetterPointer:
				UploadMemory();
				break;
				case VarsGetterPointer:
				UploadVariables();
				break;
				default:
				InterfaceMode = Setting;
				DisplayMode = Setting;
				ChangableValue = eeprom_read_word((uint16_t*)Pointers[IndexCurrentSetting]);
				break;
			}
		}
		else
		{
			SaveSetting = true;
		}
		
		PlusPushed = false;
		MinusPushed = false;
	}
}

void InstantValuesCountrol(short *p_a, short *p_b, short *p_d)
{
	static unsigned short errorCount = 0;
	
	if ((*p_a < 10 || *p_b < 10) && MeasuresLimit)
	{
		errorCount++;
		if (*p_a < 10) CurrentError = ERROR_A;
		if (*p_b < 10) CurrentError = ERROR_B;
		if (*p_a < 10 && *p_b < 10) CurrentError = ERROR_C;
		if (errorCount >= MeasuresLimit)
		{
			FaultOn;
			DisplayMode = Error;
			errorCount = 0;
		}
		return;	
	}
	
	if (errorCount) 
	{
		errorCount = 0;
		CurrentError = Off;
	}
}

void CommonControl()
{
	if (IsRun && DisplayMode == Off && (PlusPushed || MinusPushed))
	{
		DisplayMode = Current;
		DisplayTimeoutCount = DisplayTimeout;
		PlusPushed = false;
		MinusPushed = false;
		return;	
	}
	
	if (PlusPushed)
	{
		if (OCR2B != Left || !Pulse)
		{
			OCR2B = Left;
			PulseOn;
		}	
		
		ManualControl = true;
		PlusPushed = false;
	}
	
	if (MinusPushed)
	{
		if (OCR2B != Right || !Pulse)
		{
			OCR2B = Right;
			PulseOn;
		}
		
		ManualControl = true;
		MinusPushed = false;
	}
}

void SettingsControl()
{	
	static short pcount = (sizeof(Pointers)/sizeof(Pointers[0]))-1;
	
	if (PlusPushed)
	{
		if (IndexCurrentSetting < pcount) IndexCurrentSetting++;
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
		case HysteresisUpPointer:
			if (PlusPushed && ChangableValue < 5) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case HysteresisDownPointer:
			if (PlusPushed && ChangableValue < 0) ChangableValue++;
			if (MinusPushed && ChangableValue > -5) ChangableValue--;
			break;
		case PulseDurationPointer:
			if (PlusPushed && ChangableValue < 3) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case PulsesIntervalPointer:
			if (PlusPushed && ChangableValue < 60) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case DividerAPointer:
		case DividerBPointer:
		case MoveLackLimitPointer:
		case OvertimeLimitPointer:
			if (PlusPushed && ChangableValue < 99) ChangableValue++;
			if (MinusPushed && ChangableValue > 1) ChangableValue--;	
			break;
		case FactorAPointer:
		case FactorBPointer:
			if (PlusPushed && ChangableValue < 999) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case StartDelayPointer:
		case FactorSpeedPointer:
		case FactorMeasurePointer:
		case FactorEstimatePointer:
		case DisplayTimeoutPointer:
		case MeasuresLimitPointer:
			if (PlusPushed && ChangableValue < 99) ChangableValue++;
			if (MinusPushed && ChangableValue > 0) ChangableValue--;
			break;
		case IsTransmitPointer:
			if (PlusPushed && ChangableValue < 1) ChangableValue++;
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
	DisplayTimeoutCount = DisplayTimeout;
	return true;
}

bool Stop()
{
	LedOff;
	PulseOff;
	FaultOff;
	Timer0(false);
	Timer1(false);
	SetDirection(0, true);	
	Kalman(0, true);
	if (DisplayMode != Error) 
	{
		DisplayMode = Off;
		DisplayTimeoutCount = 0;
	}
	return false;
}

int main(void)
{
	unsigned short startDelayCount = 0, measureDelayCount = 0;
	short a = 0, b = 0, r = 0, d = 0;	

	Initialization();

	while(1)
	{
		if (HandleAfter8ms)
		{
			if (DisplayMode == Current)	 Print(&d);
			if (DisplayMode == Settings) Print(&Pointers[IndexCurrentSetting]);
			if (DisplayMode == Setting)	 Print(&ChangableValue);
			if (DisplayMode == Off && (Check(PORTC, PORTC4) || Check(PORTC, PORTC5))) PORTC &= 0xC0;
			
			HandleAfter8ms = false;
		}
		
		if (HandleAfter200ms)
		{	
			 ControlButtons();
			 ModesControl();
			 
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
			
			if (SettingExitCount >= 3 || IsReloadSettings) 
			{
				SettingExitCount = 0;
				IndexCurrentSetting = 0;
				InterfaceMode = Common;
				IsReloadSettings = false;
				
				if (IsRun) 
				{
					DisplayMode = Current;
					DisplayTimeoutCount = DisplayTimeout;
				}
				else DisplayMode = Off;
				if (CurrentError) DisplayMode = Error;
				LoadSettings();
			}
			
			if (Running && !IsRun) 
			{
				IsRun = Start();
				HandleAfterSecond = false;
				startDelayCount = StartDelay;
				measureDelayCount = 5;
				a = 0; b = 0; r = 0; d = 0;
				continue;
			}
			
			if (!Running && IsRun) IsRun = Stop();
			
			if (IsRun)						 
			{
				LedInv;					

				if (!measureDelayCount)
				{
					a = (short)((TCNT0 + Timer0_OverflowCount*256)/DividerA)*FactorA;
					b = (short)((TCNT1 + Timer1_OverflowCount*65535L)/DividerB)*FactorB;
					r = GetRatio(&a, &b);
					d = Kalman(Overfeed - r, false);
					
					if (IsTransmit) Transmit(&a, &b, &d);
				}
				
				if (!startDelayCount)
				{
					InstantValuesCountrol(&a, &b, &d);
					SetDirection(&d, false);		
				}
				
				TCNT0 = 0;					 
				TCNT1 = 0;
				Timer0_OverflowCount = 0;
			}
			
			if (measureDelayCount) measureDelayCount--;
			if (startDelayCount) startDelayCount--;  

			if (DisplayTimeoutCount)
			{
				DisplayTimeoutCount--;
				if (!DisplayTimeoutCount) DisplayMode = Off;
			}
			
			if (DisplayMode == Error)	PrintError();

			HandleAfterSecond = false;
		}
		
		wdt_reset();
	}
}