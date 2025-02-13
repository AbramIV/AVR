/*
 * Sandbox.c
 *
 * Created: 12.02.2025 15:24:31
 * Author : Admin
 */ 

#define F_CPU 16000000L

#define BIT_READ(REG, BIT)   (REG & (1 << BIT))
#define BIT_SET(REG, BIT)    (REG |= (1 << BIT))
#define BIT_CLEAR(REG, BIT)  (REG &= ~(1 << BIT))
#define BIT_TOGGLE(REG, BIT) (REG ^= (1 << BIT))

#define KEY			BIT_READ(PINB, 0)		    

#define LED			BIT_READ(PORTB, 5)
#define LED_ON		BIT_SET(PORTB, 5)
#define LED_OFF		BIT_CLEAR(PORTB, 5)
#define LED_TOGGLE	BIT_TOGGLE(PORTB, 5)

#define PWM_OUT		 BIT_READ(PORTD, 6)
#define PWM_OUT_ON	 BIT_SET(PORTD, 6)
#define PWM_OUT_OFF	 BIT_CLEAR(PORTD, 6)

#define PWM		BIT_READ(TCCR0B, CS02)
#define PWM_ON  BIT_SET(TCCR0B, CS02)
#define PWM_OFF BIT_CLEAR(TCCR0B, CS02)

#define ADC_START  BIT_SET(ADCSRA, ADSC);

#define PWM_MAX		  255
#define ADC_MAX		  1024
#define AVG_MAX		  32
#define DURATION_MAX  10000

#define OFF  0
#define ON   1
#define INIT 2

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

struct sTimer
{
	uint16_t ms, ms10, ms200, s;
	bool ovf;
} timer = {0, 0, 0, 0, false};

struct sVariator
{
	uint16_t power;
	uint16_t duration;
	uint16_t count;
	uint16_t step;
	uint16_t delay;
	bool active;
	bool direction;
} variator = {0, 0, 0, 0, 0, false, false};

struct sConverter
{
	uint16_t value;
	bool ready;
} converter = {0, false};

bool keyPushed = false;

void GPIO_Init(void);
void timer0_Init(void);
void timer1_Init(void);
void ADC_Init(void);

float durationAvg(uint16_t value, bool reset);
float powerAvg(uint16_t value, bool reset);
void keyControl(void);

int main(void)
{	
	GPIO_Init();
	timer0_Init();
	timer1_Init();
	ADC_Init();
	
	sei();
	
	PWM_OFF;
	
    while (1)
    {
		if (!variator.delay && !variator.active && !KEY) keyPushed = true;
		
		if (keyPushed)
		{
			variator.delay = AVG_MAX;
			keyPushed = false;
		}
		
		if (variator.active && (variator.count >= variator.step))
		{
			if (variator.direction) OCR0A++;
			else OCR0A--;
			
			if (OCR0A >= variator.power || OCR0A == 0)
			{
				LED_OFF;
				if (!variator.direction) PWM_OFF;
				variator.active = false;
			}	
			
			variator.count = 0;
		}
		
		if (converter.ready && !variator.active)
		{
			if (ADMUX == 0x40)
			{
				variator.duration = (durationAvg(converter.value, false)/ADC_MAX)*DURATION_MAX;
				ADMUX++;
			}
			else
			{
				variator.power = (powerAvg(converter.value, false)/ADC_MAX)*PWM_MAX;
				ADMUX--;
			}
			
			converter.ready = false;
		}
		
		if (timer.ms10)
		{
			if (!variator.active) ADC_START;
			
			timer.ms10 = 0;
		}
    }
}

void GPIO_Init(void)
{
	DDRB = 0b00111110;
	PORTB = 0b00000001;
	
	DDRD = 0xFF;
	PORTD = 0x00;
	
	DDRC = 0x00;
	PORTC = 0x00;
}

void timer0_Init(void)
{
	TCCR0A = (1 << COM0A1)|(1 << WGM01)|(1 << WGM00);
	TCCR0B = (1 << CS02)|(0 << CS01)|(0 << CS00); // divider 256, 1 tick = 16 us
	OCR0A = 0;
}

void timer1_Init(void)
{
	TCCR1B = (1 << CS11);  // T1clk -> fclk/8
	TIMSK1 = (1 << TOIE1); // overflow interrup -> enable
	TCNT1 = 63536; // interrupt in 2000 ticks / 1 tick -> 0.5 us
}

void ADC_Init(void)
{
	ADCSRA = 0x8F;
	ADMUX = 0x40;
}

float durationAvg(uint16_t value, bool reset)
{
	static float values[AVG_MAX];
	static unsigned short index = 0;
	static float result;
	
	if (reset)
	{
		for (int i=0; i < AVG_MAX; i++) values[i] = 0;
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % AVG_MAX;
	
	return result/AVG_MAX;
}

float powerAvg(uint16_t value, bool reset)
{
	static float values[AVG_MAX];
	static unsigned short index = 0;
	static float result;
	
	if (reset)
	{
		for (int i=0; i < AVG_MAX; i++) values[i] = 0;
		result = 0;
		index = 0;
		return 0;
	}
	
	result += value - values[index];
	values[index] = value;
	index = (index + 1) % AVG_MAX;
	
	return result/AVG_MAX;
}

ISR(TIMER1_OVF_vect)
{
	timer.ms++;
	
	if (variator.delay) 
	{
		variator.delay--;
		
		if (!variator.delay) 
		{
			LED_ON;
			variator.active = true;
			variator.direction = PWM ? false : true;
			variator.step = variator.duration/variator.power;
			if (variator.direction) PWM_ON;
		}
	}
	
	if (variator.active) variator.count++;
	
	if (timer.ms % 10 == 0) timer.ms10++;
	if (timer.ms % 1000 == 0) 
	{
		timer.s++;
		timer.ms = 0;
	}

	TCNT1 = 63536;
}

ISR(ADC_vect)
{
	converter.value = ADCW;
	converter.ready = true;
}