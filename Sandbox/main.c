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

#define LED			BIT_READ(PORTB, 5)
#define LED_ON		BIT_SET(PORTB, 5)
#define LED_OFF		BIT_CLEAR(PORTB, 5)
#define LED_TOGGLE	BIT_TOGGLE(PORTB, 5)

#define PWM_ON  BIT_SET(TCCR0B, CS02)
#define PWM_OFF BIT_CLEAR(TCCR0B, CS02)

#define INC_DURATION_MAX 10000
#define DEC_DURATION_MAX 10000
#define PWM_STEPS_MAX    255

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

typedef struct OperationalTimer
{
	uint16_t ms, ms2, ms5, ms10, ms100, ms200, ms500, s;
	bool isr;
} OperationalTimer;

struct sVariator
{
	 uint16_t duration;
	 uint16_t count;
	 uint16_t step;
	 bool active;
	 bool isInc;
} variator = {0, 0, 0, false};

OperationalTimer mainTimer = {0, 0, 0, 0, 0, 0, 0, 0, false};

//sVariator increment = {0, 0, 0, false};
//sVariator decrement = {0, 0, 0, false};

void GPIO_Init(void);
void timer0_Init(void);
void timer1_Init(void);

int main(void)
{	
	GPIO_Init();
	
	timer0_Init();
	timer1_Init();
	
	sei();
	
	variator.duration = 8000;
	variator.step = variator.duration/OCR0A;
	
    while (1) 
    {
		if (mainTimer.ms200)
		{
			LED_TOGGLE;
			mainTimer.ms200 = 0;
		}
		
		if (mainTimer.s)
		{
			
			mainTimer.s = 0;
		}
    }
}

void GPIO_Init(void)
{
	DDRB = 0b00111111;
	PORTB = 0b00000000;
	
	DDRD = 0xFF;
	PORTD = 0x00;
	
	DDRC = 0x00;
	PORTC = 0xFF;
}

void timer0_Init(void)
{
	TCCR0A = (1 << COM0A1)|(1 << WGM01)|(1 << WGM00);
	TCCR0B = (1 << CS02)|(0 << CS01)|(0 << CS00); // divider 256, 1 tick = 16 us
	OCR0A = 255;
	
	TCCR0B = 0x00;
}

void timer1_Init(void)
{
	TCCR1B = (1 << CS11);  // T1clk -> fclk/8
	TIMSK1 = (1 << TOIE1); // overflow interrup -> enable
	TCNT1 = 63536; // interrupt in 2000 ticks / 1 tick -> 0.5 us
}

ISR(TIMER1_OVF_vect)
{
	mainTimer.ms++;
	
	if (variator.active) variator.count++;
	
	if (variator.count == variator.step)
	{
		if (OCR0A == 255 || !OCR0A) 
		{
			variator.active = false;
			variator.count = 0;
		}
		else
		{
			if (variator.isInc) OCR0A++;
			else OCR0A--;
		}
	}
	
	if (mainTimer.ms % 200 == 0) mainTimer.ms200++;
	if (mainTimer.ms % 1000 == 0) 
	{
		mainTimer.s++;
		mainTimer.ms = 0;
	}

	mainTimer.isr = true;
	TCNT1 = 63536;
}