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

struct MainTimer
{
	uint16_t ms, ms2, ms5, ms10, ms100, ms200, ms500, s;
	bool isr;
} operationTimer = {0, 0, 0, 0, 0, 0, 0, 0, false};

uint16_t incTime = 8;
uint16_t decTime = 3;
uint16_t incStepCount = 0;
uint16_t decStepCount = 0;
uint16_t incStep = 0;
uint16_t decStep = 0;
bool inc = false;
bool dec = false;

void GPIO_Init(void);
void timer0_Init(void);
void timer1_Init(void);
uint16_t getStep(uint16_t period);

int main(void)
{
	GPIO_Init();
	
	timer0_Init();
	timer1_Init();
	
	PWM_ON;
	
	sei();
	
	incStep = getStep(incTime);
	inc = true;
	
    while (1) 
    {
		if (operationTimer.ms200)
		{
			LED_TOGGLE;
			operationTimer.ms200 = 0;
		}
		
		
		if (operationTimer.s)
		{
			
			operationTimer.s = 0;
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
	OCR0A = 0;
	
	TCCR0B = 0x00;
}

void timer1_Init(void)
{
	TCCR1B = (1 << CS11);  // T1clk -> fclk/8
	TIMSK1 = (1 << TOIE1); // overflow interrup -> enable
	TCNT1 = 63536; // interrupt in 2000 ticks / 1 tick -> 0.5 us
}

uint16_t getStep(uint16_t period_ms)
{
	return period_ms/PWM_STEPS_MAX;
}

ISR(TIMER1_OVF_vect)
{
	operationTimer.ms++;
	if (inc) incStepCount++;
	
	if (inc && (incStepCount == incTime))
	{
		if (OCR0A == 255) inc = false;
		OCR0A++;
		incStepCount = 0;
	}
	
	if (operationTimer.ms % 200 == 0) operationTimer.ms200++;
	if (operationTimer.ms % 1000 == 0) 
	{
		operationTimer.s++;
		operationTimer.ms = 0;
	}

	operationTimer.isr = true;
	TCNT1 = 63536;
}