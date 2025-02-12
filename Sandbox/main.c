/*
 * Sandbox.c
 *
 * Created: 12.02.2025 15:24:31
 * Author : Admin
 */ 

#define BIT_READ(REG, BIT)   (REG & (1 << BIT))
#define BIT_SET(REG, BIT)    (REG |= (1 << BIT))
#define BIT_CLEAR(REG, BIT)  (REG &= ~(1 << BIT))
#define BIT_TOGGLE(REG, BIT) (REG ^= (1 << BIT))

#define LED			BIT_READ(PORTB, 5)
#define LED_ON		BIT_SET(PORTB, 5)
#define LED_OFF		BIT_CLEAR(PORTB, 5)
#define LED_TOGGLE	BIT_TOGGLE(PORTB, 5)

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

void timer0_Init(void);
void timer1_Init(void);

int main(void)
{
	timer0_Init();
	timer1_Init();
	
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

void timer0_Init(void)
{
	TCCR0A = (1 << COM0A1)|(1 << WGM01)|(1 << WGM00);
	TCCR0B = (1 << CS02)|(0 << CS01)|(0 << CS00); // divider 256, 1 tick = 16 us
	OCR0A = 127;
	
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
	operationTimer.ms++;
	
	if (operationTimer.ms % 200 == 0) operationTimer.ms200++;
	if (operationTimer.ms % 1000 == 0) operationTimer.s++;

	operationTimer.isr = true;
	TCNT1 = 64911;
}