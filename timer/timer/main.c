/*
 * timer.c
 *
 * Created: 14.06.2019 08:27:07
 * Author : sakyazi
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

int extraTime=0;


int main(void)
{
	DDRB = 0xFF;
	TCCR0A =(1 << WGM01);
	OCR0A=234;
	TIMSK0=(1<< OCIE0A);
	sei();
	TCCR0B = (1<<CS02) | (1<<CS00);
	
	
	while(1)
	{
		
	}
	
}

ISR(TIMER0_COMPA_vect)
{
	extraTime++;
	if(extraTime > 0)
	{
		PORTB ^= (1<< PORTB5);
	}
}



