/*
 * GccApplication1.c
 *
 * Created: 13.06.2019 10:59:38
 * Author : sakyazi
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>



int main(void)
{
    DDRD=0XFF;
  
  
     while(1)
     { 
		 //PIND |= (1<< PIND1);
		 PORTD=0XFF;
		 _delay_ms(1000);
		PORTD=0X00;
		_delay_ms(1000);

     }
}

