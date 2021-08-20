/*
 * usart1.c
 *
 * Created: 17.06.2019 09:45:25
 * Author : sakyazi
 */ 


#define F_CPU 16000000UL	
#include <avr/io.h>
#include <util/delay.h>
	
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE ((F_CPU/16/USART_BAUDRATE)-1) //UBRR (USART BAUD RATE REGISTER) degeri datasheet de sayfa 205.


int main(void)
{
	UBRR0L = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRR0H = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
    UCSR0B =  (1 << TXEN0);	//enable the receiver and transmitter.
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
    
    
   
    while(1)
    {
	UDR0 ='S';
	_delay_ms(1000);
    }

}

