#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE ((F_CPU/16/USART_BAUDRATE)-1)
#define TX_BUFFER_SIZE  128
char TxBuffer[TX_BUFFER_SIZE];

uint8_t txReadPos = 0;
uint8_t txWritePos = 0;



int main(void)
{
	UBRR0L = BAUD_PRESCALE;			
	UBRR0H = (BAUD_PRESCALE >> 8);		
	UCSR0B =  (1 << TXEN0);	
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
	
	DDRB &= ~((1<<3)|(1<<5)|(1<<2));  /* Make MOSI, SCK, SS as
 						input pins */
	DDRB |= (1<<4);			/* Make MISO pin as 
						output pin */
	SPCR = (1<<SPE);			/* Enable SPI in slave mode */
	
	
	
	//sei();
	char gelen_data;
	
	while (1)
	{
		while(!(SPSR & (1<<SPIF) ));  
		gelen_data=SPDR;
	    UDR0=gelen_data;
				
	}
	
}

/*ISR(USART_TX_vect)
{
	if(txReadPos != txWritePos)
	{
		UDR0=TxBuffer[txReadPos];
		txReadPos++;
		if(txReadPos >= TX_BUFFER_SIZE)
		{
			txReadPos++;
	    }
	}
}*/
