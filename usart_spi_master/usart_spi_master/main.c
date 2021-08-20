#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define BRC ((F_CPU/16/USART_BAUDRATE)-1)
#define RX_BUFFER_SIZE  128
char rxBuffer[RX_BUFFER_SIZE];

uint8_t rxReadPos = 0;
uint8_t rxWritePos = 0;

unsigned char gonderilen;
char getChar(void);
unsigned char c ;

void SPI_Write(char data)		/* SPI write data function */
{
	char flush_buffer;
	SPDR = data;			/* Write data to SPI data register */
	while(!(SPSR & (1<<SPIF)));	/* Wait till transmission complete */
	flush_buffer = SPDR;		/* Flush received data */
	/* Note: SPIF flag is cleared by first reading SPSR (with SPIF set) and then accessing SPDR hence flush buffer used here to access SPDR after SPSR read */
}




int main(void)
{   
	/*spi register ayarlarý*/   
	DDRB |= (1<<3)|(1<<5)|(1<<2);	/* Make MOSI, SCK, SS 
						as Output pin */
	DDRB &= ~(1<<4);			/* Make MISO pin 
						as input pin */
	PORTB |= (1<<2);			/* Make high on SS pin */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);	/* Enable SPI in master mode
						with Fosc/16 */
	
	
	 
	/*usart register ayarlarý*/
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;	
	UCSR0B = (1 << RXEN0)  | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	sei();
	
	while (1)
	{
	//	c= getChar();
		SPI_Write('A');
	}
}

char getChar(void)
{
	char ret = '\0';
	
	if(rxReadPos != rxWritePos)
	{
		ret = rxBuffer[rxReadPos];
		
		rxReadPos++;
		
		if(rxReadPos >= RX_BUFFER_SIZE)
		{
			rxReadPos = 0;
		}
	}
	
	return ret;
}

//ISR(USART_RX_vect)
//{
	//rxBuffer[rxWritePos] = UDR0;
	//
	//rxWritePos++;
	//
	//if(rxWritePos >= RX_BUFFER_SIZE)
	//{
		//rxWritePos = 0;
	//}
//}
