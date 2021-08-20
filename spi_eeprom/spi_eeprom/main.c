#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define WREN  0x06


unsigned char gonderilen;
unsigned char okunan;

char spi_transfer( char data)
{
	PORTB &= ~(1 << 2);
	while(!(SPSR & (1<<SPIF)));	
	SPDR=WREN;
	PORTB |= ~(1 << 2);
	SPDR = data;                    // Start the transmission
	while (!(SPSR & (1<<SPIF)));    // Wait the end of the transmission
	return SPDR;                    // return the received byte
}

void spi_init()
{

	DDRB=(1<<5)|(1<<3) | (1<<2);   //SPI portlarý B portlarýnda olduðu için MOSI VE SCK yi çýkýþ olarak ayarlamak gerekiyor.
	DDRB &= ~(1<< 4);    //MISO input
	PORTB |= ~(1 << 2);     //disable device
	         
	SPCR=(1<<SPE)|(1<<MSTR);     // Enable SPI, Set as Master,Prescaler: Fosc/16, Enable SPI Interrupts
	PORTB &= ~(1 << 2);
	

}


char SPI_Read()	
{			
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));	
	return(SPDR);			
}



int main(void)
{
	spi_init();
	
	PORTB |= ~(1 << 2);

	while (1)
	{
		  
		
		spi_transfer('A');
		okunan = SPI_Read();
	}
}


