#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
int x;

void SPI_Init()	/* SPI Initialize function */
{
	DDRB &= ~((1<<3)|(1<<5)|(1<<2));  /* Make MOSI, SCK, SS as input pins */
	DDRB |= (1<<4);			          /* Make MISO pin as output pin */
	SPCR = (1<<SPE);			      /* Enable SPI in slave mode */
}

int main(void)
{   
	SPI_Init();
    char gelen_data;
    while (1) 
    {
		while(!(SPSR & (1<<SPIF) ));                  //Wait until transmission complete
		gelen_data=SPDR;
		if (gelen_data=='A')
		{
			x++;
			if(x==5)
			{
				x=0;
			}
		}
    }
}

