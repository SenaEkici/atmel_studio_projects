//STATUS LED kullanarak yaptðýmda olmadý.

#include <avr/io.h>
#include <avr/interrupt.h>//kesmeler için kullanýlan lib

#define F_CPU   16000000UL// system frequency 
#define BUAD    9600//baudrate -bps
#define BRC     ((F_CPU/16/BUAD) - 1)//datasheette kullanýlan formul

#define RX_BUFFER_SIZE  128//ASCII karakterleri için en az 2 olmalý

char rxBuffer[RX_BUFFER_SIZE];

uint8_t rxReadPos = 0;
uint8_t rxWritePos = 0;

char getChar(void);

int main(void)
{
	DDRB=0XFF;//led için user led çýkýþ olarak ayarlandý
	
	/*register ayarlarý datasheet sayfa 222*/
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;
	
	UCSR0B = (1 << RXEN0)  | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
		
	sei();
	
	while(1)
	{
		char c = getChar();
		
		if(c == '1')
		{
			PORTB=0XFF;
		}
		else if(c =='0')
		{
			PORTB=0X00;
		}
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

ISR(USART_RX_vect)
{
	rxBuffer[rxWritePos] = UDR0;
	
	rxWritePos++;
	
	if(rxWritePos >= RX_BUFFER_SIZE)
	{
		rxWritePos = 0;
	}
}