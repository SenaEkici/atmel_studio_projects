#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>	



char data='A';
char okunan;
char sonbit;
//TWI fonksiyonlarý
void TWI_init()
{
	 TWBR=0X62;            //bit RATE HESAPLANDI.
	 TWCR=(1<<TWINT);      //interruptlar enable edildi.
	 TWSR=0X00;            //prescaler 1 olarak ayarlandý.
}

void TWI_Start()
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);        //TWI interrupt flag-TWI start condition-TWI enable flagleri set edildi.
	while(!(TWCR &(1<<TWINT)));                    //interrupt gelene kadar bekle interrupt geldiðinde TWINT Flagi set olur.
	while((TWSR &(0XF8))!=0X08);
}

void TWI_Write_Add(char x)
{
	TWDR=x;
	TWCR=(1<<TWINT) | (1<< TWEN);
	while(!(TWCR &(1<<TWINT)));   
	while((TWSR &(0XF8))!=0X18);
}
void TWI_Write_Data(char x)
{
	TWDR=x;
	TWCR=(1<<TWINT) | (1<< TWEN);
	while(!(TWCR &(1<<TWINT)));   
	while((TWSR &(0XF8))!=0X28);
}

char TWI_Read_Ack()		
{
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA); 
	while(!(TWCR &(1<<TWINT)));   	
	return TWDR;			
}

char TWI_Read_Nack()		
{
	TWCR=(1<<TWEN)|(1<<TWINT);	
	while(!(TWCR &(1<<TWINT))==0);   
	return TWDR;
	
}
void TWI_Stop()
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);
	while((TWCR &(1<<TWSTO)));
}



int main(void)
{
	
    TWI_init();
	while (1)
	{
		TWI_Start();
		
		TWI_Write_Add(0xA0);
		
		TWI_Write_Data(data);
		
		TWI_Stop();
		
		TWI_Start();
		
		okunan=TWI_Read_Ack();
		
		TWI_Read_Nack();
			
		TWI_Stop();			
	}
    
}

