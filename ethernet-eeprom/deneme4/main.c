#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "ip_arp_udp_tcp.h"

#include "enc28j60.h"
#include "timeout.h"
#include "net.h"

#define REMOTE_PORT 9600
#define LOCAL_PORT  2000
#define BUFFER_SIZE 650
char sena[255];
static uint8_t mac_address[6] = {0x22,0x22,0x22,0x10,0x00,0x33};
static uint8_t local_host[4]  = {192,168,2,6}; //ATMEGA
static uint8_t remote_host[4] = {192,168,2,80}; //LAPTOP
	
static uint8_t gateway[4]    = {192,168,2,6}; 
static uint16_t local_port    = 2000;
static uint16_t remote_port   = 9600;

static uint8_t buf[BUFFER_SIZE+1];
static uint8_t buf2[8];
static uint8_t data[18]={0x01,0x02,0x03,0x04,0x05};
static	char buf_con;
	
	
	
 static int nPingCount = 0;
 #define F_CPU   8000000UL// system frequency
 #define F_CPU   16000000UL// system frequency
 #define BAUD    9600//baudrate -bps
 #define BAUD_PRESCALE     ((F_CPU/16/BAUD) - 1)//datasheette kullanýlan form
static void net_init (void);
static void usart_init(void);
static void TWI_init();
static void TWI_Start();
char TWI_Read_Nack();
char TWI_Read_Ack();
void TWI_Write_Data(char x);
void TWI_Write_Add(char x);
void TWI_Stop();

int main (void) {
	
	      uint16_t plen;
	      char okunan;
		  
		  DDRD =0XFF;
		  PORTD=0X00;
          net_init();
		  usart_init();
		  TWI_init();
		  int i=42;
          while(1) {
			  plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
			  if(plen==0) continue;
			    UDR0=buf[i];
			  
			 if(eth_type_is_arp_and_my_ip(buf,plen)){
				 make_arp_answer_from_request(buf);	
				 continue;
			  }
			  
			    
			   			  
    }
    return (0);
}

static void net_init (void) {
    _delay_loop_1(0);
    enc28j60Init(mac_address);
    enc28j60clkout(2);
    _delay_loop_1(0);
    enc28j60PhyWrite(PHLCON,0x476);
   _delay_loop_1(0);
    init_ip_arp_udp_tcp(mac_address,local_host,local_port);
    client_set_gwip(gateway);
}

void TWI_init()
{
	  TWSR = 0x00;
	  TWBR = 0x62;
	  TWCR = (1<<TWEN);
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
	while(!(TWCR &(1<<TWINT)));
	return TWDR;
	
}
void TWI_Stop()
{
	TWCR=(1<<TWSTO)|(1<<TWINT)|(1<<TWEN);
	while((TWCR &(1<<TWSTO)));
}
static void usart_init(void)
{
	UBRR0L = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRR0H = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
	UCSR0B =  (1 << TXEN0);	//enable the receiver and transmitter.
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}


/*
// 			   send_udp(buf2,data,sizeof(data),local_port,remote_host,remote_port);//DATA TRANSMIT-UDP-
// 			  _delay_ms(1000);
*/



/*
 TWI_Start();
 TWI_Write_Add(0xA0);
 buf_con=buf[42];
 TWI_Write_Data(buf_con);
 TWI_Stop();
 TWI_Start();
 okunan=TWI_Read_Ack();
 if(okunan='A')
 {
	 UDR0=okunan;
 }
 TWI_Read_Nack();
 TWI_Stop();




*/