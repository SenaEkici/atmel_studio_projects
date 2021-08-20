//Seri porttan aldýðý 1 veya 0 koþuluna göre farklý data gönderiyor.
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "timeout.h"
#include "net.h"

#define REMOTE_PORT 9600
#define LOCAL_PORT  2000
#define BUFFER_SIZE 650

#define F_CPU   12500000UL// system frequency

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE ((F_CPU/16/USART_BAUDRATE)-1) //UBRR (USART BAUD RATE REGISTER) degeri datasheet de sayfa 205.

static uint8_t buf1[3];
static uint8_t recvlen;

static uint8_t mac_address[6] = {0x22,0x22,0x22,0x10,0x00,0x33};
	static uint8_t from_to_mac[6]={0x64,0x51,0x06,0xA8,0x09,0xA2};
static uint8_t local_host[4]  = {192,168,2,23};
static uint8_t remote_host[4] = {192,168,2,24};
static uint8_t gateway[4]     = {192,168,2,1};
static uint16_t local_port    = 2000;
static uint16_t remote_port   = 9600;

static uint8_t buf[BUFFER_SIZE+1];

static uint8_t data[18] = {0x01,0x02,0x03,0x04,0x05};

static void net_init (void);

void usart_init(void){
	
	UBRR0L = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
	UBRR0H = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
	UCSR0B =  (1 << TXEN0);	//enable the receiver and transmitter.
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

int main(void)
{
	
    
	usart_init();
	uint8_t ledon = 0;
	uint8_t j = 0;
	// Set the clock prescaler to divide-by-1
	CLKPR=(1<<CLKPCE);
	CLKPR=0;
	
	_delay_ms(20);
	
	// initialize enc28j60
	enc28j60Init(mac_address);
	_delay_ms(50);
	
	// wait until the link is up then send a gratuitous ARP
	// to inform any conected switch about my existence
	while(!enc28j60IsLinkUp());
	_delay_ms(50);
	enc28j60gratuitousARPrequest(mac_address);

	net_init();
	while (1)
	{	
		// get the next new packet:
		recvlen = enc28j60PollPacket(BUFFER_SIZE,from_to_mac, buf1);
		if (recvlen==0){
			continue;
		}
		// toggle LED to acknowledge reception of packet

		if (ledon)
		{
			ledon=0;
			UDR0 ='S';
		}
		else
		{
			ledon=1;
			UDR0 ='A';
		}

		// set pins on port D according to user data received
		PORTD=buf[1];
		//
		j++;
		//buf[0] contains a sequence number we pass it back
		buf[1]=PINC; // read all 8 pins, PC0 to PC7
		buf[2]=j; // some data for demo purposes
		enc28j60TransmitPacket(BUFFER_SIZE,from_to_mac,buf);
		
	}
	return (0);
}


static void net_init (void) {
	_delay_loop_1(0);
	enc28j60Init(mac_address);
	enc28j60clkout(2);
	_delay_loop_1(0);
	enc28j60PhyWrite(PHLCON,0x476);//led kontrol eden register her gönderimde sarý ledi yakýp söndürüyor.
	_delay_loop_1(0);
	init_ip_arp_udp_tcp(mac_address,local_host,local_port);
	client_set_gwip(gateway);
}


