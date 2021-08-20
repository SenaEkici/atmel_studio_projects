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

static uint8_t mac_address[6] = {0x22,0x22,0x22,0x10,0x00,0x33};
static uint8_t from_to_mac[6] = {0x64,0x51,0x06,0xA8,0x09,0xA2};
/*static uint8_t dest_mac_address[6]*/
static uint8_t local_host[4]  = {192,168,2,6}; //ATMEGA
static uint8_t remote_host[4] = {192,168,2,80}; //LAPTOP

static uint16_t local_port    = 2000;
static uint16_t remote_port   = 9600;

static uint8_t buf[BUFFER_SIZE+1];
static uint8_t data[18] = {0x01,0x02,0x03,0x04,0x05};

static uint8_t recvlen;
static uint8_t buf2[BUFFER_SIZE+1];

static void net_init (void);


#define F_CPU   16000000UL// system frequency
#define BAUD    9600//baudrate -bps
#define BAUD_PRESCALE     ((F_CPU/16/BAUD) - 1)//datasheette kullanýlan formul

int main(void)
{


 	UBRR0L = BAUD_PRESCALE;			/* Load lower 8-bits of the baud rate */
 	UBRR0H = (BAUD_PRESCALE >> 8);		/* Load upper 8-bits*/
 	UCSR0B =  (1 << TXEN0);	//enable the receiver and transmitter.
 	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);



    net_init();
	
	
	
    while (1) 
    {
		//make_arp_answer_from_request(buf);
		send_udp(buf,data,sizeof(data),local_port, remote_host, remote_port);
		
		
		_delay_ms(1000);
		// get the next new packet:
		//recvlen = enc28j60PollPacket(BUFFER_SIZE,from_to_mac, buf);
				
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
	
	//client_set_gwip(gateway);
}

