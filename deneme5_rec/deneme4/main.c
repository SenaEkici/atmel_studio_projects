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
	
//static uint8_t gateway[4]    = {192,168,2,6}; 
static uint16_t local_port    = 2000;
static uint16_t remote_port   = 9600;

static uint8_t buf[BUFFER_SIZE+1];
static uint8_t buf2[8];
static uint8_t data[18]={0x01,0x02,0x03,0x04,0x05};
 static int nPingCount = 0;

static void net_init (void);

int main (void) {
	
	      uint16_t plen;
	      uint16_t  dat_p;
		   DDRD =0XFF;
		  
          net_init();
          while(1) {
			send_udp(buf2,data,sizeof(data),local_port,remote_host,remote_port);
			_delay_ms(1000);
		    
			 plen=enc28j60PacketReceive(BUFFER_SIZE, buf);
			 
			 if(plen==0) continue;
		  
			 if(eth_type_is_arp_and_my_ip(buf,plen)){
				 make_arp_answer_from_request(buf);
				 continue;
			  }

 			 ////// check if ip packets are for us:
               if(eth_type_is_ip_and_my_ip(buf,plen)==0){
                continue;
               }
			    

               if(buf[IP_PROTO_P]==IP_PROTO_ICMP_V && buf[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V){
 				 ////// a ping packet, let's send reply
                 make_echo_reply_from_request(buf,plen);
                 nPingCount ++;
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
    //client_set_gwip(gateway);
}