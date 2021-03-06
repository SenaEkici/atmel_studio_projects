/* @project 
 * 
 * License to access, copy or distribute this file.
 * This file or any portions of it, is Copyright (C) 2012, Radu Motisan ,  http://www.pocketmagic.net . All rights reserved.
 * @author Radu Motisan, radu.motisan@gmail.com
 * 
 * This file is protected by copyright law and international treaties. Unauthorized access, reproduction 
 * or distribution of this file or any portions of it may result in severe civil and criminal penalties.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * 
 * @purpose Eth interface for Atmega microcontrollers
 * http://www.pocketmagic.net/?p=2866
 */



#include "hw_enc28j60.h"
#include "ip_arp_udp_tcp.h" 
#include "ENC28J60.h"


ENC28J60::ENC28J60(){ }

uint16_t ENC28J60::m_fill_tcp_data_p(uint8_t *buf,uint16_t pos, const char *progmem_s){
	return fill_tcp_data_p(buf, pos, progmem_s);
}

uint16_t ENC28J60::m_fill_tcp_data(uint8_t *buf,uint16_t pos, const char *s){
	return fill_tcp_data(buf,pos, s);
}


void ENC28J60::m_enc28j60Init(uint8_t* macaddr){
	enc28j60Init(macaddr);
}

void ENC28J60::m_enc28j60clkout(uint8_t clk){
	enc28j60clkout(clk);
}

void ENC28J60::m_enc28j60PhyWrite(uint8_t address, uint16_t data){
	enc28j60PhyWrite(address,  data);
}

uint16_t ENC28J60::m_enc28j60PacketReceive(uint16_t len, uint8_t* packet){
	return enc28j60PacketReceive(len, packet);
}


void ENC28J60::m_init_ip_arp_udp_tcp(uint8_t *mymac,uint8_t *myip,uint8_t wwwp){
	init_ip_arp_udp_tcp(mymac,myip,wwwp);
}

uint8_t ENC28J60::m_eth_type_is_arp_and_my_ip(uint8_t *buf,uint16_t len){
	return eth_type_is_arp_and_my_ip(buf,len);
}

void ENC28J60::m_make_arp_answer_from_request(uint8_t *buf){
	make_arp_answer_from_request(buf);
}

uint8_t ENC28J60::m_eth_type_is_ip_and_my_ip(uint8_t *buf,uint16_t len){
	return eth_type_is_ip_and_my_ip(buf, len);
}


void ENC28J60::m_make_echo_reply_from_request(uint8_t *buf,uint16_t len){
	make_echo_reply_from_request(buf,len);
}

void ENC28J60::m_make_tcp_synack_from_syn(uint8_t *buf){
	make_tcp_synack_from_syn(buf);
}	

void ENC28J60::m_init_len_info(uint8_t *buf){
	init_len_info(buf);
}

uint16_t ENC28J60::m_get_tcp_data_pointer(void){
	return get_tcp_data_pointer();
}

void ENC28J60::m_make_tcp_ack_from_any(uint8_t *buf){
	make_tcp_ack_from_any(buf);
}

void ENC28J60::m_make_tcp_ack_with_data(uint8_t *buf,uint16_t dlen){
	make_tcp_ack_with_data(buf,dlen);
}



void ENC28J60::m_init_leds() {
	enc28j60PhyWrite(PHLCON,0x476);
}









