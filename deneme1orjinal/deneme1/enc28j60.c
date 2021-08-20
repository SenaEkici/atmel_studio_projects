#include <avr/io.h>

#include "ethernet.h"
#include "enc28j60.h"

//
#define F_CPU 12500000UL  // 12.5 MHz
#ifndef ALIBC_OLD
#include <util/delay_basic.h>
#else
#include <avr/delay.h>
#endif
/*
// 
// #undef PROGMEM
// #define PROGMEM __attribute__(( section(".progmem.data") ))
// #undef PSTR
// #define PSTR(s) (__extension__({static prog_char c[] PROGMEM = (s); &c[0];}))
*/



static uint8_t Enc28j60Bank;
static int16_t gNextPacketPtr;
#define ENC28J60_CONTROL_PORT   PORTB
#define ENC28J60_CONTROL_DDR    DDRB
#define ENC28J60_CONTROL_CS     PORTB2
#define ENC28J60_CONTROL_SO PORTB4
#define ENC28J60_CONTROL_SI PORTB3
#define ENC28J60_CONTROL_SCK PORTB5
// set CS to 0 = active
#define CSACTIVE ENC28J60_CONTROL_PORT&=~(1<<ENC28J60_CONTROL_CS)
// set CS to 1 = passive
#define CSPASSIVE ENC28J60_CONTROL_PORT|=(1<<ENC28J60_CONTROL_CS)
//
#define waitspi() while(!(SPSR&(1<<SPIF)))
/*const char arpreqhdr[] PROGMEM ={8,6,0,1,8,0,6,4,0,1};*/
uint8_t enc28j60ReadOp(uint8_t op, uint8_t address)
{
	CSACTIVE;
	// issue read command
	SPDR = op | (address & ADDR_MASK);
	waitspi();
	// read data
	SPDR = 0x00;
	waitspi();
	// do dummy read if needed (for mac and mii, see datasheet page 29)
	if(address & 0x80)
	{
		SPDR = 0x00;
		waitspi();
	}
	// release CS
	CSPASSIVE;
	return(SPDR);
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
	CSACTIVE;
	// issue write command
	SPDR = op | (address & ADDR_MASK);
	waitspi();
	// write data
	SPDR = data;
	waitspi();
	CSPASSIVE;
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data)
{
	CSACTIVE;
	// issue read command
	SPDR = ENC28J60_READ_BUF_MEM;
	waitspi();
	while(len)
	{
		len--;
		// read data
		SPDR = 0x00;
		waitspi();
		*data = SPDR;
		data++;
	}
	*data='\0';
	CSPASSIVE;
}

void enc28j60WriteBuffer(uint16_t len, uint8_t* data)
{
	CSACTIVE;
	// issue write command
	SPDR = ENC28J60_WRITE_BUF_MEM;
	waitspi();
	while(len)
	{
		len--;
		// write data
		SPDR = *data;
		data++;
		waitspi();
	}
	CSPASSIVE;
}

void enc28j60SetBank(uint8_t address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		// set the bank
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

uint8_t enc28j60Read(uint8_t address)
{
	// set the bank
	enc28j60SetBank(address);
	// do the read
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}

// read upper 8 bits
uint16_t enc28j60PhyReadH(uint8_t address)
{

	// Set the right address and start the register read operation
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);
	_delay_loop_1(40); // 10us

	// wait until the PHY read completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY);

	// reset reading bit
	enc28j60Write(MICMD, 0x00);
	
	return (enc28j60Read(MIRDH));
}

void enc28j60Write(uint8_t address, uint8_t data)
{
	// set the bank
	enc28j60SetBank(address);
	// do the write
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}


void enc28j60PhyWrite(uint8_t address, uint16_t data)
{
	// set the PHY register address
	enc28j60Write(MIREGADR, address);
	// write the PHY data
	enc28j60Write(MIWRL, data);
	enc28j60Write(MIWRH, data>>8);
	// wait until the PHY write completes
	while(enc28j60Read(MISTAT) & MISTAT_BUSY){
		_delay_loop_1(40); // 10us
	}
}

void enc28j60clkout(uint8_t clk)
{
	//setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk & 0x7);
}

void enc28j60Init(uint8_t* macaddr)
{
	// initialize I/O
	// ss as output:
	ENC28J60_CONTROL_DDR |= 1<<ENC28J60_CONTROL_CS;
	CSPASSIVE; // ss=0
	//
	ENC28J60_CONTROL_DDR  |= 1<<ENC28J60_CONTROL_SI | 1<<ENC28J60_CONTROL_SCK; // mosi, sck output
	ENC28J60_CONTROL_DDR|= 1<<ENC28J60_CONTROL_SO; // MISO is input
	//
	ENC28J60_CONTROL_PORT|= 1<<ENC28J60_CONTROL_SI; // MOSI low
	ENC28J60_CONTROL_PORT|= 1<<ENC28J60_CONTROL_SCK; // SCK low
	//
	// initialize SPI interface
	// master mode and Fosc/2 clock:
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR |= (1<<SPI2X);
	// perform system reset
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	_delay_loop_2(0); // 20ms
	// check CLKRDY bit to see if reset is complete
	// The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// set receive buffer start address
	gNextPacketPtr = RXSTART_INIT;
	// Rx start
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address
	enc28j60Write(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// TX end
	enc28j60Write(ETXNDL, TXSTOP_INIT&0xFF);
	enc28j60Write(ETXNDH, TXSTOP_INIT>>8);
	// do bank 1 stuff, packet filter:
	// For broadcast packets we allow only ARP packtets
	// All other packets should be unicast only for our mac (MAADR)
	//
	// The pattern to match on is therefore
	// Type     ETH.DST
	// ARP      BROADCAST
	// 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
	// in binary these poitions are:11 0000 0011 1111
	// This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	enc28j60Write(EPMM0, 0x3f);
	enc28j60Write(EPMM1, 0x30);
	enc28j60Write(EPMCSL, 0xf9);
	enc28j60Write(EPMCSH, 0xf7);
	//
	//
	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	enc28j60Write(MACON2, 0x00);
	// enable automatic padding to 60bytes and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
	// Do not send packets longer than MAX_FRAMELEN:
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);
	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, macaddr[0]);
	enc28j60Write(MAADR4, macaddr[1]);
	enc28j60Write(MAADR3, macaddr[2]);
	enc28j60Write(MAADR2, macaddr[3]);
	enc28j60Write(MAADR1, macaddr[4]);
	enc28j60Write(MAADR0, macaddr[5]);
	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

// read the revision of the chip:
uint8_t enc28j60getrev(void)
{
	return(enc28j60Read(EREVID));
}

// link status
uint8_t enc28j60linkup(void)
{
	// bit 10 (= bit 3 in upper reg)
	return(enc28j60PhyReadH(PHSTAT2) && 4);
}

void enc28j60PacketSend(uint16_t len, uint8_t* packet)
{
	// Check no transmit in progress
	while (enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
	{
		// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
		if( (enc28j60Read(EIR) & EIR_TXERIF) ) {
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
	}
	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);
	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);
	// write per-packet control byte (0x00 means use macon3 settings)
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

// just probe if there might be a packet
uint8_t enc28j60hasRxPkt(void)
{
	if( enc28j60Read(EPKTCNT) ==0 ){
		return(0);
	}
	return(1);
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet)
{
	uint16_t rxstat;
	uint16_t len;
	// check if a packet has been received and buffered
	//if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
	// The above does not work. See Rev. B4 Silicon Errata point 6.
	if( enc28j60Read(EPKTCNT) ==0 ){
		return(0);
	}

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (gNextPacketPtr &0xFF));
	enc28j60Write(ERDPTH, (gNextPacketPtr)>>8);
	// read the next packet pointer
	gNextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	gNextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the packet length (see datasheet page 43)
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	len-=4; //remove the CRC count
	// read the receive status (see datasheet page 43)
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= ((uint16_t)enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0))<<8;
	// limit retrieve length
	if (len>maxlen-1){
		len=maxlen-1;
	}
	// check CRC and symbol errors (see datasheet page 44, table 7-3):
	// The ERXFCON.CRCEN is set by default. Normally we should not
	// need to check this.
	if ((rxstat & 0x80)==0){
		// invalid
		len=0;
		}else{
		// copy the packet from the receive buffer
		enc28j60ReadBuffer(len, packet);
	}
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	enc28j60Write(ERXRDPTL, (gNextPacketPtr &0xFF));
	enc28j60Write(ERXRDPTH, (gNextPacketPtr)>>8);
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out.
	// However, compensate for the errata point 13, rev B4: enver write an even address!
	if ((gNextPacketPtr - 1 < RXSTART_INIT)
	|| (gNextPacketPtr -1 > RXSTOP_INIT)) {
		enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
		enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
		} else {
		enc28j60Write(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
		enc28j60Write(ERXRDPTH, (gNextPacketPtr-1)>>8);
	}
	// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return(len);
}
uint8_t enc28j60PollPacket( uint8_t maxlen, uint8_t* from_mac,uint8_t* buf)
{
	uint8_t len;
	uint16_t currentPacketPtr = gNextPacketPtr;
	uint16_t address;
	uint16_t framelen;

	// check if a packet has been received and buffered
	enc28j60SetBank(1);
	if (enc28j60Read(EPKTCNT) == 0)
	{
		return (0);
	}

	enc28j60SetBank(0);
	// Somehow, the read pointer is NOT already at the start of the next packet
	// even though we leave it in that state
	enc28j60Write(ERDPTL, (gNextPacketPtr&0xff));
	enc28j60Write(ERDPTH, (gNextPacketPtr)>>8);
	// Read the next packet pointer
	gNextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	gNextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the frame length
	framelen  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	framelen |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	if (maxlen> framelen-14){ // subtract eth source, dest and length fields
		maxlen=framelen-14;
	}
	address = currentPacketPtr + ETH_SRC_MAC_P + 6;	 // note +6 for receive vectors
	if (address > RXSTOP_INIT)
	{
		address -= (RXSTOP_INIT - RXSTART_INIT + 1);
	}
	enc28j60Write(ERDPTL, address & 0xff);
	enc28j60Write(ERDPTH, (address)>>8);
	enc28j60ReadBuffer(6, from_mac);
	// A value of less than 0x05dc in the EtherType has to be interpreted as
	// length. This is what we do here.
	// The length is 16 bit. The upper 8 bits must be zero
	// otherwise it is not our packet.
	len = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	if (len !=0){
		len=0;
		goto NEXTPACKET;
	}
	// read the lower byte of the length field
	len = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	// limit retrieve length to maxlen, ignoring anything else
	if (len > maxlen) {
		len=maxlen;
	}
	// copy payload data from the receive buffer
	enc28j60ReadBuffer(len, buf);

	NEXTPACKET:
	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out.
	// However, compensate for the errata point 13, rev B4: enver write an even address!
	if ((gNextPacketPtr - 1 < RXSTART_INIT)
	|| (gNextPacketPtr -1 > RXSTOP_INIT))
	{
		enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
		enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
	}
	else
	{
		enc28j60Write(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
		enc28j60Write(ERXRDPTH, (gNextPacketPtr-1)>>8);
	}

	// Decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return (len);
}


//------------------------------------------------------------------------------
// Send a packet.
// The to_mac contains the mac of the receiver
// len is the length of data in the buffer buf.
void enc28j60TransmitPacket( uint8_t len, uint8_t* to_mac,uint8_t* buf)
{
	uint16_t address;
	enc28j60SetBank(0);
	
	// (control byte, and SRC MAC have already been set during init)
	
	#ifdef ENC28J60_HAS_PENDING_TRANSMIT_ON_TRANSMIT
	// Check no transmit in progress
	while (enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
	{
		// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
		if( (enc28j60Read(EIR) & EIR_TXERIF) )
		{
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
	}
	#else
	// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if( (enc28j60Read(EIR) & EIR_TXERIF) )
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
	}
	#endif
	// Set the write pointer to start of transmit buffer area
	// +1 to skip the per packet control byte and write directly the mac
	// The control byte was set to zero during initialisation and remains like that.
	enc28j60Write(EWRPTL, (TXSTART_INIT+1)&0xFF);
	enc28j60Write(EWRPTH, (TXSTART_INIT+1)>>8);
	enc28j60WriteBuffer(6, to_mac);

	// Set the write pointer to the first byte of the EtherType field
	// (field after the mac address). This is the 802.3 length field.
	address = TXSTART_INIT + 1 + ETH_TYPE_H_P;
	enc28j60Write(EWRPTL, address & 0xFF);
	enc28j60Write(EWRPTH, address >> 8);
	// write the length of the data in the ethernet type field.
	// The type field to be interpreted by the receiver as ieee802.3 length if
	// the value is less than 0x05dc (see e.g. http://www.cavebear.com/archive/cavebear/Ethernet/type.html ):
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0);
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, len);
	// Copy the payload into the transmit buffer
	enc28j60WriteBuffer(len, buf); // remove dest mac and write the rest
	// Set the TXND pointer to correspond to the payload size given
	address = (TXSTART_INIT + ETH_HEADER_LEN + len );
	enc28j60Write(ETXNDL, address & 0xFF);
	enc28j60Write(ETXNDH, address >> 8);
	
	// Send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

//------------------------------------------------------------------------------