// Written by Rogier Schouten http://www.rogiershikes.tk
// Based on code by Guido Socher http://www.tuxgraphics.org
// Idea modified and further updated by Guido Socher
//
// License: GPL V2
#include <avr/io.h>

#define F_CPU 12500000UL  // 12.5 MHz
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "ethernet.h"
#include "enc28j60.h"
#include "enc28j60_int.h"


#define ENC28J60_CONTROL_PORT   PORTB
#define ENC28J60_CONTROL_DDR    DDRB
#define ENC28J60_CONTROL_CS     PORTB2
#define ENC28J60_CONTROL_SO PORTB4
#define ENC28J60_CONTROL_SI PORTB3
#define ENC28J60_CONTROL_SCK PORTB5


// Maximum packet length: this software has a limit of 255 payload data bytes
// and then there is some ethernet overhead (srd, dst, len, fcs)
#define ENC28J60_MAX_PACKET_LEN ((uint16_t)273)

/// SPI handling macros
#define CSACTIVE ENC28J60_CONTROL_PORT&=~(1<<ENC28J60_CONTROL_CS)
// set CS to 1 = passive
#define CSPASSIVE ENC28J60_CONTROL_PORT|=(1<<ENC28J60_CONTROL_CS)
#define waitspi() while (!( SPSR & (1 << SPIF)))

const char myip[4] ={192,168,2,24}; // some ip that does not exist in your network
// -----------------------------------------------------------------------------
const char arpreqhdr[] PROGMEM ={8,6,0,1,8,0,6,4,0,1};
static uint8_t gBank;
static uint16_t gNextPacketPtr; // position where next packet will be written

// -----------------------------------------------------------------------------

/// Read a control register in the controller assuming that the currently
/// selected bank is correct
#define enc28j60ReadReg(ADRESS) 	enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ADRESS)
uint8_t enc28j60ReadOp( uint8_t op, uint8_t address)
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



// -----------------------------------------------------------------------------

/// Write a control register in the controller assuming that the currently
/// selected bank is correct
void enc28j60WriteOp( uint8_t op, uint8_t address, uint8_t data)
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

#define enc28j60WriteReg(ADRESS, DATA) 	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, ADRESS, DATA)

// -----------------------------------------------------------------------------

/// Set the control register bank to use  
void enc28j60SetBank( uint8_t bank)
{
	// set the bank
	if (bank != gBank)
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (bank & 0x03));
		gBank = bank;
	}		
}

// -----------------------------------------------------------------------------

/// Read from the Buffer Memory
void enc28j60ReadBuffer( uint8_t len, uint8_t* data)
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
	CSPASSIVE;
}

// -----------------------------------------------------------------------------

/// Write to the Buffer Memory
void enc28j60WriteBuffer( uint8_t len, uint8_t* data)
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

// -----------------------------------------------------------------------------

/// Read upper 8 bits of a PHY register
uint8_t enc28j60PhyReadH( uint8_t address)
{

	// Set the right address and start the register read operation
	enc28j60SetBank(2);
	enc28j60WriteReg(MIREGADR, address);
	enc28j60WriteReg(MICMD, MICMD_MIIRD);
	_delay_us(15);

	// wait until the PHY read completes
	enc28j60SetBank(3);
	while(enc28j60ReadReg(MISTAT) & MISTAT_BUSY);

	// reset reading bit
	enc28j60SetBank(2);
	enc28j60WriteReg(MICMD, 0x00);
	return (enc28j60ReadReg(MIRDH));
}

// -----------------------------------------------------------------------------

/// Write a PHY register
void enc28j60PhyWrite( uint8_t address, uint16_t data)
{
	enc28j60SetBank(2);
	
	// set the PHY register address
	enc28j60WriteReg(MIREGADR, address);
	// write the PHY data
	enc28j60WriteReg(MIWRL, data);
	enc28j60WriteReg(MIWRH, data>>8);
	// wait until the PHY write completes
	enc28j60SetBank(3);
	while(enc28j60ReadReg(MISTAT) & MISTAT_BUSY)
	{
		_delay_us(15);
	}
}

// -----------------------------------------------------------------------------

void enc28j60Init( uint8_t* macaddr)
{
	gBank = 0xFF; // non-existent 
	
	// Initialize atmel I/O
    ENC28J60_CONTROL_DDR |= 1<<ENC28J60_CONTROL_CS;
    CSPASSIVE; // ss=0

    ENC28J60_CONTROL_DDR  |= 1<<ENC28J60_CONTROL_SI | 1<<ENC28J60_CONTROL_SCK; // mosi, sck output
    ENC28J60_CONTROL_DDR|= 1<<ENC28J60_CONTROL_SO; // MISO is input

    ENC28J60_CONTROL_PORT|= 1<<ENC28J60_CONTROL_SI; // MOSI low
    ENC28J60_CONTROL_PORT|= 1<<ENC28J60_CONTROL_SCK; // SCK low

	// Initialize atmel SPI interface
	// master mode and Fosc/2 clock:
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR |= (1<<SPI2X);
	
	// Reset controller
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0x1F, ENC28J60_SOFT_RESET);
	_delay_ms(50); // Note that polling CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.

	// A.S.A.P. setup CLKout: 2 is 12.5MHz:
	enc28j60SetBank(3);
	enc28j60WriteReg(ECOCON, 2);
	
	// Do bank 0 stuff
	enc28j60SetBank(0);
	// Initialize receive buffer
	// 16-bit transfers, must write low byte first
	// Set receive buffer start address
	gNextPacketPtr = RXSTART_INIT;
	// Rx start
	enc28j60WriteReg(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60WriteReg(ERXSTH, RXSTART_INIT>>8);
	// Set receive pointer address
	enc28j60WriteReg(ERXRDPTL, RXSTART_INIT&0xFF);
	enc28j60WriteReg(ERXRDPTH, RXSTART_INIT>>8);
	// RX end
	enc28j60WriteReg(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60WriteReg(ERXNDH, RXSTOP_INIT>>8);
	// TX start
	enc28j60WriteReg(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60WriteReg(ETXSTH, TXSTART_INIT>>8);
	// TX end (initialize for a packet with a payload of 1 byte)
	uint16_t address = (TXSTART_INIT + ETH_HEADER_LEN + 1);
	enc28j60WriteReg(ETXNDL, address & 0xFF);
	enc28j60WriteReg(ETXNDH, address >> 8);
	// Prepare the parts of the transmit packet that never change 
	// Write per-packet control byte (0x00 means use macon3 settings)
	enc28j60WriteReg(EWRPTL, (TXSTART_INIT) & 0xFF);
	enc28j60WriteReg(EWRPTH, (TXSTART_INIT) >> 8);
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	// Write broadcast address as DST MAC to have some default setting.
	// we can still overwrite later
	uint8_t i = 0;
	while (i < 6)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0xFF);	
		i++;
	}
	// Set our MAC address as the SRC MAC into the transmit buffer
	// Set the write pointer to start of transmit buffer area
	enc28j60WriteBuffer(6, macaddr);
	// First EtherType/length byte is always 0, initialize second byte to 1 
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);	
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x01);	
	
	// Do bank 1 stuff, packet filter: 
	enc28j60SetBank(1);
	// Only allow unicast packets destined for us and that have a correct CRC
	enc28j60WriteReg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN);

	// Do bank 2 stuff
	enc28j60SetBank(2);
	// Enable MAC receive, disable flow control (only needed in full-duplex)
	enc28j60WriteReg(MACON1, MACON1_MARXEN);
	// Bring MAC out of reset
	enc28j60WriteReg(MACON2, 0x00);
	// Enable automatic padding to 60bytes and CRC operations
	// Also, force half-duplex operation
	enc28j60WriteReg(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// Half-duplex only: back-off settings
	enc28j60WriteReg(MACON4, MACON4_DEFER|MACON4_BPEN|MACON4_NOBKOFF);
	// Set the maximum packet size which the controller will accept
	// Do not send packets longer than ENC28J60_MAX_PACKET_LEN:
	enc28j60WriteReg(MAMXFLL, ENC28J60_MAX_PACKET_LEN & 0xFF);
	enc28j60WriteReg(MAMXFLH, ENC28J60_MAX_PACKET_LEN >> 8);
	// set inter-frame gap (non-back-to-back)
	enc28j60WriteReg(MAIPGL, 0x12);
	enc28j60WriteReg(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60WriteReg(MABBIPG, 0x12);
	
	// Do bank 3 stuff
	enc28j60SetBank(3);
	// Write MAC address; NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60WriteReg(MAADR5, macaddr[0]);
	enc28j60WriteReg(MAADR4, macaddr[1]);
	enc28j60WriteReg(MAADR3, macaddr[2]);
	enc28j60WriteReg(MAADR2, macaddr[3]);
	enc28j60WriteReg(MAADR1, macaddr[4]);
	enc28j60WriteReg(MAADR0, macaddr[5]);
	
	// Do PHY stuff
	// Magjack leds configuration, LEDB=yellow LEDA=green
	// 0x476 is PHLCON LEDA=link status, LEDB=receive/transmit
	// enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
	enc28j60PhyWrite(PHLCON,0x476);
	// No loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

	// Enable interrutps
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// Enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);	
}

// -----------------------------------------------------------------------------

uint8_t enc28j60IsLinkUp(void)
{
	return (enc28j60PhyReadH(PHSTAT2) & PHSTAT2H_LSTAT);
}

// -----------------------------------------------------------------------------
// read an ethernet packet, returns the amount of bytes read.
uint8_t enc28j60PollPacket( uint8_t maxlen, uint8_t* from_mac,uint8_t* buf) 
{
	uint8_t len;
	uint16_t currentPacketPtr = gNextPacketPtr;
	uint16_t address;
	uint16_t framelen;

	// check if a packet has been received and buffered
	enc28j60SetBank(1);
	if (enc28j60ReadReg(EPKTCNT) == 0)
	{
		return (0);
	}

	enc28j60SetBank(0);
	// Somehow, the read pointer is NOT already at the start of the next packet
	// even though we leave it in that state
	enc28j60WriteReg(ERDPTL, (gNextPacketPtr&0xff));    
	enc28j60WriteReg(ERDPTH, (gNextPacketPtr)>>8);
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
	enc28j60WriteReg(ERDPTL, address & 0xff);
	enc28j60WriteReg(ERDPTH, (address)>>8);
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
		enc28j60WriteReg(ERXRDPTL, (RXSTOP_INIT)&0xFF);
		enc28j60WriteReg(ERXRDPTH, (RXSTOP_INIT)>>8);
	}
	else
	{
		enc28j60WriteReg(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
		enc28j60WriteReg(ERXRDPTH, (gNextPacketPtr-1)>>8);
	}

	// Decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
        return (len);
}

//------------------------------------------------------------------------------
// send gratuitous ARP request (spontaneous arp request) to teach any
// switches what our mac is.
void enc28j60gratuitousARPrequest( uint8_t* macaddr)
{
	uint8_t i = 0;
	uint16_t address;
	enc28j60SetBank(0);
	
	// (control byte, and SRC MAC have already been set during init)
	
#ifdef ENC28J60_HAS_PENDING_TRANSMIT_ON_TRANSMIT	
	// Check no transmit in progress
	while (enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
	{		
		// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
		if( (enc28j60ReadReg(EIR) & EIR_TXERIF) )
		{
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
	}
#else
	// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if( (enc28j60ReadReg(EIR) & EIR_TXERIF) )
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
	}
#endif
	// Set the write pointer to start of transmit buffer area
	// +1 to skip the per packet control byte and write directly the mac
	// The control byte was set to zero during initialisation and remains like that.
        enc28j60WriteReg(EWRPTL, (TXSTART_INIT+1)&0xFF);
        enc28j60WriteReg(EWRPTH, (TXSTART_INIT+1)>>8);
	// write a broadcase destination mac (all ff):
	while (i < 6)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0xFF);	
		i++;
	}
	// The mac in the ethernet field does not need to be changed.
	// Set the write pointer to the first byte of the EtherType field 
	address = TXSTART_INIT + 1 + ETH_TYPE_H_P;
	enc28j60WriteReg(EWRPTL, address & 0xFF);
	enc28j60WriteReg(EWRPTH, address >> 8);
	// there are 10 fixed bytes in the arp request
	i=0;
        while (i<10){
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, pgm_read_byte(arpreqhdr+i));	
		i++;
        }
	i=0;
	while (i < 6)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, macaddr[i]);	
		i++;
	}
	i=0;
	while (i < 4)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, myip[i]);	
		i++;
	}
	// target data:
	i=0;
	while (i < 6)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0xff);	
		i++;
	}
	// to self, for gratuitous arp:
	i=0;
	while (i < 4)
	{
		enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, myip[i]);	
		i++;
	}
	// Set the TXND pointer to correspond to the payload size given
	address = (TXSTART_INIT + 42 );
	enc28j60WriteReg(ETXNDL, address & 0xFF);
	enc28j60WriteReg(ETXNDH, address >> 8);
	
	// Send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
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
		if( (enc28j60ReadReg(EIR) & EIR_TXERIF) )
		{
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
	}
#else
	// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if( (enc28j60ReadReg(EIR) & EIR_TXERIF) )
	{
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
	}
#endif
	// Set the write pointer to start of transmit buffer area
	// +1 to skip the per packet control byte and write directly the mac
	// The control byte was set to zero during initialisation and remains like that.
        enc28j60WriteReg(EWRPTL, (TXSTART_INIT+1)&0xFF);
        enc28j60WriteReg(EWRPTH, (TXSTART_INIT+1)>>8);
	enc28j60WriteBuffer(6, to_mac);

	// Set the write pointer to the first byte of the EtherType field 
	// (field after the mac address). This is the 802.3 length field.
	address = TXSTART_INIT + 1 + ETH_TYPE_H_P;
	enc28j60WriteReg(EWRPTL, address & 0xFF);
	enc28j60WriteReg(EWRPTH, address >> 8);
	// write the length of the data in the ethernet type field.
	// The type field to be interpreted by the receiver as ieee802.3 length if
	// the value is less than 0x05dc (see e.g. http://www.cavebear.com/archive/cavebear/Ethernet/type.html ):
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0);	
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, len);	
	// Copy the payload into the transmit buffer
	enc28j60WriteBuffer(len, buf); // remove dest mac and write the rest
	// Set the TXND pointer to correspond to the payload size given
	address = (TXSTART_INIT + ETH_HEADER_LEN + len );
	enc28j60WriteReg(ETXNDL, address & 0xFF);
	enc28j60WriteReg(ETXNDH, address >> 8);
	
	// Send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}


void enc28j60clkout(uint8_t clk)
{
	//setup clkout: 2 is 12.5MHz:
	enc28j60WriteReg(ECOCON, clk & 0x7);
}
	
//------------------------------------------------------------------------------

