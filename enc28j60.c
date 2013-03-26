//*****************************************************************************
// Title		: Microchip ENC28J60 Ethernet Interface Driver
// Author		: Pascal Stang (c)2005
// Created		: 9/22/2005
// Revised		: 9/22/2005
// Version		: 0.1
//*****************************************************************************

#include "ch.h"
#include "hal.h"
#include "../../os/various/chprintf.h"

#include "enc28j60.h"
#include "uip-conf.h"

// ENC28J60 Control Registers
// Control register definitions are a combination of address,
// bank number, and Ethernet/MAC/PHY indicator bits.
// - Register address	(bits 0-4)
// - Bank number		(bits 5-6)
// - MAC/PHY indicator	(bit 7)
#define ADDR_MASK                   0x1F
#define BANK_MASK                   0x60
#define SPRD_MASK                   0x80

// All-bank registers
#define EIE                         0x1B
#define EIR                         0x1C
#define ESTAT                       0x1D
#define ECON2                       0x1E
#define ECON1                       0x1F

#define BANK0                       0x00
#define BANK1                       0x20
#define BANK2                       0x40
#define BANK3                       0x60

// Bank 0 registers
#define ERDPTL                      (0x00 | BANK0)
#define ERDPTH                      (0x01 | BANK0)
#define EWRPTL                      (0x02 | BANK0)
#define EWRPTH                      (0x03 | BANK0)
#define ETXSTL                      (0x04 | BANK0)
#define ETXSTH                      (0x05 | BANK0)
#define ETXNDL                      (0x06 | BANK0)
#define ETXNDH                      (0x07 | BANK0)
#define ERXSTL                      (0x08 | BANK0)
#define ERXSTH                      (0x09 | BANK0)
#define ERXNDL                      (0x0A | BANK0)
#define ERXNDH                      (0x0B | BANK0)
#define ERXRDPTL                    (0x0C | BANK0)
#define ERXRDPTH                    (0x0D | BANK0)
#define ERXWRPTL                    (0x0E | BANK0)
#define ERXWRPTH                    (0x0F | BANK0)
#define EDMASTL                     (0x10 | BANK0)
#define EDMASTH                     (0x11 | BANK0)
#define EDMANDL                     (0x12 | BANK0)
#define EDMANDH                     (0x13 | BANK0)
#define EDMADSTL                    (0x14 | BANK0)
#define EDMADSTH                    (0x15 | BANK0)
#define EDMACSL                     (0x16 | BANK0)
#define EDMACSH                     (0x17 | BANK0)

#define EHT0                        (0x00 | BANK1)
#define EHT1                        (0x01 | BANK1)
#define EHT2                        (0x02 | BANK1)
#define EHT3                        (0x03 | BANK1)
#define EHT4                        (0x04 | BANK1)
#define EHT5                        (0x05 | BANK1)
#define EHT6                        (0x06 | BANK1)
#define EHT7                        (0x07 | BANK1)
#define EPMM0                       (0x08 | BANK1)
#define EPMM1                       (0x09 | BANK1)
#define EPMM2                       (0x0A | BANK1)
#define EPMM3                       (0x0B | BANK1)
#define EPMM4                       (0x0C | BANK1)
#define EPMM5                       (0x0D | BANK1)
#define EPMM6                       (0x0E | BANK1)
#define EPMM7                       (0x0F | BANK1)
#define EPMCSL                      (0x10 | BANK1)
#define EPMCSH                      (0x11 | BANK1)
#define EPMOL                       (0x14 | BANK1)
#define EPMOH                       (0x15 | BANK1)
#define EWOLIE                      (0x16 | BANK1)
#define EWOLIR                      (0x17 | BANK1)
#define ERXFCON                     (0x18 | BANK1)
#define EPKTCNT                     (0x19 | BANK1)

#define MACON1                      (0x00 | BANK2)
#define MACON3                      (0x02 | BANK2)
#define MACON4                      (0x03 | BANK2)
#define MABBIPG                     (0x04 | BANK2)
#define MAIPGL                      (0x06 | BANK2)
#define MAIPGH                      (0x07 | BANK2)
#define MACLCON1                    (0x08 | BANK2)
#define MACLCON2                    (0x09 | BANK2)
#define MAMXFLL                     (0x0A | BANK2)
#define MAMXFLH                     (0x0B | BANK2)
#define MAPHSUP                     (0x0D | BANK2)
#define MICON                       (0x11 | BANK2)
#define MICMD                       (0x12 | BANK2)
#define MIREGADR                    (0x14 | BANK2)
#define MIWRL                       (0x16 | BANK2)
#define MIWRH                       (0x17 | BANK2)
#define MIRDL                       (0x18 | BANK2)
#define MIRDH                       (0x19 | BANK2)

#define MAADR5                      (0x00 | BANK3)
#define MAADR6                      (0x01 | BANK3)
#define MAADR3                      (0x02 | BANK3)
#define MAADR4                      (0x03 | BANK3)
#define MAADR1                      (0x04 | BANK3)
#define MAADR2                      (0x05 | BANK3)
#define EBSTSD                      (0x06 | BANK3)
#define EBSTCON                     (0x07 | BANK3)
#define EBSTCSL                     (0x08 | BANK3)
#define EBSTCSH                     (0x09 | BANK3)
#define MISTAT                      (0x0A | BANK3 | 0x80)
#define EREVID                      (0x12 | BANK3)
#define ECOCON                      (0x15 | BANK3)
#define EFLOCON                     (0x17 | BANK3)
#define EPAUSL                      (0x18 | BANK3)
#define EPAUSH                      (0x19 | BANK3)

// PHY registers
#define PHCON1                      0x00
#define PHSTAT1                     0x01
#define PHHID1                      0x02
#define PHHID2                      0x03
#define PHCON2                      0x10
#define PHSTAT2                     0x11
#define PHIE                        0x12
#define PHIR                        0x13
#define PHLCON                      0x14

// ENC28J60 EIE Register Bit Definitions
#define EIE_INTIE                   0x80
#define EIE_PKTIE                   0x40
#define EIE_DMAIE                   0x20
#define EIE_LINKIE                  0x10
#define EIE_TXIE                    0x08
#define EIE_WOLIE                   0x04
#define EIE_TXERIE                  0x02
#define EIE_RXERIE                  0x01

// ENC28J60 EIR Register Bit Definitions
#define EIR_PKTIF                   0x40
#define EIR_DMAIF                   0x20
#define EIR_LINKIF                  0x10
#define EIR_TXIF                    0x08
#define EIR_WOLIF                   0x04
#define EIR_TXERIF                  0x02
#define EIR_RXERIF                  0x01

// ENC28J60 ESTAT Register Bit Definitions
#define ESTAT_INT                   0x80
#define ESTAT_LATECOL               0x10
#define ESTAT_RXBUSY                0x04
#define ESTAT_TXABRT                0x02
#define ESTAT_CLKRDY                0x01

// ENC28J60 ECON2 Register Bit Definitions
#define ECON2_AUTOINC               0x80
#define ECON2_PKTDEC                0x40
#define ECON2_PWRSV                 0x20
#define ECON2_VRPS                  0x08

// ENC28J60 ECON1 Register Bit Definitions
#define ECON1_TXRST                 0x80
#define ECON1_RXRST                 0x40
#define ECON1_DMAST                 0x20
#define ECON1_CSUMEN                0x10
#define ECON1_TXRTS                 0x08
#define ECON1_RXEN                  0x04
#define ECON1_BSEL1                 0x02
#define ECON1_BSEL0                 0x01

// ENC28J60 MACON1 Register Bit Definitions
#define MACON1_LOOPBK               0x10
#define MACON1_TXPAUS               0x08
#define MACON1_RXPAUS               0x04
#define MACON1_PASSALL              0x02
#define MACON1_MARXEN               0x01

// ENC28J60 MACON2 Register Bit Definitions
#define MACON2_MARST                0x80
#define MACON2_RNDRST               0x40
#define MACON2_MARXRST              0x08
#define MACON2_RFUNRST              0x04
#define MACON2_MATXRST              0x02
#define MACON2_TFUNRST              0x01

// ENC28J60 MACON3 Register Bit Definitions
#define MACON3_PADCFG2              0x80
#define MACON3_PADCFG1              0x40
#define MACON3_PADCFG0              0x20
#define MACON3_TXCRCEN              0x10
#define MACON3_PHDRLEN              0x08
#define MACON3_HFRMLEN              0x04
#define MACON3_FRMLNEN              0x02
#define MACON3_FULDPX               0x01

/* MACON4 */
#define MACON4_DEFER                0x40
#define MACON4_BPEN                 0x20
#define MACON4_NOBKOFF              0x10

// ENC28J60 MICMD Register Bit Definitions
#define MICMD_MIISCAN               0x02
#define MICMD_MIIRD                 0x01

// ENC28J60 MISTAT Register Bit Definitions
#define MISTAT_NVALID               0x04
#define MISTAT_SCAN                 0x02
#define MISTAT_BUSY                 0x01

// ENC28J60 PHY PHCON1 Register Bit Definitions
#define PHCON1_PRST                 0x8000
#define PHCON1_PLOOPBK              0x4000
#define PHCON1_PPWRSV               0x0800
#define PHCON1_PDPXMD               0x0100

// ENC28J60 PHY PHSTAT1 Register Bit Definitions
#define PHSTAT1_PFDPX               0x1000
#define PHSTAT1_PHDPX               0x0800
#define PHSTAT1_LLSTAT              0x0004
#define PHSTAT1_JBSTAT              0x0002

// ENC28J60 PHY PHCON2 Register Bit Definitions
#define PHCON2_FRCLINK              0x4000
#define PHCON2_TXDIS                0x2000
#define PHCON2_JABBER               0x0400
#define PHCON2_HDLDIS               0x0100

// ENC28J60 Packet Control Byte Bit Definitions
#define PKTCTRL_PHUGEEN             0x08
#define PKTCTRL_PPADEN              0x04
#define PKTCTRL_PCRCEN              0x02
#define PKTCTRL_POVERRIDE           0x01

/* SPI instructions */
#define READ_CTRL_REG               0x00
#define READ_BUF_MEM                0x3A
#define WRITE_CTRL_REG              0x40
#define WRITE_BUF_MEM               0x7A
#define BIT_FIELD_SET               0x80
#define BIT_FIELD_CLR               0xA0
#define SOFT_RESET                  0xFF

#define TXSTART_INIT                0x0000
#define RXSTART_INIT                0x0600
#define RXSTOP_INIT                 0x1FFF
#define MAX_FRAMELEN                1518
#define ETHERNET_MIN_PACKET_LENGTH  0x3C

static uint8_t    _bank             = 0xFF;
static uint16_t   _nextPacket       = 0;
static SPIDriver *_spip             = NULL;

static uint8_t _readOp(uint8_t op, uint8_t addr)
{
  uint8_t data;
  uint8_t tx[3] = { op | (addr & ADDR_MASK), 0, 0 };
  uint8_t rx[3] = { 0 };

  spiSelect(_spip);
  if (addr & 0x80) {
    spiExchange(_spip, 3, tx, rx);
    data = rx[2];
  } else {
    spiExchange(_spip, 2, tx, rx);
    data = rx[1];
  }
  spiUnselect(_spip);

  return data;
}

static void _writeOp(uint8_t op, uint8_t addr, uint8_t data)
{
  uint8_t tx[2] = { op | (addr & ADDR_MASK), data };

  spiSelect(_spip);
  spiSend(_spip, 2, tx);
  spiUnselect(_spip);
}

void _readBuffer(uint16_t len, uint8_t *data)
{
  uint8_t tx[] = { READ_BUF_MEM };

  spiSelect(_spip);
  spiSend(_spip, 1, tx);
  spiReceive(_spip, len, data);
  spiUnselect(_spip);
}

void _writeBuffer(uint16_t len, uint8_t *data)
{
  uint8_t tx[] = { WRITE_BUF_MEM };

  spiSelect(_spip);
  spiSend(_spip, 1, tx);
  spiSend(_spip, len, data);
  spiUnselect(_spip);
}

static void _setBank(uint8_t address)
{
  if ((address & BANK_MASK) != _bank) {
    _writeOp(BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
    _writeOp(BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
    _bank = (address & BANK_MASK);
  }
}

static uint8_t _read(uint8_t addr)
{
  _setBank(addr);
  return _readOp(READ_CTRL_REG, addr);
}

static void _write(uint8_t addr, uint8_t data)
{
  _setBank(addr);
  _writeOp(WRITE_CTRL_REG, addr, data);
}

#if 0
static uint16_t _phyRead(uint8_t address)
{
  uint16_t data;

  // Set the right address and start the register read operation
  _write(MIREGADR, address);
  _write(MICMD, MICMD_MIIRD);

  // wait until the PHY read completes
  while (_read(MISTAT) & MISTAT_BUSY) ;

  // quit reading
  _write(MICMD, 0x00);

  // get data value
  data  = _read(MIRDL);
  data |= _read(MIRDH);

  // return the data
  return data;
}
#endif

static void _phyWrite(uint8_t address, uint16_t data)
{
  _write(MIREGADR, address);

  _write(MIWRL, data);
  _write(MIWRH, data >> 8);

  while (_read(MISTAT) & MISTAT_BUSY)
    ;
}

void enc28j60Init(SPIDriver *spip)
{
  _spip = spip;

  _writeOp(SOFT_RESET, 0, SOFT_RESET);

  chThdSleepMilliseconds(10);
  while (!_readOp(READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY) ;

  /* Initialization - chapter 6.0 */

  /* Start address of the receive buffer */
  _write(ERXSTL, RXSTART_INIT & 0xFF);
  _write(ERXSTH, RXSTART_INIT >> 8);

  /* Receive pointer */
  _write(ERXRDPTL, RXSTART_INIT & 0xFF);
  _write(ERXRDPTH, RXSTART_INIT >> 8);

  /* End addres of the receive buffer */
  _write(ERXNDL, RXSTOP_INIT & 0xFF);
  _write(ERXNDH, RXSTOP_INIT >> 8);

  /* Start address of the transmit buffer */
  _write(ETXSTL, TXSTART_INIT & 0xFF);
  _write(ETXSTH, TXSTART_INIT >> 8);

  _nextPacket = RXSTART_INIT;

  /* MAC initialization */

  /* Enable frame reception */
  _write(MACON1, MACON1_MARXEN);

  /* FIXME: enable full-duplex mode? */
  _write(MACON3, MACON3_PADCFG1 | MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);

  _write(MACON4, MACON4_DEFER);

  /* Maximum frame length */
  _write(MAMXFLL, MAX_FRAMELEN & 0xFF);
  _write(MAMXFLH, MAX_FRAMELEN >> 8);

  /* Back-to-back inter-packet gap delay */
  _write(MABBIPG, 0x12);

  /* Non-back-to-back inter-packet gap delay */
  _write(MAIPGL, 0x12);
  _write(MAIPGH, 0x0C);

  _write(MAADR1, UIP_ETHADDR0);
  _write(MAADR2, UIP_ETHADDR1);
  _write(MAADR3, UIP_ETHADDR2);
  _write(MAADR4, UIP_ETHADDR3);
  _write(MAADR5, UIP_ETHADDR4);
  _write(MAADR6, UIP_ETHADDR5);

  /* PHY initialization */

  /* Half-duplex loopback disable */
  _phyWrite(PHCON2, PHCON2_HDLDIS);

  /* Enable interrupt for packet reception*/
  _writeOp(BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);

  /* Receive enable */
  _writeOp(BIT_FIELD_SET, ECON1, ECON1_RXEN);

  /* LEDs */
  _phyWrite(PHLCON, 0x0472);
}

void enc28j60PacketSend(uint16_t len1, uint8_t *pkt1, uint16_t len2, uint8_t *pkt2)
{
  //Errata: Transmit Logic reset
  _writeOp(BIT_FIELD_SET, ECON1, ECON1_TXRST);
  _writeOp(BIT_FIELD_CLR, ECON1, ECON1_TXRST);

  // Set the write pointer to start of transmit buffer area
  _write(EWRPTL, TXSTART_INIT & 0xFF);
  _write(EWRPTH, TXSTART_INIT >> 8);

  // write per-packet control byte
  _writeOp(WRITE_BUF_MEM, 0, 0x00);

  // copy the packet into the transmit buffer
  _writeBuffer(len1, pkt1);
  if (len2 > 0)
    _writeBuffer(len2, pkt2);

  // Set the TXND pointer to correspond to the packet size given
  _write(ETXNDL,  TXSTART_INIT + len1 + len2);
  _write(ETXNDH, (TXSTART_INIT + len1 + len2) >> 8);

  // send the contents of the transmit buffer onto the network
  _writeOp(BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t *pkt)
{
  uint16_t rxstat;
  uint16_t len;

  if (!_read(EPKTCNT))
    return 0;

  // Set the read pointer to the start of the received packet
  _write(ERDPTL, _nextPacket);
  _write(ERDPTH, _nextPacket >> 8);

  // read the next packet pointer
  _nextPacket  = _readOp(READ_BUF_MEM, 0);
  _nextPacket |= _readOp(READ_BUF_MEM, 0) << 8;

  // read the packet length
  len  = _readOp(READ_BUF_MEM, 0);
  len |= _readOp(READ_BUF_MEM, 0) << 8;

  // read the receive status
  rxstat  = _readOp(READ_BUF_MEM, 0);
  rxstat |= _readOp(READ_BUF_MEM, 0) << 8;

  // limit retrieve length
  // (we reduce the MAC-reported length by 4 to remove the CRC)
  len -= 4;
  if (len >= maxlen)
    len = maxlen;

  // copy the packet from the receive buffer
  _readBuffer(len, pkt);

  // Move the RX read pointer to the start of the next received packet
  // This frees the memory we just read out
  _write(ERXRDPTL, _nextPacket);
  _write(ERXRDPTH, _nextPacket >> 8);

  // decrement the packet counter indicate we are done with this packet
  _writeOp(BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

  return len;
}

void enc28j60EnableInt(void)
{
  _writeOp(BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);
}

void enc28j60DisableInt(void)
{
  _writeOp(BIT_FIELD_CLR, EIE, EIE_INTIE);
}

void enc28j60ClearTxInt(void)
{
  _writeOp(BIT_FIELD_CLR, EIR, EIR_TXIF);
}

void enc28j60RegDump(BaseSequentialStream *p)
{
	chprintf(p, "%x:",    _read(MAADR1));
	chprintf(p, "%x:",    _read(MAADR2));
	chprintf(p, "%x:",    _read(MAADR3));
	chprintf(p, "%x:",    _read(MAADR4));
	chprintf(p, "%x:",    _read(MAADR5));
	chprintf(p, "%x\r\n", _read(MAADR6));

	chprintf(p, "REVID: 0x%x\r\n", _read(EREVID));
	chprintf(p, "ECON1=0x%x ECON2=0x%x ESTAT=0x%x\r\n",
	            _read(ECON1), _read(ECON2), _read(ESTAT));
	chprintf(p, "EIR=0x%x EIE=0x%x\r\n", _read(EIR), _read(EIE));
	chprintf(p, "MACON1=0x%0x MACON2=0x%x MACON3=0x%x MACON4=0x%x\r\n",
	            _read(MACON1), 0/*_read(MACON2)*/, _read(MACON3), _read(MACON4));
	chprintf(p, "ERXST=0x%x%x ERXND=0x%x%x\r\n",
	            _read(ERXSTH), _read(ERXSTL), _read(ERXNDH), _read(ERXNDL));
	chprintf(p, "ERXWRPT=0x%x%x ERXRDPT=0x%x%x\r\n",
	            _read(ERXWRPTH), _read(ERXWRPTL), _read(ERXRDPTH), _read(ERXRDPTL));
	chprintf(p, "ERXFCON=0x%x EPKTCNT=0x%x MAMXFL=0x%x%x\r\n",
	            _read(ERXFCON), _read(EPKTCNT), _read(MAMXFLH), _read(MAMXFLL));
	chprintf(p, "ETXST=0x%x%x ETXND=0x%x%x\r\n",
	            _read(ETXSTH), _read(ETXSTL), _read(ETXNDH), _read(ETXNDL));
	chprintf(p, "MACLCON1=0x%x MACLCON2=0x%x MAPHSUP=0x%x\r\n",
	            _read(MACLCON1), _read(MACLCON2), _read(MAPHSUP));
}
