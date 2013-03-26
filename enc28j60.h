//*****************************************************************************
// Title		: Microchip ENC28J60 Ethernet Interface Driver
// Author		: Pascal Stang (c)2005
//*****************************************************************************

#ifndef ENC28J60_H
#define ENC28J60_H

#include "ch.h"
#include "hal.h"

void     enc28j60Init(SPIDriver *spip);
void     enc28j60PacketSend(uint16_t len1, uint8_t *pkt1, uint16_t len2, uint8_t *pkt2);
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t *pkt);
void     enc28j60RegDump(BaseSequentialStream *p);
void     enc28j60EnableInt(void);
void     enc28j60DisableInt(void);
void     enc28j60ClearTxInt(void);

#endif
