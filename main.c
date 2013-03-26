/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"
#include "../../os/various/chprintf.h"

#include "uip.h"
#include "uip_arp.h"
//#include "httpd.h"

#include "enc28j60.h"

static BaseSequentialStream *serp = (BaseSequentialStream *) &SD1;

const SPIConfig enc28j60_config = {
  IOPORT2,
  PB0,
  SPI_MODE_0,
  SPI_MSB_FIRST,
  SPI_SCK_FOSC_8,
  NULL,
};

const SPIDriver *spip = &SPID;

static Thread *tp = NULL;

CH_IRQ_HANDLER(INT0_vect) {
  CH_IRQ_PROLOGUE();

  chSysLockFromIsr();
  chEvtSignalI(tp, (eventmask_t) 1);
  chSysUnlockFromIsr();

  CH_IRQ_EPILOGUE();
}

uint8_t tx[3];
uint8_t rx[3];

static WORKING_AREA(waThread1, 256);

static void ARPTimerHandler(void);
static void PeriodicTimerHandler(void);

static msg_t Thread1(void *p) {

  (void)p;
  uint8_t t = 50;

  while (TRUE) {
    if (!t--) {
      palTogglePad(IOPORT2, PORTB_LED1);
      t = 50;
    }
    //enc28j60RegDump(serp);
    ARPTimerHandler();
    PeriodicTimerHandler();
    chThdSleepMilliseconds(10);
  }
  return 0;
}

static void send()
{
  //chprintf(serp, "send(): uip_len=%d\r\n", uip_len);
  if (uip_len <= UIP_LLH_LEN + 40) {
    enc28j60PacketSend(uip_len, (uint8_t *) uip_buf, 0, 0);
  } else {
    enc28j60PacketSend(54, (uint8_t *) uip_buf , uip_len - UIP_LLH_LEN - 40, (uint8_t*) uip_appdata);
  }
}

static void PeriodicTimerHandler(void)
{
  int i;

  for (i = 0; i < UIP_CONNS; i++) {
    uip_periodic(i);
    if (uip_len > 0) {
      uip_arp_out();
      send();
    }
  }
}

static void ARPTimerHandler(void)
{
  uip_arp_timer();
}

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
static void FrameReceivedHandler(void)
{
  while (1) {
    uip_len = (uint16_t) enc28j60PacketReceive(UIP_BUFSIZE, (uint8_t *) uip_buf);
    if (uip_len > 0) {
      //chprintf(serp, "uip_len=%d\r\n", uip_len);
      //chprintf(serp, "BUF->type=%x\r\n", BUF->type);
      if (BUF->type == HTONS(UIP_ETHTYPE_IP)) {
        uip_arp_ipin();
        //chprintf(serp, "Calling uip_input()\r\n");
        uip_input();
        //chprintf(serp, "uip_len=%d\r\n", uip_len);
        if (uip_len > 0) {
          uip_arp_out();
          //chprintf(serp, "Calling send()\r\n");
          send();
        }
      }
      else if (BUF->type == HTONS(UIP_ETHTYPE_ARP)) {
        uip_arp_arpin();
        if (uip_len > 0) {
          //chprintf(serp, "Calling send()\r\n");
          send();
        }
      }
    } else break;
  }
}

int main(void) {

  systime_t tick = chTimeNow();

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palClearPad(IOPORT2, PORTB_LED1);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /* PB0<SS>, PB1<SCK>, PB2<MOSI>, PD1<RESET> */
  DDRB  |= ((1 << DDB0) | (1 << DDB1) | (1 << DDB2));
  PORTB |= ((1 << PB0) | (1 << PB1) | (1 << PB2));
  DDRD  |= (1 << DDD1);

  /* Activate RESET line on enc28j60 */
  PORTD &= ~(1 << PD1);
  chThdSleepMilliseconds(1);
  PORTD |= (1 << PD1);

  /* Falling edge interrupt on INT0 */
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);

  spiStart(spip, &enc28j60_config);

  chThdSleepMilliseconds(10);
  enc28j60Init(spip);
  enc28j60RegDump(serp);

  chThdCreateStatic(waThread1, sizeof(waThread1), LOWPRIO, Thread1, NULL);

  tp = chThdSelf();
  EIMSK |= (1 << INT0);

  uip_init();
  //httpd_init();
  hello_world_init();

  while (1) {
    chEvtWaitAny((eventmask_t) 1);
    //chprintf(serp, "New packet\r\n");
    enc28j60DisableInt();
    FrameReceivedHandler();
    enc28j60EnableInt();
    enc28j60ClearTxInt();
    //enc28j60RegDump(serp);
  }
}
