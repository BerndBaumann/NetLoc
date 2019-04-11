/*
 * Copyright (c) 2017, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * \file
 *		EVB1000 Contiki Platform Main
 *
 * \author
 * 		Pablo Corbalan <p.corbalanpelegrin@unitn.it>
 */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "dev/watchdog.h"
#include "dev/radio.h"
#include "dev/button-sensor.h"
//#include "sys/node-id.h"
#include "lib/random.h"
#include "net/netstack.h"
#include <stdio.h>
#include "serial-line.h"
/*---------------------------------------------------------------------------*/
/* For IPv6 Stack */
#include "net/queuebuf.h"
#include "net/ip/tcpip.h"
#include "net/ip/uip.h"
/*---------------------------------------------------------------------------*/
#include "stm32l1xx_nucleo.h"
#include "stm32cube_hal_init.h"
#include "leds.h"
#include "dw1000-arch.h"
/*---------------------------------------------------------------------------*/
/* DW1000 Radio Driver */
#include "deca_device_api.h"
/*---------------------------------------------------------------------------*/
unsigned short node_id = 0;
/*---------------------------------------------------------------------------*/

SENSORS(&button_sensor);

/*---------------------------------------------------------------------------*/
/* This function must be called after initializing the DW1000 radio */
static void
set_rf_params(void)
{
    uint16_t short_addr;
    uint32_t part_id, lot_id;
    uint8_t saddr[2]; /* Short address -- 2 bytes */
    uint8_t laddr[8]; /* Long address -- 8 bytes */
    uint8_t rladdr[8]; /* Reverse Long address -- 8 bytes */

    /* Read from the DW1000 OTP memory the DW1000 PART and LOT IDs */
    part_id = dwt_getpartid();
    lot_id = dwt_getlotid();

    /* Compute the Link Layer address depending on the PART and LOT IDs */
    /* In little endian */
    /* Short address */
    saddr[0] = 0xFF & part_id;
    saddr[1] = 0xFF & (part_id >> 8);
    short_addr = ((0xFFFF & saddr[0]) << 8) | (0xFFFF & saddr[1]);

    /* Long address */
    laddr[0] = (part_id & 0x000000FF);
    laddr[1] = (part_id & 0x0000FF00) >> 8;
    laddr[2] = (part_id & 0x00FF0000) >> 16;
    laddr[3] = (part_id & 0xFF000000) >> 24;

    laddr[4] = (lot_id & 0x000000FF);
    laddr[5] = (lot_id & 0x0000FF00) >> 8;
    laddr[6] = (lot_id & 0x00FF0000) >> 16;
    laddr[7] = (lot_id & 0xFF000000) >> 24;

    /* Set the LLADDR */
#if LINKADDR_SIZE == 2
    memcpy(&linkaddr_node_addr, saddr, LINKADDR_SIZE);
#else /* LINKADDR_SIZE == 2 */
    memcpy(&linkaddr_node_addr, laddr, LINKADDR_SIZE);
#endif /* LINKADDR_SIZE == 2 */

#if NETSTACK_CONF_WITH_IPV6
    memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
    queuebuf_init();
    process_start(&tcpip_process, NULL);
#endif /* NETSTACK_CONF_WITH_IPV6 */

    /* We reverse the long address to set it on the radio chip */
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        rladdr[i] = laddr[7 - i];
    }

    /* Set up the IEEE 802.15.4 Short and Long address
     * to be able to enable HW frame filtering */
    NETSTACK_RADIO.set_value(RADIO_PARAM_16BIT_ADDR, short_addr);
    NETSTACK_RADIO.set_object(RADIO_PARAM_64BIT_ADDR, rladdr, 8);

    /* Set the PAN ID -- required for HW frame filtering */
    NETSTACK_RADIO.set_value(RADIO_PARAM_PAN_ID, IEEE802154_PANID);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Main function for EVB1000 platform
 */
int
main(void)
{
    stm32cube_hal_init();
    SdkEvalSpiInit();
    GpioIRQInit();

    leds_init();

    /* Init Contiki Clock module */
    clock_init();

    ctimer_init();
    /* Init rtimer based on RTC clock */
    rtimer_init();

    /* Init the independent watchdog */
    watchdog_init();

    process_init();

    process_start(&etimer_process, NULL);


    energest_init();
    ENERGEST_ON(ENERGEST_TYPE_CPU);

    /* Init network stack */
    netstack_init();

    /* Set the link layer addresses and the PAN ID */
    set_rf_params();

    /* Init pseudo-random generator with a chip-id based seed */
    random_init(0xFFFF & dwt_getpartid());

    process_start(&sensors_process, NULL);

    serial_line_init();

#if LINKADDR_SIZE == 2
    char str[25];
    snprintf(str,
             25,
             "Contiki on node %3d.%3d\n",
             linkaddr_node_addr.u8[0],
             linkaddr_node_addr.u8[1]);
    printf(str);
#else
    char str[100];
    snprintf(str,
             100,
             "\n\nContiki on node %02x %02x %02x %02x %02x %02x %02x %02x\n",
             linkaddr_node_addr.u8[0],
             linkaddr_node_addr.u8[1],
             linkaddr_node_addr.u8[2],
             linkaddr_node_addr.u8[3],
             linkaddr_node_addr.u8[4],
             linkaddr_node_addr.u8[5],
             linkaddr_node_addr.u8[6],
             linkaddr_node_addr.u8[7]);
    printf(str);
#endif

    debug_gpio_init();

    /* Start application processes */
    autostart_start(autostart_processes);

    /* Start Independent WDG */
    watchdog_start();

    while (1)
    {
        uint8_t r;
        do
        {
            r = process_run();
            watchdog_periodic();
        }
        while (r > 0);

    }

    return 0;
}
/*---------------------------------------------------------------------------*/
