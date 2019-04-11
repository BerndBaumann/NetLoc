/*
 * Copyright (c) 2015, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


#define DW1000_CONF_RANGING_ENABLED 1

#include "deca_device_api.h"

#define APP_RADIO_CONF 1

#if APP_RADIO_CONF == 1
#define DW1000_CONF_CHANNEL        4
#define DW1000_CONF_PRF            DWT_PRF_16M
#define DW1000_CONF_PLEN           DWT_PLEN_128
#define DW1000_CONF_PAC            DWT_PAC8
#define DW1000_CONF_SFD_MODE       0
#define DW1000_CONF_DATA_RATE      DWT_BR_6M8
#define DW1000_CONF_PHR_MODE       DWT_PHRMODE_STD
#define DW1000_CONF_PREAMBLE_CODE  7
#define DW1000_CONF_SFD_TIMEOUT    (129 + 8 - 8)

#elif APP_RADIO_CONF == 2
#define DW1000_CONF_CHANNEL        2
#define DW1000_CONF_PRF            DWT_PRF_64M
#define DW1000_CONF_PLEN           DWT_PLEN_1024
#define DW1000_CONF_PAC            DWT_PAC32
#define DW1000_CONF_SFD_MODE       0
#define DW1000_CONF_DATA_RATE      DWT_BR_110K
#define DW1000_CONF_PHR_MODE       DWT_PHRMODE_STD
#define DW1000_CONF_PREAMBLE_CODE  9
#define DW1000_CONF_SFD_TIMEOUT    (1025 + 64 - 32)
#endif


#define NETSTACK_CONF_NETWORK    sicslowpan_driver
#define NETSTACK_CONF_MAC        nullmac_driver
#define NETSTACK_CONF_RDC        nullrdc_driver

#define DW1000_CONF_AUTOACK         0
#define DW1000_CONF_FRAMEFILTER     0
#define DW1000_CONF_RANGING_ENABLED 1

#define DW1000_CONF_READ_RXDIAG     1

#define NETSTACK_CONF_WITH_IPV6 1

#define UIP_CONF_ROUTER 1

//#define RPL_CONF_STATS 1
//#define RPL_CONF_WITH_DAO_ACK 1
//#define RPL_CONF_OF_OCP RPL_OCP_OF0
//#define NULLRDC_CONF_802154_AUTOACK    1
//#define NULLRDC_CONF_802154_AUTOACK_HW 0
//#define NULLRDC_CONF_ACK_WAIT_TIME (RTIMER_SECOND) see contiki-conf.h
//#define NULLRDC_CONF_SEND_802154_ACK 1
//#define RPL_CONF_DIO_REFRESH_DAO_ROUTES 1

//#define NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK range_cb_neighbor_added

//-----------------------------------------------------------------------------

#ifndef WITH_NON_STORING
#define WITH_NON_STORING 0 /* Set this to run with non-storing mode */
#endif /* WITH_NON_STORING */

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES

//#define TEST_MORE_ROUTES
//#ifdef TEST_MORE_ROUTES
///* configure number of neighbors and routes */
//#define NBR_TABLE_CONF_MAX_NEIGHBORS     10
//#define UIP_CONF_MAX_ROUTES   30
//#else
///* configure number of neighbors and routes */
//#define NBR_TABLE_CONF_MAX_NEIGHBORS     10
//#define UIP_CONF_MAX_ROUTES   10
//#endif /* TEST_MORE_ROUTES */

/* Define as minutes */
//#define RPL_CONF_DEFAULT_LIFETIME_UNIT   60

/* 10 minutes lifetime of routes */
//#define RPL_CONF_DEFAULT_LIFETIME        10

//#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME 1

#if WITH_NON_STORING
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 40 /* Number of links maintained at the root. Can be set to 0 at non-root nodes. */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES 0 /* No need for routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NON_STORING /* Mode of operation*/
#endif /* WITH_NON_STORING */

#define SERVER_REPLY 1

#endif
