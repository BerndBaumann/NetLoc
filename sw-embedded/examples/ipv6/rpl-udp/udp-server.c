/*
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
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/nbr-table.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"


#include "dw1000.h"
#include "dw1000-ranging.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define UDP_EXAMPLE_ID  190

static struct uip_udp_conn *server_conn;


/*---------------------------------------------------------------------------*/
#define RANGING_TIMEOUT (CLOCK_SECOND * 2)
#define APP_RADIO_CONF 1
dwt_config_t radio_config = {
#if APP_RADIO_CONF == 1
        .chan = 4,
        .prf = DWT_PRF_16M,
        .txPreambLength = DWT_PLEN_128,
        .dataRate = DWT_BR_6M8,
        .txCode = 7,
        .rxCode = 7,
        .rxPAC = DWT_PAC8,
        .nsSFD = 0 /* standard */,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (129 + 8 - 8),
#elif APP_RADIO_CONF == 2
        .chan = 2,
        .prf = DWT_PRF_64M,
        .txPreambLength = DWT_PLEN_1024,
        .dataRate = DWT_BR_110K,
        .txCode = 9,
        .rxCode = 9,
        .rxPAC = DWT_PAC32,
        .nsSFD = 0 /* non-standard */,
        .phrMode = DWT_PHRMODE_STD,
        .sfdTO = (1025 + 64 - 32),
#else
#error App: radio config is not set
#endif
};


PROCESS(neighbor_ranging, "Ranging process");
PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process, &neighbor_ranging);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *appdata;

  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    PRINTF("DATA recv [%ddB] '%s' from ", (int8_t)packetbuf_attr(PACKETBUF_ATTR_RSSI), appdata);
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("\n");
#if SERVER_REPLY
    PRINTF("DATA sending reply\n");
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
    uip_create_unspecified(&server_conn->ripaddr);
#endif
  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();
  dw1000_configure(&radio_config);

  // tx power setting
  // 7 6 5 4 3 2 1 0  [bit number]
  // c c c f f f f f  [c=coarse gain (attenuator), f=fine gain(mixer)]
  // | | | | | | | |
  // | | | | | | | `- +0.5dB gain
  // | | | | | | `--- +1.0dB gain
  // | | | | | `----- +2.0dB gain
  // | | | | `------- +4.0dB gain
  // | | | `--------- +8.0dB gain
  // | | |            (18dB - c c c = coarse gain)
  // | | `----------- -3.0dB gain
  // | `------------- -6.0dB gain
  // `--------------- -12 dB gain

  // channel 4, 16MHz PRF. default: 0x38383838 = 001 11000
  uint32_t txpower = 0xa0a0a0a0;
  dwt_write32bitreg(TX_POWER_ID, txpower);

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started. nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

#if UIP_CONF_ROUTER
/* The choice of server address determines its 6LoWPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct
 * uncompressed addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 0
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */
  
  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high 
     packet reception rates. */
  NETSTACK_MAC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initializing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

static struct etimer rng_to;
float neighbor_dist;
linkaddr_t rng_dst_;
uip_ds6_nbr_t *rng_nbr;

PROCESS_THREAD(neighbor_ranging, ev, data)
{
    PROCESS_BEGIN();

    while (1) {
        etimer_set(&rng_to, CLOCK_SECOND * 60);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&rng_to));

        for (rng_nbr = nbr_table_head(ds6_neighbors);
             rng_nbr != NULL;
             rng_nbr = nbr_table_next(ds6_neighbors, rng_nbr)) {
            printf("RANGING:: Start ranging with %x%x\n", rng_nbr->ipaddr.u8[8], rng_nbr->ipaddr.u8[9]);
            rng_dst_.u8[0] = 0x2 ^ rng_nbr->ipaddr.u8[8];
            rng_dst_.u8[1] = rng_nbr->ipaddr.u8[9];
            range_with(&rng_dst_, DW1000_RNG_DS);
            etimer_set(&rng_to, CLOCK_SECOND / 2);
            PROCESS_YIELD_UNTIL((ev == ranging_event || etimer_expired(&rng_to)));
            if (etimer_expired(&rng_to)) {
                printf("RANGING:: ranging timeout\n");
            } else {
                ranging_data_t *d = (ranging_data_t*)data;
                if (d->status) {
                    printf("RANGING:: ranging success: %f \n", d->distance);
                    neighbor_dist = (float)d->distance;
                } else {
                    printf("RANGING:: ranging failed\n");
                    neighbor_dist = 0.0f;
                }
            }
        }
    }


    PROCESS_END();
}
