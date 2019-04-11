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
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/nbr-table.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif

#include "dw1000.h"
#include "dw1000-ranging.h"

#include <stdio.h>
#include <string.h>

#include "dev/serial-line.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ip/uiplib.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#ifndef PERIOD
#define PERIOD 30
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		30

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK
void range_cb_neighbor_added(const linkaddr_t *addr);
#endif

static struct etimer rng_to;
float neighbor_dist;
linkaddr_t rng_dst_;
uip_ds6_nbr_t *rng_nbr;

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


/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
PROCESS(neighbor_ranging, "ranging process");
AUTOSTART_PROCESSES(&udp_client_process, &neighbor_ranging);
/*---------------------------------------------------------------------------*/
static int seq_id;
static int reply;

static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    reply++;
    printf("DATA recv '%s' (s:%d, r:%d)\n", str, seq_id, reply);
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  char buf[MAX_PAYLOAD_LEN];
  //process_start(&neighbor_ranging, NULL);

#ifdef SERVER_REPLY
  uint8_t num_used = 0;
  uip_ds6_nbr_t *nbr;

  nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    nbr = nbr_table_next(ds6_neighbors, nbr);
    num_used++;
  }

  if(seq_id > 0) {
    ANNOTATE("#A r=%d/%d,color=%s,n=%d %d\n", reply, seq_id,
             reply == seq_id ? "GREEN" : "RED", uip_ds6_route_num_routes(), num_used);
  }
#endif /* SERVER_REPLY */

  seq_id++;
  PRINTF("DATA sent to %d 'Hello %d'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  sprintf(buf, "Hello %d from the client", seq_id);
  uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
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
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoWPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our
 * link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the
 * 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::. At present Wireshark copies Context/128 and
 * then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit
 * compressed address of fd00::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed
 * addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 0
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uiplib_ipaddrconv("fd00::a723:10:e75f:2010", &server_ipaddr);
  //uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0xa723, 0x0010, 0xe75f, 0x2010);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif
}

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

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;
  static struct ctimer backoff_timer;
#if WITH_COMPOWER
  static int print = 0;
#endif

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
  // `--------------- -9.0dB gain

  // channel 4, 16MHz PRF. default: 0x38383838 = 001 11000
  uint32_t txpower = 0x20202020;
  dwt_write32bitreg(TX_POWER_ID, txpower);

  seq_id = 1000;

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started nbr:%d routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }

    if(ev == serial_line_event_message && data != NULL) {
      char *str;
      str = data;
      if(str[0] == 'r') {
        uip_ds6_route_t *r;
        uip_ipaddr_t *nexthop;
        uip_ds6_defrt_t *defrt;
        uip_ipaddr_t *ipaddr;
        defrt = NULL;
        if((ipaddr = uip_ds6_defrt_choose()) != NULL) {
          defrt = uip_ds6_defrt_lookup(ipaddr);
        }
        if(defrt != NULL) {
          PRINTF("DefRT: :: -> %02d", defrt->ipaddr.u8[15]);
          PRINTF(" lt:%lu inf:%d\n", stimer_remaining(&defrt->lifetime),
                 defrt->isinfinite);
        } else {
          PRINTF("DefRT: :: -> NULL\n");
        }

        for(r = uip_ds6_route_head();
            r != NULL;
            r = uip_ds6_route_next(r)) {
          nexthop = uip_ds6_route_nexthop(r);
          PRINTF("Route: %02d -> %02d", r->ipaddr.u8[15], nexthop->u8[15]);
          /* PRINT6ADDR(&r->ipaddr); */
          /* PRINTF(" -> "); */
          /* PRINT6ADDR(nexthop); */
          PRINTF(" lt:%lu\n", r->state.lifetime);

        }
      }
    }

    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);

#if WITH_COMPOWER
      if (print == 0) {
	powertrace_print("#P");
      }
      if (++print == 3) {
	print = 0;
      }
#endif

    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

#ifdef NETSTACK_CONF_ROUTING_NEIGHBOR_ADDED_CALLBACK
void range_cb_neighbor_added(const linkaddr_t *addr)
{
    process_start(&neighbor_ranging, addr);
}
#endif
