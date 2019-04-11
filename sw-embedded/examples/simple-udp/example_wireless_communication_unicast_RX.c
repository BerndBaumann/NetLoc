/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"  // Contiki core
#include "contiki-net.h"
#include <stdio.h>
#include "button-sensor.h"
#include "dev/leds.h"

#include "simple-udp.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"
#include "net/ip/uiplib.h"

#include "net/rpl/rpl.h"

#include "deca_regs.h"

// UDP payload structure
struct udp_payload{
	uint32_t	sequence_number;
	uint8_t		seat_row;
	uint8_t		seat_column;
	float		temperature;
	uint32_t	energy;
	char		name[15];
};
#define ENERGEST_CONF_ON 0

#define UDP_PORT 4000
#define UIP_DS6_DEFAULT_PREFIX 0xaaaa

#define SEND_INTERVAL		(10 * CLOCK_SECOND)

static struct simple_udp_connection unicast_connection;

/*---------------------------------------------------------------------------*/
PROCESS(unicast_receiver_process, "Unicast receiver example process");
AUTOSTART_PROCESSES(&unicast_receiver_process);
/*---------------------------------------------------------------------------*/

// Floor function
float my_floor(float num){
	if(num>=0.0f) return (float) ((int)num);
	else return(float)((int)num-1);
}

static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
    printf("RECVD [%d] ", (int8_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));
	if(datalen == sizeof(struct udp_payload)){
		struct udp_payload mesg;
		memcpy(&mesg, data, sizeof(struct udp_payload));

		//printf("Data received from row=%d, col=%d: %lu, %u, %u, %s\n", mesg.seat_row, mesg.seat_column, mesg.sequence_number, mesg.seat_row, mesg.seat_column, mesg.name);
		printf("%lu %u %u ", mesg.sequence_number, mesg.seat_row, mesg.seat_column);
		printf("%ld.%02d ", (long) mesg.temperature, (unsigned) ((mesg.temperature-my_floor(mesg.temperature))*100));
		printf("%ld ", (long) mesg.energy);
		printf("%s\n", mesg.name);
	}
	else{
		printf("Error: data received on port %d from port %d with length %d (unknown length != %d)\n", receiver_port, sender_port, datalen, sizeof(struct udp_payload));
	}
}
/*---------------------------------------------------------------------------*/
static uip_ipaddr_t *
set_global_address(void)
{

	static uip_ipaddr_t ipaddr;
	uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
	uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
	uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

	printf("IPv6 addresses: ");
	uint16_t i = 0;
	for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		uint8_t state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused &&
				(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
			printf("\n");
		}
	}

	return &ipaddr;
}
/*---------------------------------------------------------------------------*/
static void
create_rpl_dag(uip_ipaddr_t *ipaddr)
{
	struct uip_ds6_addr *root_if;

	root_if = uip_ds6_addr_lookup(ipaddr);
	if(root_if != NULL) {
		rpl_dag_t *dag;
		uip_ipaddr_t prefix;

		rpl_set_root(RPL_DEFAULT_INSTANCE, ipaddr);
		dag = rpl_get_any_dag();
		uip_ip6addr(&prefix, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
		rpl_set_prefix(dag, &prefix, 64);
		PRINTF("created a new RPL dag\n");
	} else {
		PRINTF("failed to create a new RPL DAG\n");
	}
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(unicast_receiver_process, ev, data)
{
	PROCESS_EXITHANDLER()
		  PROCESS_BEGIN();

	printf("RECEIVER PROCESS STARTED\n");

	uip_ipaddr_t *ipaddr;

	ipaddr = set_global_address();
	printf("my address is ");
	uip_debug_ipaddr_print(ipaddr);
	printf("\n\n");

	create_rpl_dag(ipaddr);

	// Register simple UDP
	simple_udp_register(&unicast_connection, UDP_PORT, NULL, UDP_PORT, receiver);

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
    uint32_t txpower = 0xa0a0a0a0;
    dwt_write32bitreg(TX_POWER_ID, txpower);

	while(1) {
		PROCESS_WAIT_EVENT();
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
