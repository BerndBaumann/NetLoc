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


#include "dw1000.h"
#include "dw1000-ranging.h"

#define ENERGEST_CONF_ON 0

// Etimer settings
#define SEND_INTERVAL	(CLOCK_SECOND * 1)
static struct etimer periodic_timer;
static radio_value_t radio_actual_tx_power = 0;

// UDP settings and payload structure
#define UDP_PORT	4000
#define	UIP_DS6_DEFAULT_PREFIX	0xaaaa
static struct simple_udp_connection unicast_connection;
struct udp_payload{
	uint32_t	sequence_number;
	uint8_t		seat_row;
	uint8_t		seat_column;
	float		temperature;
	uint32_t	energy;
	char		name[15];
};
static struct udp_payload message;

static uip_ipaddr_t *my_ip_address;

static int status;
static struct etimer rng_to;

// Energest variables
#if ENERGEST_CONF_ON
	// Current consumption of the Sensortag [mA] (see CC2650 datasheet)
	#define TRANSMIT_CURRENT 6.1 // 6.1 at 0 dBm and 9.1 at +5 dBm
	#define LISTEN_CURRENT 	 5.9
	#define CPU_CURRENT  	 (1.45 + 1.48) // 1.45 + 31ua/MHz
	#define LPM_CURRENT  	 0.0027

	// Suppy voltage of the Sensortag [V]
	#define SUPPLY_VOLTAGE   3.3

	// Duration of CR2032 battery in mAh
	#define BATTERY_MAH 200

	// Measurement variables
	uint32_t rx_ticks = 0, tx_ticks = 0, cpu_ticks = 0, lpm_ticks = 0;
	uint32_t energy_rx = 0, energy_tx = 0, energy_cpu = 0, energy_lpm = 0, energy_total = 0;
	uint32_t average_power = 0, total_milliseconds = 0;
#endif

/*---------------------------------------------------------------------------*/

// Floor function
float my_floor(float num){
	if(num>=0.0f) return (float) ((int)num);
	else return(float)((int)num-1);
}


/*---------------------------------------------------------------------------*/

static void udp_received(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen) {
	if(datalen == sizeof(struct udp_payload)){
		struct udp_payload mesg;
		memcpy(&mesg, data, sizeof(struct udp_payload));
		printf("Data received on port %d from port %d with length %d\n", receiver_port, sender_port, datalen);
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

PROCESS(wireless_communication_process, "Wireless Communication process");
AUTOSTART_PROCESSES(&wireless_communication_process);
PROCESS_THREAD(wireless_communication_process, ev, data)
{
	PROCESS_EXITHANDLER()
	PROCESS_BEGIN();

	printf("TRANSMITTER PROCESS STARTED\n");

	radio_actual_tx_power = 5;   // [dBm]
	//NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, radio_actual_tx_power);

	my_ip_address = set_global_address();
	// UDP register
	simple_udp_register(&unicast_connection, UDP_PORT, NULL, UDP_PORT, udp_received);

	while(1) {
		// Wait for a fixed amount of time
		etimer_set(&periodic_timer, SEND_INTERVAL);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

		#if ENERGEST_CONF_ON
			// Compute difference in clock ticks
			rx_ticks = energest_type_time(ENERGEST_TYPE_LISTEN) - rx_ticks;
			tx_ticks = energest_type_time(ENERGEST_TYPE_TRANSMIT) - tx_ticks;
			cpu_ticks = energest_type_time(ENERGEST_TYPE_CPU) - cpu_ticks;
			lpm_ticks = energest_type_time(ENERGEST_TYPE_LPM) - lpm_ticks;

			// Display difference in clock ticks
			printf("Radio consumption\n");
			printf("Ticks in transmission: %lu\n", tx_ticks);
			printf("Ticks in reception: %lu\n", rx_ticks);
			printf("CPU consumption\n");
			printf("Ticks in active mode: %lu\n", cpu_ticks);
			printf("Ticks in low-power mode: %lu\n", lpm_ticks);

			// Computation of power consumption in uJ
			energy_rx = ((rx_ticks * 1000) / RTIMER_SECOND) * LISTEN_CURRENT * SUPPLY_VOLTAGE;
			energy_tx = ((tx_ticks * 1000) / RTIMER_SECOND) * TRANSMIT_CURRENT * SUPPLY_VOLTAGE;
			energy_cpu = ((cpu_ticks * 1000) / RTIMER_SECOND) * CPU_CURRENT * SUPPLY_VOLTAGE;
			energy_lpm = ((lpm_ticks * 1000) / RTIMER_SECOND) * LPM_CURRENT * SUPPLY_VOLTAGE;
			energy_total = energy_rx + energy_tx + energy_cpu + energy_lpm;

			// Display energy consumption
			printf("Energy consumption radio: %lu uJ\n", energy_rx + energy_tx);
			printf("Energy consumption-CPU: %lu uJ\n", energy_cpu);
			printf("Energy consumption-LPM: %lu uJ\n", energy_lpm);
			printf("Total energy consumption in mJ: %lu\n", (energy_total / 1000));

			// Derive power consumption in uW
			total_milliseconds = ((cpu_ticks + lpm_ticks) * 1000) / RTIMER_SECOND;
			average_power = (energy_total * 1000) / total_milliseconds;
			printf("Total milliseconds of observation: %lu\n", total_milliseconds);
			printf("Average power consumption in uW: %lu\n", average_power);
			uint32_t average_current_ua = (average_power / SUPPLY_VOLTAGE);
			printf("Average current consumption in uA: %lu\n", average_current_ua);
			uint32_t number_of_hours = (BATTERY_MAH * 1000) / average_current_ua;
			printf("Battery duration in hours: %lu\n", number_of_hours);

			// Restart energest measurement
			energest_flush();
			rx_ticks = energest_type_time(ENERGEST_TYPE_LISTEN);
			tx_ticks = energest_type_time(ENERGEST_TYPE_TRANSMIT);
			cpu_ticks = energest_type_time(ENERGEST_TYPE_CPU);
			lpm_ticks = energest_type_time(ENERGEST_TYPE_LPM);
		#endif

		// Create the message to be sent
		message.sequence_number++;
		message.seat_row = 42;
		message.seat_column = 7;
		#if ENERGEST_CONF_ON
		 	// Sending energy consumption in mW
			message.energy = (energy_total);
		#else
			message.energy = 0;
		#endif
		strcpy(message.name, "ABCDEF");

        // tx power setting
        // 7 6 5 4 3 2 1 0  [bit number]
        // c c c f f f f f  [c=coarse gain (attenuator), f=fine gain(mixer)]
        // | | | | | | | |  ( f f f f f / 2 = fine gain [0dB ... 15.5dB])
        // | | | | | | | `- +0.5dB gain
        // | | | | | | `--- +1.0dB gain
        // | | | | | `----- +2.0dB gain
        // | | | | `------- +4.0dB gain
        // | | | `--------- +8.0dB gain
        // | | |            (18dB - c c c = coarse gain [-3dB ... -18dB])
        // | | `----------- -3.0dB gain
        // | `------------- -6.0dB gain
        // `--------------- -9.0dB gain

        // channel 4, 16MHz PRF. default: 0x38383838 = 001 11000
        uint32_t txpower = 0xa0a0a0a0;
        dwt_write32bitreg(TX_POWER_ID, txpower);

		printf("Start ranging ... ");
		linkaddr_t dst = {{0x9e, 0x23}};
		status = range_with(&dst, DW1000_RNG_DS);
		etimer_set(&rng_to, CLOCK_SECOND / 2);
        PROCESS_YIELD_UNTIL((ev == ranging_event || etimer_expired(&rng_to)));
        if (etimer_expired(&rng_to)) {
            printf("ranging timeout\n");
        } else {
            ranging_data_t *d = (ranging_data_t*)data;
            if (d->status) {
                printf("ranging success: %f \n", d->distance);
                message.temperature = (float)d->distance;
            } else {
                printf("ranging failed\n");
                message.temperature = 0.0f;
            }
        }

        // Setting node address
        // Deriving the address of the node we are talking to
        uip_ipaddr_t recipient_address;
        //uiplib_ipaddrconv("fe80::212:4b00:69e:d589", &recipient_address);
        //uiplib_ipaddrconv("fe80::a223:10:e75f:2010", &recipient_address);
        uiplib_ipaddrconv("fe80::9c23:10:e75f:2010", &recipient_address);


	    // Sending the message
	    if(uip_ipaddr_cmp(my_ip_address, &recipient_address)) {
	    	printf("ERROR: Sending to myself: stopping...\n");
	    	uip_debug_ipaddr_print(&recipient_address);
	    	printf(" = ");
	    	uip_debug_ipaddr_print(my_ip_address);
	    	printf("\n");
	     }
	    else {
	    	simple_udp_sendto(&unicast_connection, &message, sizeof(struct udp_payload), &recipient_address);
	        printf("Sending message number %lu to ", message.sequence_number);
	    	uip_debug_ipaddr_print(&recipient_address);
	    	printf("\n");
	    }

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
