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

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
//#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "dw1000-ranging.h"
#include "sys/rtimer.h"
//#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(example_rime_process, "Example rime");
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
#define RANGING_TIMEOUT (CLOCK_SECOND / 2)
#define APP_RADIO_CONF 1

#if 1
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTDEBUG(...)
#endif

#if 0
#define PRINTLOG(...) printf(__VA_ARGS__)
#else
#define PRINTLOG(...)
#endif
/*---------------------------------------------------------------------------*/
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
linkaddr_t dst_;
process_event_t start_ranging_event_;
static struct etimer et;
static struct etimer timeout;
static int status;
uint32_t startt;
uint32_t delay_;

uint32_t success_counter_;
uint32_t timeout_counter_;
uint32_t failure_counter_;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(range_process, ev, data) {
	PROCESS_BEGIN();

	success_counter_ = 0;
	timeout_counter_ = 0;
	failure_counter_ = 0;
	dw1000_configure(&radio_config);
	//dwt_setpreambledetecttimeout(0xfff0);

	if (linkaddr_node_addr.u8[0] == 0xa5 && linkaddr_node_addr.u8[1] == 0x23) {
		dst_.u8[0] = 0x34; // I am A, dst is node B
		dst_.u8[1] = 0xc6;
	} else {
	    linkaddr_copy(&dst_, &linkaddr_node_addr);
		dst_.u8[0] = 0xa5; // I am B, dst is node A
		dst_.u8[1] = 0x23;
	}

	PRINTDEBUG("my address is 0x%02x 0x%02x\n", linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);

	etimer_set(&et, 5 * CLOCK_SECOND);
	PROCESS_WAIT_UNTIL(etimer_expired(&et));
	PRINTDEBUG("I am %02x%02x ranging with %02x%02x\n", linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1], dst_.u8[0], dst_.u8[1]);

	delay_ = CLOCK_SECOND;
	if (linkaddr_node_addr.u8[0] == 160) {
		PRINTDEBUG("... waiting for poll\n");
		while (1) {
			PROCESS_YIELD_UNTIL(0);
		}
	}
	PRINTLOG("Success,SuccessTotal,Timeout,TimeoutTotal,Failure,FailureTotal,DelaySet,DelayMeasured,Distance\n");
	while (1) {
		PRINTDEBUG("\n--- --- ---\n[s/t/f] = [%lu/%lu/%lu]\n", success_counter_, timeout_counter_, failure_counter_);
		PRINTDEBUG("R req\n");
		startt = RTIMER_NOW();
		status = range_with(&dst_, DW1000_RNG_DS);
		if (!status) {
			uint32_t stop = RTIMER_NOW() - startt;
			failure_counter_++;
			PRINTDEBUG("R req failed after %3.3fms\n", (float)(stop * 1000)/(float)RTIMER_SECOND);
			delay_ = CLOCK_SECOND;
			PRINTDEBUG("new delay = %3.3fms\n", (float)(delay_ * 1000)/(float)CLOCK_SECOND);
			PRINTLOG("0,%lu,0,%lu,1,%lu,%3.3f,%3.3fd,0.0\n", success_counter_, timeout_counter_, failure_counter_, (float)(delay_ * 1000)/(float)CLOCK_SECOND, (float)(stop * 1000)/(float)RTIMER_SECOND);
		} else {
			etimer_set(&timeout, RANGING_TIMEOUT);
			PROCESS_YIELD_UNTIL(
					(ev == ranging_event || etimer_expired(&timeout)));
			if (etimer_expired(&timeout)) {
				uint32_t stop = RTIMER_NOW() - startt;
				timeout_counter_++;
				PRINTDEBUG("R TIMEOUT after %3.3fms\n", (float)(stop * 1000)/(float)RTIMER_SECOND);
				delay_ += (uint32_t)((float)delay_ / 10.0f) + 1;
				if (delay_ > (CLOCK_SECOND * 2))
					delay_ = (CLOCK_SECOND * 2);
				PRINTDEBUG("new delay = %3.3fms\n", (float)(delay_ * 1000)/(float)CLOCK_SECOND);
				PRINTLOG("0,%lu,1,%lu,0,%lu,%3.3f,%3.3f,0.0\n", success_counter_, timeout_counter_, failure_counter_, (float)(delay_ * 1000)/(float)CLOCK_SECOND, (float)(stop * 1000)/(float)RTIMER_SECOND);
			} else if (((ranging_data_t *) data)->status) {
				uint32_t stop = RTIMER_NOW() - startt;
				ranging_data_t *d = data;
				success_counter_++;
				PRINTDEBUG("R success: %f bias %f after %3.3fms\n", d->raw_distance, d->distance,(float)(stop * 1000)/(float)RTIMER_SECOND);
				delay_ -= (uint32_t)((float)delay_ / 10.0f) + 1;
				if (delay_ <= 2) {
					delay_ = 2;
				}
				PRINTDEBUG("new delay = %3.3fms\n", (float)(delay_ * 1000)/(float)CLOCK_SECOND);
				PRINTLOG("1,%lu,0,%lu,0,%lu,%3.3f,%3.3f,%3.3f\n", success_counter_, timeout_counter_, failure_counter_, (float)(delay_ * 1000)/(float)CLOCK_SECOND, (float)(stop * 1000)/(float)RTIMER_SECOND, d->raw_distance);
			} else {
				uint32_t stop = RTIMER_NOW() - startt;
				failure_counter_++;
				PRINTDEBUG("R FAIL after %3.3fms\n", (float)(stop * 1000)/(float)RTIMER_SECOND);
				delay_ = CLOCK_SECOND;
				PRINTDEBUG("new delay = %3.3fms\n", (float)(delay_ * 1000)/(float)CLOCK_SECOND);
				PRINTLOG("0,%lu,0,%lu,1,%lu,%3.3f,%3.3f,0.0\n", success_counter_, timeout_counter_, failure_counter_, (float)(delay_ * 1000)/(float)CLOCK_SECOND, (float)(stop * 1000)/(float)RTIMER_SECOND);
			}
		}
		etimer_set(&et, 16);
		PROCESS_WAIT_UNTIL(etimer_expired(&et));
	}

	PRINTDEBUG("process end\n");
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
  PRINTDEBUG("### ### ### unicast message received from %d.%d ### ### ### %s\n",
	 from->u8[0], from->u8[1], (char*)packetbuf_dataptr());

  if (from->u8[0] == dst_.u8[0] && from->u8[1] == dst_.u8[1]) {
    process_post(&example_rime_process, start_ranging_event_, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }
  // status = 0 ... MAC_TX_OK
  // status = 1 ... MAC_TX_COLLISION
  // status = 2 ... MAC_TX_NOACK
  // status = 4 ... MAC_TX_ERR
  PRINTDEBUG("unicast message sent to %d.%d: status %d num_tx %d\n",
    dest->u8[0], dest->u8[1], status, num_tx);
}
/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
static void recv_bc(struct broadcast_conn *b, linkaddr_t *from) {
	PRINTDEBUG("broadcast received from %d.%d: %s\n", from->u8[0], from->u8[1], (char*)packetbuf_dataptr());

	if (from->u8[0] == dst_.u8[0] && from->u8[1] == dst_.u8[1]) {
		process_post(&example_rime_process, start_ranging_event_, NULL);
	}
}
static const struct broadcast_callbacks bc_callbacks = {recv_bc};
static struct broadcast_conn bc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_rime_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  dw1000_configure(&radio_config);

  start_ranging_event_ = process_alloc_event();

  unicast_open(&uc, 146, &unicast_callbacks);
  broadcast_open(&bc, 180, &bc_callbacks);

  static struct etimer et;
  if (linkaddr_node_addr.u8[0] == 165 && linkaddr_node_addr.u8[1] == 35) {
	  PRINTDEBUG("I am A, dst_ is node B\n");
      linkaddr_copy(&dst_, &linkaddr_node_addr);
	  dst_.u8[0] = 160; // I am A, dst is node B
	  dst_.u8[1] = 35;
  } else {
	  PRINTDEBUG("I am B, dst_ is node A\n");
      linkaddr_copy(&dst_, &linkaddr_node_addr);
	  dst_.u8[0] = 165; // I am B, dst is node A
	  dst_.u8[1] = 35;
  }

  while(1) {

    etimer_set(&et, CLOCK_SECOND*2);

    PROCESS_WAIT_EVENT_UNTIL(ev == start_ranging_event_ || etimer_expired(&et));

    if (ev == start_ranging_event_) {
        PRINTDEBUG("\n### ### ### PROCESS_START ### ### ###\n");
      //process_start(&range_process, NULL);
      break;
    } else {
      packetbuf_copyfrom("Hello", 6);
      if (linkaddr_node_addr.u8[0] == 160) {
        unicast_send(&uc, &dst_);

//        packetbuf_clear();
//        packetbuf_copyfrom("Hello", 6);
//        broadcast_send(&bc);
      }
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

