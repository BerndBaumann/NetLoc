#include "contiki.h"
#include "net/rime/rime.h"
#include "leds.h"
//#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "dw1000-ranging.h"
#include "sys/rtimer.h"
//#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
PROCESS(range_process, "Test range process");
AUTOSTART_PROCESSES(&range_process);
/*---------------------------------------------------------------------------*/
#define RANGING_TIMEOUT (CLOCK_SECOND / 2)
#define APP_RADIO_CONF 1

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

//-----------------------------------------------------------------------------
linkaddr_t dst = {{0x34, 0xc6}};
//linkaddr_t dst = {{0x00, 0x0c}};
//linkaddr_t dst = {{0x11, 0xc6}};

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(range_process, ev, data)
{
  static struct etimer et;
  static struct etimer timeout;
  static int status;

  PROCESS_BEGIN();

  dw1000_configure(&radio_config);

  if(linkaddr_node_addr.u8[0] == 0x11) {

    etimer_set(&et, 5 * CLOCK_SECOND);
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    printf("I am %02x%02x ranging with %02x%02x\n",
           linkaddr_node_addr.u8[0],
           linkaddr_node_addr.u8[1],
           dst.u8[0],
           dst.u8[1]);

    while(1) {
//      printf("R req\n");
      status = range_with(&dst, DW1000_RNG_DS);
      if(!status) {
//        printf("R req failed\n");
      } else {
        etimer_set(&timeout, RANGING_TIMEOUT);
        PROCESS_YIELD_UNTIL((ev == ranging_event || etimer_expired(&timeout)));
        if(etimer_expired(&timeout)) {
//          printf("R TIMEOUT\n");
        } else if(((ranging_data_t *)data)->status) {
          ranging_data_t *d = data;
          printf("%f, %f\n", d->raw_distance, d->distance);
        } else {
//          printf("R FAIL\n");
        }
      }
      etimer_set(&et, 16);
      PROCESS_WAIT_UNTIL(etimer_expired(&et));
    }
  }
  printf("I am the ranging target\n");
  PROCESS_END();
}
