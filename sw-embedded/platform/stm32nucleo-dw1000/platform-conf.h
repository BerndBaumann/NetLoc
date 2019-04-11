#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__
/*---------------------------------------------------------------------------*/
#include <inttypes.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define PLATFORM_HAS_LEDS 1
#define PLATFORM_HAS_BUTTON 1
#define PLATFORM_HAS_RADIO 1

#define LEDS_GREEN  1 /*Nucleo LED*/

#define LEDS_CONF_ALL 1

/*---------------------------------------------------------------------------*/
#define F_CPU                   32000000ul
#define RTIMER_ARCH_SECOND              32768
#define PRESCALER       ((F_CPU / (RTIMER_ARCH_SECOND * 2)))

#define UART1_CONF_TX_WITH_INTERRUPT        0
#define WITH_SERIAL_LINE_INPUT              1
#define TELNETD_CONF_NUMLINES               6

/*---------------------------------------------------------------------------*/
/* define ticks/second for slow and fast clocks. Notice that these should be a
   power of two, eg 64,128,256,512 etc, for efficiency as POT's can be optimized
   well. */
#ifndef CLOCK_CONF_SECOND
#define CLOCK_CONF_SECOND             128UL
#endif
/* One tick: 62.5 ms */

// defined in contiki-conf.h
#ifndef RTIMER_CLOCK_DIFF
#define RTIMER_CLOCK_DIFF(a, b)     ((signed short)((a) - (b)))
#endif

/*---------------------------------------------------------------------------*/
typedef unsigned long clock_time_t;
//typedef unsigned long long rtimer_clock_t; see contiki-conf.h
/*---------------------------------------------------------------------------*/
#define CC_CONF_REGISTER_ARGS          0
#define CC_CONF_FUNCTION_POINTER_ARGS  1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS                1

#define CCIF
#define CLIF
/*---------------------------------------------------------------------------*/
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef  int32_t s32_t;
//typedef unsigned short uip_stats_t; see contiki-conf.h
/*---------------------------------------------------------------------------*/
#define MULTICHAN_CONF_SET_CHANNEL(x)
#define MULTICHAN_CONF_READ_RSSI(x) 0
/*---------------------------------------------------------------------------*/

#endif /* __PLATFORM_CONF_H__ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
