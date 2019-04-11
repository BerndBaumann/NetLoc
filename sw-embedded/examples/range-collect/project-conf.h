#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

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
#define DW1000_CONF_SFD_MODE       1
#define DW1000_CONF_DATA_RATE      DWT_BR_110K
#define DW1000_CONF_PHR_MODE       DWT_PHRMODE_STD
#define DW1000_CONF_PREAMBLE_CODE  9
#define DW1000_CONF_SFD_TIMEOUT    (1025 + 64 - 32)
#endif


#define NETSTACK_CONF_NETWORK rime_driver
#define NETSTACK_CONF_MAC nullmac_driver
#define NETSTACK_CONF_RDC nullrdc_driver


#endif /* PROJECT_CONF_H_ */
