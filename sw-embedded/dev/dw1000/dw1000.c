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

/**
 * \file
 *      Contiki DW1000 Driver Source File
 *
 * \author
 *      Pablo Corbalan <p.corbalanpelegrin@unitn.it>
 *      Timofei Istomin <tim.ist@gmail.com>
 */

#include "dw1000.h"
#include "dw1000-arch.h"
#include "dw1000-ranging.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "net/ipv6/uip-ds6.h"
//#include "board.h" /* To be removed after debugging */
#include "leds.h" /* To be removed after debugging */
#include <stdbool.h>
#include "dev/watchdog.h"
/*---------------------------------------------------------------------------*/
#include "deca_device_api.h"
#include "deca_regs.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

#define DEBUG_LEDS 1
#undef LEDS_TOGGLE
#if DEBUG_LEDS
#define LEDS_TOGGLE(x) leds_toggle(x)
#else
#define LEDS_TOGGLE(x)
#endif
/*---------------------------------------------------------------------------*/
typedef enum {
  FRAME_RECEIVED = 1,
  RECV_TO,
  RECV_ERROR,
  TX_SUCCESS,
} dw1000_event_t;

/*---------------------------------------------------------------------------*/
/* Configuration constants */
/*#define DW1000_RX_AFTER_TX_DELAY 60 */
#define DW1000_RX_AFTER_TX_DELAY 0
/*---------------------------------------------------------------------------*/
/* Static variables */
#if DEBUG
static dw1000_event_t dw_dbg_event;
static uint32_t radio_status;
#endif
static uint16_t data_len; /* received data length (payload without CRC) */
static bool frame_pending;
static bool auto_ack_enabled;
static bool wait_ack_txdone;
static volatile bool tx_done; /* flag indicating the end of TX */
static bool dw1000_is_on; /* 0 if radio is off, non-zero if radio is on */
/*---------------------------------------------------------------------------*/
PROCESS(dw1000_process, "DW1000 driver");
#if DEBUG
PROCESS(dw1000_dbg_process, "DW1000 dbg process");
#endif
/*---------------------------------------------------------------------------*/
/* Declaration of static radio callback functions.
 * NOTE: For all events, corresponding interrupts are cleared and necessary
 * resets are performed. In addition, in the RXFCG case, received frame
 * information and frame control are read before calling the callback. If
 * double buffering is activated, it will also toggle between reception
 * buffers once the reception callback processing has ended.
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);
/*---------------------------------------------------------------------------*/
/* DW1000 Radio Driver Static Functions */
static int dw1000_init(void);
static int dw1000_prepare(const void *payload, unsigned short payload_len);
static int dw1000_transmit(unsigned short transmit_len);
static int dw1000_send(const void *payload, unsigned short payload_len);
static int dw1000_radio_read(void *buf, unsigned short buf_len);
static int dw1000_channel_clear(void);
static int dw1000_receiving_packet(void);
static int dw1000_pending_packet(void);
static int dw1000_on(void);
static int dw1000_off(void);
static radio_result_t dw1000_get_value(radio_param_t param, radio_value_t *value);
static radio_result_t dw1000_set_value(radio_param_t param, radio_value_t value);
static radio_result_t dw1000_get_object(radio_param_t param, void *dest, size_t size);
static radio_result_t dw1000_set_object(radio_param_t param, const void *src, size_t size);
/*---------------------------------------------------------------------------*/
/* Callback to process RX good frame events */
static void
rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  /*LEDS_TOGGLE(LEDS_GREEN); */
#if DW1000_RANGING_ENABLED
  if(cb_data->rx_flags & DWT_CB_DATA_RX_FLAG_RNG) {
    if (dw1000_unicast_ranging_flag) {
      dw1000_receive_ranging_ack(cb_data);
    } else {
      if (cb_data->datalength < 25) { // TODO: find better indicator than packet length
        dw1000_rng_ok_cb(cb_data);
      } else {
        dw1000_receive_ranging_uc(cb_data);
      }
    }
    return;
  }
  /* got a non-ranging packet: reset the ranging module if */
  /* it was in the middle of ranging */
  dw1000_range_reset();
#endif

  data_len = cb_data->datalength - DW1000_CRC_LEN;
  /* Set the appropriate event flag */
  frame_pending = true;

  /* if we have auto-ACKs enabled and an ACK was requested, */
  /* don't signal the reception until the TX done interrupt */
  if(auto_ack_enabled && (cb_data->status & SYS_STATUS_AAT)) {
    /*leds_on(LEDS_ORANGE); */
    wait_ack_txdone = true;
  } else {
    wait_ack_txdone = false;
    process_poll(&dw1000_process);
  }
}
/*---------------------------------------------------------------------------*/
void dw1000_receive_ranging_uc(const dwt_cb_data_t *cb_data) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  // get rx timestamp
  uint8_t ts_tab[5];
  uint64_t poll_rx_ts_64 = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for(i = 4; i >= 0; i--) {
    poll_rx_ts_64 <<= 8;
    poll_rx_ts_64 |= ts_tab[i];
  }
  // calculate delay for sending ranging - ack
  uint32_t resp_tx_time = (poll_rx_ts_64 + (1000 * 65536)) >> 8;
  dwt_setdelayedtrxtime(resp_tx_time);
  uint64_t resp_tx_ts_64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + 16455; // TODO: set antenna delay
  // create tx buffer and copy into radio memory
  uint8_t uc_rng[13];
  memset(uc_rng, 0, 13);
  uc_rng[0] = 0x02;
  uc_rng[1] = 0;
  dwt_readrxdata(&uc_rng[2], 1, 2);
  uint32_t responder_rx_ts = (uint32_t)poll_rx_ts_64;
  uint32_t responder_tx_ts = (uint32_t)resp_tx_ts_64;
  uc_rng[3] = (uint8_t)(poll_rx_ts_64 >> 24);
  uc_rng[4] = (uint8_t)(poll_rx_ts_64 >> 16);
  uc_rng[5] = (uint8_t)(poll_rx_ts_64 >>  8);
  uc_rng[6] = (uint8_t)(poll_rx_ts_64 & 0xff);
  uc_rng[7] = (uint8_t)(resp_tx_ts_64 >> 24);
  uc_rng[8] = (uint8_t)(resp_tx_ts_64 >> 16);
  uc_rng[9] = (uint8_t)(resp_tx_ts_64 >>  8);
  uc_rng[10] = (uint8_t)(resp_tx_ts_64 & 0xff);
//  for (i = 0; i < 11; i++)
//    printf(" %02x", uc_rng[i]);
//  printf("\n");
  dwt_writetxdata(13, uc_rng, 0); // + 2 byte CRC
  // enable ranging flag (txctrl)
  dwt_writetxfctrl(13, 0, 1);
  // starttx delayed
  int ret = dwt_starttx(DWT_START_TX_DELAYED);

  // prevent nullrdc layer from sending another ACK
  dw1000_unicast_ranging_flag = true;

  // set frame pending = 1 and poll radio process
  wait_ack_txdone = false;
  frame_pending = true;
  data_len = cb_data->datalength - DW1000_CRC_LEN;
  process_poll(&dw1000_process);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
/*---------------------------------------------------------------------------*/
void
dw1000_receive_ranging_ack(const dwt_cb_data_t *cb_data) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  // read rx data
  // [fcf_h][fcf_l][seq num][responder rx_ts][responder tx_ts]
  uint8_t uc_ack[13];
  memset(uc_ack, 0, 13);
  if (cb_data->datalength != 13) {
    PRINTF("Wrong data length in unicast ranging ACK %d\n", cb_data->datalength);
    frame_pending = true;
    process_poll(&dw1000_process);
    return;
  }
  dwt_readrxdata(uc_ack, 11, 0);
  uint32_t responder_rx_ts = (((uint32_t)uc_ack[3]) << 24) + (((uint32_t)uc_ack[4]) << 16) + (((uint32_t)uc_ack[5]) << 8) + ((uint32_t)uc_ack[6]);
  uint32_t responder_tx_ts = (((uint32_t)uc_ack[7]) << 24) + (((uint32_t)uc_ack[8]) << 16) + (((uint32_t)uc_ack[9]) << 8) + ((uint32_t)uc_ack[10]);
  uint32_t uc_tx_ts = dwt_readtxtimestamplo32();
  uint32_t uc_ack_rx_ts = dwt_readrxtimestamplo32();
  int32_t rtt_initiator = (int32_t)(responder_rx_ts - uc_tx_ts);
  int32_t rtt_responder = (int32_t)(responder_tx_ts - uc_ack_rx_ts);
//  int32_t carrier_integrator = dwt_readcarrierintegrator();
//  float clockOffsetRatio = (float)carrier_integrator * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6);
//  double tof = ((rtt_initiator - rtt_responder * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
  double tof = ((rtt_initiator - rtt_responder) / 2.0) * DWT_TIME_UNITS;
  double not_corrected = tof * 299702547.0;
//  double corrected = not_corrected - dwt_getrangebias(DW1000_CHANNEL, not_corrected, DW1000_PRF);
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_ll_lookup(&unicast_rng_addr);
  if (nbr == NULL) {
    PRINTF("ERROR: could not find %d.%d\n", unicast_rng_addr.u8[0], unicast_rng_addr.u8[1]);
  } else {
    nbr->distance_cm = (uint16_t)(not_corrected * 100.0);
    PRINTF("UNICAST-ACK [%d] received %d (%f)\n", uc_ack[2], nbr->distance_cm, not_corrected);
  }
  frame_pending = true;
  wait_ack_txdone = false;
  data_len = 3;
  dummy_ack[0] = uc_ack[0];
  dummy_ack[1] = uc_ack[1];
  dummy_ack[2] = uc_ack[2];
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  process_poll(&dw1000_process);
}

/*---------------------------------------------------------------------------*/
/* Callback to process RX timeout events */
static void
rx_to_cb(const dwt_cb_data_t *cb_data)
{
#if DW1000_RANGING_ENABLED
  dw1000_range_reset();
#endif
  dw1000_unicast_ranging_flag = false;
  linkaddr_copy(&unicast_rng_addr, &linkaddr_null);
#if DEBUG
  dw_dbg_event = RECV_TO;
  radio_status = cb_data->status;
  process_poll(&dw1000_dbg_process);
#endif
  dw1000_on();

//  LEDS_TOGGLE(LEDS_YELLOW);
  /* Set LED PC7 */
}
/*---------------------------------------------------------------------------*/
/* Callback to process RX error events */
static void
rx_err_cb(const dwt_cb_data_t *cb_data)
{
#if DW1000_RANGING_ENABLED
  dw1000_range_reset();
#endif
  dw1000_unicast_ranging_flag = false;
  linkaddr_copy(&unicast_rng_addr, &linkaddr_null);
#if DEBUG
  dw_dbg_event = RECV_ERROR;
  radio_status = cb_data->status;
  process_poll(&dw1000_dbg_process);
#endif
  dw1000_on();

  /* Set LED PC8 */
  /*LEDS_TOGGLE(LEDS_RED); // not informative with frame filtering */
}
/* Callback to process TX confirmation events */
static void
tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* Set LED PC9 */
  /*LEDS_TOGGLE(LEDS_ORANGE); */

  tx_done = 1; /* to stop waiting in dw1000_transmit() */

  /*if we are sending an auto ACK, signal the frame reception here */
  if(wait_ack_txdone) {
    wait_ack_txdone = false;
    process_poll(&dw1000_process);
  }
}
/*---------------------------------------------------------------------------*/

int
dw1000_configure(dwt_config_t *cfg)
{

  int8_t irq_status = dw1000_disable_interrupt();
#if DW1000_RANGING_ENABLED
  dw1000_range_reset(); /* In case we were ranging */
#endif
  dwt_forcetrxoff();
  dw1000_enable_interrupt(irq_status);

  /* Configure DW1000 */
  dwt_configure(cfg);
#if DW1000_RANGING_ENABLED
  dw1000_range_reconfigure(cfg); /* TODO: check the returned status */
#endif
  // configure calibrated values
  dw1000_load_transmitter_calibration(cfg->chan, cfg->prf);

  dw1000_on();

#if DEBUG == 1
  PRINTF("DW1000 Radio Configuration: \n");
  PRINTF("\t Channel: %u\n", DW1000_CHANNEL);
  PRINTF("\t PRF: %u\n", DW1000_PRF);
  PRINTF("\t PLEN: %u\n", DW1000_PLEN);
  PRINTF("\t PAC Size: %u\n", DW1000_PAC);
  PRINTF("\t Preamble Code: %u\n", DW1000_PREAMBLE_CODE);
  PRINTF("\t SFD: %u\n", DW1000_SFD_MODE);
  PRINTF("\t Data Rate: %u\n", DW1000_DATA_RATE);
  PRINTF("\t PHR Mode: %u\n", DW1000_PHR_MODE);
  PRINTF("\t SFD Timeout: %u\n", DW1000_SFD_TIMEOUT);
#endif
  return 1;
}
static int
dw1000_init(void)
{
  dwt_config_t default_cfg;

  /* Initialize arch-dependent DW1000 */
  dw1000_arch_init();

  /* Configure DW1000 GPIOs to show TX/RX activity with the LEDs */
#if DEBUG_LEDS == 1
  dwt_setleds(DWT_LEDS_ENABLE);
#endif

  auto_ack_enabled = false;

#if DW1000_FRAMEFILTER == 1
  dw1000_set_value(RADIO_PARAM_RX_MODE, RADIO_RX_MODE_ADDRESS_FILTER);
#endif /* DW1000_FRAMEFILTER */

  /* Set the DW1000 ISR */
  dw1000_set_isr(dwt_isr);

  /* Register TX/RX callbacks. */
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
                   DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT |
                   DWT_INT_ARFE | DWT_INT_TFRB, 1);

#if DW1000_RANGING_ENABLED
  dw1000_ranging_init();
#endif

  /* Set the configuration values */
  default_cfg.chan = DW1000_CHANNEL;
  default_cfg.prf = DW1000_PRF;
  default_cfg.txPreambLength = DW1000_PLEN;
  default_cfg.rxPAC = DW1000_PAC;
  default_cfg.txCode = DW1000_PREAMBLE_CODE;
  default_cfg.rxCode = DW1000_PREAMBLE_CODE;
  default_cfg.nsSFD = DW1000_SFD_MODE;
  default_cfg.dataRate = DW1000_DATA_RATE;
  default_cfg.phrMode = DW1000_PHR_MODE;
  default_cfg.sfdTO = DW1000_SFD_TIMEOUT;

  dw1000_configure(&default_cfg);

  dw1000_on();

  /* Start DW1000 process */
  process_start(&dw1000_process, NULL);
#if DEBUG
  process_start(&dw1000_dbg_process, NULL);
#endif /* DEBUG */

  dw1000_last_lqi = 110;
  dw1000_last_rssi = 0;
  dw1000_last_rxtime = 0;
  linkaddr_copy(&unicast_rng_addr, &linkaddr_null);

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_prepare(const void *payload, unsigned short payload_len)
{
  uint8_t frame_len;

#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    return 1;   /* error */
  }
#endif

  frame_len = payload_len + DW1000_CRC_LEN;

#if DEBUG_
  if (dw1000_unicast_ranging_flag) {
    PRINTF("sending (%d) bytes: ", payload_len);
    int i;
    PRINTF("\n");
    for (i = 0; i < payload_len; i++) {
        if (i % 4 == 0)
            PRINTF(" ");
        if (i % 8 == 0)
            PRINTF("\n");
        PRINTF("%02x ", *((uint8_t *)(payload + i)));
    }
    PRINTF("\n");
  }
#endif

  /* Write frame data to DW1000 and prepare transmission */
  dwt_writetxdata(frame_len, (uint8_t *)payload, 0); /* Zero offset in TX buffer. */
  if (dw1000_unicast_ranging_flag) {
    dwt_writetxfctrl(frame_len, 0, 1);
    linkaddr_copy(&unicast_rng_addr, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
  } else {
    dwt_writetxfctrl(frame_len, 0, 0); /* Zero offset in TX buffer, no ranging. */
    linkaddr_copy(&unicast_rng_addr, &linkaddr_null);
  }
  /* TODO: check the return status of the operations above */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_transmit(unsigned short transmit_len)
{
  // TODO: remove after debugging -- begin
  if (dw1000_unicast_ranging_flag)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  // TODO: remove after debugging -- end

  int ret;
  int8_t irq_status = dw1000_disable_interrupt();
#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    dw1000_enable_interrupt(irq_status);
    return RADIO_TX_ERR;
  }
#endif

  /* Switch off radio before setting it to transmit
   * It also clears pending interrupts */
  dwt_forcetrxoff();

  /* Radio starts listening certain delay (in UWB microseconds) after TX */
  dwt_setrxaftertxdelay(DW1000_RX_AFTER_TX_DELAY);

  tx_done = false;

  /* Start transmission, indicating that a response is expected so that reception
   * is enabled automatically after the frame is sent and the delay set by
   * dwt_setrxaftertxdelay() has elapsed. */
  ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  dw1000_enable_interrupt(irq_status);

  // TODO: remove after debugging -- begin
  if (dw1000_unicast_ranging_flag) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }
  // TODO: remove after debugging -- end

  if(ret != DWT_SUCCESS) {
    return RADIO_TX_ERR;
  }

  watchdog_periodic();
  while(!tx_done) {
    /* do nothing, could go to LPM mode */
    asm("");
    /* TODO: add a timeout */
  }
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_send(const void *payload, unsigned short payload_len)
{
  if (dw1000_unicast_ranging_flag) {
    if (payload_len == 3 &&
        dummy_ack[0] == ((uint8_t*)payload)[0] &&
        dummy_ack[1] == ((uint8_t*)payload)[1] &&
        dummy_ack[2] == ((uint8_t*)payload)[2]) {
      // do nothing, because this ACK has been sent already
      // during unicast-ranging
      dw1000_unicast_ranging_flag = false;
      return RADIO_TX_OK;
    }
  }
  if(0 == dw1000_prepare(payload, payload_len)) {
    return dw1000_transmit(payload_len);
  } else {
    return RADIO_TX_ERR;
  }
}
/*---------------------------------------------------------------------------*/
static int
dw1000_radio_read(void *buf, unsigned short buf_len)
{
  if(!frame_pending) {
    return 0;
  }
  if (dw1000_unicast_ranging_flag) {
    *((uint8_t*)buf    ) = dummy_ack[0];
    *((uint8_t*)buf + 1) = dummy_ack[1];
    *((uint8_t*)buf + 2) = dummy_ack[2];
    dw1000_unicast_ranging_flag = false;
  } else {
    dwt_readrxdata(buf, buf_len, 0);
  }
  frame_pending = false;
  return buf_len;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_channel_clear(void)
{
#if DW1000_RANGING_ENABLED
  if(dw1000_is_ranging()) {
    return 0;
  }
#endif /* DW1000_RANGING_ENABLED */

  if(wait_ack_txdone) {
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_receiving_packet(void)
{
  /* TODO: fix this by checking the actual radio status */
  if(wait_ack_txdone) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_pending_packet(void)
{

  // TODO: remove after debugging -- begin
  if (dw1000_unicast_ranging_flag)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  // TODO: remove after debugging -- end
  return frame_pending;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_on(void)
{
  /* Enable RX */
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  dw1000_is_on = true;
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
dw1000_off(void)
{
  /* Turn off the transceiver */
  /*dwt_forcetrxoff(); */
  dw1000_is_on = false;
  return 0;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_get_value(radio_param_t param, radio_value_t *value)
{
    switch(param) {
        case RADIO_PARAM_CHANNEL:
        {
            *value = DW1000_CHANNEL;
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_LAST_LINK_QUALITY:
        {
             *value = dw1000_last_lqi;
             return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_LAST_RSSI:
        {
            *value = dw1000_last_rssi;
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_PAN_ID:
        {
            *value = IEEE802154_CONF_PANID;
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_POWER_MODE:
        {
            if (dw1000_is_on) {
                *value = RADIO_POWER_MODE_ON;
            } else {
                *value = RADIO_POWER_MODE_OFF;
            }
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_RSSI:
        {
            *value = dw1000_last_rssi;
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_RX_MODE:
        {
            if (DW1000_CONF_FRAMEFILTER) {
                *value |= RADIO_RX_MODE_ADDRESS_FILTER;
            }
            if (DW1000_CONF_AUTOACK) {
                *value |= RADIO_RX_MODE_AUTOACK;
            }
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_TXPOWER:
        {
            *value = -41;
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_TX_MODE:
        {
            *value = 0; // no cca-send
            return RADIO_RESULT_OK;
        }

        case RADIO_PARAM_16BIT_ADDR:
        {
            if (LINKADDR_SIZE != 2) {
                return RADIO_RESULT_ERROR;
            } else {
                linkaddr_copy(value, &linkaddr_node_addr);
            }
            return RADIO_RESULT_OK;
        }

        case RADIO_CONST_CHANNEL_MIN:
        {
            *value = 1;
            return RADIO_RESULT_OK;
        }

        case RADIO_CONST_CHANNEL_MAX:
        {
            *value = 7;
            return RADIO_RESULT_OK;
        }

        case RADIO_CONST_TXPOWER_MIN:
        case RADIO_CONST_TXPOWER_MAX:
        {
            *value = -41;
            return RADIO_RESULT_OK;
        }

        case DW1000_UCRNG:
        {
            *value = dw1000_unicast_ranging_flag;
            return RADIO_RESULT_OK;
        }

        //case RADIO_PARAM_CCA_THRESHOLD:
        default:
        {
            return RADIO_RESULT_NOT_SUPPORTED;
        }
    }
    return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_set_value(radio_param_t param, radio_value_t value)
{
  switch(param) {
  case RADIO_PARAM_PAN_ID:
    dwt_setpanid(value & 0xFFFF);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_16BIT_ADDR:
    dwt_setaddress16(value & 0xFFFF);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if(value & RADIO_RX_MODE_ADDRESS_FILTER) {
      dwt_enableframefilter(DWT_FF_COORD_EN | DWT_FF_BEACON_EN | DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_MAC_EN);
#if DW1000_AUTOACK
      /* Auto-ack is only possible if frame filtering is activated */
      dwt_enableautoack(DW1000_AUTOACK_DELAY);
      auto_ack_enabled = true;
#endif
    } else {
      dwt_enableframefilter(DWT_FF_NOTYPE_EN);
      auto_ack_enabled = false;
    }
    return RADIO_RESULT_OK;
  case DW1000_UCRNG:
    {
      if (value == 1) {
        dw1000_unicast_ranging_flag = true;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        // configure radio
        // set antenna delays
        // set rxtimeout
        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0xfff0);
      } else {
        dw1000_unicast_ranging_flag = false;
      }
    }
  return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_get_object(radio_param_t param, void *dest, size_t size)
{
  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || dest == NULL) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    dwt_geteui((uint8_t *)dest);
    return RADIO_RESULT_OK;
  } else if (param == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
      *(rtimer_clock_t*)dest = dw1000_last_rxtime;
      return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
dw1000_set_object(radio_param_t param, const void *src, size_t size)
{
  if(param == RADIO_PARAM_64BIT_ADDR) {
    if(size != 8 || src == NULL) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    dwt_seteui((uint8_t *)src);
    return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw1000_process, ev, data)
{
  PROCESS_BEGIN();

  /*PRINTF("dw1000_process: started\n"); */

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("dwr frame\n");

    if(!frame_pending) {
      /* received a frame but it was already read (e.g. ACK) */
      /* re-enable rx */
      dw1000_on();
      continue;
    }

    if(data_len > PACKETBUF_SIZE) {
      frame_pending = false;
      dw1000_on();
      continue; /* packet is too big, drop it */
    }

    /* Clear packetbuf to avoid having leftovers from previous receptions */
    packetbuf_clear();

    /* Copy the received frame to packetbuf */
    dw1000_radio_read(packetbuf_dataptr(), data_len);
    packetbuf_set_datalen(data_len);

#if DW1000_READ_RXDIAG
    if (!dw1000_is_ranging()) {
        dwt_readaccdata(cir_buffer, CIR_BUF_LEN, 0);
        dwt_readdiagnostics(&dw1000_diagnostics);
        dw1000_last_rxtime = RTIMER_NOW();

        /* rssi [dBm] = 10*log10((C*2^17)/N^2)-A
         * C = CIR_PWR (reg 0x12)
         * A = constant -> 113.77 for 16MHz PRF, 121.74 for 64MHz PRF
         * N = RXPACC (reg 0x10)
         */
        dw1000_last_lqi = 107;
        float C = (float)(((uint32_t)dw1000_diagnostics.maxGrowthCIR) << 17);
        float A = (DW1000_CONF_PRF == DWT_PRF_64M) ? 113.77f : 121.74f;
        dw1000_last_rssi = 10.0f * (log10f(C) -
                log10f(dw1000_diagnostics.rxPreamCount * dw1000_diagnostics.rxPreamCount))
                - A;

        packetbuf_set_attr(PACKETBUF_ATTR_RSSI, dw1000_last_rssi);
        packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, dw1000_last_lqi);
        packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, dw1000_last_rxtime);
        PRINTF("DIAGNOSTICS: rssi %d, timestamp %lu\n", dw1000_last_rssi, dw1000_last_rxtime);
    } // if (!dw1000_is_ranging())
#endif

    /* Re-enable RX to keep listening */
    dw1000_on();
    PRINTF("dw1000_process: calling recv cb, len %d\n", data_len);
#if DEBUG
  PRINTF("recvd (%d) bytes: ", data_len);
  int i;
  PRINTF("\n");
  for (i = 0; i < data_len; i++) {
      if (i % 4 == 0)
          PRINTF(" ");
      if (i % 8 == 0)
          PRINTF("\n");
      PRINTF("%02x ", *((uint8_t *)(packetbuf_dataptr() + i)));
  }
  PRINTF("\n");
#endif
    NETSTACK_RDC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
const struct radio_driver dw1000_driver =
{
  dw1000_init,
  dw1000_prepare,
  dw1000_transmit,
  dw1000_send,
  dw1000_radio_read,
  dw1000_channel_clear,
  dw1000_receiving_packet,
  dw1000_pending_packet,
  dw1000_on,
  dw1000_off,
  dw1000_get_value,
  dw1000_set_value,
  dw1000_get_object,
  dw1000_set_object
};
/*---------------------------------------------------------------------------*/

bool
range_with(linkaddr_t *dst, dw1000_rng_type_t type)
{
#if DW1000_RANGING_ENABLED
  return dw1000_range_with(dst, type);
#else
  return false;
#endif
}
#if DEBUG
PROCESS_THREAD(dw1000_dbg_process, ev, data)
{
  static struct etimer et;
  static uint32_t r1;
  static uint8_t r2;
  PROCESS_BEGIN();
  while(1) {
    etimer_set(&et, CLOCK_SECOND * 10);
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_POLL) {
      r1 = radio_status;
      printf("RX ERR(%u) %02x %02x %02x %02x\n",
             dw_dbg_event, (uint8_t)(r1 >> 24), (uint8_t)(r1 >> 16),
             (uint8_t)(r1 >> 8), (uint8_t)r1);
      prettyPrintSysStatus(r1, r2);
      printf("\n");
    }
    if(etimer_expired(&et)) {
      r1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
      r2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
      printf("*** SYS_STATUS %02x %02x %02x %02x %02x ***\n",
             (uint8_t)(r1 >> 24), (uint8_t)(r1 >> 16), (uint8_t)(r1 >> 8),
             (uint8_t)(r1), r2);
      prettyPrintSysStatus(r1, r2);
      printf("\n");
      dw_dbg_event = 0;
    }
  }
  PROCESS_END();
}

void prettyPrintSysStatus(uint32_t r1, uint8_t r2) {

    if( r1 & SYS_STATUS_IRQS     ) PRINTF("Interrupt Request Status READ ONLY\n");
    if( r1 & SYS_STATUS_CPLOCK   ) PRINTF("Clock PLL Lock\n");
    if( r1 & SYS_STATUS_ESYNCR   ) PRINTF("External Sync Clock Reset\n");
    if( r1 & SYS_STATUS_AAT      ) PRINTF("Automatic Acknowledge Trigger\n");
    if( r1 & SYS_STATUS_TXFRB    ) PRINTF("Transmit Frame Begins\n");
    if( r1 & SYS_STATUS_TXPRS    ) PRINTF("Transmit Preamble Sent\n");
    if( r1 & SYS_STATUS_TXPHS    ) PRINTF("Transmit PHY Header Sent\n");
    if( r1 & SYS_STATUS_TXFRS    ) PRINTF("Transmit Frame Sent: This is set when the transmitter has completed the sending of a frame\n");
    if( r1 & SYS_STATUS_RXPRD    ) PRINTF("Receiver Preamble Detected status\n");
    if( r1 & SYS_STATUS_RXSFDD   ) PRINTF("Receiver Start Frame Delimiter Detected.\n");
    if( r1 & SYS_STATUS_LDEDONE  ) PRINTF("LDE processing done\n");
    if( r1 & SYS_STATUS_RXPHD    ) PRINTF("Receiver PHY Header Detect\n");
    if( r1 & SYS_STATUS_RXPHE    ) PRINTF("Receiver PHY Header Error\n");
    if( r1 & SYS_STATUS_RXDFR    ) PRINTF("Receiver Data Frame Ready\n");
    if( r1 & SYS_STATUS_RXFCG    ) PRINTF("Receiver FCS Good\n");
    if( r1 & SYS_STATUS_RXFCE    ) PRINTF("Receiver FCS Error\n");
    if( r1 & SYS_STATUS_RXRFSL   ) PRINTF("Receiver Reed Solomon Frame Sync Loss\n");
    if( r1 & SYS_STATUS_RXRFTO   ) PRINTF("Receive Frame Wait Timeout\n");
    if( r1 & SYS_STATUS_LDEERR   ) PRINTF("Leading edge detection processing error\n");
    if( r1 & SYS_STATUS_reserved ) PRINTF("bit19 reserved\n");
    if( r1 & SYS_STATUS_RXOVRR   ) PRINTF("Receiver Overrun\n");
    if( r1 & SYS_STATUS_RXPTO    ) PRINTF("Preamble detection timeout\n");
    if( r1 & SYS_STATUS_GPIOIRQ  ) PRINTF("GPIO interrupt\n");
    if( r1 & SYS_STATUS_SLP2INIT ) PRINTF("SLEEP to INIT\n");
    if( r1 & SYS_STATUS_RFPLL_LL ) PRINTF("RF PLL Losing Lock\n");
    if( r1 & SYS_STATUS_CLKPLL_LL) PRINTF("Clock PLL Losing Lock\n");
    if( r1 & SYS_STATUS_RXSFDTO  ) PRINTF("Receive SFD timeout\n");
    if( r1 & SYS_STATUS_HPDWARN  ) PRINTF("Half Period Delay Warning\n");
    if( r1 & SYS_STATUS_TXBERR   ) PRINTF("Transmit Buffer Error\n");
    if( r1 & SYS_STATUS_AFFREJ   ) PRINTF("Automatic Frame Filtering rejection\n");
    if( r1 & SYS_STATUS_HSRBP    ) PRINTF("Host Side Receive Buffer Pointer\n");
    if( r1 & SYS_STATUS_ICRBP    ) PRINTF("IC side Receive Buffer Pointer READ ONLY\n");

    if( r2 & (uint8_t)(SYS_STATUS_RXRSCS >> 32) ) PRINTF("Receiver Reed-Solomon Correction Status\n");
    if( r2 & (uint8_t)(SYS_STATUS_RXPREJ >> 32) ) PRINTF("Receiver Preamble Rejection\n");
    if( r2 & (uint8_t)(SYS_STATUS_TXPUTE >> 32) ) PRINTF("Transmit power up time error\n");
}
#endif /* DEBUG */

/*---------------------------------------------------------------------------*/
uint32_t
calculateFrameLengthUs(dwt_config_t radio_config, uint32_t payload_bytes)
{
    uint32_t frame_len_us = 0;
    uint32_t sfd_len = 0;
    uint32_t pre_len = 0;

    if (radio_config.nsSFD == 1)
    {
        // non-standard SFD
        if (radio_config.dataRate == DWT_BR_110K)
        {
            sfd_len = 64;
        }
        else if (radio_config.dataRate == DWT_BR_850K)
        {
            sfd_len = 16;
        }
        else
        {
            sfd_len = 8;
        }
    }
    else
    {
        // 802154 standard SFD
        if (radio_config.dataRate == DWT_BR_110K)
        {
            sfd_len = 64;
        }
        else
        {
            sfd_len = 8;
        }
    }

    switch (radio_config.txPreambLength)
    {
        case DWT_PLEN_4096:
            pre_len = 4096;
            break;
        case DWT_PLEN_2048:
            pre_len = 2048;
            break;
        case DWT_PLEN_1536:
            pre_len = 1536;
            break;
        case DWT_PLEN_1024:
            pre_len = 1024;
            break;
        case DWT_PLEN_512:
            pre_len = 512;
            break;
        case DWT_PLEN_256:
            pre_len = 256;
            break;
        case DWT_PLEN_128:
            pre_len = 128;
            break;
        case DWT_PLEN_64:
            pre_len = 64;
            break;
    }

    pre_len += sfd_len;
    // Convert preamble length from symbols to time. Length of symbol is defined
    // in IEEE 802.15.4 standard.
    // preamble symbol duration: @16MHZ: 496/499.2MHz=993.59ns, @64MHZ: 508/499.2MHz=1017.63ns
    if (radio_config.prf == DWT_PRF_16M)
    {
        pre_len *= 99359; //preamble symbol duration = 993.59ns
    }
    else
    {
        pre_len *= 101763; //preamble symbol duration = 1017.63ns
    }

    // Compute the number of symbols for the given length.
    // 48...including 48 bits for Reed-Solomon code; if data lengths > 330 bits (55*6) we need 48 'new' bits for RS-Coder. (63/55)-code --> 8 bits RS per 55 bits --> 48/8 = 6*55bits = 330bits

    frame_len_us = payload_bytes * 8 + (1 + ((payload_bytes * 8 - 1) / 330)) * 48;
    // Convert from symbols to time and add PHY header length
    if (radio_config.dataRate == DWT_BR_110K)
    {
        frame_len_us *= 820513; //symbol duration ~8205ns
        frame_len_us += 17230800; //PHY header duration: ~21*8205ns (Decawave uses 21 bits for PHD)
    }
    else if (radio_config.dataRate == DWT_BR_850K)
    {
        frame_len_us *= 102564; //symbol duration ~1025.64ns
        frame_len_us += 2153900; //PHY header duration: ~21*1025.64ns (Decawave uses 21 bits for PHD)
    }
    else
    {
        frame_len_us *= 12821; //symbol duration ~128.2ns
        frame_len_us += 2153900; //PHY header duration: ~21*1025.64ns (Decawave uses 21 bits for PHD)
    }

    frame_len_us += pre_len;

    // convert to micro-seconds
    frame_len_us = 1 + ((frame_len_us - 1) / 100000);

    return frame_len_us;
}
