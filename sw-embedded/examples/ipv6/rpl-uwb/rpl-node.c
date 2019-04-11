/*
 * rpl-node.c
 *
 *  Created on: Jul 26, 2018
 *      Author: b3
 */

#include <stdio.h>

#include "contiki.h"

/**
 * Networking libraries
 */
#include "net/ip/uip.h"
#include "net/ip/uiplib.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "net/packetbuf.h"

/**
 * Decawave driver includes
 */
#include "dw1000.h"
#include "dw1000-ranging.h"

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/**
 * \brief udp_server_process This process implements the DAG root and
 * the data sink of the IPv6 network. A node must start the udp_server_process
 * or the udp_client_process, but not both.
 */
PROCESS(udp_server_process, "UDP server process");

/**
 * \brief udp_client_process This process is executed by all nodes in the
 * network. They try to join a DODAG and send their sensor data periodically
 * to a UDP port or the server. A client node must not be a server node at the
 * same time.
 */
PROCESS(udp_client_process, "UDP client process");

/**
 * \brief role_assignment_process This process runs only once at start to
 * decide whether the node will be a client or a server. Currently, this is
 * decided based on the node's address. After a role is assigned, this process
 * terminates.
 */
PROCESS(role_assignment_process, "Role assignment process");

/**
 * \brief process pointer to dynamically notify the current application
 * process after ranging. This value is set once, during role assignment.
 */
struct process *udp_proc_;

/**
 * \brief neighbor_ranging This process iterates through the node's neighbor
 * table and measures the distance to each one. The measured values are
 * stored in a global data structure, which is then sent to the sink.
 */
PROCESS(neighbor_ranging, "ranging process");

/**
 * \brief Auto start with role assignment.
 */
AUTOSTART_PROCESSES(&role_assignment_process);


//-----------------------------------------------------------------------------
// networking stuff
static struct uip_udp_conn *udp_conn_;
linkaddr_t server_addr64_ = {{0xa5, 0x23, 0x00, 0x10, 0xe7, 0x5f, 0x20, 0x10}};
static uip_ipaddr_t server_ipaddr128_;
static uip_ipaddr_t ipaddr_;
struct uip_ds6_addr *root_if_;

#define NUM_MAX_NETWORK_CLIENTS 7
static uip_ipaddr_t clients_addresses_[NUM_MAX_NETWORK_CLIENTS];

//-----------------------------------------------------------------------------
// data exchange stuff
struct ranging_tuple_t {
  uint16_t neighbor_addr;
  uint16_t neighbor_dist;
};
struct udp_payload_t {
  uint32_t seq_num;
  struct ranging_tuple_t measurement[NBR_TABLE_CONF_MAX_NEIGHBORS];
};
struct udp_payload_t udp_packet_;
void printUdpPayload(struct udp_payload_t* p, uint8_t id_h, uint8_t id_l);
uint32_t current_seq_num_;
struct etimer udp_timeout_;

//-----------------------------------------------------------------------------
// ranging stuff
struct etimer rng_to_;
linkaddr_t rng_dst_;
uip_ds6_nbr_t *rng_nbr_;
static uint8_t counter_;
static process_event_t rng_done_event_;
static process_event_t start_ranging_event_;

//-----------------------------------------------------------------------------
PROCESS_THREAD(role_assignment_process, ev, data)
{
  PROCESS_BEGIN();

  printf("node address: %02x.%02x.%02x.%02x.%02x.%02x.%02x.%02x\n",
         linkaddr_node_addr.u8[0],
         linkaddr_node_addr.u8[1],
         linkaddr_node_addr.u8[2],
         linkaddr_node_addr.u8[3],
         linkaddr_node_addr.u8[4],
         linkaddr_node_addr.u8[5],
         linkaddr_node_addr.u8[6],
         linkaddr_node_addr.u8[7]);

  uiplib_ipaddrconv("fd00::a723:10:e75f:2010", &server_ipaddr128_);

  memset(&udp_packet_, 0, sizeof(udp_packet_));
  memset(&clients_addresses_, 0, sizeof(clients_addresses_));

  if (linkaddr_cmp(&linkaddr_node_addr, &server_addr64_)) {
      printf("ROLE server\n");
      udp_proc_ = &udp_server_process;
      process_start(&udp_server_process, NULL);
  } else {
      printf("ROLE client\n");
      udp_proc_ = &udp_client_process;
      process_start(&udp_client_process, NULL);
  }

  process_start(&neighbor_ranging, NULL);
  start_ranging_event_ = process_alloc_event();
  rng_done_event_ = process_alloc_event();

  PROCESS_END();
}

//-----------------------------------------------------------------------------
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();

  // set up own IP address
  uip_ip6addr(&ipaddr_, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr_, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr_, 0, ADDR_AUTOCONF);

  // open UDP connection
  udp_conn_ = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  if(udp_conn_ == NULL) {
    printf("Error: upd_new() failed\n");
    PROCESS_EXIT();
  }
  udp_bind(udp_conn_, UIP_HTONS(UDP_CLIENT_PORT));

  // 20 seconds startup delay
  etimer_set(&udp_timeout_, CLOCK_SECOND * 20);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));

  // 1. wait for UDP request by sink node
  // 2. range to all neighbors
  // 3. send neighbor table to sink
  while (1) {
    //---------------------------
    printf("Waiting for sink\n");
    PROCESS_WAIT_EVENT_UNTIL((ev == tcpip_event) && uip_newdata());
    if (uip_datalen() != sizeof(udp_packet_)) {
      printf("ERROR: wrong packet length\n");
      continue;
    }
    struct udp_payload_t* p = uip_appdata;
    uint8_t idx;
    uint32_t tmp = 0;
    for (idx = 0; idx < NBR_TABLE_CONF_MAX_NEIGHBORS; idx++) {
      tmp += p->measurement[idx].neighbor_addr;
      tmp += p->measurement[idx].neighbor_dist;
    }
    if (tmp != 0) {
      printf("ERROR: wrong payload: ");
      printUdpPayload(uip_appdata, linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
      continue;
    }
    current_seq_num_ = p->seq_num;
    uip_ip6addr_copy(&server_ipaddr128_, &UIP_IP_BUF->srcipaddr);
    printf("UDP ranging request recvd. %lu\n", current_seq_num_);
    // adding some delay so the server has time to become idle
    etimer_set(&udp_timeout_, CLOCK_SECOND / 2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));

    //----------------------------------------------------------
    process_post(&neighbor_ranging, start_ranging_event_, NULL);
    PROCESS_WAIT_EVENT_UNTIL((ev == rng_done_event_));

    //--------------------
    udp_packet_.seq_num = current_seq_num_;
    uip_udp_packet_sendto(
        udp_conn_,
        (int8_t*)&udp_packet_,
        sizeof(udp_packet_),
        &server_ipaddr128_,
        UIP_HTONS(UDP_SERVER_PORT));
  } // while (1)

  PROCESS_END();
}

//-----------------------------------------------------------------------------
PROCESS_THREAD(udp_server_process, ev, data)
{
  PROCESS_BEGIN();

  // set up  own IP address
  uip_ip6addr(&ipaddr_, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr_, &uip_lladdr);

  // initialize new DAG
  uip_ds6_addr_add(&ipaddr_, 0, ADDR_MANUAL);
  root_if_ = uip_ds6_addr_lookup(&ipaddr_);
  if(root_if_ != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr_);
    uip_ip6addr(&ipaddr_, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr_, 64);
    printf("Success: created a new RPL dag\n");
  } else {
    printf("Error: failed to create a new RPL DAG\n");
  }

  udp_conn_ = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(udp_conn_ == NULL) {
    printf("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(udp_conn_, UIP_HTONS(UDP_SERVER_PORT));

  printf("Created a server connection with remote address ");
  //PRINT6ADDR(&udp_conn_->ripaddr);
  printf(" local/remote port %u/%u\n",
         UIP_HTONS(udp_conn_->lport),
         UIP_HTONS(udp_conn_->rport));

  // 20 seconds startup delay
  etimer_set(&udp_timeout_, CLOCK_SECOND * 10);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));

  current_seq_num_ = 42;
  // 1. range to all neighbors
  // 2.0 for each address in the network:
  //        2.1 request neighbor table
  //        2.2 receive and print neighbor table
  while (1) {
    //----------------------------------------------------------
    etimer_set(&udp_timeout_, CLOCK_SECOND * 2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));
    process_post(&neighbor_ranging, start_ranging_event_, NULL);
    PROCESS_WAIT_EVENT_UNTIL((ev == rng_done_event_));
    udp_packet_.seq_num = current_seq_num_;
    printUdpPayload(&udp_packet_, linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

    //---------------------------------------------------------
    etimer_set(&udp_timeout_, CLOCK_SECOND * 1);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));
    memset(&clients_addresses_, 0, sizeof(clients_addresses_));
    uip_ds6_route_t *r;
    uint8_t i;
    for(r = uip_ds6_route_head(), i = 0;
        (r != NULL) && (i < NUM_MAX_NETWORK_CLIENTS);
        r = uip_ds6_route_next(r), i++) {
      PRINT6ADDR(&r->ipaddr); printf("\n");
      clients_addresses_[i] = r->ipaddr;
    }
    for (counter_ = 0; counter_ < NUM_MAX_NETWORK_CLIENTS; counter_++) {
      if (clients_addresses_[counter_].u16[0] == 0x0000) {
        continue;
      }
      memset(&udp_packet_, 0, sizeof(udp_packet_));
      udp_packet_.seq_num = current_seq_num_;
      uip_udp_packet_sendto(
          udp_conn_,
          (int8_t*)&udp_packet_,
          sizeof(udp_packet_),
          &clients_addresses_[counter_],
          UIP_HTONS(UDP_CLIENT_PORT));
      etimer_set(&udp_timeout_, CLOCK_SECOND * 10);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_) ||
                               ((ev == tcpip_event) && uip_newdata()));
      if (etimer_expired(&udp_timeout_)) {
        printf("ERROR: UDP timeout (sn=%lu). addr = ", current_seq_num_);
        PRINT6ADDR(&clients_addresses_[counter_]);
        printf("\n");
      } else {
        memcpy(&udp_packet_, uip_appdata, sizeof(udp_packet_));
        printUdpPayload(&udp_packet_,
                        UIP_IP_BUF->srcipaddr.u8[8] ^ 0x2,
                        UIP_IP_BUF->srcipaddr.u8[9]);

        etimer_set(&udp_timeout_, CLOCK_SECOND * 2);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&udp_timeout_));
      }
    } // for (counter_ = 0; counter_ < NUM_MAX_NETWORK_CLIENTS; counter_++) {
    current_seq_num_++;
  } // while(1)

  PROCESS_END();
}

//-----------------------------------------------------------------------------
PROCESS_THREAD(neighbor_ranging, ev, data)
{
    PROCESS_BEGIN();

    while (1) {
        PROCESS_WAIT_EVENT_UNTIL((ev == start_ranging_event_));
        memset(&udp_packet_, 0, sizeof(udp_packet_));
        for (rng_nbr_ = nbr_table_head(ds6_neighbors), counter_ = 0;
             rng_nbr_ != NULL;
             rng_nbr_ = nbr_table_next(ds6_neighbors, rng_nbr_), counter_++) {
            rng_dst_.u8[0] = 0x2 ^ rng_nbr_->ipaddr.u8[8];
            rng_dst_.u8[1] = rng_nbr_->ipaddr.u8[9];
            range_with(&rng_dst_, DW1000_RNG_DS);
            etimer_set(&rng_to_, CLOCK_SECOND / 2);
            PROCESS_YIELD_UNTIL((ev == ranging_event || etimer_expired(&rng_to_)));
            if (etimer_expired(&rng_to_)) {
                printf("ERROR: RANGING:: timeout\n");
                continue;
            } else {
                ranging_data_t *d = (ranging_data_t*)data;
                if (d->status) {
                    printf("RANGING:: with %02x.%02x success: %f \n",
                           rng_nbr_->ipaddr.u8[8],
                           rng_nbr_->ipaddr.u8[9],
                           d->distance);
                    udp_packet_.measurement[counter_].neighbor_addr =
                        (uint16_t)((((uint16_t)rng_dst_.u8[0]) << 8) +
                                   (((uint16_t)rng_dst_.u8[1]) & 0x00ff));
                    udp_packet_.measurement[counter_].neighbor_dist =
                        (uint16_t)(d->distance * 100.0);
                } else {
                    printf("ERROR: RANGING:: with %02x.%02x failed\n",
                           rng_nbr_->ipaddr.u8[8],
                           rng_nbr_->ipaddr.u8[9]);
                    udp_packet_.measurement[counter_].neighbor_addr =
                        (uint16_t)((((uint16_t)rng_dst_.u8[0]) << 8) +
                                   (((uint16_t)rng_dst_.u8[1]) & 0x00ff));
                    udp_packet_.measurement[counter_].neighbor_dist = 0;
                } // if (d->status)
            } // if (etimer_expired(&rng_to_))
        } // for (rng_nbr_ = nbr_table_head(ds6_neighbors), counter_ = 0;
        process_post(udp_proc_, rng_done_event_, NULL);
    } // while (1)

    PROCESS_END();
}

//-----------------------------------------------------------------------------
void printUdpPayload(struct udp_payload_t* p, uint8_t id_h, uint8_t id_l) {
  // RPLUWB, <16bit addr>, <seqnum>, <nbr0 addr>, <nbr0 dist>, ...\n
  printf("RPLUWB, %02x%02x, %4lu", id_h, id_l, p->seq_num);
  uint8_t idx;
  for (idx = 0; idx < NBR_TABLE_CONF_MAX_NEIGHBORS; idx++) {
    printf(", %04x, %4d", p->measurement[idx].neighbor_addr,
        p->measurement[idx].neighbor_dist);
  }
  printf("\n");
}

