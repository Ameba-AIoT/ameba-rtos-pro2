/**
 * @file
 * DHCP client API
 */

/*
 * Copyright (c) 2001-2004 Leon Woestenberg <leon.woestenberg@gmx.net>
 * Copyright (c) 2001-2004 Axon Digital Design B.V., The Netherlands.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Leon Woestenberg <leon.woestenberg@gmx.net>
 *
 */
#ifndef LWIP_HDR_DHCP_H
#define LWIP_HDR_DHCP_H

#include "lwip/opt.h"

#if LWIP_DHCP /* don't build if not configured for use in lwipopts.h */

#include "lwip/netif.h"
#include "lwip/udp.h"

#if LWIP_DHCP_DOES_ACD_CHECK
#include "lwip/acd.h"
#endif /* LWIP_DHCP_DOES_ACD_CHECK */

#ifdef __cplusplus
extern "C" {
#endif

/** Define DHCP_TIMEOUT_SIZE_T in opt.h if you want use a different integer than u16_t.
 *  Especially useful if DHCP_COARSE_TIMER_SECS is in smaller units, so timeouts easily reach UINT16_MAX and more */
#ifdef DHCP_TIMEOUT_SIZE_T
typedef DHCP_TIMEOUT_SIZE_T dhcp_timeout_t;
#else /* DHCP_TIMEOUT_SIZE_T */
typedef u16_t dhcp_timeout_t;
#endif /* DHCP_TIMEOUT_SIZE_T*/
/** period (in seconds) of the application calling dhcp_coarse_tmr() */
#ifndef DHCP_COARSE_TIMER_SECS
#if CONFIG_LWIP_DHCP_COARSE_TIMER
#define DHCP_COARSE_TIMER_SECS CONFIG_LWIP_DHCP_COARSE_TIMER    //Realtek add
#else
#define DHCP_COARSE_TIMER_SECS  60
#endif
#endif /* DHCP_COARSE_TIMER_SECS */
/** period (in milliseconds) of the application calling dhcp_coarse_tmr() */
#define DHCP_COARSE_TIMER_MSECS (DHCP_COARSE_TIMER_SECS * 1000UL)
/** period (in milliseconds) of the application calling dhcp_fine_tmr() */
#define DHCP_FINE_TIMER_MSECS   500

#define DHCP_BOOT_FILE_LEN      128U

#define DHCP_FLAG_SUBNET_MASK_GIVEN 0x01
#define DHCP_FLAG_EXTERNAL_MEM      0x02

/* AutoIP cooperation flags (struct dhcp.autoip_coop_state) */
typedef enum {
  DHCP_AUTOIP_COOP_STATE_OFF  = 0,
  DHCP_AUTOIP_COOP_STATE_ON   = 1
} dhcp_autoip_coop_state_enum_t;

struct dhcp
{
  /** transaction identifier of last sent request */
  u32_t xid;
  /** track PCB allocation state */
  u8_t pcb_allocated;
  /** current DHCP state machine state */
  u8_t state;
  /** retries of current request */
  u8_t tries;
  /** see DHCP_FLAG_* */
  u8_t flags;

  dhcp_timeout_t request_timeout; /* #ticks with period DHCP_FINE_TIMER_SECS for request timeout */
  dhcp_timeout_t t1_timeout;  /* #ticks with period DHCP_COARSE_TIMER_SECS for renewal time */
  dhcp_timeout_t t2_timeout;  /* #ticks with period DHCP_COARSE_TIMER_SECS for rebind time */
  dhcp_timeout_t t1_renew_time;  /* #ticks with period DHCP_COARSE_TIMER_SECS until next renew try */
  dhcp_timeout_t t2_rebind_time; /* #ticks with period DHCP_COARSE_TIMER_SECS until next rebind try */
  dhcp_timeout_t lease_used; /* #ticks with period DHCP_COARSE_TIMER_SECS since last received DHCP ack */
  dhcp_timeout_t t0_timeout; /* #ticks with period DHCP_COARSE_TIMER_SECS for lease time */
  ip_addr_t server_ip_addr; /* dhcp server address that offered this lease (ip_addr_t because passed to UDP) */
  ip4_addr_t offered_ip_addr;
  ip4_addr_t offered_sn_mask;
  ip4_addr_t offered_gw_addr;

  u32_t offered_t0_lease; /* lease period (in seconds) */
  u32_t offered_t1_renew; /* recommended renew time (usually 50% of lease period) */
  u32_t offered_t2_rebind; /* recommended rebind time (usually 87.5 of lease period)  */
#if LWIP_DHCP_BOOTP_FILE
  ip4_addr_t offered_si_addr;
  char boot_file_name[DHCP_BOOT_FILE_LEN];
#endif /* LWIP_DHCP_BOOTPFILE */
#if LWIP_DHCP_DOES_ACD_CHECK
  /** acd struct */
  struct acd acd;
#endif /* LWIP_DHCP_DOES_ACD_CHECK */
  u32_t seconds_elapsed;            //Realtek add end
};


void dhcp_set_struct(struct netif *netif, struct dhcp *dhcp);
/** Remove a struct dhcp previously set to the netif using dhcp_set_struct() */
#define dhcp_remove_struct(netif) netif_set_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP, NULL)
void dhcp_cleanup(struct netif *netif);
err_t dhcp_start(struct netif *netif);
err_t dhcp_renew(struct netif *netif);
err_t dhcp_release(struct netif *netif);
void dhcp_stop(struct netif *netif);
void dhcp_release_and_stop(struct netif *netif);
void dhcp_inform(struct netif *netif);
void dhcp_network_changed_link_up(struct netif *netif);

u8_t dhcp_supplied_address(const struct netif *netif);
/* to be called every minute */
void dhcp_coarse_tmr(void);
/* to be called every half second */
void dhcp_fine_tmr(void);

#if LWIP_DHCP_GET_NTP_SRV
/** This function must exist, in other to add offered NTP servers to
 * the NTP (or SNTP) engine.
 * See LWIP_DHCP_MAX_NTP_SERVERS */
extern void dhcp_set_ntp_servers(u8_t num_ntp_servers, const ip4_addr_t* ntp_server_addrs);
#endif /* LWIP_DHCP_GET_NTP_SRV */

#define netif_dhcp_data(netif) ((struct dhcp*)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP))

#ifdef __cplusplus
}
#endif

#endif /* LWIP_DHCP */

#endif /*LWIP_HDR_DHCP_H*/
