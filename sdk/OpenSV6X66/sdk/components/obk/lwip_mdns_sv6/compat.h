/* SV6X66 adaptation of lwIP's BSD-licensed mDNS responder, imported from
 * OpenIOTdocker/important_files/esp-idf-v5.5.2-source.tar.gz,
 * esp-idf/components/lwip/lwip/src/apps/mdns and src/include/lwip.
 * This header is private to this component; SDK structures remain unchanged. */
#ifndef OBK_SV_MDNS_COMPAT_H
#define OBK_SV_MDNS_COMPAT_H
#include "obk_config.h"
#include "lwip/opt.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/ip.h"
#include "lwip/igmp.h"
#include "lwip/pbuf.h"
#include "lwip/mem.h"
#include "lwip/timers.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if LWIP_IPV6
#error SV6X66 mDNS compatibility supports IPv4 only
#endif
#define LWIP_IPV4 1
#define LWIP_MDNS_RESPONDER ENABLE_DRIVER_MDNS
#define LWIP_MDNS_SEARCH 0
#define MDNS_RESP_USENETIF_EXTCALLBACK 0
#define LWIP_NETIF_EXT_STATUS_CALLBACK 0
#define LWIP_ASSERT_CORE_LOCKED() ((void)0)
#ifndef LWIP_RAND
#define LWIP_RAND() ((u32_t)rand())
#endif
#ifndef PACK_STRUCT_FLD_8
#define PACK_STRUCT_FLD_8(x) PACK_STRUCT_FIELD(x)
#endif
#define IP_IS_V6(a) 0
#define IP_IS_V6_VAL(a) 0
#define IP_IS_V4_VAL(a) 1
#define ip_2_ip4(a) (a)
#define netif_ip4_addr(n) (&(n)->ip_addr)
#define netif_ip4_netmask(n) (&(n)->netmask)
#define ip4_addr_isany_val(a) ((a).addr == 0)
#define ip4_addr_net_eq(a,b,m) ip_addr_netcmp(a,b,m)
#define ip_addr_eq(a,b) ip_addr_cmp(a,b)
#define ip_current_input_netif() ip_current_netif()
#define IPADDR4_INIT_BYTES(a,b,c,d) { PP_HTONL(((u32_t)(a)<<24)|((u32_t)(b)<<16)|((u32_t)(c)<<8)|(d)) }
#define IPADDR_TYPE_ANY 0
#define IP_ANY_TYPE IP_ADDR_ANY
#define udp_new_ip_type(t) udp_new()
#define udp_set_multicast_ttl(p,t) ((p)->mcast_ttl=(t))
#define igmp_joingroup_netif(n,a) igmp_joingroup(&(n)->ip_addr,(ip_addr_t *)(a))
#define igmp_leavegroup_netif(n,a) igmp_leavegroup(&(n)->ip_addr,(ip_addr_t *)(a))
#define lwip_itoa(b,s,v) ((void)snprintf(b,s,"%d",v))
void *sv_mdns_calloc(size_t,size_t);
#define mem_calloc sv_mdns_calloc
int sv_mdns_strnicmp(const char *,const char *,size_t);
#define lwip_strnicmp sv_mdns_strnicmp
struct pbuf *sv_mdns_pbuf_skip(struct pbuf *,u16_t,u16_t *);
err_t sv_mdns_pbuf_take_at(struct pbuf *,const void *,u16_t,u16_t);
#define pbuf_skip sv_mdns_pbuf_skip
#define pbuf_take_at sv_mdns_pbuf_take_at
void *sv_mdns_client_data(struct netif *);
void sv_mdns_set_client_data(struct netif *,void *);
#define netif_get_client_data(n,id) sv_mdns_client_data(n)
#define netif_set_client_data(n,id,p) sv_mdns_set_client_data(n,p)
#define netif_alloc_client_data_id() 0
void sv_mdns_timeout(u32_t,sys_timeout_handler,void *);
void sv_mdns_untimeout(sys_timeout_handler,void *);
void sv_mdns_cancel_timers(void);
#ifndef SV_MDNS_TIMER_IMPLEMENTATION
#ifdef sys_timeout
#undef sys_timeout
#endif
#define sys_timeout sv_mdns_timeout
#define sys_untimeout sv_mdns_untimeout
#endif

/* Four aligned, fixed slots; the responder alone owns this pool. */
#define LWIP_MEMPOOL_DECLARE(name,count,size,desc) \
  static union {void *align;u8_t bytes[size];} name##_slots[count]; \
  static u8_t name##_used[count]; \
  static void *name##_alloc(void) {unsigned i;for(i=0;i<count;i++) \
    if(!name##_used[i]) {name##_used[i]=1;return name##_slots[i].bytes;}return NULL;} \
  static void name##_free(void *p) {unsigned i;for(i=0;i<count;i++) \
    if(p==name##_slots[i].bytes) {name##_used[i]=0;return;}}
#define LWIP_MEMPOOL_INIT(n) memset(n##_used,0,sizeof(n##_used))
#define LWIP_MEMPOOL_ALLOC(n) n##_alloc()
#define LWIP_MEMPOOL_FREE(n,p) n##_free(p)
void sv_mdns_responder_shutdown(void);
#endif
