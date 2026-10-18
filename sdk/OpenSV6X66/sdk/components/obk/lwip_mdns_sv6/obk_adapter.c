#include "sv6x66_platform.h"
#include "new_common.h"
#include "hal/hal_wifi.h"
#include "logging/logging.h"
#if ENABLE_DRIVER_MDNS
#include "lwip/apps/mdns.h"
#include "lwip/apps/mdns_priv.h"
#include "lwip/tcpip.h"

static struct netif *attached;
static netif_status_callback_fn previous_remove;
static OsTaskHandle core_task;
static volatile unsigned suspended;
static volatile unsigned request_state;
static unsigned request_start;
static void (*request_callback)(void);
static void (*cleanup_callback)(void);

static void removed(struct netif *netif);
void sv_mdns_detach_netif(struct netif *n)
{
	/* Release every delayed packet before the SDK may free its STA netif. */
	sv_mdns_responder_shutdown();
	if(attached==n) {
		n->remove_callback=previous_remove;
		attached=NULL;
	}
}
static void apply(void *unused)
{
	unsigned run;
	OS_DeclareCritical();
	(void)unused;core_task=OS_TaskGetCurrHandle();
	OS_EnterCritical();run=!request_start || !suspended;OS_ExitCritical();
	if(run)request_callback();
	OS_EnterCritical();request_state=request_state==3?0:2;OS_ExitCritical();
}

/* Static requests survive timeout. States: idle, queued, done, abandoned. */
static int exchange(void (*callback)(void),unsigned start,unsigned wait_ms)
{
	unsigned waited=0,state;
	OS_DeclareCritical();
	for(;;) {
		OS_EnterCritical();state=request_state;
		if(state==0)request_state=1;
		OS_ExitCritical();
		if(state==0)break;
		if(wait_ms && waited>=wait_ms)return 0;
		OS_MsDelay(10);waited+=10;
	}
	request_start=start;
	request_callback=callback;
	if(tcpip_callback_with_block(apply,NULL,0)!=ERR_OK) {
		OS_EnterCritical();request_state=0;OS_ExitCritical();return 0;
	}
	for(;;) {
		OS_EnterCritical();state=request_state;
		if(state==2) {request_state=0;OS_ExitCritical();return 1;}
		else if(wait_ms && waited>=wait_ms)request_state=3;
		OS_ExitCritical();
		if(wait_ms && waited>=wait_ms)return 0;
		OS_MsDelay(10);waited+=10;
	}
}
static void removed(struct netif *netif)
{
	netif_status_callback_fn old=previous_remove;
	OS_DeclareCritical();
	OS_EnterCritical();suspended=1;OS_ExitCritical();
	if(OS_TaskGetCurrHandle()==core_task)cleanup_callback();
	else {
		/* Unknown SDK callback context: do not return to netif free until TCPIP
			* has detached. This last-resort path may wait under queue pressure. */
		while(!exchange(cleanup_callback,0,0))OS_MsDelay(10);
	}
	if(old)old(netif);
}
int SV6X66_MDNS_BeforeWifiStop(void)
{
	unsigned idle;
	OS_DeclareCritical();
	OS_EnterCritical();suspended=1;idle=!attached && !request_state;OS_ExitCritical();
	if(idle)return 1;
	if(!cleanup_callback || !exchange(cleanup_callback,0,2000)) {
		addLogAdv(LOG_ERROR,LOG_FEATURE_HTTP,"MDNS cleanup pending; WiFi transition deferred");
		return 0;
	}
	return 1;
}
struct netif *SV6X66_MDNS_GetStaNetif(void)
{
	struct netif *n;
	u32_t ip;
	if(attached)return attached;
	ip=ipaddr_addr(HAL_GetMyIPString());
	for(n=netif_list;n;n=n->next)
		if(netif_is_up(n) && n->ip_addr.addr==ip && ip)return n;
	return NULL;
}
void sv_mdns_attach_netif(struct netif *n)
{
	attached=n;previous_remove=n->remove_callback;n->remove_callback=removed;
}
void SV6X66_MDNS_Dispatch(void (*callback)(void),void (*cleanup)(void),int start)
{
	unsigned idle;
	OS_DeclareCritical();
	OS_EnterCritical();
	suspended=!start;cleanup_callback=cleanup;idle=!attached && !request_state;
	OS_ExitCritical();
	if(!start && idle)return;
	if(!exchange(callback,start,2000)) {
		addLogAdv(LOG_ERROR,LOG_FEATURE_HTTP,start?
			"DRV_MDNS: TCPIP operation pending or unavailable":
			"DRV_MDNS: TCPIP cleanup pending");
	}
}
#else
int SV6X66_MDNS_BeforeWifiStop(void) {return 1;}
#endif
