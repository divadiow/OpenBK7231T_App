#define SV_MDNS_TIMER_IMPLEMENTATION
#include "compat.h"

#if LWIP_MDNS_RESPONDER

static struct netif *owner;
static void *host;
/* IPv4: five response timers, one probe timer and four TC-question timers. */
static struct {sys_timeout_handler handler;void *arg;} timers[16];

/* Pair with lwIP's mem_free, not the OBK allocation wrapper. */
void *sv_mdns_calloc(size_t count,size_t size)
{
    size_t bytes;
    void *p;
    if(size && count>(size_t)-1/size)return NULL;
    bytes=count*size;
    if(!bytes || bytes>(mem_size_t)-1)return NULL;
    p=mem_malloc((mem_size_t)bytes);
    if(p)memset(p,0,bytes);
    return p;
}

void *sv_mdns_client_data(struct netif *n) {return n==owner?host:NULL;}
void sv_mdns_set_client_data(struct netif *n,void *p)
{owner=p?n:NULL;host=p;}

static void dispatch(void *arg)
{
    unsigned i=(unsigned)((char *)arg-(char *)timers)/sizeof(timers[0]);
    sys_timeout_handler handler=timers[i].handler;
    void *value=timers[i].arg;
    timers[i].handler=NULL;
    if(handler)handler(value);
}
void sv_mdns_timeout(u32_t ms,sys_timeout_handler h,void *arg)
{
    unsigned i;
    for(i=0;i<16;i++)if(!timers[i].handler) {
        timers[i].handler=h;timers[i].arg=arg;
        sys_timeout(ms,dispatch,&timers[i]);return;
    }
}
void sv_mdns_untimeout(sys_timeout_handler h,void *arg)
{
    unsigned i;
    for(i=0;i<16;i++)if(timers[i].handler==h && timers[i].arg==arg) {
        sys_untimeout(dispatch,&timers[i]);timers[i].handler=NULL;return;
    }
}
void sv_mdns_cancel_timers(void)
{
    unsigned i;
    for(i=0;i<16;i++)if(timers[i].handler) {
        sys_untimeout(dispatch,&timers[i]);timers[i].handler=NULL;
    }
}
struct pbuf *sv_mdns_pbuf_skip(struct pbuf *p,u16_t offset,u16_t *remaining)
{
    while(p && offset>=p->len) {offset-=p->len;p=p->next;}
    *remaining=offset;return p;
}
err_t sv_mdns_pbuf_take_at(struct pbuf *p,const void *src,u16_t length,u16_t offset)
{
    const u8_t *s=src;
    if(!p || offset>p->tot_len || length>p->tot_len-offset)return ERR_ARG;
    p=sv_mdns_pbuf_skip(p,offset,&offset);
    while(length && p) {
        u16_t n=LWIP_MIN(length,p->len-offset);
        MEMCPY((u8_t *)p->payload+offset,s,n);
        s+=n;length-=n;p=p->next;offset=0;
    }
    return length?ERR_ARG:ERR_OK;
}
int sv_mdns_strnicmp(const char *a,const char *b,size_t n)
{
    while(n--) {
        unsigned char x=*a++,y=*b++;
        if(x>='A' && x<='Z')x+='a'-'A';
        if(y>='A' && y<='Z')y+='a'-'A';
        if(x!=y)return (int)x-(int)y;
        if(!x)return 0;
    }
    return 0;
}
#endif
