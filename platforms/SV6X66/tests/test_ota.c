#include "../ota_stage.h"
#include "../sv6x66_layout.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define LENGTH (SV6X66_APP_START + 64)
static uint8_t image[48 + LENGTH];
typedef struct { int marker, opened, closed, verified, published, fault; uint32_t written; } mock_t;
static int disarm(void *p) { mock_t *m=p; if(m->fault==1 || (m->fault==7 && m->verified))return -1; m->marker=0; return 0; }
static int begin(void *p) { mock_t *m=p; assert(!m->marker); if(m->fault==2)return -1; m->opened=1; return 0; }
static int write_data(void *p,const uint8_t *b,size_t n) { mock_t *m=p; assert(m->opened && !m->marker); (void)b; if(m->fault==3)return -1; m->written+=n; return 0; }
static int close_data(void *p) { mock_t *m=p; assert(m->opened); m->opened=0; m->closed=1; return m->fault==4 ? -1:0; }
static int verify(void *p,uint32_t n,const uint8_t md5[16]) { mock_t *m=p; (void)md5; assert(m->closed && m->written==n && !m->marker); if(m->fault==5)return -1; m->verified=1; return 0; }
static int publish(void *p,const uint8_t md5[16]) { mock_t *m=p; (void)md5; assert(m->verified && !m->marker); m->marker=1; if(m->fault==6 || m->fault==7)return -1; m->published=1; return 0; }
static void put32(uint8_t *p,uint32_t n) { p[0]=n;p[1]=n>>8;p[2]=n>>16;p[3]=n>>24; }
static uint32_t crc(const uint8_t *p,size_t n) { uint32_t c=~0u; while(n--) { c^=*p++; for(int k=0;k<8;k++) c=(c>>1)^(0xedb88320u&(0u-(c&1))); } return c^~0u; }
static void fixture(void) {
 memset(image,0,sizeof(image)); memcpy(image,"OBKSV616",8);put32(image+8,1);put32(image+12,LENGTH);
 put32(image+16,SV6X66_APP_START);put32(image+20,SV6X66_FS_START);put32(image+44,crc(image,44));
 uint8_t *b=image+48;put32(b+4,40);put32(b+8,80);put32(b+12,4);put32(b+16,SV6X66_MAIN_SIZE);put32(b+20,SV6X66_FLASH_SIZE);put32(b+36,SV6X66_RAW_SIZE);
}
static void setup(ota_stage_t *s,mock_t *m,int fault) { memset(m,0,sizeof(*m));m->marker=1;m->fault=fault;ota_stage_ops_t ops={m,disarm,begin,write_data,close_data,verify,publish};assert(!ota_stage_init(s,&ops,sizeof(image))); }
int main(void) {
 ota_stage_t s;mock_t m;fixture();setup(&s,&m,0);
 for(size_t i=0;i<sizeof(image);i++) assert(!ota_stage_feed(&s,image+i,1));
 assert(!ota_stage_finish(&s));assert(m.marker && m.published && m.verified);assert(ota_stage_feed(&s,image,1));
 for(int fault=1;fault<=7;fault++) { fixture();setup(&s,&m,fault);int result=ota_stage_feed(&s,image,sizeof(image));if(!result)result=ota_stage_finish(&s);assert(result);assert(!m.published);if(fault!=1 && fault!=7)assert(!m.marker);if(fault==7)assert(m.marker && m.verified && s.disarm_failed);if(fault==1)assert(s.disarm_failed); }
 fixture();setup(&s,&m,0);assert(!ota_stage_feed(&s,image,sizeof(image)-1));assert(ota_stage_finish(&s));assert(!m.marker&&!m.published);
 fixture();setup(&s,&m,0);assert(!ota_stage_feed(&s,image,20));assert(ota_stage_finish(&s));assert(!m.published);
 fixture();setup(&s,&m,0);image[44]^=1;assert(ota_stage_feed(&s,image,sizeof(image)));assert(!m.opened&&!m.published);
 fixture();setup(&s,&m,0);put32(image+16,0);put32(image+44,crc(image,44));assert(ota_stage_feed(&s,image,sizeof(image)));assert(!m.opened&&!m.published);
 fixture();setup(&s,&m,0);put32(image+12,SV6X66_FS_START+1);put32(image+44,crc(image,44));assert(ota_stage_feed(&s,image,sizeof(image)));assert(!m.opened&&!m.published);
 fixture();setup(&s,&m,0);image[48+36]^=1;assert(!ota_stage_feed(&s,image,sizeof(image)));assert(ota_stage_finish(&s));assert(!m.verified&&!m.published&&!m.marker);
 fixture();setup(&s,&m,0);assert(!ota_stage_feed(&s,image,sizeof(image)));assert(ota_stage_feed(&s,image,1));assert(!m.published&&!m.marker);
 fixture();setup(&s,&m,0);assert(!ota_stage_feed(&s,NULL,0));ota_stage_abort(&s);assert(ota_stage_feed(&s,image,1));assert(!m.published);
 puts("OTA staging fault tests passed");return 0;
}
