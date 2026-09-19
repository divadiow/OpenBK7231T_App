/* Compile the actual driver with transport/HAL stubs. Not a radio/network test. */
#include "stub.h"
#include "src/driver/drv_wemo.c"

static int checks;
#define CHECK(x) do { checks++; if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static char output[32768], ssdp[2048];
static int roles[64], channels[64], types[64], values[64];
static int led_running, led_power, writes, sent, registrations, fail_at = -1, alloc_fail;
static int stop_on_write;
static const char *name = "Original name", *ip = "192.0.2.42";
static unsigned char hardware_mac[6] = {0x10,0x20,0xAB,0xCD,0xEF,0x12};
static struct { const char *path; int method; http_callback_fn handler; } callbacks[32];

void *test_malloc(size_t n) { if (alloc_fail) return NULL; return malloc(n); }
int CHANNEL_GetType(int c) { return types[c]; }
int h_isChannelRelay(int c) { int i; for(i=0;i<64;i++) if(channels[i]==c && roles[i]) return 1; return 0; }
int CHANNEL_Get(int c) { return values[c]; }
void CHANNEL_Set(int c,int v,int flags) { CHECK(flags==0); values[c]=v; writes++; }
int LED_IsLEDRunning(void) { return led_running; }
int LED_GetEnableAll(void) { return led_power; }
void LED_SetEnableAll(int v) { led_power=v; writes++; }
int CMD_ExecuteCommand(const char *command,int flags) {
    int c, value = !strcmp(command,"POWER ON");
    CHECK(value || !strcmp(command,"POWER OFF"));CHECK(flags==0);
#if ENABLE_LED_BASIC
    if(led_running) {LED_SetEnableAll(value);return 0;}
#endif
    for(c=0;c<CHANNEL_MAX;c++) if(h_isChannelRelay(c) || CHANNEL_GetType(c)==ChType_Toggle) {CHANNEL_Set(c,value,0);break;}
    return 0;
}
void WiFI_GetMacAddress(char *mac) { memcpy(mac,hardware_mac,6); }
const char *HAL_GetMyIPString(void) { return ip; }
const char *CFG_GetDeviceName(void) { return name; }
void addLogAdv(int a,int b,const char *fmt,...) { (void)a;(void)b;(void)fmt; }
int my_strnicmp(const char *a,const char *b,int n) { while(n--) { if(!*a || !*b || ((*a|32)!=(*b|32))) return 1; a++;b++; } return 0; }
int poststr(http_request_t *r,const char *s) {
    (void)r;
    if(s) { CHECK(strlen(output)+strlen(s)<sizeof(output)); strcat(output,s); }
    if(stop_on_write) {stop_on_write=0;WEMO_Shutdown();}
    return 0;
}
int hprintf255(http_request_t *r,const char *fmt,...) { char buf[255];int n;va_list ap;va_start(ap,fmt);n=vsnprintf(buf,sizeof(buf),fmt,ap);va_end(ap);CHECK(n>=0 && n<(int)sizeof(buf));return poststr(r,buf); }
void http_setup(http_request_t *r,const char *mime) { hprintf255(r,"HTTP/1.1 %d OK\r\nContent-type: %s\r\n\r\n",r->responseCode,mime); }
void poststr_escaped(http_request_t *r,char *s) { char b[2]={0};for(;*s;s++){switch(*s){case '&':poststr(r,"&amp;");break;case '<':poststr(r,"&lt;");break;case '>':poststr(r,"&gt;");break;case '"':poststr(r,"&quot;");break;default:b[0]=*s;poststr(r,b);}} }
int http_rest_error(http_request_t *r,int code,char *msg) {r->responseCode=code;return hprintf255(r,"{\"error\":%d,\"message\":\"%s\"}",code,msg);}
int HTTP_RegisterCallback(const char *path,int method,http_callback_fn cb,int auth) {
    int i;CHECK(auth==0);
    if(registrations==fail_at || registrations>=32) return -4;
    for(i=0;i<registrations;i++) if(callbacks[i].handler==cb && !strcmp(callbacks[i].path,path) && callbacks[i].method==method) return i;
    callbacks[registrations].path=path;callbacks[registrations].method=method;callbacks[registrations++].handler=cb;return 0;
}
void DRV_SSDP_SendReply(struct sockaddr_in *addr,const char *message) {(void)addr;CHECK(strlen(message)<sizeof(ssdp));strcpy(ssdp,message);sent++;}

static http_request_t request(const char *body,const char *header) {
    http_request_t r={0}; r.responseCode=200;r.bodystart=(char*)body;
    if(body) r.contentLength=r.bodylen=(int)strlen(body);
    if(header) {r.headers[0]=(char*)header;r.numheaders=1;}
    output[0]=0;return r;
}
static const char *soap(const char *action,const char *args,int meta) {
    static char b[8192];
    snprintf(b,sizeof(b),"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<s:Envelope xmlns:s=\"%s\" s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\"><s:Body><u:%s xmlns:u=\"%s\">%s</u:%s></s:Body></s:Envelope>",WEMO_SOAP_NS,action,meta?WEMO_META_SERVICE:WEMO_SERVICE,args,action);return b;
}
static void reset(void) {
    WEMO_Shutdown();g_wemo_registered=0;registrations=0;fail_at=-1;
    memset(callbacks,0,sizeof(callbacks));memset(roles,0,sizeof(roles));memset(channels,0,sizeof(channels));memset(types,0,sizeof(types));memset(values,0,sizeof(values));
    led_running=led_power=writes=sent=alloc_fail=stop_on_write=0;output[0]=ssdp[0]=0;name="Original name";ip="192.0.2.42";WEMO_Init();CHECK(g_wemo_enabled);
}
static void fault(const char *body,const char *header,int code,int meta) {
    http_request_t r=request(body,header);int before=writes;char text[64];
    if(meta) WEMO_MetaInfo1(&r);else WEMO_BasicEvent1(&r);
    snprintf(text,sizeof(text),"<errorCode>%d</errorCode>",code);
    CHECK(r.responseCode==500);CHECK(strstr(output,text)!=NULL);CHECK(writes==before);CHECK(strstr(output,"<s:Fault>")!=NULL);
}
static void test_ota(void) {
    http_request_t r;int c,role;
    reset();name="Original & <name> \"quote\"";types[1]=ChType_Toggle;roles[7]=1;channels[7]=2;
    r=request(soap("SetBinaryState","<BinaryState>1</BinaryState>",0),NULL);WEMO_BasicEvent1(&r);
    CHECK(values[1]==1 && values[2]==0);CHECK(strstr(output,"<u:SetBinaryStateResponse")!=NULL);
    r=request(NULL,NULL);WEMO_Setup(&r);
    CHECK(strstr(output,"Original &amp; &lt;name&gt; &quot;quote&quot;")!=NULL);
    CHECK(strstr(output,"uuid:Socket-1_0-201612ABCDEF12")!=NULL);
    CHECK(strstr(output,"<serialNumber>192.0.2.42</serialNumber>")!=NULL);
    CHECK(strstr(output,"<presentationURL>http://192.0.2.42:80/")!=NULL);
    CHECK(strstr(output,"<binaryState>1</binaryState>")!=NULL);
    /* Exhaustive pairwise ordering: original eligible roles + virtual toggle. */
    for(role=1;role<=6;role++) for(c=0;c<CHANNEL_MAX;c++) {
        int t;for(t=0;t<CHANNEL_MAX;t++) {
            memset(types,0,sizeof(types));memset(roles,0,sizeof(roles));
            roles[7]=role;channels[7]=c;types[t]=ChType_Toggle;
            CHECK(WEMO_FindMainChannel()==(c<t?c:t));
        }
    }
    reset();roles[7]=2;channels[7]=2;
    r=request(soap("SetBinaryState","<BinaryState>1</BinaryState>",0),NULL);WEMO_BasicEvent1(&r);CHECK(values[2]==1);
    r=request(soap("GetBinaryState","<BinaryState>0</BinaryState>",0),NULL);WEMO_BasicEvent1(&r);CHECK(values[2]==1);CHECK(strstr(output,"<BinaryState>1</BinaryState>")!=NULL);
#if ENABLE_LED_BASIC
    led_running=1;led_power=1;
    r=request(soap("SetBinaryState","<BinaryState>0</BinaryState>",0),NULL);WEMO_BasicEvent1(&r);CHECK(led_power==0 && values[2]==1);
#endif
    reset();r=request(soap("SetBinaryState","<BinaryState>1</BinaryState>",0),NULL);WEMO_BasicEvent1(&r);CHECK(writes==0 && strstr(output,"<BinaryState>0</BinaryState>"));
}
static void test_soap(void) {
    const char *bad[]={"", "garbage", "-1", "2", "1xyz", "01", "1 0", "true", "0.0", "&#49;"};
    const char *malformed[]={"<BinaryState>1", "<BinaryState/>", "<BinaryState>1</Wrong>", "<BinaryState>1</BinaryState><BinaryState>0</BinaryState>", "<wrapper><BinaryState>1</BinaryState></wrapper>", "<BinaryState><x/>1</BinaryState>", "<!-- <BinaryState>1</BinaryState> -->", "<Other x=\"<BinaryState>1</BinaryState>\"/>"};
    char b[8192];size_t i;int state,action;http_request_t r;
    reset();types[1]=ChType_Toggle;
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++){snprintf(b,sizeof(b),"<BinaryState>%s</BinaryState>",bad[i]);fault(soap("SetBinaryState",b,0),NULL,402,0);}
    for(i=0;i<sizeof(malformed)/sizeof(malformed[0]);i++) fault(soap("SetBinaryState",malformed[i],0),NULL,402,0);
    fault(soap("NotSetBinaryState","<BinaryState>1</BinaryState>",0),NULL,401,0);
    fault(soap("GetMetaInfo","",1),NULL,401,0);
    fault(soap("GetBinaryState","",0),"SOAPACTION: \"urn:Belkin:service:basicevent:1#SetBinaryState\"",401,0);
    fault(soap("SetBinaryState","<BinaryState>1</BinaryState>",0),"SOAPACTION: \"urn:Belkin:service:metainfo:1#SetBinaryState\"",401,0);
    fault(NULL,NULL,402,0);
    r=request(soap("SetBinaryState","<BinaryState> \r\n1\t </BinaryState>",0),"sOaPaCtIoN: \"urn:Belkin:service:basicevent:1#SetBinaryState\"\r\n");
    WEMO_BasicEvent1(&r);CHECK(r.responseCode==200 && values[1]==1);
    r=request("<Envelope xmlns=\"" WEMO_SOAP_NS "\" xmlns:b=\"" WEMO_SERVICE "\"><Body><b:SetBinaryState><b:BinaryState>0</b:BinaryState></b:SetBinaryState></Body></Envelope>",NULL);
    WEMO_BasicEvent1(&r);CHECK(r.responseCode==200 && values[1]==0);
    r=request("<s:Envelope xmlns:s=\"" WEMO_SOAP_NS "\"><s:Body><GetBinaryState xmlns=\"" WEMO_SERVICE "\"/></s:Body></s:Envelope>",NULL);
    WEMO_BasicEvent1(&r);CHECK(r.responseCode==200 && strstr(output,"GetBinaryStateResponse"));
    r=request("<s:Envelope xmlns:s=\"" WEMO_SOAP_NS "\"><s:Header><client xmlns=\"urn:example\">legacy controller</client></s:Header><s:Body><u:GetBinaryState xmlns:u=\"" WEMO_SERVICE "\"/></s:Body></s:Envelope>",NULL);
    WEMO_BasicEvent1(&r);CHECK(r.responseCode==200 && strstr(output,"GetBinaryStateResponse"));
    /* Truncate a valid request at every byte, without a NUL-termination promise. */
    strcpy(b,soap("SetBinaryState","<BinaryState>1</BinaryState>",0));
    for(i=0;i<strlen(b);i++){state=-1;action=WEMO_ParseSOAP(b,(int)i,0,&state);CHECK(action<0);}
    r=request(b,NULL);r.bodylen--;WEMO_BasicEvent1(&r);CHECK(r.responseCode==500 && values[1]==0);
    r=request(b,NULL);r.headers[0]="SOAPACTION: \"urn:Belkin:service:basicevent:1#SetBinaryState\"";r.headers[1]=r.headers[0];r.numheaders=2;WEMO_BasicEvent1(&r);CHECK(r.responseCode==500 && values[1]==0);
    strcat(b,"<extra/>");fault(b,NULL,402,0);
    fault("<!DOCTYPE x><s:Envelope/>",NULL,402,0);
    fault("<s:Envelope xmlns:s=\"" WEMO_SOAP_NS "\" xmlns:s=\"other\"><s:Body/></s:Envelope>",NULL,402,0);
    fault("<s:Envelope xmlns:s=\"" WEMO_SOAP_NS "\"><s:Body><s:x:GetBinaryState/></s:Body></s:Envelope>",NULL,402,0);
}
static void test_metadata_and_lifecycle(void) {
    http_request_t r;int i;
    reset();r=request(soap("GetMetaInfo","",1),NULL);WEMO_MetaInfo1(&r);
    CHECK(r.responseCode==200);CHECK(strstr(output,"1020ABCDEF12|192.0.2.42|OpenBeken|" WEMO_TEST_VERSION_XML "||Socket")!=NULL);
    fault(soap("GetBinaryState","",0),NULL,401,1);fault(soap("GetExtMetaInfo","",1),NULL,401,1);
    r=request(NULL,NULL);WEMO_MetaInfoService(&r);CHECK(!strstr(output,"GetExtMetaInfo"));CHECK(strstr(output,"<direction>out</direction>"));
    for(i=0;i<100;i++){WEMO_Shutdown();r=request(NULL,NULL);WEMO_Setup(&r);CHECK(r.responseCode==404);WEMO_Init();CHECK(g_wemo_enabled && registrations==5);}
    /* The old four endpoints still work if only four callback slots are free. */
    WEMO_Shutdown();g_wemo_registered=0;registrations=0;fail_at=4;WEMO_Init();CHECK(g_wemo_enabled);r=request(NULL,NULL);WEMO_Setup(&r);CHECK(r.responseCode==200);
    WEMO_Shutdown();g_wemo_registered=0;registrations=0;fail_at=2;WEMO_Init();CHECK(!g_wemo_enabled && registrations==2);
    fail_at=-1;WEMO_Init();CHECK(g_wemo_enabled && registrations==5);
    registrations=32;WEMO_Shutdown();WEMO_Init();CHECK(g_wemo_enabled);
    r=request(NULL,NULL);stop_on_write=1;WEMO_Setup(&r);CHECK(strstr(output,"Socket-1_0-201612ABCDEF12")!=NULL);CHECK(!g_wemo_enabled);
}
static void test_ssdp(void) {
    struct sockaddr_in address={0};int mode;
    reset();
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: urn:Belkin:device:**\r\n\r\n")==1);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nsT: urn:belkin:device:controllee:1\r\n\r\n")==3);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: upnp:rootdevice\r\n\r\n")==2);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: ssdp:all\r\n\r\n")==2);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: ssdpsearch:all\r\n\r\n")==2);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: unrelated\r\nX: urn:Belkin:device:**\r\n\r\n")==0);
    CHECK(DRV_WEMO_GetSearchType("M-SEARCH * HTTP/1.1\r\nST: urn:Belkin:device:**\r\nST: ssdp:all\r\n\r\n")==0);
    for(mode=1;mode<=3;mode++){sent=0;DRV_WEMO_Send_Advert_To(mode,&address);CHECK(sent==1);CHECK(strstr(ssdp,"LOCATION: http://192.0.2.42:80/setup.xml\r\n"));CHECK(strstr(ssdp,"USN: uuid:Socket-1_0-201612ABCDEF12::"));CHECK(strstr(ssdp,mode==1?"ST: urn:Belkin:device:**\r\n":mode==3?"ST: urn:Belkin:device:controllee:1\r\n":"ST: upnp:rootdevice\r\n"));}
    WEMO_Shutdown();sent=0;DRV_WEMO_Send_Advert_To(1,&address);CHECK(sent==0);WEMO_Init();alloc_fail=1;DRV_WEMO_Send_Advert_To(1,&address);CHECK(sent==0);alloc_fail=0;DRV_WEMO_Send_Advert_To(1,&address);CHECK(sent==1);
}
static void test_mutations(void) {
    char b[1024];const char *base=soap("SetBinaryState","<BinaryState>1</BinaryState>",0);size_t n=strlen(base),i;int state;unsigned int rng=0x12345;
    for(i=0;i<10000;i++) {int j;strcpy(b,base);for(j=0;j<3;j++){rng=rng*1664525U+1013904223U;b[rng%n]=(char)(rng>>24);}state=-1;WEMO_ParseSOAP(b,(int)n,0,&state);}
    CHECK(1);
}
int main(void) {test_ota();test_soap();test_metadata_and_lifecycle();test_ssdp();test_mutations();WEMO_Shutdown();printf("PASS %d checks, plus 10000 bounded parser mutations (LED=%d)\n",checks,ENABLE_LED_BASIC);return 0;}
