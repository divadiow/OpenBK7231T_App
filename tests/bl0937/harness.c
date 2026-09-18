/* Execute production C with a deterministic HAL/clock.
 * This is not a BL602/RTL hardware emulator. See README for model limitations.
 */
#include "test_support.h"
#include "src/driver/drv_pwrCal.c"
#include "src/driver/drv_bl0937.c"
#include "src/driver/drv_bl_shared.c"

static portTickType clock_ticks;
static unsigned wall_ms, tick_remainder, injection_ms, deferred_ms;
static unsigned repeated_injections, tick_countdown, tick_calls;
static unsigned write_before_ms, write_after_ms, shared_pause_ms;
static unsigned pulse_remainder, cf1_remainder;
static uint64_t emitted_power;
static int physical_sel = 1;
static int mock_inverse, pin_cf=7, failures, reports, sel_writes;
int mock_irq_depth;
#define observed_v DRV_GetReading(OBK_VOLTAGE)
#define observed_i DRV_GetReading(OBK_CURRENT)
#define observed_p DRV_GetReading(OBK_POWER)
#define total_wh (datasetlist[0].sensors[OBK_CONSUMPTION_TOTAL].lastReading)
static CommandFn commands[3];
static const char *token;
static float cfg[4];
static bool cfg_set[4];
static void (*handlers[64])(int);
static const char *scenario;
static unsigned cfg_flags;
static int relay_on=1, movingavg_cnt=1, mqtt_calls, invalid_mqtt;
struct MockConfig g_cfg;
int g_secondsElapsed;
static void check(int ok,const char*expr,int line) {
    if(!ok) { ++failures; fprintf(stderr,"FAIL %s:%d %s\n",scenario,line,expr); }
}
#define CHECK(x) check(!!(x),#x,__LINE__)
static int near(float a,float b) { return isfinite(a) && fabsf(a-b)<0.0001f*fmaxf(1.0f,fabsf(b)); }
static void emit(unsigned power,unsigned cf1) {
    while(power--) if(handlers[GPIO_HLW_CF]) handlers[GPIO_HLW_CF](GPIO_HLW_CF);
    while(cf1--) if(handlers[GPIO_HLW_CF1]) handlers[GPIO_HLW_CF1](GPIO_HLW_CF1);
}
static void advance(unsigned ms,unsigned power_hz,unsigned cf1_hz) {
    CHECK(mock_irq_depth==0);
    uint64_t sum=(uint64_t)tick_remainder+(uint64_t)ms*TICK_HZ;
    clock_ticks=(portTickType)(clock_ticks+sum/1000U); tick_remainder=(unsigned)(sum%1000U);
    wall_ms+=ms;
    unsigned pn=ms*power_hz+pulse_remainder, vn=ms*cf1_hz+cf1_remainder;
    pulse_remainder=pn%1000;cf1_remainder=vn%1000;
    emitted_power+=pn/1000;
    emit(pn/1000,vn/1000);
}
static void signal(unsigned ms,unsigned power_hz) {
    advance(ms,power_hz,(physical_sel!=mock_inverse)?1000U:100U);
}
portTickType xTaskGetTickCount(void) {
    portTickType sampled=clock_ticks;
    ++tick_calls;
    if(tick_countdown) --tick_countdown;
    if(repeated_injections) { --repeated_injections; signal(20,100); }
    if(injection_ms && !tick_countdown) {
        unsigned ms=injection_ms; injection_ms=0;
        /* Simulate preemption immediately after loading the tick value.
         * GPIO interrupts continue while the main task is not scheduled.
         */
        if(mock_irq_depth) deferred_ms=ms;
        else signal(ms,100);
    }
    return sampled;
}
void mock_restore_irq(int depth) {
    mock_irq_depth=depth;
    if(!depth && deferred_ms) { unsigned ms=deferred_ms; deferred_ms=0; signal(ms,100); }
}
void addLogAdv(int level,int feature,const char*format,...) { (void)level;(void)feature;(void)format; }
int PIN_FindPinIndexForRole(int role,int fallback) {
    if(role==IOR_BL0937_SEL_n) return mock_inverse?24:fallback;
    if(role==IOR_BL0937_SEL) return mock_inverse?fallback:24;
    if(role==IOR_BL0937_CF) return pin_cf;
    if(role==IOR_BL0937_CF1) return 8;
    return fallback;
}
void HAL_PIN_Setup_Output(int pin) { (void)pin; }
void HAL_PIN_Setup_Input_Pullup(int pin) { (void)pin; }
void HAL_PIN_SetOutputValue(int pin,int value) {
    (void)pin;++sel_writes;
    if(write_before_ms) { unsigned ms=write_before_ms;write_before_ms=0;signal(ms,100); }
    physical_sel=value;cf1_remainder=0;
    if(write_after_ms) { unsigned ms=write_after_ms;write_after_ms=0;signal(ms,100); }
}
void HAL_AttachInterrupt(int pin,int mode,void(*fn)(int)) { (void)mode;CHECK(pin>=0&&pin<64);handlers[pin]=fn; }
void HAL_DetachInterrupt(int pin) { CHECK(pin>=0&&pin<64);handlers[pin]=NULL; }
void CMD_RegisterCommand(const char*name,CommandFn fn,const void*ctx) {
    (void)ctx;
    if(!strcmp(name,"VoltageSet")) commands[0]=fn;
    if(!strcmp(name,"CurrentSet")) commands[1]=fn;
    if(!strcmp(name,"PowerSet")) commands[2]=fn;
}
void Tokenizer_TokenizeString(const char*str,int flags) { (void)flags;token=str; }
int Tokenizer_CheckArgsCountAndPrintWarning(const char*cmd,int count) { (void)cmd;(void)count;return !token||!*token; }
float Tokenizer_GetArgFloat(int index) { (void)index;return strtof(token,NULL); }
const char*CMD_GetResultString(int res) { (void)res;return "bad argument"; }
float CFG_GetPowerMeasurementCalibrationFloat(int key,float fallback) { return cfg_set[key]?cfg[key]:fallback; }
void CFG_SetPowerMeasurementCalibrationFloat(int key,float value) { cfg_set[key]=true;cfg[key]=value; }
static void window(unsigned ms,unsigned power_hz) { signal(ms,power_hz);BL0937_RunEverySecond();CHECK(mock_irq_depth==0); }
static void warm(void) { for(int k=0;k<4;k++) window(1000,100); }
static void test_nominal(void) {
    for(int k=0;k<16;k++) { window(1000,100);if(k>=3) { CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));CHECK(near(observed_p,150)); } }
}
#if TEST_FIXED
static void test_settling(void) {
    /* Independent fixed 1-second mode schedule: first window corrupt,
       second window clean. Normal and inverse physical SEL are tested. */
    for(int k=0;k<24;k++) {
        unsigned cf1=(k%2==0)?19000U:((physical_sel!=mock_inverse)?1000U:100U);
        advance(1000,100,cf1); BL0937_RunEverySecond();
        if(k>=3) {CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));}
        CHECK(near(observed_p,150));
    }
}
static void test_threshold(void) {
    int writes=sel_writes, n=reports;uint32_t base=g_p_pulses;
    BL0937_RunEverySecond();CHECK(reports==n);CHECK(sel_writes==writes);
    advance(500-TICK_MS,1000,1000);BL0937_RunEverySecond();
    CHECK(reports==n);CHECK(g_p_pulses==base+500-TICK_MS);CHECK(sel_writes==writes);
    advance(TICK_MS,1000,1000);BL0937_RunEverySecond();
    CHECK(reports==n+1);CHECK(g_p_pulses==base+500);CHECK(sel_writes==writes);CHECK(near(observed_p,1500));
    CHECK(isnan(observed_v)&&isnan(observed_i));
    advance(500,1000,1000);BL0937_RunEverySecond();CHECK(sel_writes==writes);
    advance(500,1000,(physical_sel!=mock_inverse)?1000:100);BL0937_RunEverySecond();
    CHECK(sel_writes==writes+1);CHECK(!isnan(mock_inverse?observed_i:observed_v));
}
static void test_delays(void) {
    const unsigned ds[]={1000,4000,20,980,1000,5000,0,20,980,1000,800,1200,1000};
    warm();
    for(unsigned k=0;k<sizeof(ds)/sizeof(*ds);k++) {
        window(ds[k],100);CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));
        CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));CHECK(near(observed_p,150));
    }
}
static void test_calibration_isolation(void) {
    warm();float rv=latest_raw_voltage,ri=latest_raw_current;
    (void)PwrCal_ScalePower(0.625f);CHECK(latest_raw_voltage==rv);CHECK(latest_raw_current==ri);CHECK(latest_raw_power==0.625f);
    (void)PwrCal_ScaleVoltage(1234.5f);CHECK(latest_raw_current==ri);CHECK(latest_raw_power==0.625f);
    (void)PwrCal_ScaleCurrent(34.75f);CHECK(latest_raw_voltage==1234.5f);CHECK(latest_raw_power==0.625f);
    CHECK(commands[2](NULL,"PowerSet","1.25",0)==CMD_RES_OK);CHECK(near(PwrCal_ScalePower(0.625f),1.25f));
}
static void test_restart(int inverse_only) {
    warm();int n=reports;uint32_t total=g_p_pulses;
    if(inverse_only)mock_inverse=!mock_inverse;else pin_cf=9;
    BL0937_RunEverySecond();CHECK(reports==n);CHECK(g_invertSEL==mock_inverse);
    CHECK(isnan(final_v)&&isnan(final_c));CHECK(g_p_pulses==total);
    if(!inverse_only){CHECK(GPIO_HLW_CF==9);CHECK(handlers[7]==NULL);CHECK(handlers[9]!=NULL);}
    window(1000,100);CHECK(reports==n+1);CHECK(near(observed_p,150));CHECK(isnan(observed_v)&&isnan(observed_i));
    for(int k=0;k<3;k++)window(1000,100);
    CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));
    CHECK(fabs(total_wh-(emitted_power*1.5/3600.0))<1e-6);
}
static void test_restart_pending(void) {
    warm();signal(1000,100);mock_inverse=!mock_inverse;
    BL0937_RunEverySecond();
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
    for(int k=0;k<4;k++)window(1000,100);
    CHECK(near(observed_p,150));
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
}
static void test_retry_exhaustion(void) {
    warm();int n=reports,writes=sel_writes;uint32_t before=g_p_pulses;
    signal(1000,100);repeated_injections=8;unsigned calls=tick_calls;
    BL0937_RunEverySecond();CHECK(tick_calls-calls==8);CHECK(reports==n);CHECK(sel_writes==writes);
    CHECK(g_p_pulses>before);window(1000,100);CHECK(near(observed_p,150));
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
}
static void test_second_read_preemption(void) {
    warm();signal(1000,100);tick_countdown=2;injection_ms=1500;BL0937_RunEverySecond();
    CHECK(near(observed_p,150));window(1000,100);CHECK(near(observed_p,150));
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
}
static void test_delayed_write(int after) {
    window(1000,100);signal(1000,100);
    if(after)write_after_ms=1500;else write_before_ms=1500;
    BL0937_RunEverySecond();CHECK(near(observed_p,150));
    int writes=sel_writes;
    /* A callback immediately after delayed SEL must NOT finish settling.
       The next full post-write interval is also discarded. */
    advance(20,100,19000);BL0937_RunEverySecond();
    advance(980,100,19000);BL0937_RunEverySecond();CHECK(sel_writes==writes);
    window(1000,100);CHECK(sel_writes==writes+1);
    CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
}
static void test_publish_delay(void) {
    warm();signal(1000,100);shared_pause_ms=1500;BL0937_RunEverySecond();
    CHECK(near(observed_p,150));window(1000,100);
    CHECK(near(observed_p,150));CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-6);
}
static void test_partial_updates(void) {
    window(1000,100);CHECK(isnan(observed_v)&&isnan(observed_i));CHECK(near(observed_p,150));
    CHECK(!BL_HasEnergySensorReading(OBK_VOLTAGE));CHECK(!BL_HasEnergySensorReading(OBK_CURRENT));
    CHECK(!BL_HasEnergySensorReading(OBK_POWER_APPARENT));CHECK(BL_HasEnergySensorReading(OBK_POWER));
    window(1000,100);CHECK(isfinite(mock_inverse?observed_i:observed_v));
    CHECK(isnan(mock_inverse?observed_v:observed_i));
    for(int k=0;k<8;k++)window(1000,100);
    CHECK(isfinite(observed_v)&&isfinite(observed_i));CHECK(invalid_mqtt==0);
}
static void test_movingavg(void) {
    movingavg_cnt=4;
    test_partial_updates();CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));
    double old=total_wh;float previous=observed_p;
    window(1000,200);
    #ifdef ENABLE_BL_MOVINGAVG
    float expected=((4-1)*previous+300)/4;
#else
    (void)previous;float expected=300;
#endif
    CHECK(near(observed_p,expected));CHECK(fabs(total_wh-old-expected/3600.0)<1e-6);
}
static void test_relay_policy(void) {
    g_cfg.pins.roles[0]=IOR_Relay;g_cfg.pins.channels[0]=0;
    cfg_flags=1U<<OBK_FLAG_POWER_FORCE_ZERO_IF_RELAYS_OPEN;relay_on=0;
    for(int k=0;k<6;k++) { window(1000,100); } CHECK(observed_p==0);CHECK(observed_i==0);CHECK(total_wh==0);
    relay_on=1;window(1000,100);CHECK(observed_p==150);CHECK(fabs(total_wh-150.0/3600)<1e-6);
}
static void test_bad_intervals(void) {
    int n=reports;
    const float invalid[]={0,-1,NAN,INFINITY};
    for(unsigned k=0;k<4;k++)BL_ProcessUpdateWithInterval(230,1,230,NAN,invalid[k]);
    BL_ProcessUpdateWithInterval(230,1,NAN,NAN,1);CHECK(reports==n);CHECK(total_wh==0);
}
static void test_twin(void) {
#if ENABLE_BL_TWIN
    BL_ProcessUpdateEx(1,230,1,230,NAN,0.25f);CHECK(datasetlist[1].sensors[OBK_CONSUMPTION_TOTAL].lastReading==0.25);
    CHECK(total_wh==0);window(1000,100);CHECK(fabs(total_wh-150.0/3600)<1e-6);
    CHECK(datasetlist[1].sensors[OBK_CONSUMPTION_TOTAL].lastReading==0.25);
    BL_ProcessUpdateEx(-1,230,1,230,NAN,0.25f);BL_ProcessUpdateEx(2,230,1,230,NAN,0.25f);
    CHECK(datasetlist[1].sensors[OBK_CONSUMPTION_TOTAL].lastReading==0.25);
#endif
}
static void test_rate_precision(void) {
    warm();
    const unsigned ds[]={500,500,1500,500,2000,500,500,1000};
    for(int k=0;k<80;k++) {
        window(ds[k%8],1000);
        CHECK(near(observed_p,1500) || (k==0&&near(observed_p,150)));
    }
    window(1000,1000);
    CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));
    CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));
    CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<1e-4);
}
static void test_random_timing(void) {
    warm();uint32_t rng=1234567;
    for(int k=0;k<2000;k++) {
        rng=rng*1664525U+1013904223U;
        unsigned ms=(rng%4==0)?20U:500U+(rng%401)*10U;
        window(ms,100);
        CHECK(near(observed_v,1000*DEFAULT_VOLTAGE_CAL));CHECK(near(observed_i,100*DEFAULT_CURRENT_CAL));CHECK(near(observed_p,150));
    }
    window(1000,100);CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<0.001);
}
static void test_random_load(void) {
    uint32_t rng=875;
    for(int k=0;k<2000;k++) {
        rng=rng*1664525U+1013904223U;
        unsigned ms=(rng%4==0)?20U:500U+(rng%401)*10U;
        unsigned hz=(rng>>8)%1500;
        window(ms,hz);
        CHECK(isfinite(observed_p));CHECK(observed_p<=2251);
    }
    window(1000,100);CHECK(fabs(total_wh-emitted_power*1.5/3600.0)<0.001);
}
#endif

static void test_preemption(void) {
    warm();signal(1000,100);injection_ms=1500;BL0937_RunEverySecond();
    float first=observed_p;CHECK(near(observed_p,150));
    window(1000,100);float second=observed_p;CHECK(near(observed_p,150));
    printf("{\"first_power\":%.9g,\"next_power\":%.9g}\n",first,second);
}
static void test_startup_energy(int late) {
    for(int k=0;k<4;k++) window(1000,(late?(k==3):(k<3))?600U:0U);
    double expected=late?0.25:0.75;CHECK(fabs(total_wh-expected)<1e-6);
    printf("{\"energy_wh\":%.9g,\"expected_wh\":%.9g,\"reports\":%d}\n",total_wh,expected,reports);
}
static void test_zero(void) { for(int k=0;k<8;k++) { advance(1000,0,0);BL0937_RunEverySecond(); }CHECK(observed_v==0&&observed_i==0&&observed_p==0); }
static void test_powermax(void) { warm();window(1000,3000);CHECK(near(observed_p,150));window(1000,0);CHECK(observed_p==0); }
static void test_legacy_calibration(void) {
    /* Hash bit-level float outputs over both scaling modes and large/signed
     * integer inputs. Runner compares base and head for identical hashes.
     */
    uint64_t hash=UINT64_C(14695981039346656037);uint32_t rng=24681357;
    for(int mode=0;mode<2;mode++) {
        PwrCal_Init((pwr_cal_type_t)mode,0.13253012048f,0.0118577075f,1.5f);
        for(int k=0;k<10000;k++) {
            rng=rng*1664525U+1013904223U; int v=(int)(rng&INT_MAX);if(rng&1)v=-v;
            rng=rng*1664525U+1013904223U; int p=(int)(rng&INT_MAX);if(rng&1)p=-p;
            float i=(float)(rng%100000U)/37.0f;float out[4];
            PwrCal_Scale(v,i,p,&out[0],&out[1],&out[2]);out[3]=PwrCal_ScalePowerOnly(p);
            const unsigned char*b=(const unsigned char*)out;
            for(unsigned n=0;n<sizeof(out);n++) {hash^=b[n];hash*=UINT64_C(1099511628211);}
        }
    }
    /* Legacy power-only call must not overwrite PowerSet's raw source. */
    memset(cfg_set,0,sizeof(cfg_set));PwrCal_Init(PWR_CAL_MULTIPLY,1,1,1);
    float v,i,p;PwrCal_Scale(100,10,100,&v,&i,&p);(void)PwrCal_ScalePowerOnly(900);
    CHECK(commands[2](NULL,"PowerSet","150",0)==CMD_RES_OK);CHECK(near(PwrCal_ScalePowerOnly(100),150));
    printf("{\"hash\":\"%016llx\",\"vectors\":20000}\n",(unsigned long long)hash);
}
int main(int argc,char**argv) {
    if(argc<2) { return 2; }
    scenario=argv[1];mock_inverse=argc>2?atoi(argv[2]):0;
    if(!strcmp(scenario,"wrap")) clock_ticks=(portTickType)((portTickType)-1-(250U*TICK_HZ/1000U));
#if TEST_FIXED
    if(!strcmp(scenario,"counter_wrap")){g_p_pulses=UINT32_MAX-10U;g_vc_pulses=UINT32_MAX-10U;}
#endif
    BL0937_Init();
    if(!strcmp(scenario,"nominal")||!strcmp(scenario,"wrap"))test_nominal();
    else if(!strcmp(scenario,"preemption"))test_preemption();
    else if(!strcmp(scenario,"startup_late"))test_startup_energy(1);
    else if(!strcmp(scenario,"startup_early"))test_startup_energy(0);
    else if(!strcmp(scenario,"zero"))test_zero();
    else if(!strcmp(scenario,"powermax"))test_powermax();
    else if(!strcmp(scenario,"legacy_calibration"))test_legacy_calibration();
#if TEST_FIXED
    else if(!strcmp(scenario,"settling"))test_settling();
    else if(!strcmp(scenario,"threshold"))test_threshold();
    else if(!strcmp(scenario,"delays"))test_delays();
    else if(!strcmp(scenario,"calibration_isolation"))test_calibration_isolation();
    else if(!strcmp(scenario,"restart"))test_restart(0);
    else if(!strcmp(scenario,"inverse_restart"))test_restart(1);
    else if(!strcmp(scenario,"restart_pending"))test_restart_pending();
    else if(!strcmp(scenario,"retry_exhaustion"))test_retry_exhaustion();
    else if(!strcmp(scenario,"second_read_preemption"))test_second_read_preemption();
    else if(!strcmp(scenario,"delayed_sel_before"))test_delayed_write(0);
    else if(!strcmp(scenario,"delayed_sel_after"))test_delayed_write(1);
    else if(!strcmp(scenario,"publish_delay"))test_publish_delay();
    else if(!strcmp(scenario,"partial"))test_partial_updates();
    else if(!strcmp(scenario,"movingavg"))test_movingavg();
    else if(!strcmp(scenario,"relay_policy"))test_relay_policy();
    else if(!strcmp(scenario,"bad_intervals"))test_bad_intervals();
    else if(!strcmp(scenario,"twin"))test_twin();
    else if(!strcmp(scenario,"rate_precision"))test_rate_precision();
    else if(!strcmp(scenario,"random_timing"))test_random_timing();
    else if(!strcmp(scenario,"random_load"))test_random_load();
    else if(!strcmp(scenario,"counter_wrap"))test_nominal();
#endif
    else return 2;
    printf("{\"scenario\":\"%s\",\"failures\":%d,\"tick_ms\":%d,\"tick_bits\":%d,\"beken\":%d,\"inverted\":%d}\n",scenario,failures,TICK_MS,TICK_BITS,PLATFORM_BEKEN,mock_inverse);
    return failures?1:0;
}

/* External I/O and services only. No power/energy formulas are mocked. */
void poststr(http_request_t*r,const char*s) {(void)r;(void)s;}
void hprintf255(http_request_t*r,const char*f,...) {(void)r;(void)f;}
int DRV_IsRunning(const char*s) {(void)s;return 1;}
int TIME_IsTimeSynced(void) {return 0;}
time_t TIME_GetCurrentTime(void) {return 0;}
int TIME_GetMDay(void) {return 1;}
int TIME_GetTimesZoneOfsSeconds(void) {return 0;}
int OTA_GetProgress(void) {return 0;}
int CHANNEL_Get(int ch) {(void)ch;return relay_on;}
int CFG_HasFlag(int flag) {
    if(shared_pause_ms) {unsigned ms=shared_pause_ms;shared_pause_ms=0;signal(ms,100);}
    return (cfg_flags>>flag)&1U;
}
void HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA*d) {memset(d,0,sizeof(*d));d->actual_mday=-1;}
void HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA*d) {(void)d;}
void HAL_FlashVars_SaveTotalConsumption(float value) {CHECK(isfinite(value));++reports;}
int Tokenizer_GetArgsCount(void) {return token&&*token?1:0;}
int Tokenizer_GetArgInteger(int i) {(void)i;return token?atoi(token):0;}
float Tokenizer_GetArgFloatDefault(int i,float fallback) {return i<Tokenizer_GetArgsCount()?Tokenizer_GetArgFloat(i):fallback;}
const char* Tokenizer_GetArg(int i) {(void)i;return token;}
void EventHandlers_ProcessVariableChange_Integer(enum EventCode e,int prev,int now) {(void)e;(void)prev;(void)now;}
int MQTT_IsReady(void) {return 1;}
void MQTT_PublishMain_StringFloat(const char*n,float v,int digits,int flags) {(void)n;(void)digits;(void)flags;++mqtt_calls;if(!isfinite(v))++invalid_mqtt;}
void MQTT_PublishMain_StringString(const char*n,const char*v,int flags) {(void)n;(void)v;(void)flags;++mqtt_calls;}
cJSON *cJSON_CreateObject(void) {return calloc(1,sizeof(cJSON));}
cJSON *cJSON_CreateArray(void) {return cJSON_CreateObject();}
cJSON *cJSON_CreateNumber(double n) {(void)n;return cJSON_CreateObject();}
void cJSON_AddNumberToObject(cJSON*j,const char*k,double n) {(void)j;(void)k;(void)n;}
void cJSON_AddStringToObject(cJSON*j,const char*k,const char*n) {(void)j;(void)k;(void)n;}
void cJSON_AddItemToArray(cJSON*j,cJSON*a) {(void)j;free(a);}
void cJSON_AddItemToObject(cJSON*j,const char*k,cJSON*a) {(void)j;(void)k;free(a);}
char *cJSON_PrintUnformatted(cJSON*j) {(void)j;char*s=malloc(3);strcpy(s,"{}");return s;}
void cJSON_Delete(cJSON*j) {free(j);}
float XJ_MovingAverage_float(float prev,float now) {
    if(prev<=0 || movingavg_cnt<=1)return now;
    return (((movingavg_cnt-1)*prev+now)/movingavg_cnt);
}
