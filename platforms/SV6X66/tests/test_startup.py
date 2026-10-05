#!/usr/bin/env python3
"""Exercise APP_Init's task-create success/failure behavior on the host."""
from pathlib import Path
import argparse
import subprocess
import tempfile

PLATFORM = Path(__file__).resolve().parents[1]
APP = PLATFORM.parents[1]
HEADERS = ("new_common.h", "ota_stage.h", "cJSON/cJSON.h", "sys/backtrace.h", "sys/xip.h", "sys/sys_clock.h", "fsal.h", "stock_mount.h", "netstack.h", "wifi_api.h", "idmanage/pbuf.h", "security/drv_security.h", "uart/drv_uart.h", "rf/rf_api.h")
ENV = r'''#include <stddef.h>
#include <stdint.h>
typedef void *SSV_FS; typedef void (*OsTask)(void *); typedef void *OsTaskHandle;
typedef struct { void *malloc_fn; void *free_fn; } cJSON_Hooks;
struct st_rf_table { unsigned char boot_flag; };
#define OS_SUCCESS 0
#define OS_TASK_PRIO1 1
#define UART_INT_RXFIFO_TRGLVL_1 1
#define UART_WORD_LEN_8 8
#define UART_STOP_BIT_1 1
#define UART_PARITY_DISABLE 0
#define DUT_STA 0
#define WIFI_INIT() ((void)0)
#define configASSERT(x) ((void)(x))
int startup_printf(const char *, ...);
#define printf startup_printf
void xip_init(void); void xip_enter(void); void drv_uart_init(void);
void drv_uart_set_fifo(int,int); void drv_uart_set_format(int,int,int,int);
unsigned long sys_bus_clock(void); void OS_Init(void); void OS_StatInit(void);
void OS_MemInit(void); void OS_PsramInit(void); void *OS_MemAlloc(size_t);
void OS_MemFree(void *); void cJSON_InitHooks(cJSON_Hooks *);
int build_default_rf_table(struct st_rf_table *); int load_rf_table_to_mac(struct st_rf_table *);
void load_rf_table_from_flash(void); int ota_stage_layout_valid(const uint8_t *);
SSV_FS FS_init(void); int SV6X66_StockHeaderValid(const uint8_t *); SSV_FS SV6X66_StockMount(void);
void SV6X66_StorageInit(void); int SV6X66_OTAStartup(void); int SV6X66_NetInit(void);
void netstack_init(void *); int DUT_wifi_start(int); void Main_Init(void);
void Main_OnEverySecond(void); void OS_MsDelay(unsigned);
unsigned char OS_TaskCreate(OsTask,const char *,unsigned short,void *,unsigned,OsTaskHandle *);
void OS_StartScheduler(void); void print_callstack(void);
'''
HARNESS = r'''#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
void APP_Init(void);
struct st_rf_table ssv_rf_table;
static jmp_buf exit_point; static int task_result, observed, scheduler_calls;
int startup_printf(const char *fmt, ...) { if (strstr(fmt,"task creation failed")){observed=2;longjmp(exit_point,1);} return 0; }
#define VOID_FN(name) void name(void) {}
VOID_FN(xip_init) VOID_FN(xip_enter) VOID_FN(drv_uart_init) VOID_FN(OS_Init)
VOID_FN(OS_StatInit) VOID_FN(OS_MemInit) VOID_FN(OS_PsramInit)
VOID_FN(load_rf_table_from_flash) VOID_FN(SV6X66_StorageInit)
void netstack_init(void *p){(void)p;}
VOID_FN(Main_Init) VOID_FN(Main_OnEverySecond) VOID_FN(print_callstack)
void drv_uart_set_fifo(int a,int b){(void)a;(void)b;}
void drv_uart_set_format(int a,int b,int c,int d){(void)a;(void)b;(void)c;(void)d;}
unsigned long sys_bus_clock(void){return 1;}
void *OS_MemAlloc(size_t n){return malloc(n);} void OS_MemFree(void *p){free(p);}
void cJSON_InitHooks(cJSON_Hooks *p){(void)p;}
int build_default_rf_table(struct st_rf_table *p){(void)p;return 0;}
int load_rf_table_to_mac(struct st_rf_table *p){(void)p;return 0;}
int ota_stage_layout_valid(const uint8_t *p){(void)p;return 0;}
void *FS_init(void){return NULL;}
int SV6X66_StockHeaderValid(const uint8_t *p){(void)p;return 0;}
void *SV6X66_StockMount(void){return NULL;}
int SV6X66_OTAStartup(void){return 0;} int SV6X66_NetInit(void){return 1;}
int DUT_wifi_start(int mode){(void)mode;return 0;} void OS_MsDelay(unsigned ms){(void)ms;}
unsigned char OS_TaskCreate(void (*fn)(void *),const char *name,unsigned short stack,void *arg,unsigned pri,void **handle){
 (void)fn;(void)name;(void)stack;(void)arg;(void)pri;(void)handle; return (unsigned char)task_result;
}
void OS_StartScheduler(void){++scheduler_calls;observed=1;longjmp(exit_point,1);}
int main(int argc,char **argv){if(argc!=2)return 10;task_result=atoi(argv[1]);if(setjmp(exit_point)==0)APP_Init();
 if(task_result==1&&(observed!=1||scheduler_calls!=1))return 11;
 if(task_result==0&&(observed!=2||scheduler_calls!=0))return 12;return 0;}
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-source', type=Path, default=PLATFORM / 'main.c')
    parser.add_argument('--stock', action='store_true', help='exercise the stock-firmware startup branch')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="sv6166f-startup-") as tmp:
        temp=Path(tmp); stubs=temp/"stubs"
        for header in HEADERS:
            path=stubs/header; path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text("/* declarations supplied by test_startup_env.h */\n")
        env=stubs/"test_startup_env.h"; env.write_text(ENV)
        harness=temp/"startup_harness.c"; harness.write_text(HARNESS)
        executable=temp/"test_startup"
        subprocess.run(["gcc","-std=gnu11","-w"] + (["-DSV6X66_STOCK_CKW04=1"] if args.stock else []) + ["-include",str(env),"-I"+str(stubs),str(args.main_source.resolve()),str(harness),"-o",str(executable)],cwd=APP,check=True)
        for result in ("1","0"):
            subprocess.run([str(executable),result],cwd=APP,check=True,timeout=10)
    print("APP_Init startup task result checks passed (1=success, 0=failure)")

if __name__ == "__main__": main()
