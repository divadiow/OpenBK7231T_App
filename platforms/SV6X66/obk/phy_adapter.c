#include "sv6x66_platform.h"
#include "soc_types.h"
#include "soc_defs.h"
#include "phy/drv_phy.h"
#include "rf/rf_api.h"
#include "wificonf.h"
#include "ssvradio.h"
#include "turismo/ssv6006C_reg.h"
#include "turismo/ssv6006C_aux.h"
#include "phy/turismo_common.h"

/* Experimental CKW04 bring-up: the factory boot sector contains executable
	* code at the SDK's fixed XTAL/bus-header addresses. Keep that boot sector
	* intact and supply the physically confirmed 26 MHz crystal explicitly.
	* The unmodified SDK macros retain RF patching and fast calibration logic. */
extern IEEE80211STATUS gwifistatus;
extern struct st_rf_table *p_rf_table;
extern struct ssv6006_padpd gdpd;
extern struct ssv6006_cal_result cal_result;
extern OsMutex g_phy_mutex;
extern void drv_phy_set_bbscale(struct ssv6006_padpd *dpd);

/* These complete declarations refer to the existing SDK archive tables, not
	* copies. Bounds and shared-state sizes are pinned to this SDK's symbol ABI;
	* inspect them again in each built ELF before hardware use. */
extern ssv_cabrio_reg ssv6006_turismoC_rf_setting[14];
extern ssv_cabrio_reg ssv6006_turismoC_phy_setting[75];
typedef char rf_entry_size_must_be_8[(sizeof(ssv_cabrio_reg)==8)?1:-1];
typedef char rf_table_size_must_be_116[(sizeof(struct st_rf_table)==116)?1:-1];
typedef char dpd_size_must_be_540[(sizeof(struct ssv6006_padpd)==540)?1:-1];
typedef char calibration_size_must_be_204[(sizeof(struct ssv6006_cal_result)==204)?1:-1];
typedef char phy_mutex_size_must_be_4[(sizeof(OsMutex)==4)?1:-1];
typedef char dcdc_offset_must_be_65c[(offsetof(IEEE80211STATUS,dcdc_disable)==0x65c)?1:-1];

/* Version labels are metadata from the unmodified SDK table headers. Those
	* headers define strong arrays, so including them here would duplicate data.
	* Keep all SDK calibration diagnostics, including timeout/failure messages. */
#define SSV6006_TURISMOC_RF_TABLE_VER "19.00"
#define SSV6006_TURISMOC_PHY_TABLE_VER "19.00"
#undef PRINT
#undef PRINT_ERR
#undef PRINT_INFO
#define PRINT SV6X66_Printf
#define PRINT_ERR SV6X66_Printf
#define PRINT_INFO SV6X66_Printf

void __wrap_drv_phy_cali(void)
{
	struct ssv6006_patch patch={0};
	struct ssv6006_cal_result *cal=&cal_result;
	int band;
	OS_STATUS mutex_result;

	drv_phy_set_bbscale(&gdpd);
	patch.dcdc=gwifistatus.dcdc_disable!=1;
	patch.xtal=XTAL26M;
	patch.cpu_clk=CLK_80M;
	if(p_rf_table) {TIR_FILL_PATCH(p_rf_table,patch);}
	band=is_5g_band_support()?AG_BAND_BOTH:G_BAND_ONLY;
	SV6X66_Printf("CKW04 PHY: crystal=26 MHz bus=80 MHz nominal band=%d\r\n",band);
	/* The SDK macro uses cal-> without parentheses: pass a pointer variable.
		* This is its fast-calibration/restore path, not the broken IQK macro. */
	_INIT_TURISMOC_SYS(patch,band,cal);
	if(SV6X66_LogUartSetBaud())for(;;){}
	if(cal->cal_done) {
		_RESTORE_CAL(band,cal);
	} else {
		TU_INIT_TURISMOC_FAST_CALI(cal,band);
	}
	if(!g_phy_mutex) {
		mutex_result=OS_MutexInit(&g_phy_mutex);
		if(mutex_result!=OS_SUCCESS)SV6X66_Printf("PHY mutex initialization failed: rc=%d\r\n",(int)mutex_result);
	}
	SV6X66_Printf("CKW04 PHY: clock selector=%u (expected %u)\r\n",
		(unsigned int)GET_CLK_DIGI_SEL,(unsigned int)CLK_80M);
}
