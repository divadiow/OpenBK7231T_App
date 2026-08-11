#if PLATFORM_TXW81X

#include "../../obk_config.h"
#include "../../new_common.h"
#include "../../new_cfg.h"
#include "../../logging/logging.h"
#include "../../httpserver/new_http.h"
#include "../hal_ota.h"
#include "hal/spi_nor.h"
#include "lib/ota/fw.h"
#include "lib/ota/fwinfo.h"
#include "sys_config.h"
#include "dev.h"
#include "devid.h"
#include "hal/dvp.h"
#include "ef_cfg.h"

/*
 * The TXW81X SDK OTA API takes a uint16 length. Keep every write small even
 * when the HTTP server has already buffered a large part of the request body.
 * The vendor TCP updater also feeds libota_write_fw() in 1460-byte pieces.
 */
#define TXW81X_OTA_IO_CHUNK       1460
#define TXW81X_OTA_PROGRESS_STEP  (64 * 1024)
#define TXW81X_OTA_TRACE_BYTES      8
#define TXW81X_OTA_FLASH_STACK     "FPV-v2.5.3.7-40416-v5-marker-stable"

extern struct spi_nor_flash flash0;
extern uint32 get_flash_cap(void);
extern void mcu_watchdog_feed(void);

/*
 * libotaV2_write_fw() is supplied only inside Taixin's precompiled
 * libcommon.a. Its object also contains a weak ota_fwinfo_get() fallback that
 * returns -1. On this build that can win an internal reference even though
 * project/device.c has the intended callback.
 *
 * platforms/TXW81X/OpenBeken.mk uses --defsym to bind ota_fwinfo_get directly
 * to this named implementation. This removes ambiguity inside the archive and
 * also gives us enough state to report where the opaque SDK fails next.
 */
typedef struct txw81x_ota_sdk_diag_s
{
	int customer_bypass_seen;
	int malloc_seen;
	int malloc_size;
	uint32 malloc_ptr;
	int fwinfo_seen;
	int fwinfo_result;
	uint32 flash_size;
	uint32 sector_size;
	uint32 block_size;
	uint32 ota_addr0;
	uint32 ota_addr1;
	uint32 flash_bus;
	int loader_mark_seen;
	uint32 loader_mark;
	int spi_open_count;
	int spi_open_result;
	int spi_read_count;
	uint32 spi_read_addr;
	uint32 spi_read_len;
	int spi_close_count;
	int sector_erase_count;
	uint32 sector_erase_addr;
	int block_erase_count;
	uint32 block_erase_addr;
	int spi_write_count;
	uint32 spi_write_addr;
	uint32 spi_write_len;
	int current_loader_seen;
	uint32 current_loader_addr;
	uint32 source_data_ptr;
	uint32 sdk_data_ptr;
	uint8 sdk_before[TXW81X_OTA_TRACE_BYTES];
	uint8 customer_before[TXW81X_OTA_TRACE_BYTES];
	uint8 customer_after[TXW81X_OTA_TRACE_BYTES];
	uint8 erase_before[TXW81X_OTA_TRACE_BYTES];
	uint8 erase_after[TXW81X_OTA_TRACE_BYTES];
	uint8 sdk_after[TXW81X_OTA_TRACE_BYTES];
} txw81x_ota_sdk_diag_t;

static volatile int g_txw81x_ota_trace_active;
static volatile int g_txw81x_ota_first_call_active;
static volatile uint8 *g_txw81x_ota_trace_data;
static volatile txw81x_ota_sdk_diag_t g_txw81x_ota_diag;
static uint8 g_txw81x_ota_chunk[TXW81X_OTA_IO_CHUNK] __attribute__((aligned(4)));
static uint8 g_txw81x_ota_saved_marker[2];

static void TXW81X_OTA_Snapshot(volatile uint8 *dst, const uint8 *src)
{
	int i;
	if(dst == NULL)
	{
		return;
	}
	for(i = 0; i < TXW81X_OTA_TRACE_BYTES; i++)
	{
		dst[i] = src != NULL ? src[i] : 0;
	}
}

static void TXW81X_OTA_RestoreFirstMarker(void)
{
	if(g_txw81x_ota_trace_active && g_txw81x_ota_first_call_active &&
		g_txw81x_ota_trace_data != NULL)
	{
		g_txw81x_ota_trace_data[0] = g_txw81x_ota_saved_marker[0];
		g_txw81x_ota_trace_data[1] = g_txw81x_ota_saved_marker[1];
	}
}

static void TXW81X_OTA_ResetSdkDiag(void)
{
	memset((void*)&g_txw81x_ota_diag, 0, sizeof(g_txw81x_ota_diag));
}

static void TXW81X_OTA_LogSdkDiag(void)
{
	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: customer-bypass=%d malloc=%d size=%d ptr=0x%08x",
		g_txw81x_ota_diag.customer_bypass_seen,
		g_txw81x_ota_diag.malloc_seen,
		g_txw81x_ota_diag.malloc_size,
		g_txw81x_ota_diag.malloc_ptr);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: fwinfo=%d result=%d flash=0x%08x sector=0x%08x block=0x%08x",
		g_txw81x_ota_diag.fwinfo_seen,
		g_txw81x_ota_diag.fwinfo_result,
		g_txw81x_ota_diag.flash_size,
		g_txw81x_ota_diag.sector_size,
		g_txw81x_ota_diag.block_size);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: addr0=0x%08x addr1=0x%08x bus=0x%08x loader-mark=%d/0x%08x",
		g_txw81x_ota_diag.ota_addr0,
		g_txw81x_ota_diag.ota_addr1,
		g_txw81x_ota_diag.flash_bus,
		g_txw81x_ota_diag.loader_mark_seen,
		g_txw81x_ota_diag.loader_mark);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: spi-open=%d last-result=%d spi-read=%d addr=0x%08x len=%u close=%d",
		g_txw81x_ota_diag.spi_open_count,
		g_txw81x_ota_diag.spi_open_result,
		g_txw81x_ota_diag.spi_read_count,
		g_txw81x_ota_diag.spi_read_addr,
		g_txw81x_ota_diag.spi_read_len,
		g_txw81x_ota_diag.spi_close_count);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: sector-erase=%d/0x%08x block-erase=%d/0x%08x write=%d addr=0x%08x len=%u",
		g_txw81x_ota_diag.sector_erase_count,
		g_txw81x_ota_diag.sector_erase_addr,
		g_txw81x_ota_diag.block_erase_count,
		g_txw81x_ota_diag.block_erase_addr,
		g_txw81x_ota_diag.spi_write_count,
		g_txw81x_ota_diag.spi_write_addr,
		g_txw81x_ota_diag.spi_write_len);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK trace: current-loader=%d/0x%08x free-heap=%u",
		g_txw81x_ota_diag.current_loader_seen,
		g_txw81x_ota_diag.current_loader_addr,
		(unsigned int)xPortGetFreeHeapSize());

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK bytes: source=0x%08x sdk=0x%08x before=%02x %02x %02x %02x customer=%02x %02x/%02x %02x",
		g_txw81x_ota_diag.source_data_ptr,
		g_txw81x_ota_diag.sdk_data_ptr,
		g_txw81x_ota_diag.sdk_before[0], g_txw81x_ota_diag.sdk_before[1],
		g_txw81x_ota_diag.sdk_before[2], g_txw81x_ota_diag.sdk_before[3],
		g_txw81x_ota_diag.customer_before[0], g_txw81x_ota_diag.customer_before[1],
		g_txw81x_ota_diag.customer_after[0], g_txw81x_ota_diag.customer_after[1]);

	ADDLOG_ERROR(LOG_FEATURE_OTA,
		"TXW81X SDK bytes: erase=%02x %02x/%02x %02x after=%02x %02x %02x %02x",
		g_txw81x_ota_diag.erase_before[0], g_txw81x_ota_diag.erase_before[1],
		g_txw81x_ota_diag.erase_after[0], g_txw81x_ota_diag.erase_after[1],
		g_txw81x_ota_diag.sdk_after[0], g_txw81x_ota_diag.sdk_after[1],
		g_txw81x_ota_diag.sdk_after[2], g_txw81x_ota_diag.sdk_after[3]);
}

int32 txw81x_ota_fwinfo_get(struct ota_fwinfo *pinfo)
{
	uint32 detected_size;
	int32 ret = 0;

	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.fwinfo_seen = 1;
	}

	if(pinfo == NULL)
	{
		ret = -1;
		goto done;
	}

	/* Refresh the capacity so a 1 MiB part cannot retain the 2 MiB fallback. */
	detected_size = get_flash_cap();
	if(detected_size != 0)
	{
		flash0.size = detected_size;
	}

	if(flash0.size == 0 || flash0.sector_size == 0 || flash0.bus == NULL)
	{
		ret = -1;
		goto done;
	}

	pinfo->flash0 = &flash0;
	pinfo->flash1 = &flash0;
	pinfo->addr0 = 0;
	pinfo->addr1 = flash0.size / 2;

	if((pinfo->addr1 & (flash0.sector_size - 1)) != 0)
	{
		ret = -1;
	}

done:
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.fwinfo_result = ret;
		g_txw81x_ota_diag.flash_size = flash0.size;
		g_txw81x_ota_diag.sector_size = flash0.sector_size;
		g_txw81x_ota_diag.block_size = flash0.block_size;
		g_txw81x_ota_diag.ota_addr0 = 0;
		g_txw81x_ota_diag.ota_addr1 = flash0.size / 2;
		g_txw81x_ota_diag.flash_bus = (uint32)(uintptr_t)flash0.bus;
	}
	return ret;
}

/*
 * The following wrappers do not alter SDK results. They only record which
 * opaque libcommon.a stage was reached. Logging is deferred until after the
 * SDK call returns, so the wrappers cannot recursively allocate or log while
 * the flash driver is active.
 */
extern void *__real__os_malloc(int size);
void *__wrap__os_malloc(int size)
{
	void *ptr = __real__os_malloc(size);
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.malloc_seen = 1;
		g_txw81x_ota_diag.malloc_size = size;
		g_txw81x_ota_diag.malloc_ptr = (uint32)(uintptr_t)ptr;
	}
	return ptr;
}

extern uint32 __real_get_loader_mark(void);
uint32 __wrap_get_loader_mark(void)
{
	uint32 value = __real_get_loader_mark();
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.loader_mark_seen = 1;
		g_txw81x_ota_diag.loader_mark = value;
	}
	return value;
}

extern int32 __real_spi_nor_open(struct spi_nor_flash *flash);
int32 __wrap_spi_nor_open(struct spi_nor_flash *flash)
{
	int32 result;
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.spi_open_count++;
	}
	result = __real_spi_nor_open(flash);
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.spi_open_result = result;
	}
	return result;
}

extern void __real_spi_nor_read(struct spi_nor_flash *flash,
	uint32 addr, uint8 *buf, uint32 len);
void __wrap_spi_nor_read(struct spi_nor_flash *flash,
	uint32 addr, uint8 *buf, uint32 len)
{
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.spi_read_count++;
		g_txw81x_ota_diag.spi_read_addr = addr;
		g_txw81x_ota_diag.spi_read_len = len;
	}
	__real_spi_nor_read(flash, addr, buf, len);
}

extern void __real_spi_nor_close(struct spi_nor_flash *flash);
void __wrap_spi_nor_close(struct spi_nor_flash *flash)
{
	__real_spi_nor_close(flash);
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.spi_close_count++;
	}
}

extern void __real_spi_nor_sector_erase(struct spi_nor_flash *flash, uint32 addr);
void __wrap_spi_nor_sector_erase(struct spi_nor_flash *flash, uint32 addr)
{
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.sector_erase_count++;
		g_txw81x_ota_diag.sector_erase_addr = addr;
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.erase_before,
			(const uint8*)g_txw81x_ota_trace_data);
	}
	mcu_watchdog_feed();
	__real_spi_nor_sector_erase(flash, addr);
	mcu_watchdog_feed();
	TXW81X_OTA_RestoreFirstMarker();
	if(g_txw81x_ota_trace_active)
	{
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.erase_after,
			(const uint8*)g_txw81x_ota_trace_data);
	}
}

extern void __real_spi_nor_block_erase(struct spi_nor_flash *flash, uint32 addr);
void __wrap_spi_nor_block_erase(struct spi_nor_flash *flash, uint32 addr)
{
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.block_erase_count++;
		g_txw81x_ota_diag.block_erase_addr = addr;
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.erase_before,
			(const uint8*)g_txw81x_ota_trace_data);
	}
	mcu_watchdog_feed();
	__real_spi_nor_block_erase(flash, addr);
	mcu_watchdog_feed();
	TXW81X_OTA_RestoreFirstMarker();
	if(g_txw81x_ota_trace_active)
	{
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.erase_after,
			(const uint8*)g_txw81x_ota_trace_data);
	}
}

extern void __real_spi_nor_write(struct spi_nor_flash *flash,
	uint32 addr, uint8 *data, uint32 len);
void __wrap_spi_nor_write(struct spi_nor_flash *flash,
	uint32 addr, uint8 *data, uint32 len)
{
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.spi_write_count++;
		g_txw81x_ota_diag.spi_write_addr = addr;
		g_txw81x_ota_diag.spi_write_len = len;
	}
	mcu_watchdog_feed();
	__real_spi_nor_write(flash, addr, data, len);
	mcu_watchdog_feed();
}

extern uint32 __real_get_current_loader_addr(void);
uint32 __wrap_get_current_loader_addr(void)
{
	uint32 value = __real_get_current_loader_addr();
	if(g_txw81x_ota_trace_active)
	{
		g_txw81x_ota_diag.current_loader_seen = 1;
		g_txw81x_ota_diag.current_loader_addr = value;
	}
	return value;
}

/*
 * Taixin's helper emits "Not encrypt firmware" for the SDK's own
 * APP_compress.bin when AesEnable=0. Validate the firmware header, explicitly
 * accept that normal unencrypted format, and preserve the original customer-ID
 * check for genuinely encrypted images.
 */
extern int32 __real_fwinfo_check_customer_id(const uint8 *data);

int32 __wrap_fwinfo_check_customer_id(const uint8 *data)
{
	int32 err = 0;
	uint16 aes_enabled;

	if(data == NULL)
	{
		ADDLOG_ERROR(LOG_FEATURE_OTA,
			"TXW81X OTA: customer-ID check received a null header");
		return -1;
	}

	if(g_txw81x_ota_trace_active)
	{
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.customer_before, data);
	}
	aes_enabled = fwinfo_get_fw_aes_en(data, &err);
	if(g_txw81x_ota_trace_active)
	{
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.customer_after, data);
	}
	if(err != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_OTA,
			"TXW81X OTA: invalid firmware header, fwinfo error %d", err);
		return err;
	}

	if(aes_enabled == 0)
	{
		if(g_txw81x_ota_trace_active)
		{
			g_txw81x_ota_diag.customer_bypass_seen = 1;
		}
		/* Do not log while the SDK trace wrappers are active: logging may
		 * allocate memory and would contaminate the first-call diagnostics. */
		return 0;
	}

	return __real_fwinfo_check_customer_id(data);
}

static void TXW81X_OTA_QuiesceCamera(void)
{
#if DVP_EN
	struct dvp_device *dvp = (struct dvp_device*)dev_get(HG_DVP_DEVID);
	if(dvp != NULL)
	{
		ADDLOG_INFO(LOG_FEATURE_OTA,
			"TXW81X OTA: stopping DVP before flash erase/write");
		dvp_close(dvp);
		rtos_delay_milliseconds(5);
	}
#endif
}

static int TXW81X_OTA_GetStagingLimit(uint32 *start, uint32 *limit)
{
	uint32 detected_size = get_flash_cap();
	uint32 reserved_end;

	if(detected_size != 0)
	{
		flash0.size = detected_size;
	}
	if(flash0.size == 0 || flash0.sector_size == 0 || flash0.size <= (2 * flash0.sector_size))
	{
		return -1;
	}

	*start = flash0.size / 2;
	reserved_end = flash0.size - (2 * flash0.sector_size);

	/* OpenBeken EasyFlash begins at 0xEF000 on TXW81X. A 64 KiB block
	 * erase at 0xE0000 would destroy it, so cap OTA staging at the
	 * beginning of EasyFlash and force the SDK into sector-erase mode. */
	if((uint32)EF_START_ADDR < reserved_end)
	{
		reserved_end = (uint32)EF_START_ADDR;
	}

	if(reserved_end <= *start)
	{
		return -1;
	}
	*limit = reserved_end - *start;
	return 0;
}

int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr)
{
	int total = 0;
	int towrite = 0;
	char* writebuf = request->bodystart;
	int buffered = request->bodylen;
	int ret = 0;
	int res = 0;
	int next_progress = TXW81X_OTA_PROGRESS_STEP;
	int camera_quiesced = 0;
	uint32 staging_start = 0;
	uint32 staging_limit = 0;

	(void)maxaddr;

	ADDLOG_INFO(LOG_FEATURE_OTA, "TXW81X OTA post len %d, initially buffered %d",
		request->contentLength, request->bodylen);

	if(request->contentLength <= 0)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA, "Content-length is 0");
		goto update_ota_exit;
	}

	if(TXW81X_OTA_GetStagingLimit(&staging_start, &staging_limit) != 0)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA,
			"TXW81X OTA: invalid flash geometry (size=0x%08x sector=0x%08x bus=0x%08x)",
			flash0.size, flash0.sector_size, (uint32)(uintptr_t)flash0.bus);
		goto update_ota_exit;
	}

	ADDLOG_INFO(LOG_FEATURE_OTA,
		"TXW81X OTA: flash-stack=%s flash=0x%08x staging-start=0x%08x staging-end=0x%08x staging-limit=%u free-heap=%u",
		TXW81X_OTA_FLASH_STACK, flash0.size, staging_start,
		staging_start + staging_limit, staging_limit,
		(unsigned int)xPortGetFreeHeapSize());

	if((uint32)request->contentLength > staging_limit)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA,
			"TXW81X OTA image is too large: %d bytes, maximum %u bytes",
			request->contentLength, staging_limit);
		goto update_ota_exit;
	}

	towrite = request->contentLength;

	if(buffered < 0)
	{
		buffered = 0;
	}
	else if(buffered > towrite)
	{
		buffered = towrite;
	}

	TXW81X_OTA_ResetSdkDiag();
	mcu_watchdog_feed();

	while(towrite > 0)
	{
		int writelen;
		uint16 marker;

		if(buffered <= 0)
		{
			int recvmax = request->receivedLenmax;

			if(recvmax <= 0 || recvmax > TXW81X_OTA_IO_CHUNK)
			{
				recvmax = TXW81X_OTA_IO_CHUNK;
			}
			if(recvmax > towrite)
			{
				recvmax = towrite;
			}

			writebuf = request->received;
			mcu_watchdog_feed();
			buffered = recv(request->fd, writebuf, recvmax, 0);
			mcu_watchdog_feed();
			if(buffered <= 0)
			{
				ADDLOG_ERROR(LOG_FEATURE_OTA,
					"recv returned %d - remaining %d of %d bytes",
					buffered, towrite, request->contentLength);
				ret = -1;
				goto update_ota_exit;
			}
		}

		writelen = buffered;
		if(writelen > TXW81X_OTA_IO_CHUNK)
		{
			writelen = TXW81X_OTA_IO_CHUNK;
		}
		if(writelen > towrite)
		{
			writelen = towrite;
		}

		/* The vendor OTA routine deliberately changes the first two input
		 * bytes to 0xFF while staging the image. Never give it OpenBeken's
		 * live HTTP receive buffer. A private aligned copy also guarantees
		 * that the marker survives flash-driver activity before validation. */
		memcpy(g_txw81x_ota_chunk, writebuf, writelen);

		if(startaddr == 0)
		{
			int marker_at = -1;
			int i;

			if(writelen < 2)
			{
				ADDLOG_ERROR(LOG_FEATURE_OTA,
					"TXW81X OTA: first packet is only %d byte(s)", writelen);
				ret = -1;
				goto update_ota_exit;
			}

			marker = (uint16)g_txw81x_ota_chunk[0] |
				((uint16)g_txw81x_ota_chunk[1] << 8);

			ADDLOG_INFO(LOG_FEATURE_OTA,
				"TXW81X OTA first bytes: src=0x%08x copy=0x%08x %02x %02x %02x %02x %02x %02x %02x %02x",
				(uint32)(uintptr_t)writebuf,
				(uint32)(uintptr_t)g_txw81x_ota_chunk,
				g_txw81x_ota_chunk[0], g_txw81x_ota_chunk[1],
				writelen > 2 ? g_txw81x_ota_chunk[2] : 0,
				writelen > 3 ? g_txw81x_ota_chunk[3] : 0,
				writelen > 4 ? g_txw81x_ota_chunk[4] : 0,
				writelen > 5 ? g_txw81x_ota_chunk[5] : 0,
				writelen > 6 ? g_txw81x_ota_chunk[6] : 0,
				writelen > 7 ? g_txw81x_ota_chunk[7] : 0);

			if(marker != OTA_FLASH_MARKER)
			{
				for(i = 1; i + 1 < writelen && i < 64; i++)
				{
					uint16 candidate = (uint16)g_txw81x_ota_chunk[i] |
						((uint16)g_txw81x_ota_chunk[i + 1] << 8);
					if(candidate == OTA_FLASH_MARKER)
					{
						marker_at = i;
						break;
					}
				}
				ADDLOG_ERROR(LOG_FEATURE_OTA,
					"TXW81X OTA: invalid first marker 0x%04x; expected 0x%04x; marker found at offset %d",
					marker, OTA_FLASH_MARKER, marker_at);
				ADDLOG_ERROR(LOG_FEATURE_OTA,
					"TXW81X OTA: refusing to erase flash; upload the generated OpenTXW81X_*_ota.img unchanged");
				ret = -1;
				goto update_ota_exit;
			}

			g_txw81x_ota_saved_marker[0] = g_txw81x_ota_chunk[0];
			g_txw81x_ota_saved_marker[1] = g_txw81x_ota_chunk[1];

			if(!camera_quiesced)
			{
				TXW81X_OTA_QuiesceCamera();
				camera_quiesced = 1;
			}
		}

		g_txw81x_ota_diag.source_data_ptr = (uint32)(uintptr_t)writebuf;
		g_txw81x_ota_diag.sdk_data_ptr = (uint32)(uintptr_t)g_txw81x_ota_chunk;
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.sdk_before,
			g_txw81x_ota_chunk);
		g_txw81x_ota_trace_data = g_txw81x_ota_chunk;
		g_txw81x_ota_first_call_active = (startaddr == 0);

		mcu_watchdog_feed();
		g_txw81x_ota_trace_active = 1;
		/* Sector mode is mandatory on the 1 MiB OpenBeken layout. The final
		 * 64 KiB OTA block overlaps EasyFlash at 0xEF000 if block erase is
		 * used. is_once_earse remains false so the SDK erases on demand. */
		res = libotaV2_write_fw(request->contentLength,
			0, 1, 0, (uint32)startaddr,
			g_txw81x_ota_chunk, (uint16)writelen);
		TXW81X_OTA_Snapshot(g_txw81x_ota_diag.sdk_after,
			g_txw81x_ota_chunk);
		g_txw81x_ota_trace_active = 0;
		g_txw81x_ota_first_call_active = 0;
		g_txw81x_ota_trace_data = NULL;
		mcu_watchdog_feed();

		if(res != 0)
		{
			ADDLOG_ERROR(LOG_FEATURE_OTA,
				"libotaV2_write_fw failed at offset %d, len %d, res %d",
				startaddr, writelen, res);
			TXW81X_OTA_LogSdkDiag();
			ret = -1;
			goto update_ota_exit;
		}

		if(startaddr == 0)
		{
			if(g_txw81x_ota_diag.customer_bypass_seen)
			{
				ADDLOG_INFO(LOG_FEATURE_OTA,
					"TXW81X OTA: accepted valid unencrypted image");
			}
			ADDLOG_INFO(LOG_FEATURE_OTA,
				"TXW81X OTA first block: sector-erase=%d block-erase=%d write=%d target=0x%08x",
				g_txw81x_ota_diag.sector_erase_count,
				g_txw81x_ota_diag.block_erase_count,
				g_txw81x_ota_diag.spi_write_count,
				g_txw81x_ota_diag.spi_write_addr);
		}

		total += writelen;
		startaddr += writelen;
		towrite -= writelen;
		writebuf += writelen;
		buffered -= writelen;

		if(total >= next_progress || towrite == 0)
		{
			ADDLOG_INFO(LOG_FEATURE_OTA,
				"TXW81X OTA progress: %d/%d bytes",
				total, request->contentLength);
			while(next_progress <= total)
			{
				next_progress += TXW81X_OTA_PROGRESS_STEP;
			}
		}

		mcu_watchdog_feed();
		rtos_delay_milliseconds(10);
	}

update_ota_exit:
	g_txw81x_ota_trace_active = 0;
	g_txw81x_ota_first_call_active = 0;
	g_txw81x_ota_trace_data = NULL;
	if(ret != -1)
	{
		ADDLOG_INFO(LOG_FEATURE_OTA, "OTA is successful: %d bytes written", total);
	}
	else
	{
		ADDLOG_ERROR(LOG_FEATURE_OTA, "OTA failed after %d of %d bytes",
			total, request->contentLength);
		return http_rest_error(request, ret, "error");
	}

	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	CFG_IncrementOTACount();
	return 0;
}

#endif
