#if PLATFORM_XRADIO

#include <image/flash.h>
#include <ota/ota.h>
#include "../../obk_config.h"
#include "../../new_common.h"
#include "../../new_cfg.h"
#include "../../httpserver/new_http.h"
#include "../../logging/logging.h"

#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0x08
#endif

uint32_t flash_read(uint32_t flash, uint32_t addr, void* buf, uint32_t size);
#define FLASH_INDEX_XR809 0

int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr)
{
	int total = 0;
	int towrite = request->bodylen;
	char* writebuf = request->bodystart;
	int writelen = request->bodylen;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "OTA post len %d, bodylen %d, rxmax %d", request->contentLength, request->bodylen, request->receivedLenmax);

#if ENABLE_DRIVER_IR || ENABLE_DRIVER_IRREMOTEESP
	if(DRV_IsRunning("IR")) DRV_StopDriver("IR");
#endif
#if ENABLE_DRIVER_TINYIR_NEC
	if(DRV_IsRunning("TinyIR_NEC")) DRV_StopDriver("TinyIR_NEC");
#endif
#if ENABLE_DRIVER_RC
	if(DRV_IsRunning("RC")) DRV_StopDriver("RC");
#endif

	bool recvfp = true;

	ota_status_t ota_update_rest_init(void* url)
	{
		return OTA_STATUS_OK;
	}
	ota_status_t ota_update_rest_get(uint8_t* buf, uint32_t buf_size, uint32_t* recv_size, uint8_t* eof_flag)
	{
		const int max_empty_recv_retries = 15000; /* ~30s at 2ms */
		int empty_recv_retries = 0;

		*recv_size = 0;
		*eof_flag = 0;

		if (recvfp)
		{
			int bsize = (writelen > (int)buf_size ? (int)buf_size : writelen);
			if (bsize > 0)
			{
				memcpy(buf, writebuf + startaddr, bsize);
				startaddr += bsize;
				*recv_size = bsize;
				total += bsize;
				towrite -= bsize;
				writelen -= bsize;
				recvfp = writelen > 0;
				return OTA_STATUS_OK;
			}
			recvfp = false;
		}

		while (towrite > 0)
		{
			int want = (towrite > (int)buf_size ? (int)buf_size : towrite);
			if (want > request->receivedLenmax)
			{
				want = request->receivedLenmax;
			}
			if (want <= 0)
			{
				ADDLOG_ERROR(LOG_FEATURE_OTA, "recv want invalid %d, remaining %d, buf_size %u, rxmax %d",
					want, towrite, buf_size, request->receivedLenmax);
				*eof_flag = 1;
				return OTA_STATUS_ERROR;
			}

			writelen = recv(request->fd, buf, want, MSG_DONTWAIT);
			if (writelen > 0)
			{
				*recv_size = writelen;
				total += writelen;
				towrite -= writelen;
				*eof_flag = (towrite <= 0);
				return OTA_STATUS_OK;
			}

			/* On this XRadio HTTP path, recv() can transiently return <= 0 while the
			 * browser is still streaming the POST body. Do not treat the first -1 as EOF.
			 */
			empty_recv_retries++;
			if ((empty_recv_retries == 1) || ((empty_recv_retries % 1000) == 0))
			{
				ADDLOG_DEBUG(LOG_FEATURE_OTA, "recv retry %d, ret %d, remaining %d, want %d",
					empty_recv_retries, writelen, towrite, want);
			}
			if (empty_recv_retries >= max_empty_recv_retries)
			{
				ADDLOG_INFO(LOG_FEATURE_OTA, "recv gave no data after %d retries, ret %d, remaining %d",
					empty_recv_retries, writelen, towrite);
				*eof_flag = 1;
				return OTA_STATUS_OK;
			}
			rtos_delay_milliseconds(2);
		}

		*eof_flag = 1;
		return OTA_STATUS_OK;
	}

	int ret = 0;
	uint32_t* verify_value;
	ota_verify_t verify_type;
	ota_verify_data_t verify_data;

	if (request->contentLength > 0)
	{
		towrite = request->contentLength;
	}
	else
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA, "Content-length is 0");
		goto update_ota_exit;
	}

	ota_init();

	if (ota_update_image(NULL, ota_update_rest_init, ota_update_rest_get) != OTA_STATUS_OK)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA, "ota_update_image failed");
		goto update_ota_exit;
	}

	if (ota_get_verify_data(&verify_data) != OTA_STATUS_OK)
	{
		ADDLOG_INFO(LOG_FEATURE_OTA, "ota_get_verify_data not ok, OTA_VERIFY_NONE");
		verify_type = OTA_VERIFY_NONE;
		verify_value = NULL;
	}
	else
	{
		verify_type = verify_data.ov_type;
		ADDLOG_INFO(LOG_FEATURE_OTA, "ota_get_verify_data ok");
		verify_value = (uint32_t*)(verify_data.ov_data);
	}

	if (ota_verify_image(verify_type, verify_value) != OTA_STATUS_OK)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA, "OTA verify image failed");
		goto update_ota_exit;
	}

update_ota_exit:
	if (ret != -1)
	{
		ADDLOG_INFO(LOG_FEATURE_OTA, "OTA is successful");
	}
	else
	{
		ADDLOG_ERROR(LOG_FEATURE_OTA, "OTA failed.");
		return http_rest_error(request, ret, "error");
	}
	ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d total bytes written", total);
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	CFG_IncrementOTACount();
	return 0;
}

int HAL_FlashRead(char*buffer, int readlen, int startaddr) {
	int res;
	//uint32_t flash_read(uint32_t flash, uint32_t addr,void *buf, uint32_t size)
	res = flash_read(0, startaddr, buffer, readlen);
	return res;
}
#endif