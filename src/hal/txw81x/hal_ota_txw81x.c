#if PLATFORM_TXW81X

#include "../../obk_config.h"
#include "../../new_common.h"
#include "../../new_cfg.h"
#include "../../logging/logging.h"
#include "../../httpserver/new_http.h"
#include "../hal_ota.h"
#include "lib/ota/fw.h"

/*
 * The TXW81X SDK OTA API takes a uint16 length.  Keep every write small even
 * when the HTTP server has already buffered a large part of the request body.
 * The vendor TCP updater also feeds libota_write_fw() in roughly MTU-sized
 * pieces.
 */
#define TXW81X_OTA_IO_CHUNK 1460

int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr)
{
	int total = 0;
	int towrite = 0;
	char* writebuf = request->bodystart;
	int buffered = request->bodylen;
	int ret = 0;
	int res = 0;

	(void)maxaddr;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "OTA post len %d, initially buffered %d",
		request->contentLength, request->bodylen);

	if(request->contentLength <= 0)
	{
		ret = -1;
		ADDLOG_ERROR(LOG_FEATURE_OTA, "Content-length is 0");
		goto update_ota_exit;
	}

	towrite = request->contentLength;

	/* Do not consume beyond the declared HTTP request body. */
	if(buffered < 0)
	{
		buffered = 0;
	}
	else if(buffered > towrite)
	{
		buffered = towrite;
	}

	while(towrite > 0)
	{
		int writelen;

		/* Once the bytes already held after the HTTP headers are exhausted,
		 * continue streaming the body from the same socket. */
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
			buffered = recv(request->fd, writebuf, recvmax, 0);
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

		res = libota_write_fw(request->contentLength, startaddr,
			(uint8*)writebuf, (uint16)writelen);

		/* TXW81X follows the normal C convention used by the vendor updater:
		 * zero is success; any non-zero return value is an error. */
		if(res != 0)
		{
			ADDLOG_ERROR(LOG_FEATURE_OTA,
				"libota_write_fw failed at offset %d, len %d, res %d",
				startaddr, writelen, res);
			ret = -1;
			goto update_ota_exit;
		}

		ADDLOG_INFO(LOG_FEATURE_OTA, "Writelen %d at %d, res %d",
			writelen, total, res);

		total += writelen;
		startaddr += writelen;
		towrite -= writelen;
		writebuf += writelen;
		buffered -= writelen;

		rtos_delay_milliseconds(10);
	}

update_ota_exit:
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

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d total bytes written", total);
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	CFG_IncrementOTACount();
	return 0;
}

#endif
