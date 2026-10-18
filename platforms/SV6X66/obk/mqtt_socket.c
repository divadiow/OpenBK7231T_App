#include "sv6x66_platform.h"
#include "lwip/apps/mqtt.h"
#include "new_common.h"
#include "new_cfg.h"
#include "new_pins.h"
#include <string.h>
#include <errno.h>

#define PACKET_MAX 4096
#define PENDING_MAX 16
typedef struct {
	uint16_t id;
	uint8_t expect;
	uint32_t since;
	mqtt_request_cb_t callback;
	void *argument;
} pending_request;
struct sv6_mqtt_client {
	int fd, connected, worker, stop;
	uint16_t sequence, keepalive;
	uint32_t last_received, last_sent, ping_since;
	SemaphoreHandle_t mutex, transmit;
	mqtt_connection_cb_t connection;
	void *connection_arg, *incoming_arg;
	mqtt_incoming_publish_cb_t publish;
	mqtt_incoming_data_cb_t data;
	pending_request pending[PENDING_MAX];
	uint16_t inbound_qos2[PENDING_MAX];
};
static int lock(mqtt_client_t *client) {
	return client && xSemaphoreTake(client->mutex, portMAX_DELAY) == pdTRUE;
}
static void unlock(mqtt_client_t *client) {
	xSemaphoreGive(client->mutex);
}
static int wait_io(int fd, int writing, uint32_t timeout_ms) {
	fd_set set;
	struct timeval timeout;
	FD_ZERO(&set);
	FD_SET(fd, &set);
	timeout.tv_sec = timeout_ms / 1000;
	timeout.tv_usec = (timeout_ms % 1000) * 1000;
	return select(fd + 1, writing ? NULL : &set, writing ? &set : NULL, NULL, &timeout) > 0 ? 0 : -1;
}
static int send_packet(mqtt_client_t *client, uint8_t control, const void *body, size_t length) {
	uint8_t header[5];
	size_t h = 1, remaining = length, sent = 0;
	int fd, result = -1;
	if (length > PACKET_MAX || xSemaphoreTake(client->transmit, portMAX_DELAY) != pdTRUE)
		return -1;
	header[0] = control;
	do {
		uint8_t digit = remaining % 128;
		remaining /= 128;
		if (remaining)
			digit |= 128;
		header[h++] = digit;
	} while (remaining && h < 5);
	lock(client);
	fd = client->fd;
	unlock(client);
	if (fd < 0)
		goto done;
	while (sent < h) {
		int n;
		if (wait_io(fd, 1, 5000))
			goto done;
		n = send(fd, header + sent, h - sent, 0);
		if (n <= 0)
			goto done;
		sent += n;
	}
	sent = 0;
	while (sent < length) {
		int n;
		if (wait_io(fd, 1, 5000))
			goto done;
		n = send(fd, (const uint8_t *)body + sent, length - sent, 0);
		if (n <= 0)
			goto done;
		sent += n;
	}
	lock(client);
	client->last_sent = OS_GetSysTick();
	unlock(client);
	result = 0;
done:
	xSemaphoreGive(client->transmit);
	return result;
}
static int put_string(uint8_t *buffer, size_t *used, const char *text) {
	size_t n = text ? strlen(text) : 0;
	if (n > 65535 || *used + 2 + n > PACKET_MAX)
		return -1;
	buffer[(*used)++] = n >> 8;
	buffer[(*used)++] = n;
	if (n)
		memcpy(buffer + *used, text, n);
	*used += n;
	return 0;
}
static uint16_t reserve(mqtt_client_t *client, uint8_t expect, mqtt_request_cb_t callback, void *arg) {
	uint32_t i, j;
	uint16_t id = 0;
	lock(client);
	for (i = 0; i < PENDING_MAX; i++)
		if (!client->pending[i].id)
			break;
	if (i == PENDING_MAX)
		goto done;
	do {
		id = ++client->sequence;
		if (!id)
			id = ++client->sequence;
		for (j = 0; j < PENDING_MAX; j++)
			if (client->pending[j].id == id)
				break;
	} while (j != PENDING_MAX);
	client->pending[i] = (pending_request){id, expect, OS_GetSysTick(), callback, arg};
done:
	unlock(client);
	return id;
}
static void complete(mqtt_client_t *client, uint16_t id, uint8_t type, err_t error) {
	mqtt_request_cb_t callback = NULL;
	void *arg = NULL;
	uint32_t i;
	lock(client);
	for (i = 0; i < PENDING_MAX; i++)
		if (client->pending[i].id == id && (type == 0 || client->pending[i].expect == type)) {
			callback = client->pending[i].callback;
			arg = client->pending[i].argument;
			memset(&client->pending[i], 0, sizeof(client->pending[i]));
			break;
		}
	unlock(client);
	if (callback)
		callback(arg, error);
}
static int reply_id(mqtt_client_t *client, uint8_t command, uint16_t id) {
	uint8_t body[2] = {id >> 8, id};
	return send_packet(client, command, body, 2);
}
static int receive_exact(mqtt_client_t *client, void *buffer, size_t bytes) {
	uint8_t *p = buffer;
	while (bytes) {
		int n;
		if (client->stop || wait_io(client->fd, 0, 5000))
			return -1;
		n = recv(client->fd, p, bytes, 0);
		if (n <= 0)
			return -1;
		p += n;
		bytes -= n;
	}
	return 0;
}
static int dispatch(mqtt_client_t *client, uint8_t control, uint8_t *body, size_t length) {
	uint8_t type = control >> 4;
	uint16_t id;
	uint32_t i;
	if (type != 3 && control != (uint8_t)((type << 4) | (type == 6 ? 2 : 0)))
		return -1;
	if (type == 2) {
		/* Every connection requests Clean Session, so Session Present must0. */
		if (length != 2 || body[0] || body[1] > 5)
			return -1;
		lock(client);
		client->connected = body[1] == 0;
		unlock(client);
		if (client->connection)
			client->connection(client, client->connection_arg, (mqtt_connection_status_t)body[1]);
		return body[1] ? -1 : 0;
	}
	if (type == 13) {
		if (length)
			return -1;
		client->ping_since = 0;
		return 0;
	}
	if (type == 3) {
		uint16_t topic_length;
		uint8_t qos = (control >> 1) & 3;
		size_t at = 2;
		char topic[257];
		int duplicate = 0;
		if (length < 2 || qos == 3)
			return -1;
		topic_length = ((uint16_t)body[0] << 8) | body[1];
		if (!topic_length || topic_length > 256 || topic_length > length - at)
			return -1;
		memcpy(topic, body + at, topic_length);
		topic[topic_length] = 0;
		at += topic_length;
		id = 0;
		if (qos) {
			if (length - at < 2)
				return -1;
			id = ((uint16_t)body[at] << 8) | body[at + 1];
			at += 2;
			if (!id)
				return -1;
		}
		if (qos == 2) {
			uint32_t vacant = PENDING_MAX;
			for (i = 0; i < PENDING_MAX; i++) {
				if (client->inbound_qos2[i] == id)
					duplicate = 1;
				if (!client->inbound_qos2[i])
					vacant = i;
			}
			if (!duplicate) {
				if (vacant == PENDING_MAX)
					return -1;
				client->inbound_qos2[vacant] = id;
			}
		}
		if (!duplicate) {
			if (client->publish)
				client->publish(client->incoming_arg, topic, length - at);
			if (client->data)
				client->data(client->incoming_arg, body + at, length - at, MQTT_DATA_FLAG_LAST);
		}
		return qos ? reply_id(client, qos == 1 ? 0x40 : 0x50, id) : 0;
	}
	if ((type >= 4 && type <= 7) || type == 9 || type == 11) {
		if (length < 2)
			return -1;
		id = ((uint16_t)body[0] << 8) | body[1];
		if (!id)
			return -1;
		if (type == 5) {
			int found = 0;
			lock(client);
			for (i = 0; i < PENDING_MAX; i++)
				if (client->pending[i].id == id && (client->pending[i].expect == 5 || client->pending[i].expect == 7)) {
					client->pending[i].expect = 7;
					found = 1;
					break;
				}
			unlock(client);
			return found ? reply_id(client, 0x62, id) : -1;
		}
		if (type == 6) {
			for (i = 0; i < PENDING_MAX; i++)
				if (client->inbound_qos2[i] == id)
					client->inbound_qos2[i] = 0;
			return reply_id(client, 0x70, id);
		}
		if (type == 9) {
			if (length != 3)
				return -1;
			complete(client, id, type, body[2] == 0x80 ? ERR_ABRT : ERR_OK);
		} else {
			if (length != 2)
				return -1;
			complete(client, id, type, ERR_OK);
		}
		return 0;
	}
	return -1;
}
static void worker(void *argument) {
	mqtt_client_t *client = argument;
	uint8_t *packet = malloc(PACKET_MAX), control, digit;
	uint32_t now;
	int failed = packet ? 0 : 1;
	while (!failed && !client->stop) {
		fd_set set;
		struct timeval timeout = {1, 0};
		int ready;
		uint32_t i;
		FD_ZERO(&set);
		FD_SET(client->fd, &set);
		ready = select(client->fd + 1, &set, NULL, NULL, &timeout);
		if (ready < 0)
			break;
		if (ready) {
			size_t length = 0, multiplier = 1;
			unsigned int digits = 0;
			if (receive_exact(client, &control, 1))
				break;
			do {
				if (++digits > 4 || receive_exact(client, &digit, 1)) {
					failed = 1;
					break;
				}
				length += (digit & 127) * multiplier;
				multiplier *= 128;
			} while (digit & 128);
			if (failed || length > PACKET_MAX || receive_exact(client, packet, length) || dispatch(client, control, packet, length))
				break;
			client->last_received = OS_GetSysTick();
		}
		now = OS_GetSysTick();
		if (!client->connected && (uint32_t)(now - client->last_received) > OS_MS2TICK(10000))
			break;
		for (i = 0; i < PENDING_MAX; i++) {
			uint16_t expired = 0;
			lock(client);
			if (client->pending[i].id && (uint32_t)(now - client->pending[i].since) > OS_MS2TICK(30000))
				expired = client->pending[i].id;
			unlock(client);
			if (expired)
				complete(client, expired, 0, ERR_TIMEOUT);
		}
		if (client->connected && client->keepalive) {
			if (client->ping_since && (uint32_t)(now - client->ping_since) > OS_MS2TICK(client->keepalive * 1000u))
				break;
			if (!client->ping_since && (uint32_t)(now - client->last_sent) >= OS_MS2TICK(client->keepalive * 500u)) {
				if (send_packet(client, 0xc0, NULL, 0))
					break;
				client->ping_since = now;
			}
		}
	}
	lock(client);
	{
		int fd = client->fd;
		client->fd = -1;
		client->connected = 0;
		unlock(client);
		if (fd >= 0)
			lwip_close(fd);
	}
	free(packet);
	for (now = 0; now < PENDING_MAX; now++) {
		uint16_t id;
		lock(client);
		id = client->pending[now].id;
		unlock(client);
		if (id)
			complete(client, id, 0, ERR_ABRT);
	}
	if (!client->stop && client->connection)
		client->connection(client, client->connection_arg, MQTT_CONNECT_DISCONNECTED);
	lock(client);
	client->worker = 0;
	unlock(client);
	vTaskDelete(NULL);
}
mqtt_client_t *mqtt_client_new(void) {
	mqtt_client_t *client = calloc(1, sizeof(*client));
	if (!client)
		return NULL;
	client->fd = -1;
	client->mutex = xSemaphoreCreateMutex();
	client->transmit = xSemaphoreCreateMutex();
	if (!client->mutex || !client->transmit) {
		if (client->mutex)
			vSemaphoreDelete(client->mutex);
		if (client->transmit)
			vSemaphoreDelete(client->transmit);
		free(client);
		return NULL;
	}
	return client;
}
u8_t mqtt_client_is_connected(mqtt_client_t *client) {
	int connected = 0;
	if (lock(client)) {
		connected = client->connected;
		unlock(client);
	}
	return connected;
}
void mqtt_set_inpub_callback(mqtt_client_t *client, mqtt_incoming_publish_cb_t publish, mqtt_incoming_data_cb_t data, void *arg) {
	if (lock(client)) {
		client->publish = publish;
		client->data = data;
		client->incoming_arg = arg;
		unlock(client);
	}
}
err_t mqtt_client_connect(mqtt_client_t *client, const ip_addr_t *ip, u16_t port, mqtt_connection_cb_t callback, void *arg, const struct mqtt_connect_client_info_t *info) {
	uint8_t *body, flags = 2;
	size_t used = 0;
	struct sockaddr_in address;
	int fd;
	if (!client || !ip || !info || !info->client_id || info->will_qos > 2)
		return ERR_ARG;
	/* The persisted field exists even when MQTT_USE_TLS removes its accessor.
		* Reject a retained TLS request instead of silently connecting plaintext. */
	if (g_cfg.mqtt_use_tls) {
		SV6X66_Printf("MQTT TLS is unsupported on this profile; plaintext fallback refused\r\n");
		return ERR_ARG;
	}
	lock(client);
	if (client->worker) {
		unlock(client);
		return ERR_INPROGRESS;
	}
	unlock(client);
	body = malloc(PACKET_MAX);
	if (!body)
		return ERR_MEM;
	if (put_string(body, &used, "MQTT"))
		goto invalid;
	body[used++] = 4;
	if (info->client_user)
		flags |= 128;
	if (info->client_pass)
		flags |= 64;
	if (info->will_topic)
		flags |= 4 | (info->will_qos << 3) | (info->will_retain ? 32 : 0);
	body[used++] = flags;
	body[used++] = info->keep_alive >> 8;
	body[used++] = info->keep_alive;
	if (put_string(body, &used, info->client_id))
		goto invalid;
	if (info->will_topic && (put_string(body, &used, info->will_topic) || put_string(body, &used, info->will_msg)))
		goto invalid;
	if (info->client_user && put_string(body, &used, info->client_user))
		goto invalid;
	if (info->client_pass && put_string(body, &used, info->client_pass))
		goto invalid;
	fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		free(body);
		return ERR_MEM;
	}
	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	address.sin_addr.s_addr = ip->addr;
	{
		unsigned long nonblocking = 1;
		int error = 0;
		socklen_t error_bytes = sizeof(error);
		if (lwip_ioctl(fd, FIONBIO, &nonblocking) < 0) {
			lwip_close(fd);
			free(body);
			return ERR_CONN;
		}
		if (connect(fd, (struct sockaddr *)&address, sizeof(address)) < 0 &&
			(wait_io(fd, 1, 10000) || getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &error_bytes) < 0 || error)) {
			lwip_close(fd);
			free(body);
			return ERR_CONN;
		}
	}
	lock(client);
	client->fd = fd;
	client->stop = 0;
	client->connected = 0;
	client->worker = 1;
	client->connection = callback;
	client->connection_arg = arg;
	client->keepalive = info->keep_alive;
	client->last_received = client->last_sent = OS_GetSysTick();
	client->ping_since = 0;
	memset(client->inbound_qos2, 0, sizeof(client->inbound_qos2));
	unlock(client);
	if (send_packet(client, 0x10, body, used) || rtos_create_thread(NULL, 2, "mqtt_io", worker, 6144, client)) {
		lock(client);
		client->fd = -1;
		client->worker = 0;
		unlock(client);
		lwip_close(fd);
		free(body);
		return ERR_CONN;
	}
	free(body);
	return ERR_OK;
invalid:
	free(body);
	return ERR_ARG;
}
void mqtt_disconnect(mqtt_client_t *client) {
	if (!client)
		return;
	send_packet(client, 0xe0, NULL, 0);
	lock(client);
	client->stop = 1;
	client->connected = 0;
	unlock(client);
}
err_t mqtt_sub_unsub(mqtt_client_t *client, const char *topic, u8_t qos, mqtt_request_cb_t callback, void *arg, u8_t subscribe) {
	uint8_t *body;
	size_t used = 2;
	uint16_t id;
	int result;
	if (!mqtt_client_is_connected(client) || !topic || qos > 2)
		return ERR_CONN;
	body = malloc(PACKET_MAX);
	if (!body)
		return ERR_MEM;
	if (put_string(body, &used, topic) || (subscribe && used == PACKET_MAX)) {
		free(body);
		return ERR_ARG;
	}
	if (subscribe)
		body[used++] = qos;
	id = reserve(client, subscribe ? 9 : 11, callback, arg);
	if (!id) {
		free(body);
		return ERR_MEM;
	}
	body[0] = id >> 8;
	body[1] = id;
	result = send_packet(client, subscribe ? 0x82 : 0xa2, body, used);
	free(body);
	if (result) {
		complete(client, id, 0, ERR_CONN);
		return ERR_CONN;
	}
	return ERR_OK;
}
err_t mqtt_publish(mqtt_client_t *client, const char *topic, const void *payload, u16_t bytes, u8_t qos, u8_t retain, mqtt_request_cb_t callback, void *arg) {
	uint8_t *body;
	size_t used = 0;
	uint16_t id = 0;
	int result;
	if (!mqtt_client_is_connected(client) || !topic || qos > 2 || (!payload && bytes))
		return ERR_CONN;
	body = malloc(PACKET_MAX);
	if (!body)
		return ERR_MEM;
	if (put_string(body, &used, topic) || used + bytes + (qos ? 2 : 0) > PACKET_MAX) {
		free(body);
		return ERR_BUF;
	}
	if (qos) {
		id = reserve(client, qos == 1 ? 4 : 5, callback, arg);
		if (!id) {
			free(body);
			return ERR_MEM;
		}
		body[used++] = id >> 8;
		body[used++] = id;
	}
	if (bytes)
		memcpy(body + used, payload, bytes);
	used += bytes;
	result = send_packet(client, 0x30 | (qos << 1) | (retain ? 1 : 0), body, used);
	free(body);
	if (result) {
		if (id)
			complete(client, id, 0, ERR_CONN);
		return ERR_CONN;
	}
	if (!qos && callback)
		callback(arg, ERR_OK);
	return ERR_OK;
}
