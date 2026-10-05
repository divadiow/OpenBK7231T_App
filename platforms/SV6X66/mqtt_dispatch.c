// All raw lwIP operations run in the SDK TCP/IP task. Keep SDK core locking unchanged.
#include "obk_platform.h"
#include "my_lwip2_mqtt_replacement.h"
#include <string.h>

extern mqtt_client_t *mqtt_client_new_core(void);
extern err_t mqtt_client_connect_core(mqtt_client_t *, const ip_addr_t *, u16_t, mqtt_connection_cb_t, void *, const struct mqtt_connect_client_info_t *);
extern void mqtt_disconnect_core(mqtt_client_t *);
extern u8_t mqtt_client_is_connected_core(mqtt_client_t *);
extern void mqtt_set_inpub_callback_core(mqtt_client_t *, mqtt_incoming_publish_cb_t, mqtt_incoming_data_cb_t, void *);
extern err_t mqtt_sub_unsub_core(mqtt_client_t *, const char *, u8_t, mqtt_request_cb_t, void *, u8_t);
extern err_t mqtt_publish_core(mqtt_client_t *, const char *, const void *, u16_t, u8_t, u8_t, mqtt_request_cb_t, void *);

static OsTaskHandle tcpip_task;
enum operation { NET_INIT, MQTT_NEW, MQTT_CONNECT, MQTT_DISCONNECT, MQTT_CONNECTED, MQTT_CALLBACKS, MQTT_SUB, MQTT_PUBLISH, NET_DNS };
typedef struct {
    enum operation operation;
    xSemaphoreHandle completion;
    err_t result;
    mqtt_client_t *client;
    const char *text;
    const void *payload;
    u16_t length, port;
    u8_t qos, retain, subscribe, connected;
    const ip_addr_t *address;
    ip_addr_t *resolved;
    mqtt_connection_cb_t connection_cb;
    mqtt_request_cb_t request_cb;
    mqtt_incoming_publish_cb_t publish_cb;
    mqtt_incoming_data_cb_t data_cb;
    dns_found_callback dns_cb;
    void *arg;
    const struct mqtt_connect_client_info_t *info;
} net_call_t;
static void execute(void *arg)
{
    net_call_t *call = arg;
    call->result = ERR_OK;
    switch (call->operation) {
    case NET_INIT: tcpip_task = OS_TaskGetCurrHandle(); break;
    case MQTT_NEW: call->client = mqtt_client_new_core(); break;
    case MQTT_CONNECT: call->result = mqtt_client_connect_core(call->client, call->address, call->port, call->connection_cb, call->arg, call->info); break;
    case MQTT_DISCONNECT: mqtt_disconnect_core(call->client); break;
    case MQTT_CONNECTED: call->connected = mqtt_client_is_connected_core(call->client); break;
    case MQTT_CALLBACKS: mqtt_set_inpub_callback_core(call->client, call->publish_cb, call->data_cb, call->arg); break;
    case MQTT_SUB: call->result = mqtt_sub_unsub_core(call->client, call->text, call->qos, call->request_cb, call->arg, call->subscribe); break;
    case MQTT_PUBLISH: call->result = mqtt_publish_core(call->client, call->text, call->payload, call->length, call->qos, call->retain, call->request_cb, call->arg); break;
    case NET_DNS: call->result = dns_gethostbyname(call->text, call->resolved, call->dns_cb, call->arg); break;
    }
    if (call->completion) xSemaphoreGive(call->completion);
}
static err_t dispatch(net_call_t *call)
{
    err_t result;
    if (tcpip_task && OS_TaskGetCurrHandle() == tcpip_task) {
        execute(call);
        return call->result;
    }
    if (!tcpip_task && call->operation != NET_INIT) return ERR_IF;
    call->completion = xSemaphoreCreateBinary();
    if (!call->completion) return ERR_MEM;
    result = tcpip_callback_with_block(execute, call, 0);
    if (result == ERR_OK) {
        // An accepted callback owns this stack request until completion. Never time out.
        xSemaphoreTake(call->completion, portMAX_DELAY);
        result = call->result;
    }
    vSemaphoreDelete(call->completion);
    return result;
}
int SV6X66_NetInit(void)
{
    net_call_t call = {0};
    call.operation = NET_INIT;
    return dispatch(&call) == ERR_OK;
}
mqtt_client_t *mqtt_client_new(void)
{
    net_call_t call = {0}; call.operation = MQTT_NEW;
    return dispatch(&call) == ERR_OK ? call.client : NULL;
}
err_t mqtt_client_connect(mqtt_client_t *client, const ip_addr_t *ip, u16_t port, mqtt_connection_cb_t cb, void *arg, const struct mqtt_connect_client_info_t *info)
{
    if (!client || !ip || !info || !info->client_id) return ERR_ARG;
    net_call_t call = {0}; call.operation = MQTT_CONNECT;
    call.client = client; call.address = ip; call.port = port; call.connection_cb = cb; call.arg = arg; call.info = info;
    return dispatch(&call);
}
void mqtt_disconnect(mqtt_client_t *client)
{
    if (!client) return;
    net_call_t call = {0}; call.operation = MQTT_DISCONNECT; call.client = client;
    dispatch(&call);
}
u8_t mqtt_client_is_connected(mqtt_client_t *client)
{
    if (!client) return 0;
    net_call_t call = {0}; call.operation = MQTT_CONNECTED; call.client = client;
    return dispatch(&call) == ERR_OK ? call.connected : 0;
}
void mqtt_set_inpub_callback(mqtt_client_t *client, mqtt_incoming_publish_cb_t pub, mqtt_incoming_data_cb_t data, void *arg)
{
    if (!client) return;
    net_call_t call = {0}; call.operation = MQTT_CALLBACKS;
    call.client = client; call.publish_cb = pub; call.data_cb = data; call.arg = arg;
    dispatch(&call);
}
err_t mqtt_sub_unsub(mqtt_client_t *client, const char *topic, u8_t qos, mqtt_request_cb_t cb, void *arg, u8_t sub)
{
    if (!client || !topic) return ERR_ARG;
    net_call_t call = {0}; call.operation = MQTT_SUB;
    call.client = client; call.text = topic; call.qos = qos; call.request_cb = cb; call.arg = arg; call.subscribe = sub;
    return dispatch(&call);
}
err_t mqtt_publish(mqtt_client_t *client, const char *topic, const void *payload, u16_t length, u8_t qos, u8_t retain, mqtt_request_cb_t cb, void *arg)
{
    if (!client || !topic || (!payload && length)) return ERR_ARG;
    net_call_t call = {0}; call.operation = MQTT_PUBLISH;
    call.client = client; call.text = topic; call.payload = payload; call.length = length;
    call.qos = qos; call.retain = retain; call.request_cb = cb; call.arg = arg;
    return dispatch(&call);
}
err_t SV6X66_DNSLookup(const char *name, ip_addr_t *address, dns_found_callback cb, void *arg)
{
    if (!name || !address) return ERR_ARG;
    net_call_t call = {0}; call.operation = NET_DNS;
    call.text = name; call.resolved = address; call.dns_cb = cb; call.arg = arg;
    return dispatch(&call);
}
