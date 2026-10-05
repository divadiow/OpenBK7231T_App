#include "include/test_net_env.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

mqtt_client_t *mqtt_client_new(void);
err_t mqtt_client_connect(mqtt_client_t *, const ip_addr_t *, u16_t,
                          mqtt_connection_cb_t, void *, const struct mqtt_connect_client_info_t *);
void mqtt_disconnect(mqtt_client_t *);
u8_t mqtt_client_is_connected(mqtt_client_t *);
void mqtt_set_inpub_callback(mqtt_client_t *, mqtt_incoming_publish_cb_t,
                             mqtt_incoming_data_cb_t, void *);
err_t mqtt_sub_unsub(mqtt_client_t *, const char *, u8_t,
                     mqtt_request_cb_t, void *, u8_t);
err_t mqtt_publish(mqtt_client_t *, const char *, const void *, u16_t,
                   u8_t, u8_t, mqtt_request_cb_t, void *);
err_t SV6X66_DNSLookup(const char *, ip_addr_t *, dns_found_callback, void *);
int SV6X66_NetInit(void);

#define QUEUE_CAPACITY 256
#define CLIENT_CAPACITY 128

typedef struct { tcpip_callback_fn callback; void *arg; } queued_call_t;
struct test_sem { sem_t sem; };

static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_ready = PTHREAD_COND_INITIALIZER;
static queued_call_t queue_items[QUEUE_CAPACITY];
static unsigned queue_head, queue_tail;
static int stop_worker, fail_next_queue, fail_next_sem;
static pthread_t worker_id;
static int worker_ready;
static unsigned wrong_thread_calls, core_calls, queue_calls;
static mqtt_client_t clients[CLIENT_CAPACITY];
static unsigned clients_created;
static int next_publish_result, next_sub_result, trigger_nested_publish;
static mqtt_client_t *nested_client;
static int nested_publish_result;
static void *stored_incoming_arg;
static mqtt_incoming_publish_cb_t stored_publish_cb;
static mqtt_incoming_data_cb_t stored_data_cb;
static void *incoming_test_arg;
static int connection_callback_status = -1;

static pthread_mutex_t dns_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t dns_changed = PTHREAD_COND_INITIALIZER;
static int dns_mode, dns_callback_count;
static const char *dns_name_seen;
static void *dns_arg_seen;
static ip_addr_t *dns_result_seen;
static dns_found_callback pending_dns_callback;
static ip_addr_t *pending_dns_address;
static void *pending_dns_arg;
static const char *pending_dns_name;

static int queue_push(tcpip_callback_fn callback, void *arg)
{
    unsigned next;
    pthread_mutex_lock(&queue_mutex);
    if (fail_next_queue) {
        fail_next_queue = 0;
        pthread_mutex_unlock(&queue_mutex);
        return 0;
    }
    next = (queue_tail + 1u) % QUEUE_CAPACITY;
    if (next == queue_head) {
        pthread_mutex_unlock(&queue_mutex);
        return 0;
    }
    queue_items[queue_tail].callback = callback;
    queue_items[queue_tail].arg = arg;
    queue_tail = next;
    queue_calls++;
    pthread_cond_signal(&queue_ready);
    pthread_mutex_unlock(&queue_mutex);
    return 1;
}

static void *tcpip_worker(void *unused)
{
    (void)unused;
    pthread_mutex_lock(&queue_mutex);
    worker_id = pthread_self();
    worker_ready = 1;
    pthread_cond_broadcast(&queue_ready);
    for (;;) {
        queued_call_t item;
        while (queue_head == queue_tail)
            pthread_cond_wait(&queue_ready, &queue_mutex);
        item = queue_items[queue_head];
        queue_head = (queue_head + 1u) % QUEUE_CAPACITY;
        if (stop_worker && item.callback == NULL)
            break;
        pthread_mutex_unlock(&queue_mutex);
        if (item.callback != NULL)
            item.callback(item.arg);
        pthread_mutex_lock(&queue_mutex);
    }
    pthread_mutex_unlock(&queue_mutex);
    return NULL;
}

OsTaskHandle OS_TaskGetCurrHandle(void) { return pthread_self(); }

xSemaphoreHandle xSemaphoreCreateBinary(void)
{
    struct test_sem *sem;
    pthread_mutex_lock(&queue_mutex);
    if (fail_next_sem) {
        fail_next_sem = 0;
        pthread_mutex_unlock(&queue_mutex);
        return NULL;
    }
    pthread_mutex_unlock(&queue_mutex);
    sem = (struct test_sem *)malloc(sizeof(*sem));
    if (sem == NULL || sem_init(&sem->sem, 0, 0) != 0) {
        free(sem);
        return NULL;
    }
    return sem;
}

int xSemaphoreGive(xSemaphoreHandle sem)
{
    return sem != NULL && sem_post(&sem->sem) == 0 ? pdTRUE : pdFALSE;
}

int xSemaphoreTake(xSemaphoreHandle sem, uint32_t timeout)
{
    (void)timeout;
    if (sem == NULL)
        return pdFALSE;
    while (sem_wait(&sem->sem) != 0) {
        if (errno != EINTR)
            return pdFALSE;
    }
    return pdTRUE;
}

void vSemaphoreDelete(xSemaphoreHandle sem)
{
    if (sem != NULL) {
        sem_destroy(&sem->sem);
        free(sem);
    }
}

err_t tcpip_callback_with_block(tcpip_callback_fn callback, void *arg, u8_t block)
{
    (void)block;
    return queue_push(callback, arg) ? ERR_OK : ERR_MEM;
}

static void record_core_call(void)
{
    core_calls++;
    if (!pthread_equal(pthread_self(), worker_id))
        wrong_thread_calls++;
}

mqtt_client_t *mqtt_client_new_core(void)
{
    mqtt_client_t *client;
    record_core_call();
    assert(clients_created < CLIENT_CAPACITY);
    client = &clients[clients_created];
    memset(client, 0, sizeof(*client));
    client->id = (int)++clients_created;
    return client;
}

err_t mqtt_client_connect_core(mqtt_client_t *client, const ip_addr_t *ip, u16_t port,
                              mqtt_connection_cb_t callback, void *arg,
                              const struct mqtt_connect_client_info_t *info)
{
    record_core_call();
    assert(client != NULL && ip != NULL && port == 1883 && info != NULL);
    client->connected = 1;
    if (callback != NULL)
        callback(client, arg, MQTT_CONNECT_ACCEPTED);
    return ERR_OK;
}

void mqtt_disconnect_core(mqtt_client_t *client)
{
    record_core_call();
    assert(client != NULL);
    client->connected = 0;
}

u8_t mqtt_client_is_connected_core(mqtt_client_t *client)
{
    record_core_call();
    assert(client != NULL);
    return (u8_t)client->connected;
}

void mqtt_set_inpub_callback_core(mqtt_client_t *client,
                                  mqtt_incoming_publish_cb_t publish_cb,
                                  mqtt_incoming_data_cb_t data_cb, void *arg)
{
    record_core_call();
    assert(client != NULL);
    stored_publish_cb = publish_cb;
    stored_data_cb = data_cb;
    stored_incoming_arg = arg;
}

err_t mqtt_sub_unsub_core(mqtt_client_t *client, const char *topic, u8_t qos,
                          mqtt_request_cb_t callback, void *arg, u8_t subscribe)
{
    record_core_call();
    assert(client != NULL && topic != NULL && qos <= 2 && subscribe <= 1);
    if (trigger_nested_publish && callback != NULL) {
        nested_client = client;
        callback(arg, ERR_OK);
    }
    return next_sub_result;
}

err_t mqtt_publish_core(mqtt_client_t *client, const char *topic,
                        const void *payload, u16_t length, u8_t qos,
                        u8_t retain, mqtt_request_cb_t callback, void *arg)
{
    record_core_call();
    assert(client != NULL && topic != NULL && (payload != NULL || length == 0));
    assert(qos <= 2 && retain <= 1);
    (void)callback;
    (void)arg;
    return next_publish_result;
}

err_t dns_gethostbyname(const char *name, ip_addr_t *address,
                        dns_found_callback callback, void *arg)
{
    record_core_call();
    assert(name != NULL && address != NULL && callback != NULL);
    if (dns_mode == 0) {
        address->addr = 0x01020304u;
        /* Cached/numeric results are returned without calling the callback. */
        return ERR_OK;
    }
    pending_dns_callback = callback;
    pending_dns_address = address;
    pending_dns_arg = arg;
    pending_dns_name = name;
    return ERR_INPROGRESS;
}

static void connection_callback(mqtt_client_t *client, void *arg,
                                mqtt_connection_status_t status)
{
    assert(client != NULL && arg == &connection_callback_status);
    connection_callback_status = (int)status;
}

static void request_callback(void *arg, err_t result)
{
    assert(arg == &nested_publish_result && result == ERR_OK);
    nested_publish_result = mqtt_publish(nested_client, "nested/topic", "x", 1,
                                         0, 0, NULL, NULL);
}

static void incoming_publish_callback(void *arg, const char *topic, u32_t length)
{
    assert(arg == incoming_test_arg && strcmp(topic, "incoming/topic") == 0 && length == 3);
}

static void incoming_data_callback(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    assert(arg == incoming_test_arg && data != NULL && length == 3 && flags == 1);
}


static void dns_callback(const char *name, const ip_addr_t *address, void *arg)
{
    pthread_mutex_lock(&dns_mutex);
    dns_name_seen = name;
    dns_arg_seen = arg;
    dns_result_seen = (ip_addr_t *)address;
    dns_callback_count++;
    pthread_cond_broadcast(&dns_changed);
    pthread_mutex_unlock(&dns_mutex);
}

static void complete_pending_dns(void *unused)
{
    (void)unused;
    assert(pending_dns_callback != NULL);
    pending_dns_callback(pending_dns_name, pending_dns_address, pending_dns_arg);
    pending_dns_callback = NULL;
}

static void wait_for_dns_callback(int expected_count)
{
    pthread_mutex_lock(&dns_mutex);
    while (dns_callback_count < expected_count)
        pthread_cond_wait(&dns_changed, &dns_mutex);
    pthread_mutex_unlock(&dns_mutex);
}

typedef struct { int iterations; } concurrent_work_t;
static void *concurrent_client_work(void *arg)
{
    concurrent_work_t *work = (concurrent_work_t *)arg;
    for (int i = 0; i < work->iterations; ++i) {
        mqtt_client_t *client = mqtt_client_new();
        ip_addr_t address = { 0x7f000001u };
        struct mqtt_connect_client_info_t info = { "parallel-client", NULL, NULL, 20 };
        assert(client != NULL);
        assert(mqtt_client_connect(client, &address, 1883, NULL, NULL, &info) == ERR_OK);
        assert(mqtt_publish(client, "parallel/topic", "data", 4, 1, 0, NULL, NULL) == ERR_OK);
        assert(mqtt_client_is_connected(client) == 1);
        mqtt_disconnect(client);
    }
    return NULL;
}

int main(void)
{
    pthread_t worker, clients_threads[8];
    mqtt_client_t *client;
    ip_addr_t address = { 0x7f000001u }, dns_address = { 0 };
    struct mqtt_connect_client_info_t info = { "host-test", NULL, NULL, 30 };
    int previous_core_calls;
    char payload[] = "test";
    void *incoming_arg = &payload;
    concurrent_work_t work = { 8 };

    /* Calls before worker ownership is established fail without queueing. */
    assert(mqtt_client_new() == NULL);
    assert(queue_calls == 0);
    pthread_mutex_lock(&queue_mutex);
    assert(pthread_create(&worker, NULL, tcpip_worker, NULL) == 0);
    while (!worker_ready)
        pthread_cond_wait(&queue_ready, &queue_mutex);
    pthread_mutex_unlock(&queue_mutex);

    assert(SV6X66_NetInit() == 1);
    client = mqtt_client_new();
    assert(client != NULL);
    assert(mqtt_client_connect(client, &address, 1883, connection_callback,
                               &connection_callback_status, &info) == ERR_OK);
    assert(connection_callback_status == MQTT_CONNECT_ACCEPTED);
    assert(mqtt_client_is_connected(client) == 1);
    incoming_test_arg = incoming_arg;
    mqtt_set_inpub_callback(client, incoming_publish_callback,
                            incoming_data_callback, incoming_arg);
    assert(stored_incoming_arg == incoming_arg);
    assert(stored_publish_cb == incoming_publish_callback && stored_data_cb == incoming_data_callback);
    stored_publish_cb(stored_incoming_arg, "incoming/topic", 3);
    { const u8_t data[] = { 1, 2, 3 }; stored_data_cb(stored_incoming_arg, data, 3, 1); }
    assert(mqtt_sub_unsub(client, "normal/topic", 1, NULL, NULL, 1) == ERR_OK);
    assert(mqtt_publish(client, "normal/topic", payload, sizeof(payload), 1, 0, NULL, NULL) == ERR_OK);
    next_publish_result = ERR_ARG;
    assert(mqtt_publish(client, "result/topic", payload, sizeof(payload), 0, 0, NULL, NULL) == ERR_ARG);
    next_publish_result = ERR_OK;
    mqtt_disconnect(client);
    assert(mqtt_client_is_connected(client) == 0);

    /* Allocation and tcpip enqueue failures return promptly without core calls. */
    previous_core_calls = (int)core_calls;
    pthread_mutex_lock(&queue_mutex);
    fail_next_sem = 1;
    pthread_mutex_unlock(&queue_mutex);
    assert(mqtt_client_new() == NULL);
    assert((int)core_calls == previous_core_calls);
    pthread_mutex_lock(&queue_mutex);
    fail_next_queue = 1;
    pthread_mutex_unlock(&queue_mutex);
    assert(mqtt_publish(client, "queue/fail", "x", 1, 0, 0, NULL, NULL) == ERR_MEM);
    assert((int)core_calls == previous_core_calls);

    /* A request callback on tcpip may safely issue a synchronous nested publish. */
    trigger_nested_publish = 1;
    nested_publish_result = -99;
    assert(mqtt_sub_unsub(client, "nested/subscribe", 1, request_callback,
                          &nested_publish_result, 1) == ERR_OK);
    trigger_nested_publish = 0;
    assert(nested_publish_result == ERR_OK);

    /* DNS immediate completion and ERR_INPROGRESS callback forwarding. */
    dns_mode = 0;
    dns_callback_count = 0;
    assert(SV6X66_DNSLookup("immediate.test", &dns_address, dns_callback, &info) == ERR_OK);
    assert(dns_callback_count == 0 && dns_address.addr == 0x01020304u);
    dns_mode = 1;
    assert(SV6X66_DNSLookup("pending.test", &dns_address, dns_callback, &info) == ERR_INPROGRESS);
    assert(dns_callback_count == 0);
    assert(queue_push(complete_pending_dns, NULL));
    wait_for_dns_callback(1);
    assert(strcmp(dns_name_seen, "pending.test") == 0);
    assert(dns_arg_seen == &info && dns_result_seen == &dns_address);

    /* Concurrent callers retain per-call results while core calls stay on tcpip. */
    for (size_t i = 0; i < sizeof(clients_threads) / sizeof(clients_threads[0]); ++i)
        assert(pthread_create(&clients_threads[i], NULL, concurrent_client_work, &work) == 0);
    for (size_t i = 0; i < sizeof(clients_threads) / sizeof(clients_threads[0]); ++i)
        assert(pthread_join(clients_threads[i], NULL) == 0);
    assert(wrong_thread_calls == 0);

    pthread_mutex_lock(&queue_mutex);
    stop_worker = 1;
    pthread_mutex_unlock(&queue_mutex);
    assert(queue_push(NULL, NULL));
    assert(pthread_join(worker, NULL) == 0);
    printf("mqtt dispatcher host tests passed (%u queued calls, %u core calls)\n",
           queue_calls, core_calls);
    return 0;
}