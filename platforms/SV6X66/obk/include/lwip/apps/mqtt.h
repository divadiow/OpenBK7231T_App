#ifndef OBK_SV6X66_MQTT_H
#define OBK_SV6X66_MQTT_H
/* Original socket adapter for the OBK-facing lwIP MQTT API. This platform's
	* SDK ships lwIP1.4 without that optional application module. */
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include <stdint.h>
typedef struct sv6_mqtt_client mqtt_client_t;
struct mqtt_connect_client_info_t {
	const char *client_id,*client_user,*client_pass;
	uint16_t keep_alive;
	const char *will_topic,*will_msg;
	uint8_t will_qos,will_retain;
};
typedef enum {
	MQTT_CONNECT_ACCEPTED=0,MQTT_CONNECT_REFUSED_PROTOCOL_VERSION=1,
	MQTT_CONNECT_REFUSED_IDENTIFIER=2,MQTT_CONNECT_REFUSED_SERVER=3,
	MQTT_CONNECT_REFUSED_USERNAME_PASS=4,MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_=5,
	MQTT_CONNECT_DISCONNECTED=256,MQTT_CONNECT_TIMEOUT=257
} mqtt_connection_status_t;
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *,void *,mqtt_connection_status_t);
typedef void (*mqtt_request_cb_t)(void *,err_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *,const char *,u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *,const u8_t *,u16_t,u8_t);
#define MQTT_DATA_FLAG_LAST 1
mqtt_client_t *mqtt_client_new(void);
err_t mqtt_client_connect(mqtt_client_t *,const ip_addr_t *,u16_t,mqtt_connection_cb_t,void *,const struct mqtt_connect_client_info_t *);
void mqtt_disconnect(mqtt_client_t *);
u8_t mqtt_client_is_connected(mqtt_client_t *);
void mqtt_set_inpub_callback(mqtt_client_t *,mqtt_incoming_publish_cb_t,mqtt_incoming_data_cb_t,void *);
err_t mqtt_sub_unsub(mqtt_client_t *,const char *,u8_t,mqtt_request_cb_t,void *,u8_t);
err_t mqtt_publish(mqtt_client_t *,const char *,const void *,u16_t,u8_t,u8_t,mqtt_request_cb_t,void *);
#define mqtt_subscribe(c,t,q,cb,a) mqtt_sub_unsub(c,t,q,cb,a,1)
#define mqtt_unsubscribe(c,t,cb,a) mqtt_sub_unsub(c,t,0,cb,a,0)
#endif
