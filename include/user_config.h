#ifndef __MQTT_CONFIG_H__
#define __MQTT_CONFIG_H__

#define WIFI_AP_NAME "BridgeThings"
#define WIFI_AP_PASSWORD "password"
#define CFG_HOLDER	0x00FF55A4	/* Change this value to load default configurations */
#define CFG_LOCATION	0x3C	/* Please don't change or if you know what you doing */
#define VERSION 1.0
#define MQTT_SSL_ENABLE

/*DEFAULT CONFIGURATIONS*/
#define DATA_SEND_DELAY 30000	/* milliseconds */
#define LDR_SEND_DELAY 10000
#define MQTT_HOST     "35.154.140.175" //or "mqtt.yourdomain.com"
#define MQTT_PORT     1883
#define MQTT_BUF_SIZE   1024
#define MQTT_KEEPALIVE    120  /*second*/

#define MQTT_CLIENT_ID "B_%08X"
#define MQTT_USER ""
#define MQTT_PASS ""
#define MQTT_CLEAN_SESSION 1
#define MQTT_KEEPALIVE 120

#define STA_SSID ""
#define STA_PASS ""
#define STA_TYPE AUTH_WPA2_PSK
#define MQTT_RECONNECT_TIMEOUT  5 /*second*/

#define DEFAULT_SECURITY  0
#define QUEUE_BUFFER_SIZE       2048

#define PROTOCOL_NAMEv31  /*MQTT version 3.1 compatible with Mosquitto v0.15*/
//PROTOCOL_NAMEv311     /*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/
#define DEBUG_ON
#if defined(DEBUG_ON)
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

#endif // __MQTT_CONFIG_H__
