#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "rtc_wdt.h"
#include "esp_task_wdt.h"
#include "esp_mac.h"

#include "app_globals.h"
#include "app_mqtt.h"
#include "app_time.h"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "APP_MQTT";

static const char telem_topic[] = "/topic/qos0";

TelemDataT telem_data = {0};

#define MAX_JSON_SIZE 128
static char telem_json[MAX_JSON_SIZE];

#define BROKER_URI      "wss://mqtt.munx.xyz:443"

#define MAC_ADDRESS_SIZE    6
static uint8_t mac_address[MAC_ADDRESS_SIZE];

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    if (client != mqtt_client)
    {
        ESP_LOGW(TAG, "MQTT client handle does not match client initialized");
    }

    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) 
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            if (strncmp(event->data, "send telemetry please", event->data_len) == 0) {
                ESP_LOGI(TAG, "Sending telemetry");
                send_telemetery();
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

uint16_t mqtt_init(const char *ca_crt, const char *client_crt, const char *client_key)
{
    esp_err_t err;
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = BROKER_URI,
            .verification.certificate = ca_crt
        },
        .credentials = {
            .authentication = {
                .certificate = client_crt,
                .key = client_key
            }
        }
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK)
    {
         ESP_LOGE(TAG, "Failed to register mqtt event handler. Err:%d", err);
         return err;
    }
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK)
    {
         ESP_LOGE(TAG, "Failed to start mqtt client. Err:%d", err);
         return err;
    }
    return err;
}

void buffer_telem_data(TelemDataT *data_to_send)
{
    if ((char)(data_to_send->data[0]) != 0xFF)
    {
        memcpy(telem_data.data, data_to_send->data, DATA_STRING_SIZE);
    }
    if (data_to_send->espTemp != 0xFFFFFFFF)
    {
        telem_data.espTemp = data_to_send->espTemp;
    }
}
void set_mqtt_mac(uint8_t *mqtt_mac)
{
    memcpy(mac_address, mqtt_mac, MAC_ADDRESS_SIZE);
}

void send_telemetery(void)
{
    time_t unix_time = GetUnixTime();
    uint16_t json_size = sprintf(telem_json, "{");
#if SEND_TELEM_MAC
    json_size += sprintf(telem_json + json_size, "\"MAC\":\"%02X%02X%02X%02X%02X%02X\",",  \
        mac_address[0],mac_address[1],mac_address[2],mac_address[3],mac_address[4],mac_address[5]);
#endif
    json_size += sprintf(telem_json + json_size, "\"time\":%lld",  \
        unix_time);
    if (telem_data.data[0] != 0xFF)
    {
        json_size += sprintf(telem_json + json_size, ",\"data\":\"%s\"",  \
            telem_data.data);
    }
    if (telem_data.espTemp != 0xFFFFFFFF)
    {
        json_size += sprintf(telem_json + json_size, ",\"espTemp\":%.2f",  \
            telem_data.espTemp);
    }
    json_size += sprintf(telem_json + json_size, "}");
    if (json_size > MAX_JSON_SIZE)
    {
        ESP_LOGW(TAG,"Telemetry data too large, concatenated from %d to %d bytes", json_size, MAX_JSON_SIZE);
        json_size = MAX_JSON_SIZE;
    }
    int msg_id = esp_mqtt_client_publish(mqtt_client, telem_topic, telem_json, json_size, 0, 0);
    ESP_LOGI(TAG, "telemetry sent with msg_id=%d", msg_id);
    memset(&telem_data, 0xFF, sizeof(telem_data));  // Reset telemetry object
}