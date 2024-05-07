/* MQTT over SSL Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "nvs.h"

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

#ifdef CONFIG_IDF_TARGET_ESP32C3
#include "driver/temperature_sensor.h"
#endif

#include "app_ble.h"
#include "app_time.h"

#define FOREACH(item, array) \
    for (int keep = 1, \
             count = 0,\
             size = sizeof (array) / sizeof *(array); \
         keep && count != size; \
         keep = !keep, count++) \
            for (item = (array) + count; keep; keep = !keep) 

#define SEND_TELEM_MAC  1

#define CERTS_NAMESPACE "certs"
#define SEND_TELEMTRY_PERIOD_SECONDS    5
static const uint32_t telem_send_period_ms = SEND_TELEMTRY_PERIOD_SECONDS * 1000;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "APP_MAIN";

static const char telem_topic[] = "/topic/qos0";

#define CA_CERT_SIZE        1913
#define CLIENT_CERT_SIZE    1350
#define CLIENT_KEY_SIZE     1734

char ca_crt_arr[CA_CERT_SIZE]           = {0xFF};
char client_crt_arr[CLIENT_CERT_SIZE]   = {0xFF};
char client_key_arr[CLIENT_KEY_SIZE]    = {0xFF};

char sanity_arr[12] = {0xFF};

static uint8_t MacAddress[8];

temperature_sensor_handle_t temp_sensor = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);

typedef struct DeviceConfigT
{
    const char      *p_name;
    char            *p_arr;
    uint16_t        size;
} DeviceConfigT;

DeviceConfigT device_configs[] = {
    { .p_name = "client_crt",   .p_arr = client_crt_arr, .size = CLIENT_CERT_SIZE},
    { .p_name = "client_key",   .p_arr = client_key_arr, .size = CLIENT_KEY_SIZE},
    { .p_name = "ca_crt",       .p_arr = ca_crt_arr,     .size = CA_CERT_SIZE},
    { .p_name = "sanity",       .p_arr = sanity_arr,     .size = 12}
};

typedef struct TelemDataT
{
    char  data[10];         // Generic data string
    float espTemp;          // Temperature read by the esp32

} TelemDataT;

TelemDataT telem_data = {0};
static bool is_send_telem = false;

#define MAX_JSON_SIZE 128
static char telem_json[MAX_JSON_SIZE];

#define BROKER_URI      "wss://mqtt.munx.xyz:443"

SemaphoreHandle_t sem_telem;

// User handler for Watchdog Timer:
// If the watchdog timer is triggered, reboot.
extern void esp_task_wdt_isr_user_handler(void)
{
    abort();
}

void TelemTimerCallback( TimerHandle_t xTimer )
{
    is_send_telem = true;
}

static void send_telemetery(esp_mqtt_client_handle_t client)
{
    time_t unix_time = GetUnixTime();
    uint16_t json_size = sprintf(telem_json, "{");
#if SEND_TELEM_MAC
    json_size += sprintf(telem_json + json_size, "\"MAC\":\"%02X%02X%02X%02X%02X%02X\"",  \
        MacAddress[0],MacAddress[1],MacAddress[2],MacAddress[3],MacAddress[4],MacAddress[5]);
#endif
    json_size += sprintf(telem_json + json_size, ",\"time\":%lld",  \
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
    int msg_id = esp_mqtt_client_publish(client, telem_topic, telem_json, json_size, 0, 0);
    ESP_LOGI(TAG, "telemetry sent with msg_id=%d", msg_id);
    memset(&telem_data, 0xFF, sizeof(telem_data));  // Reset telemetry object
}

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
                send_telemetery(client);
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

static void mqtt_app_start(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    // Open
    nvs_flash_init_partition(CERTS_NAMESPACE);

    err = nvs_open_from_partition(CERTS_NAMESPACE, CERTS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to open NVS. Err: 0x%4X", err);
        return ;//err;
    }
    
    FOREACH (DeviceConfigT *config, device_configs)
    {
        ESP_LOGV(TAG, "NVS reading: %s", config->p_name);
        size_t required_size = 0;  // value will default to 0, if not set yet in NVS
        err = nvs_get_str(nvs_handle, config->p_name, NULL, &required_size);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) 
        {
            ESP_LOGE(TAG, "Failed to find NVS string %s. Err: 0x%04X", config->p_name, err);
            break;
        }
        ESP_LOGV(TAG, "%s length: %d", config->p_name, required_size);
        if (required_size > config->size)
        {
            ESP_LOGE(TAG, "NVS string %s expects maximum %d bytes, but is %d bytes", config->p_name, config->size, required_size);
            break;
        }
        err = nvs_get_str(nvs_handle, config->p_name, config->p_arr, &required_size);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to load NVS string %s. Err: 0x%04X", config->p_name, err);
            break;
        }
        ESP_LOGD(TAG, "%s", config->p_arr);
    }

    nvs_close(nvs_handle);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = BROKER_URI,
            .verification.certificate = (const char *)ca_crt_arr
        },
        .credentials = {
            .authentication = {
                .certificate = (const char *)client_crt_arr,
                .key = (const char *)client_key_arr
            }
        }
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void TelemTask(void *arg)
{
    const TickType_t xDelay_ticks = pdMS_TO_TICKS( telem_send_period_ms );
    ESP_LOGD(TAG, "Telem task entered");
    while (1)
    {
        xSemaphoreTake( sem_telem, xDelay_ticks);
        ESP_LOGD(TAG, "Sending data");
        // ESP_ERROR_CHECK( esp_task_wdt_reset() );
        // memset(&telem_data, 0xFF, sizeof(telem_data));
#ifdef CONFIG_IDF_TARGET_ESP32C3
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &telem_data.espTemp));
        ESP_LOGI(TAG, "Temperature value %.02f ℃", telem_data.espTemp);
#else
        if ((telem_data.data[0] >= 'A') && (telem_data.data[0] < 'Z'))
            telem_data.data[0]++;
        else
            telem_data.data[0]= 'A';
#endif
        send_telemetery(mqtt_client);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
#ifdef CONFIG_IDF_TARGET_ESP32C3
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
    float tsens_value;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));
    ESP_LOGI(TAG, "Temperature value %.02f ℃", tsens_value);
#endif

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("APP_MAIN", ESP_LOG_VERBOSE);
    esp_log_level_set("APP_BLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

	if (esp_efuse_mac_get_default(MacAddress) != ESP_OK)
    {
		ESP_LOGI(TAG, "Unable to read MAC address");
    }
	else
    {
		ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", \
            (uint16_t)MacAddress[0], (uint16_t)MacAddress[1], (uint16_t)MacAddress[2], (uint16_t)MacAddress[3], \
            (uint16_t)MacAddress[4], (uint16_t)MacAddress[5], (uint16_t)MacAddress[6], (uint16_t)MacAddress[7]);
    }

    /* Attempt to connect to Wi-Fi, or else wait for user to provide credentials */
    ble_prov_wifi_init();

    InitTime();

    mqtt_app_start();

    // esp_task_wdt_config_t wdt_config = {.timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true};
    // ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));

    ESP_LOGI(TAG, "***Creating resources..");
    sem_telem = xSemaphoreCreateBinary();
    if( sem_telem == NULL )
    {
        ESP_LOGE(TAG, "semaphore creation failed");
    }
    TaskHandle_t telem_task_handle = NULL;
    xTaskCreate(TelemTask, "TelemTask", 2024, NULL, 1, &telem_task_handle);


    // const BaseType_t APP_CORE = 1;
    // xTaskCreatePinnedToCore(
    //         TelemTask,
    //        "TelemTask",
    //         configMINIMAL_STACK_SIZE,
    //         NULL,
    //         1,
    //         telem_task_handle,
    //         APP_CORE
    //     );
    // ESP_ERROR_CHECK(esp_task_wdt_add(telem_task_handle));

}
