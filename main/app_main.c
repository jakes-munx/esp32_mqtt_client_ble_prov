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

#define CERTS_NAMESPACE "certs"
#define SEND_TELEMTRY_PERIOD_SECONDS    5
static const uint32_t telem_send_period_ms = SEND_TELEMTRY_PERIOD_SECONDS * 1000;

static esp_mqtt_client_handle_t mqtt_client = NULL;
static const char *TAG = "APP_MAIN";

static const char telem_topic[] = "/topic/qos0";

#define CERT_SIZE 2048

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t ca_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t ca_start[]   asm("_binary_gtsr1_pem_start");
// extern const uint8_t ca_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
#endif
extern const uint8_t ca_end[]   asm("_binary_gtsr1_pem_nd");
// extern const uint8_t ca_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

// extern const uint8_t client_cert_pem_start[]   asm("_binary_client_crt_start");
// extern const uint8_t client_key_pem_start[]   asm("_binary_client_key_start");
char client_crt_arr[CERT_SIZE] = {0xFF};
char client_key_arr[CERT_SIZE] = {0xFF};

static char telem_data = 'A';
static bool is_send_telem = false;

#define MAX_JSON_SIZE 128
static char telem_json[MAX_JSON_SIZE];

// #define BROKER_URI      "mqtts://mqtt.eclipseprojects.io:8883"
#define BROKER_URI      "wss://mqtt.munx.xyz:443"
#define BROKER_ADDRESS  "192.168.68.58"
#define BROKER_PORT     9001

const char broker_username[] = "user";
const char broker_password[] = "password";
const char client_id[] = "ESP32";

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
    uint16_t json_size = sprintf(telem_json, "{\"data\":\"%s\"}", &telem_data);
    if (json_size > MAX_JSON_SIZE)
    {
        ESP_LOGW(TAG,"Telemetry data too large, concatenated from %d to %d bytes", json_size, MAX_JSON_SIZE);
        json_size = MAX_JSON_SIZE;
    }
    int msg_id = esp_mqtt_client_publish(client, telem_topic, telem_json, json_size, 0, 0);
    ESP_LOGI(TAG, "telemetry sent with msg_id=%d", msg_id);
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
            send_telemetery(client);
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
    // const esp_partition_t *certs_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "certs");
    // #define CERT_SIZE 2048
    // if (certs_part->address == 0)
    // {
    //     ESP_LOGE(TAG, "Failed to find =certs partition");
    //     return;
    // }
    // ESP_LOGI(TAG, "[APP] Client cert addr : %p" , (void *)(certs_part->address));
    // ESP_LOGE(TAG, "[APP] Client cert : %lu" , *(uint32_t*)(0x9000));
    // // if ((*(uint32_t*)(0x9000) == 0x00000000) || (*(uint32_t*)(0x9000) == 0xFFFFFFFF))
    // // {
    //     ESP_LOGE(TAG, "[APP] No Client cert found: %lu" , *(uint32_t*)(0x9000));
    //     // return;
    // // }
    // if ((*(uint32_t*)(certs_part->address) == 0x00000000) || (*(uint32_t*)(certs_part->address) == 0xFFFFFFFF))
    // {
    //     ESP_LOGE(TAG, "[APP] No Client cert found: %lu" , *(uint32_t*)(certs_part->address));
    //     return;
    // }

    nvs_handle_t nvs_handle;
    esp_err_t err;
    static char sanity_string[12] = {0xff};
    // Open
    // err = nvs_open(CERTS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    nvs_flash_init_partition(CERTS_NAMESPACE);
    err = nvs_open_from_partition(CERTS_NAMESPACE, CERTS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to open NVS. Err: 0x%4X", err);
        return ;//err;
    }
        
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    err = nvs_get_str(nvs_handle, "sanity", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) 
    {
        ESP_LOGE(TAG, "Failed to get NVS sanity string");
        return ;//err;
    }
    printf("sanity length: %d\n", required_size);
    err = nvs_get_str(nvs_handle, "sanity", sanity_string, &required_size);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to load NVS sanity string");
        return ;//err;
    }
    printf("sanity string: %s\n", sanity_string);
    required_size = 0;
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_str(nvs_handle, "client_crt", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) 
    {
        ESP_LOGE(TAG, "Failed to get NVS client_crt blob: 0x%04X", err);
        return ;//err;
    }
    ESP_LOGI(TAG, "client key length: %d", required_size);
    if (required_size == 0) 
    {
        ESP_LOGE(TAG, "Partition empty!");
    } 
    else 
    {
        err = nvs_get_str(nvs_handle, "client_crt", client_crt_arr, &required_size);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to load NVS client_crt blob: 0x%04X", err);
            return ;//err;
        }
        ESP_LOGI(TAG, "Loaded NVS client_crt blob");
        ESP_LOGI(TAG, "%s", client_crt_arr);
    }
    required_size = 0;
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_str(nvs_handle, "client_key", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) 
    {
        ESP_LOGE(TAG, "Failed to get NVS client_key blob: 0x%04X", err);
        return ;//err;
    }
    printf("client key length: %d\n", required_size);
    if (required_size == 0) 
    {
        printf("Partition empty!\n");
    } 
    else 
    {
        err = nvs_get_str(nvs_handle, "client_key", client_key_arr, &required_size);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Failed to load NVS client_key blob: 0x%04X", err);
            return ;//err;
        }
        ESP_LOGI(TAG, "Loaded NVS client_key blob");
        ESP_LOGI(TAG, "%s", client_key_arr);
    }
    // Close
    nvs_close(nvs_handle);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = BROKER_URI,
            .verification.certificate = (const char *)ca_start
        },
        .credentials = {
            .authentication = {
                // .certificate = (const char *)client_cert_pem_start,
                // .key = (const char *)client_key_pem_start,
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
        if (telem_data < 'Z')
            telem_data++;
        else
            telem_data = 'A';
        send_telemetery(mqtt_client);
    }
}

// static void init_spiffs(void)
// {
//     ESP_LOGI(TAG, "Initializing SPIFFS");

//     esp_vfs_spiffs_conf_t conf = {
//       .base_path = "/spiffs",
//       .partition_label = NULL,
//       .max_files = 5,
//       .format_if_mount_failed = true
//     };

//     // Use settings defined above to initialize and mount SPIFFS filesystem.
//     // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
//     esp_err_t ret = esp_vfs_spiffs_register(&conf);

//     if (ret != ESP_OK) {
//         if (ret == ESP_FAIL) {
//             ESP_LOGE(TAG, "Failed to mount or format filesystem");
//         } else if (ret == ESP_ERR_NOT_FOUND) {
//             ESP_LOGE(TAG, "Failed to find SPIFFS partition");
//         } else {
//             ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
//         }
//         return;
//     }

// #ifdef CONFIG_EXAMPLE_SPIFFS_CHECK_ON_START
//     ESP_LOGI(TAG, "Performing SPIFFS_check().");
//     ret = esp_spiffs_check(conf.partition_label);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
//         return;
//     } else {
//         ESP_LOGI(TAG, "SPIFFS_check() successful");
//     }
// #endif

//     size_t total = 0, used = 0;
//     ret = esp_spiffs_info(conf.partition_label, &total, &used);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
//         esp_spiffs_format(conf.partition_label);
//         return;
//     } else {
//         ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
//     }

//     // Check consistency of reported partiton size info.
//     if (used > total) {
//         ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
//         ret = esp_spiffs_check(conf.partition_label);
//         // Could be also used to mend broken files, to clean unreferenced pages, etc.
//         // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
//             return;
//         } else {
//             ESP_LOGI(TAG, "SPIFFS_check() successful");
//         }
//     }

//     // Use POSIX and C standard library functions to work with files.
//     // First create a file.
//     ESP_LOGI(TAG, "Opening file");
//     FILE* f = fopen("/spiffs/hello.txt", "w");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for writing");
//         return;
//     }
//     fprintf(f, "Hello World!\n");
//     fclose(f);
//     ESP_LOGI(TAG, "File written");

//     // Check if destination file exists before renaming
//     struct stat st;
//     if (stat("/spiffs/foo.txt", &st) == 0) {
//         // Delete it if it exists
//         unlink("/spiffs/foo.txt");
//     }

//     // Rename original file
//     ESP_LOGI(TAG, "Renaming file");
//     if (rename("/spiffs/hello.txt", "/spiffs/foo.txt") != 0) {
//         ESP_LOGE(TAG, "Rename failed");
//         return;
//     }

//     // Open renamed file for reading
//     ESP_LOGI(TAG, "Reading file");
//     f = fopen("/spiffs/foo.txt", "r");
//     if (f == NULL) {
//         ESP_LOGE(TAG, "Failed to open file for reading");
//         return;
//     }
//     char line[64];
//     fgets(line, sizeof(line), f);
//     fclose(f);
//     // strip newline
//     char* pos = strchr(line, '\n');
//     if (pos) {
//         *pos = '\0';
//     }
//     ESP_LOGI(TAG, "Read from file: '%s'", line);

//     // All done, unmount partition and disable SPIFFS
//     esp_vfs_spiffs_unregister(conf.partition_label);
//     ESP_LOGI(TAG, "SPIFFS unmounted");
// }

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
#ifdef CONFIG_IDF_TARGET_ESP32C3
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
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

    uint8_t MacAddress[8];
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
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // ESP_ERROR_CHECK(example_connect());

    /* Attempt to connect to Wi-Fi, or else wait for user to provide credentials */
    ble_prov_wifi_init();

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
