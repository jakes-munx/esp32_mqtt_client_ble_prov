#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "app_b_ag.h"

static QueueHandle_t b_ag_uart_queue;
#define B_AG_UART_QUEUE_SIZE    10
#define MAX_BAG_UART_BUF_SIZE   256

typedef struct UartDeviceT
{
    const uart_port_t port;
    const uint8_t pin_rx;
    const uint8_t pin_tx;
    const uint8_t pin_cts;
    const uint8_t pin_rts;
    uint16_t rx_buffer_size;
    uint16_t tx_buffer_size;
    uint16_t rx_timeout;
} UartDeviceT;

static const UartDeviceT b_ag_uart = {
    .port = UART_NUM_1,
#ifdef CONFIG_IDF_TARGET_ESP32C3
    .pin_rx = GPIO_NUM_18,
    .pin_tx = GPIO_NUM_19,
    .pin_rts = 0,
    .pin_cts = 0,
#else
    .pin_rx = GPIO_NUM_5,
    .pin_tx = GPIO_NUM_4,
    .pin_rts = 0,
    .pin_cts = 0,
#endif
    .rx_buffer_size = MAX_BAG_UART_BUF_SIZE,
    .tx_buffer_size = 0,
    .rx_timeout = 1
};

int send_uart_data(const char* data, uint8_t data_len)
{
    const int txBytes = uart_write_bytes(b_ag_uart.port, data, data_len);
    return txBytes;
}

static void rx_task(void *arg)
{
    uart_event_t event;
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_VERBOSE);
    ESP_LOGI(RX_TASK_TAG, "Starting UART RX task");
    while (1) 
    {
        if (xQueueReceive(b_ag_uart_queue, (void *)&event, portMAX_DELAY)) 
        {
            ESP_LOGV(RX_TASK_TAG, "uart[%d] payload size: %d", b_ag_uart.port, event.size);
            switch (event.type) 
            {
                case UART_DATA: 
                {
                    static uint8_t uart_rx_buffer[MAX_BAG_UART_BUF_SIZE];
                    size_t length = uart_read_bytes(b_ag_uart.port, uart_rx_buffer, event.size, 0);//TASK_WATCHDOG_RESET_INTERVAL);
                    ESP_LOGD(RX_TASK_TAG, "Read %d bytes", length);
                    ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, uart_rx_buffer, length, ESP_LOG_VERBOSE);
                    ProcessRxBAgString(uart_rx_buffer, length);
                    xTaskNotifyGive( b_ag_cmd_task_handle);
                    break;
                }
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGD(RX_TASK_TAG, "uart flush event type: %d", event.type);
                    uart_flush_input(b_ag_uart.port);
                    xQueueReset(b_ag_uart_queue);
                    break;
                case UART_BREAK:
                case UART_PARITY_ERR:
                case UART_FRAME_ERR:
                default:
                    ESP_LOGD(RX_TASK_TAG, "uart error type: %d", event.type);
                    break;
            }
        }
    }
    // free(data);
}

void uart_init(void) 
{
    const uart_config_t uart_config = 
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK( uart_driver_install(b_ag_uart.port, b_ag_uart.rx_buffer_size, b_ag_uart.tx_buffer_size, B_AG_UART_QUEUE_SIZE, &b_ag_uart_queue, 0) );
    ESP_ERROR_CHECK( uart_param_config(b_ag_uart.port, &uart_config) );
    ESP_ERROR_CHECK( uart_set_pin(b_ag_uart.port, b_ag_uart.pin_tx, b_ag_uart.pin_rx, b_ag_uart.pin_rts, b_ag_uart.pin_cts) );
    ESP_ERROR_CHECK( uart_set_mode(b_ag_uart.port, UART_MODE_UART) );
    ESP_ERROR_CHECK( uart_set_rx_timeout(b_ag_uart.port, b_ag_uart.rx_timeout) );
    uart_pattern_queue_reset(b_ag_uart.port, B_AG_UART_QUEUE_SIZE);

    TaskHandle_t rx_task_handle = NULL;
    // TaskHandle_t tx_task_handle = NULL;

    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, &rx_task_handle);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, &tx_task_handle);
}