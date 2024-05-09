#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static QueueHandle_t b_ag_uart_queue;
#define B_AG_UART_QUEUE_SIZE    10

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

const UartDeviceT b_ag_uart = {
    .port = UART_NUM_1,
#ifdef CONFIG_IDF_TARGET_ESP32C3
    .pin_rx = GPIO_NUM_18,
    .pin_tx = GPIO_NUM_19,
    .pin_rts = UART_PIN_NO_CHANGE,
    .pin_cts = UART_PIN_NO_CHANGE,
#else
    .pin_rx = GPIO_NUM_5,
    .pin_tx = GPIO_NUM_4,
    .pin_rts = UART_PIN_NO_CHANGE,
    .pin_cts = UART_PIN_NO_CHANGE,
#endif
    .rx_buffer_size = 1024,
    .tx_buffer_size = 0,
    .rx_timeout = 1
};

void uart_init(void) 
{
    const uart_config_t uart_config = 
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK( uart_driver_install(b_ag_uart.port, b_ag_uart.rx_buffer_size, b_ag_uart.tx_buffer_size, B_AG_UART_QUEUE_SIZE, &b_ag_uart_queue, 0) );
    ESP_ERROR_CHECK( uart_param_config(b_ag_uart.port, &uart_config) );
    ESP_ERROR_CHECK( uart_set_pin(b_ag_uart.port, b_ag_uart.pin_tx, b_ag_uart.pin_rx, b_ag_uart.pin_rts, b_ag_uart.pin_cts) );
    ESP_ERROR_CHECK( uart_set_mode(b_ag_uart.port, UART_MODE_UART) );
    ESP_ERROR_CHECK( uart_set_rx_timeout(b_ag_uart.port, b_ag_uart.rx_timeout) );
    uart_pattern_queue_reset(b_ag_uart.port, B_AG_UART_QUEUE_SIZE);
}