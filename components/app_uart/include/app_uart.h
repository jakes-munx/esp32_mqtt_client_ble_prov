#ifndef __APP_UART_H
#define __APP_UART_H

int send_uart_data(const char* data, uint8_t data_len);
void uart_init(void);

#endif // __APP_UART_H