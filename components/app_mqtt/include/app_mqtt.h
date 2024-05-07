#ifndef __APP_MQTT_H
#define __APP_MQTT_H

#define DATA_STRING_SIZE    10
typedef struct TelemDataT
{
    char  data[DATA_STRING_SIZE];   // Generic data string
    float espTemp;                  // Temperature read by the esp32

} TelemDataT;

uint16_t mqtt_init(const char *ca_crt, const char *client_crt, const char *client_key);
void set_mqtt_mac(uint8_t *mqtt_mac);
void buffer_telem_data(TelemDataT *data_to_send);
void send_telemetery(void);

#endif // __APP_MQTT_H