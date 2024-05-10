#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"

#include "app_uart.h"
#include "app_b_ag.h"

static const char *TAG = "APP_B_AG";

TaskHandle_t b_ag_cmd_task_handle = NULL;

typedef void (*P_MESSAGEHANDLEFUNC)(BAgMsgT * const msg);

typedef struct CommsMessage
{
	uint8_t const type;
	P_MESSAGEHANDLEFUNC const handler;
} CommsMessageT;

typedef struct SWversionT
{
    uint8_t major;
    uint8_t minor;
    uint8_t build;
} SWversionT;
typedef struct EnergizerDataT
{
    SWversionT      sw_version;
    uint8_t         hw_version;
    SolarModelInfoT unit_type;
    uint16_t        temp;
    uint8_t         sw_pos;
} EnergizerDataT;
EnergizerDataT energizer_data = {0};

typedef struct EnergizerQueriesT
{
    const char      *p_data_name;
    BAgCommandT     command_type;
    uint8_t         command;
    uint8_t         parameter;
    uint8_t         n_args;
} EnergizerQueriesT;
static EnergizerQueriesT const energizer_queries[] = {
    {.p_data_name = "SWvers",   .command_type = bacVERSION,                 .command = vspSW_VERSION,       .parameter = 0, .n_args = 1},
    {.p_data_name = "HWvers",   .command_type = bacVERSION,                 .command = vspHW_VERSION,       .parameter = 0, .n_args = 1},
    {.p_data_name = "UnitType", .command_type = bacUNIT_TYPE,               .command = vspSW_VERSION,       .parameter = 0, .n_args = 0},
    {.p_data_name = "EngTemp",  .command_type = bacDIAGNOSTICS,             .command = dcINT_TEMPERATURE,   .parameter = 0, .n_args = 2},
    {.p_data_name = "SwPos",    .command_type = bacAUTOMATION_DIAGNOSTICS,  .command = adrFRONT_SW_POS,     .parameter = 0, .n_args = 1},
};
#define NUM_OF_QUERIES		    (sizeof(energizer_queries)/sizeof(energizer_queries[0]))
#define MAX_QUERY_PAYLOAD_SIZE  3

// Functions and structures to handle Get/Set and diagnostics data.
static void ProcessVersion(BAgMsgT * const msg);
static void ProcessUnitType(BAgMsgT * const msg);
static void ProcessGet(BAgMsgT * const msg);
static void ProcessDiagnostics(BAgMsgT * const msg);
static void ProcessAutomationDiagnostics(BAgMsgT * const msg);

static CommsMessageT const command_messages[] = {
	{(uint8_t)bacVERSION,					ProcessVersion},
	{(uint8_t)bacUNIT_TYPE,					ProcessUnitType},
	{(uint8_t)bacGET,						ProcessGet},
    {(uint8_t)bacDIAGNOSTICS,				ProcessDiagnostics},
    {(uint8_t)bacAUTOMATION_DIAGNOSTICS,	ProcessAutomationDiagnostics},
};
#define NUM_OF_COMMAND_TYPESS		(sizeof(command_messages)/sizeof(command_messages[0]))

#define BAG_SUB_PKT_TYPE_INDEX  1
#define BAG_MSG_BYTE_VAL_INDEX  2

BAgByteErrT ProcessRxBAgString(uint8_t * rx_buf, uint8_t rx_len)
{
    BAgByteErrT err = errNOERROR;
    static BAgMsgT BAgMsg = {0};

    if ((rx_len < MIN_MSG_LEN) || (rx_len > MAX_MSG_LEN))
    {
        ESP_LOGE(TAG, "BAg message too %s: %d", (rx_len< MIN_MSG_LEN) ? "short" : "long", rx_len);
        return err = errPKT;
    }
    if ((rx_buf[0] != STX) || (rx_buf[rx_len - 1] != ETX)) // Bag msg not wrapped in STX [] ETX
    {
        return err = errPKT;
    }

    char prev_byte = 0;

    BAgMsg.msg_complete = false;
    BAgMsg.len = 0;
    BAgMsg.chksum = 0;
    for (uint8_t bg_index = 1; bg_index < (rx_len-1); bg_index++)   // loop through payload between STX [] ETX
    {
        if(rx_buf[bg_index] == DLE && prev_byte != DLE)
        {
            prev_byte = rx_buf[bg_index];       // DLE recevied and prev byte was not a DLE
        }                                       // current byte is an inserted DLE.
        else
        {
            if (rx_buf[bg_index] == ETX && prev_byte != DLE)
            {              // End of message 
                if (BAgMsg.chksum == 0) 
                {
                    BAgMsg.len--;  // Checksum passed
                    BAgMsg.msg_complete = true;
                }
                else 
                {
                    return err = errCHKSUM; // checksum failed
                }
            }
            else
            {
                // prev_byte = 0;
                BAgMsg.chksum += rx_buf[bg_index];
                BAgMsg.buf[BAgMsg.len++] = rx_buf[bg_index];
            }
        }
    }

    uint8_t command = BAgMsg.buf[0];
    uint8_t index = 0;
    while(index< NUM_OF_COMMAND_TYPESS)
    {
        if(command == command_messages[index].type)
        {
           command_messages[index].handler(&BAgMsg);	//processes the packet 
           break;
        }
        ++index;
    }
    return err;
}

void ProcessVersion(BAgMsgT * const msg)
{
    switch(msg->buf[BAG_SUB_PKT_TYPE_INDEX]) // sub packet
    {
        case vspBUSPROTOCOL:    // protocol version.
                
        break;
        case vspSW_VERSION:     // software version.
            energizer_data.sw_version.major = msg->buf[BAG_MSG_BYTE_VAL_INDEX];
            energizer_data.sw_version.minor = msg->buf[BAG_MSG_BYTE_VAL_INDEX+1];
            energizer_data.sw_version.build = msg->buf[BAG_MSG_BYTE_VAL_INDEX+2];
            ESP_LOGI(TAG, "sw version %d.%d.%d", energizer_data.sw_version.major, energizer_data.sw_version.minor, energizer_data.sw_version.build);
        break;
        case vspHW_VERSION:     // Hardware version.
            energizer_data.hw_version = msg->buf[BAG_MSG_BYTE_VAL_INDEX];      
            ESP_LOGI(TAG, "hw version %d", energizer_data.hw_version);
        break;
    }		
}

void ProcessUnitType(BAgMsgT * const msg)
{
	if ((msg->buf[BAG_SUB_PKT_TYPE_INDEX] >= smiS20) && (msg->buf[BAG_SUB_PKT_TYPE_INDEX] < smiMAX))
    {
        energizer_data.unit_type = msg->buf[BAG_SUB_PKT_TYPE_INDEX];
        ESP_LOGI(TAG, "Unit type 0x%02X", energizer_data.unit_type);
    }
}

void ProcessGet(BAgMsgT * const msg)
{
    switch(msg->buf[BAG_SUB_PKT_TYPE_INDEX]) // sub packet
    {
        default:
            ESP_LOGI(TAG, "Unhandled GET: %d value: %d",msg->buf[BAG_MSG_BYTE_VAL_INDEX], msg->buf[BAG_MSG_BYTE_VAL_INDEX]);
        break;
    }
}

static void ProcessDiagnostics(BAgMsgT * const msg)
{
    switch(msg->buf[BAG_SUB_PKT_TYPE_INDEX]) // sub packet
    {
        case dcINT_TEMPERATURE:
            energizer_data.temp = (msg->buf[BAG_MSG_BYTE_VAL_INDEX]<<8)+(msg->buf[BAG_MSG_BYTE_VAL_INDEX+1]);       
            ESP_LOGI(TAG, "Temperature %d", energizer_data.temp);
        break;
        default:
            ESP_LOGI(TAG, "Unhandled DIAGNOSITC: %d value: %d",msg->buf[BAG_SUB_PKT_TYPE_INDEX], msg->buf[BAG_MSG_BYTE_VAL_INDEX]);
        break;
    }
}
static void ProcessAutomationDiagnostics(BAgMsgT * const msg)
{
    switch(msg->buf[BAG_SUB_PKT_TYPE_INDEX]) // sub packet
    {
        case adrFRONT_SW_POS:
            energizer_data.sw_pos = (msg->buf[BAG_MSG_BYTE_VAL_INDEX]<<8)+(msg->buf[BAG_MSG_BYTE_VAL_INDEX+1]);    
            ESP_LOGI(TAG, "Switch position %d", energizer_data.sw_pos);
        break;
        default:
            ESP_LOGI(TAG, "Unhandled AUTOMATION DIAGNOSITC: %d value: %d",msg->buf[BAG_SUB_PKT_TYPE_INDEX], msg->buf[BAG_MSG_BYTE_VAL_INDEX]);
        break;
    }
}

//--------------------------------------------------------------------SendBAg()
//Description:	packs the tx data buffer in BAg format and places into 
//				the UART buffer.
//Parameters:	uint8_t const *  data buffer
//				uint16_t length of the buffer.
//Returns:		none
//-----------------------------------------------------------------------------
void SendBAg(uint8_t const * tx_data, uint16_t len)
{
    char buffer[MAX_MSG_LEN];
    uint8_t index = 0;
    uint8_t check = 0;

    //Add STX
    buffer[(index++)] = STX;
            
    while(len--)
    {
        check = (uint8_t)(check - *tx_data);    // Decrement checksum from 0, so resolved checksum should always equal zero
        if((DLE == *tx_data || STX == *tx_data || ETX == *tx_data) && index < MAX_MSG_LEN)  // Preceed DLE,STX,ETX with a DLE
        {
            buffer[(index++)] =  DLE;
        }
        if(index < MAX_MSG_LEN)
        {
            buffer[(index++)] = *tx_data++;
        }
    }
    if((DLE == check || STX == check || ETX == check) && index < MAX_MSG_LEN) // if checksum happens to be DLE,STX,ETX; preceed with a DLE
    {
        buffer[(index++)] = DLE;
    }
    if(index < MAX_MSG_LEN)
    {
        buffer[(index++)] = check;
    }
    if(index < MAX_MSG_LEN)
    {
        buffer[(index++)] = ETX;    // End of message char
        send_uart_data(buffer,(uint16_t)index);         
    }
}

static void b_ag_cmd_task(void *arg)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
    BaseType_t xResult;

    ESP_LOGI(TAG, "Starting UART TX task");
    static uint8_t tx_data[MAX_QUERY_PAYLOAD_SIZE];
    while (1) 
    {
        ESP_LOGI(TAG, "Sending BAg commands");
        for (uint8_t query_index = 0; query_index < NUM_OF_QUERIES; query_index++)
        {
            ESP_LOGD(TAG, "Sending query: %s", energizer_queries[query_index].p_data_name);
            tx_data[0] = energizer_queries[query_index].command_type;
            tx_data[1] = energizer_queries[query_index].command;
            tx_data[2] = energizer_queries[query_index].parameter;

            SendBAg(tx_data, energizer_queries[query_index].n_args + 1);

            xResult = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );
            if (xResult == 0)   // Timeout, retry once
            {
                ESP_LOGW(TAG, "BAg command timedout, retrying..");
                SendBAg(tx_data, energizer_queries[query_index].n_args + 1);
                xResult = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );
                if (xResult == 0)   // Retry timedout, stop BAg queries
                {
                    ESP_LOGE(TAG, "BAg command failed. Stopping.");
                    break;
                }
            }
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void b_ag_init(void)
{
    uart_init();

    xTaskCreate(b_ag_cmd_task, "b_ag_cmd_task", 1024*2, NULL, configMAX_PRIORITIES-1, &b_ag_cmd_task_handle);
}