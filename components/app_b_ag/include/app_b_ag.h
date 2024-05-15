#ifndef __APP_B_AG_H
#define __APP_B_AG_H

extern TaskHandle_t b_ag_cmd_task_handle;
extern TaskHandle_t telem_task_handle;
extern SemaphoreHandle_t sem_data_ready;
extern SemaphoreHandle_t sem_uart_rx;

#define STX (uint8_t)(0x02)
#define ETX (uint8_t)(0x03)
#define DLE (uint8_t)(0x10)

#define MAX_MSG_LEN     32
#define MIN_MSG_LEN     5

#define GET_ENG_INFO    1
#define GET_ENG_DATA    2

#define ENG_INFO_READY   1
#define ENG_TELEM_READY   2

typedef enum
{
    bacVERSION = 0x00,
    bacUNIT_TYPE,
    bacGET,
    bacSET,
    bacDIAGNOSTICS,    
    bacDATALOGDOWNLOAD,
    bacAUTOMATION_CTRL_SET,
    bacAUTOMATION_CTRL_GET,
    bacAUTOMATION_DIAGNOSTICS,
	bacOUT_OF_RANGE = 0xFC,
    bacBAD_DATA	= 0xFD,
    bacBAD_LENGTH = 0xFE,
    bacBAD_COMMAND = 0xFF,
} BAgCommandT;

typedef enum
{
    smiS6        = 0x60,  
    smiS12       = 0x61,
    smiS10x      = 0x62,
    smiS20       = 0x68,
    smiS30       = 0x69,
    smiS20x      = 0x6A,
    smiS40       = 0x6B,
    smiS60       = 0x70,
    smiS80       = 0x71,
    smiS60x      = 0x72,  
    smiMAX       = smiS60x,
} SolarModelInfoT;

typedef enum
{
    vspBUSPROTOCOL,
    vspSW_VERSION,
    vspHW_VERSION,
} VersionSubPacketT;

typedef enum
{
    // Added for MidSolar, SmallSolar diverges after gscSWITCH_COUNT
    gscDACVOLTAGE = 0x00,               // 0x00-Set Dac voltage for testjig tuning
    gscADJ_DAC_VOLT_MODEL,              // 0x01-Set Adjustable max DAC for each model.
    gscBATT_VOLT_CAL_VAL,               // 0x02-battery voltage cal
    gscSOLAR_VOLT_CAL_VAL,              // 0x03-solar voltage cal
    gscSHORT_CKT_CURR_CAL_VAL,          // 0x04-short circuit current cal	
    gscLED_CTRL,                        // 0x05-LED Control 0x00 - All Led off 
                                        //			   0x01 - Battery Green led on
                                        //			   0x02 - Battery Red Led on
                                        //			   0x04 - Fence Green led on
                                        //			   0x08 - Fence Red led on
                                        //			   0x0F - All Led on 
    gscTEMP_CAL_VALUE,                  // 0x06-Temperature calibration Value								   
    gscLIGHT_SNSR_CAL_VALUE,            // 0x07-Light Sensor Calibration value
    gscLIGHT_SNSR_THRESHOLD,            // 0x08-light sensor threshold to define day/night
    gscBATT_CHRGE_CONTROL,              // 0x09-Battery Charge Control   0x00 - Clear Battery Charge control 
                                        //                          0x01 - Start Battery float charging
                                        //                          0x02 - Stop Battery  float Charging
    gscOP_MODE_CONTROL,                 // 0x0A-Unit's Operating control mode	0xA5 - Enter stop mode indefinitely
    gscFIRE_ONCE_AND_STOP,              // 0x0B-Set the unit to fire one pulse in test jig mode. 0x01-0x03=ch - Send one pulse and wait.
    gscSERIAL_NUMBER,                   // 0x0C-Write Serial number into flash.
    gscMODEL_INFO,                      // 0x0D-write the model number into flash    
    gscSAVE_CONFIG,					    // 0x0E-Saves all configuration into flash.
    gscSWITCH_COUNT,                    // 0x0F-writes the number of switches present in the unit.
    gscPRI_SNS_MEAS_DELAY,              // 0x10-Delay added to measure the primary sense voltage after a fence pulse.
    gscLOAD_SNS_CAL_VAL,                // 0x11-Load Sense Calibration Value.
    gscLOAD_SNS_LED_THRESHOLD,          // 0x12-Load Sense LED threshold 
    gscUNDR_SWING_COMP_FACTOR,          // 0x13-fence pulse Under swing compensation factor for LOAD Calculation.
    gscMULTI_STAGE_PWM_ON_TIME,         // 0x14-MultiStage On time. Sent with stage index and on time
    gscMULTI_STAGE_PWM_OFF_TIME,        // 0x15-MultiStage Off time. Sent with stage index and off time
    gscMULTI_STAGE_CHRG_PERIOD,         // 0x16-charge control period for N-1 stages. Last stage will be targeting DAC threshold
    gscTEST_MULTI_STAGE_CONFIG_PARAMS,  // 0x17-Boolean indicating whether to utilize the test parameters for multistage charge.
    gscSP_CHRG_CURR_CAL_VAL,            // 0x18-Solar panel charge current calibration value.
    gscDC_REG_VOLT_IN_CAL_VAL,          // 0x19-DC Regulator Voltage input calibration value
    gscDC_REG_VOLT_IN_2_CAL_VAL,        // 0x1A-DC Regulator Voltage input 2 calibration value   
    gscDC_REG_CHRG_CURR_OFFSET_CAL_VAL, // 0x1B-DC regulator charge current calibration value
    gscFIELD_TESTING_ENABLED,           // 0x1C-Enable serial comms for field testing.
    gscTEMP_COMP_ISC_ADJ_THRES,         // 0x1D-Sets the short ckt current threshold above which temp comp is adjusted
    gscTEMP_COMP_OFFSET,                // 0x1E-Cell temp compensation offset

    // Added for B80
    gscBATTERY_LED_THRESHOLD,           // 0x1F-Battery LED change colour to red threshold

    // Added for MiniSolar
    gscCONSOLE_DEBUG_LEVEL,             // 0X20-bit 0: FSMs, bit 1: timings, bit 2: informational, bit 3: brief charging stats, bit 4: verbose charging stats, bit 8: vary temperature
    gscSTANDBY_MINUTES,                 // 0X21-Number of minutes when in standby mode that routine solar checks will occur
    gscEXIT_FLAT_CELL_CHG_MINUTES,      // 0X22-Number of minutes when in Flat cell state, with daylight successful charging, before pulsing restarts
    gscEXIT_FLAT_CELL_MAX_DAY_MINUTES,  // 0X23-Number of minutes when in Flat cell state, with daylight available, before pulsing restarts
    gscV_DROP_MV_SIGNIFICANT_CHG,       // 0X24-Amount of V_Drop_mV that is considered significant for Flat cell recovery
    gscINSUF_CHG_TO_STANDBY_MINUTES,    // 0X25-Number of minutes when insufficient solar is available, before standby mode starts
    gscFORCE_VREF_SELF_CAL,             // 0X26-Force the current Vref voltage into config (2=auto, 1=set, 0=reset)
    gscV_DROP_MV_WAKEUP_CHG,            // 0X27-Amount of V_Drop_mV for deciding if it is worth waking up from standby mode
    gscV_DROP_MV_INSUF_CHG,             // 0X28-Amount of V_Drop_mV for deciding that there has been insufficient charge for 5 days
    gscENTER_RECOVERY_IF_LESS_THAN_MV,  // 0X29-Max cell level for entering the dead cell recovery state after brownout reset
    gscLEAVE_RECOVERY_IF_MORE_THAN_MV,  // 0X2A-Min cell level for leaving the dead cell recovery state to resume normal operation
    gscWC70_CELL_LOWER_LIMIT_MV,        // 0X2B-WC70 cell mV lower limit for testing cell voltage
    gscWC70_CELL_UPPER_LIMIT_MV,        // 0X2C-WC70 cell mV upper limit for testing cell voltage
    gscHOUR_MINUTES,                    // 0X2D-Number of minutes in an hour, to allow alg speedup
    gscSOC_ABOVE_ENERGY_THRESH_HOURS,   // 0X2E-Number of hours that SOC needs to be above Low (30%) to allow full energy
    
    gscCOMMAND_MAX                      // INSERT new commands before here, do not alter existing commands
} GetSetCommandT;

typedef enum
{
    // Added for MidSolar, not present in SmallSolar
    accAUTOMATION_ENABLED,              // 0x00-Automation Enabled (set to 2)
    accINTERNAL_TEMP_OVERRIDE,          // 0x01-internal temperature adc override flag
    accINTERNAL_TEMP_ADC,               // 0x02-internal temperature adc override value
    accBATT_VOLTAGE_OVERRIDE,           // 0x03-battery voltage adc override flag
    accBATT_VOLTAGE_SHORT_CIRCUIT_ADC,  // 0x04-battery voltage short circuit adc override value
    accBATT_VOLTAGE_OPEN_CIRCUIT_ADC,   // 0x05-battery voltage open circuit adc override value
    accSOLAR_VOLTAGE_OVERRIDE,          // 0x06-solar panel voltage override flag
    accSOLAR_VOLTAGE_ADC,               // 0x07-solar panel voltage override value (short circuit for MiniSolar)
    accSOLAR_CURRENT_OVERRIDE,          // 0x08-solar panel short circuit current override flag
    accSOLAR_CURRENT_SHORT_CIRCUIT_ADC, // 0x09-solar panel short circuit current override value
    accDC_REG_VIN_OVERRIDE,             // 0x0A-dc regulator VIN override flag
    accDC_REG_VIN_ADC,                  // 0x0B-dc regulator VIN override value
    accDC_REG_VIN2_OVERRIDE,            // 0x0C-dc regulator VIN2  override flag
    accDC_REG_VIN2_ADC,                 // 0x0D-dc regulator VIN2 override value
    accLIGHT_SENSOR_OVERRIDE,           // 0x0E-light sensor adc override flag
    accLIGHT_SENSOR_ADC,                // 0x0F-light sensor adc override value
    accPRIMARY_SENSE_OVERRIDE,          // 0x10-primary sense override flag
    accPRIMARY_SENSE_ADC,               // 0x11-primary sense override value
    accSECONDARY_SENSE_OVERRIDE,        // 0x12-secondary sense override flag
    accSECONDARY_SENSE_ADC,             // 0x13-secondary sense override value
    accUNDERSWING_SENSE_OVERRIDE,       // 0x14-underswing sense override flag
    accUNDERSWING_SENSE_ADC,            // 0x15-underswing sense override value
    accDC_REG_OUT_OVERRIDE,             // 0x16-dc reg out override flag
    accDC_REG_OUT_ADC,                  // 0x17-dc reg out override adc
    accBATT_PULSE_RATE_OVERRIDE,        // 0x18-batt pulse rate override flag
    accBATT_PULSE_RATE_VALUE,           // 0x19-batt pulse rate override value
    accTARGET_CV_DAC_OVERRIDE,          // 0x1A-target cv dac override flag
    accTARGET_CV_DAC_VALUE,             // 0x1B-target cv dac override value
    accMAGNETIC_HALL_SENSE_OVERRIDE,    // 0x1C-magnetic hall sense override flag
    accMAGNETIC_HALL_SENSE_VALUE,       // 0x1D-magnetic hall sense override value - where 0b 0000 0000 denotes the hall sense
    accBATT_MONITOR_TIMER_INC_OVERRIDE, // 0x1E-battery monitor timer increment override flag
    accBATT_MONITOR_TIMER_INC_VALUE,    // 0x1F-battery monitor timer increment override value
    accDISABLE_SLEEP_MODE,              // 0x20-disables sleep mode when in automation.
    accDISABLE_BATT_MANAGEMENT,         // 0x21-disables battery management control
    accTEST_WATCHDOG,                   // 0x22-tests watchdog - will put the unit into a while loop until the software watchdog reset it

    // Added for B80
    accCV_MEASURE_1_OVERRIDE,           // 0x23-cv measure 1 override flag
    accCV_MEASURE_1_ADC,                // 0x24-cv measure 1 override value
    accCV_MEASURE_2_OVERRIDE,           // 0x25-cv measure 2 override flag
    accCV_MEASURE_2_ADC,                // 0x26-cv measure 2 override value
    accSWITCH_POS_OVERRIDE,             // 0x27-switch pos adc override flag
    accSWITCH_POS_ADC,                  // 0x28-switch pos adc override value
    accENERGY_LEVEL_INDEX_OVERRIDE,     // 0x29-energy level override flag
    accENERGY_LEVEL_INDEX_VALUE,        // 0x2A-energy level override value
    accDEV_1_SIGNAL_OUTPUT,             // 0x2B-the signal to mirror out to DEV1 pin (0-9)
    accDEV_2_SIGNAL_OUTPUT,             // 0x2C-the signal to mirror out to DEV2 pin (0-9)

    // Added for MiniSolar
    accSOLAR_VOLTAGE_OPEN_CIRCUIT_ADC,  // 0x2D-solar panel voltage open circuit override value
    accCELL_CHARGE_FET_TESTJIG_OVERRIDE,// 0x2E-Cell Charge FET testjig override (100 = off, 101 = on)
    accTARGET_CV_TESTJIG_OVERRIDE_V,    // 0x2F-Target CV to use in WC40 override (volts)
    accDISABLE_SUB_CHARGE_INTERVALS,    // 0x30-Set to 1 to disable the sub charge intervals, 0 to restore
    accFORCE_REDUCED_ENERGY_MODE,       // 0x31-Set to 1 to force reduced energy mode even if cell healthy
            
    // Added for C1
    accDICKSON_MULTIPLIER_ENABLE,       // 0x32-Enable/Disable PWM to 7V Dickson charge pump in WC40
    accI_SOL_SC_OVERRIDE,               // 0x33-Solar short circuit current override flag
    accI_SOL_SC_ADC,                    // 0x34-Solar short circuit current override value
    accSOLAR_SHUNT_FET_TESTJIG_OVERRIDE,// 0x35-Solar short circuit FET testjig override (100 = off, 101 = on)
    
    accAUTOMATION_CTRL_MAX              // INSERT new commands before here, do not alter existing commands
} AutomationControlCommandT;

typedef enum 
{
    // Added for MidSolar, SmallSolar diverges after dcSOLAR_PANEL_SC_CURRENT
    dcBATT_VOLTAGE_ADC,				    // 0x00-Read ADC values of Battery Voltage
    dcSOLAR_VOLTAGE_ADC,			    // 0x01-Read ADC values of Solar Voltage
    dcINT_TEMPERATURE,				    // 0x02-Read Internal Temperature
    dcLIGHT_SNSR_READING,			    // 0x03-Read Light Sensor Reading
    dcSOLAR_PANEL_SC_CURRENT,		    // 0x04-Read Solar Panel Short Circuit current
    dcMAGNETIC_SNSR_STATUS,		        // 0x05-Read Hall effect snsr n status	
    dcINT_TEMPERATURE_ADC,              // 0x06-Read ADC values on temp channel.
    dcPRIMARY_SNS_VOLTAGE,              // 0x07-Read Primary Sense voltage   
    dcUNDERSWING_SNS_VOLTAGE,	        // 0x08-Read Under swing sense voltage.
    dcFINAL_LOAD_SNS_VAL,               // 0x09-Final Load Sense Value.
    dcSECONDARY_SNS_VOLTAGE,            // 0x0A-Read Secondary sense
    dcSTARTUP_PRI_SNS_VOLTAGE,          // 0x0B-Read the startup primary sense voltage
    dcDC_REG_VOLT_IN_VAL,               // 0x0C-Input voltage on the DC regulator 1 line
    dcDC_REG_VOLT_IN_2_VAL,             // 0x0D-Input voltage on the DC regulator 2 line for measuring the charge current
    dcDC_REG_VOLT_OUT_VAL,              // 0x0E-Output voltage from the DC regulator
    dcOC_BATT_VOLTAGE_ADC,              // 0x0F-Open circuit battery voltages.
    dcDC_REG_VOLT_IN_SC_VAL,            // 0x10-Input voltage on the DC regulator 1 line when reg ctrl. is turned on     
    dcBATTERY_CHARGE_STATUS,            // 0x11-Current charge status of the battery from the charge current.
    dcBATTERY_ISC_MOVING_AVG_VAL,       // 0x12-Moving average value of the short circuit current. (Hourly)
    dcBATTERY_TEMP,                     // 0x13-Battery Temperature
    dcTESTJIG_MODE_STATUS,              // 0x14-Reads the testjig mode status
    dcTESTJIG_PIN_STATUS,               // 0x15-Reads the testjig pin status

    // Added for B80
    dcCV_MEASURE_1_BEFORE,              // 0x16-Reads the CV for channel 1 before charging (0=adc 1=volts)
    dcCV_MEASURE_1_AFTER,               // 0x17-Reads the CV for channel 1 before firing (0=adc 1=volts)
    dcCV_MEASURE_2_BEFORE,              // 0x18-Reads the CV for channel 2 before charging (0=adc 1=volts)
    dcCV_MEASURE_2_AFTER,               // 0x19-Reads the CV for channel 2 before firing (0=adc 1=volts)
    dcMULTI_STAGE_PWM_ON_TIME,          // 0x1A-MultiStage On time in use. Sent with stage index and on time
    dcMULTI_STAGE_PWM_OFF_TIME,         // 0x1B-MultiStage Off time in use. Sent with stage index and off time
    dcMULTI_STAGE_CHRG_PERIOD,          // 0x1C-charge control period for N-1 stages in use. Last stage will be targeting DAC threshold
    dcMAX_CHANNEL_DIFF,                 // 0x1D-Maximum CV channel difference seen (0=adc 1=volts)
    dcCHANNEL_DIFF_COUNTER,             // 0x1E-Count of the times the CV channel difference has exceeded the threshold

    // Added for MiniSolar
    dcDEV_PIN_STATUS,                   // 0x1F-Read the live dev pin status (open=1)
    dcTARGET_CV_V,                      // 0x20-Read the Target CV in Volts
    dcVREF_RAIL_MV,                     // 0x21-Read the smoothed Vref rail in millivolts based on internal ref
    dcBOARD_DETECT_STATUS,              // 0x22-Read the board detect boot status (INTL=1, JPN=0)
    dcCELL_VOLTAGE_SMOOTH_MV,           // 0x23-Read value of Cell Voltage smoothed, no cap chg, mV
    dcSOLAR_VOLTAGE_SMOOTH_MV,          // 0x24-Read value of Solar Voltage smoothed, no cap chg, mV
    dcCELL_VOLTAGE_OC_MV,               // 0x25-Read value of Cell Voltage open circuit, no cap chg, mV
    dcSOLAR_VOLTAGE_OC_MV, 		        // 0x26-Read value of Solar Voltage open circuit, no cap chg, mV
    dcHOURS_UPTIME,                     // 0x27-Read value of uptime of micro in hours, 65000 saturate
    dcFLAT_CELL_LED_BRIGHTNESS,         // 0x28-Read value of Flat Cell LED Brightness (0-1000)
    dcCELL_CHARGE_FSM,                  // 0x29-Read value of cell_charge FSM
    dcCHARGE_FIRE_FSM,                  // 0x2A-Read value of charge_fire FSM
    dcPOWER_MODE_FSM,                   // 0x2B-Read value of power_mode FSM
    dcSTANDBY_ENTRY_FSM,                // 0x2C-Read value of standby_entry FSM
    dcBUTTON_DETECT_FSM,                // 0x2D-Read value of light_detect FSM (was button_detect FSM)
    dcWC70_FSM,                         // 0x2E-Read value of wc70 FSM
    dcSOLAR_LIGHT_FSM,                  // 0x2F-Read value of light_level FSM (was solar_light FSM)
    dcENERGY_STATE,                     // 0x30-Read value of Energy State (1=Reduced, 2=Full)
    dcHOURS_SINCE_LAST_CELL_LOW,        // 0x31-Read value of hours since last time cell was Low (below 30%) SOC
    dcMINUTES_THIS_HOUR,                // 0x32-Read value of minutes_this_hour
    dcCELL_OC_MINUTE_AV_MV,             // 0x33-Read value of cell_oc_minute_av_mv for last completed minute
    dcCELL_OC_HOUR_AV_MV,               // 0x34-Read value of cell_oc_hour_av_mv for last completed hour
    dcTIME_WAITING_FOR_STANDBY_MINS,    // 0x35-Read value of time_waiting_for_standby in whole minutes
    dcLAST_HOUR_VOLTAGE_SOC_LOW_MV,     // 0x36-Read value of last_hour_voltage_soc_low_mv
    dcLAST_HOUR_CELL_TEMPERATURE,       // 0x37-Read value of last_hour_cell_temperature
    dcCELL_OC_REAL_TIME_AV_MV,          // 0x38-Read value of real time partial calculation of cell_oc_hour_av_mv for hour in progress
    dcCELL_MAX_VOLT_MV,                 // 0x39-Read value of max cell voltage based on single sample (0=daily, 1=since reset)
    dcCELL_MEAN_MAX_VOLT_MV,            // 0x3A-Read value of max cell voltage based on mean of 5 samples (0=daily, 1=since reset)
    
    dcSWITCH_FSM,                       // 0x3B-Read value of switch FSM
    dcSOLAR_CHARGE_PERCENT_PWM,         // 0x3C-Read value of solar charging PWM
    
    dcDIAGNOSTICS_MAX                   // INSERT new responses before here, do not alter existing commands
} DiagnosticsResponseT;

typedef enum
{
    // Added for MidSolar, not present in SmallSolar
    adrENGR_RUN_STATUS,                 // 0x00-Energizer run status
    adrENGR_CHARGE_TIME,                // 0x01-Capacitor Charge time
    adrENGR_ON_TIME,                    // 0x02-Micro's normal operating time while pulsing
    adrENGR_SLEEP_TIME,                 // 0x03-engerizer sleep time between pulses.
    adrFRONT_SW_POS,                    // 0x04-Front switch position status
    adrBATT_HEALTH_STATUS,              // 0x05-Battery Health Status
    adrBATT_CHARGE_STATUS,	            // 0x06-Current Battery Charge Status
    adrBATT_SOLAR_FET_CTRL_PIN_STATE,   // 0x07-battery solar FET pin status
    adrBATT_DC_REG_CTRL_PIN_STATUS,     // 0x08-battery charge pins status
    adrBATT_SOLAR_CHARGE_CTRL_STATE,    // 0x09-Battery solar charge control state
    adrBATT_DC_REG_CHARGE_CTRL_STATE,   // 0x0A-Battery DC regulator charge control state
    adrBATT_CHARGE_TARGET_VOLTAGE,      // 0x0B-battery charge target voltage
    adrADJUSTED_PULSE_RATE,             // 0x0C-Computed adj pulse rate
    adrADJUSTED_TARGET_CV_DAC,          // 0x0D-Computed adj target CV in DAC counts.
    adrDAY_PULSE_RATE,                  // 0x0E-Day time pulse rate                                                
    adrNIGHT_PULSE_RATE,                // 0x0F-night time pulse rate
    adrTEMP_BASED_COMP_FACTOR,          // 0x10-Temperature based CV compensation factor
    adrLIGHT_SENSOR_STATUS,             // 0x11-day/night light sensor status
    adrDC_REG_CONNECTED_STATUS,	        // 0x12-connected status of DC regulator
    adrDC_REG_CHARGE_CURRENT_THRESHOLD, // 0x13-DC regulator charge current threshold to determine the charging state
    adrSOLAR_CHARGE_CURRENT_THRESHOLD,  // 0x14-Solar panel charge current threshold to determine the charging state.
    adrBATT_SAMPLE_CURR_HOUR_INDEX,     // 0x15-battery management sample current hour index
    adrBATT_SAMPLE_VOLTAGE_AT_INDEX,    // 0x16-battery management sample voltage at specified index
    adrBATT_SAMPLE_MINIMUM_VOLTAGE,     // 0x17-battery management sample minimum voltage
    adrLED_RESET_ON_COUNTS,             // 0x18-resets the on counts of the Led
    adrLED_GET_ON_COUNTS,               // 0x19-returns the on counts of the led
    adrBATT_CHARGE_COUNT,	            // 0x1A-Battery charge counter
    adrSOLAR_FET_CONTROL_PIN_STATE,     // 0x1B-Solar FET pin status
    adrBATT_FET_CONTROL_PIN_STATE,      // 0x1C-Batt FET pin status

    // Added for B80
    adrENERGY_LEVEL_INDEX,              // 0x1D-current energy level index (1-8) 8=highest
    adrVOLTAGE_MAP_INDEX,               // 0x1E-current voltage map index (0 based for lowest voltage)
    adrTEMPERATURE_MAP_INDEX,           // 0x1F-current temperature map index (0 based for lowest temp)
    adrBATTERY_LED_TEMP_THRESH_ADC,     // 0x20-ADC value with temperature compensation applied that triggers a Red LED on fence pulse
    
    adrAUTOMATION_DIAGNOSTICS_MAX       // INSERT new responses before here, do not alter existing commands
} AutomationDiagnosticsResponseT;

typedef enum
{
    errNOERROR = 0,
    errCHKSUM  = -1,
    errPKT     = -2,
} BAgByteErrT;

typedef struct
{
    uint8_t len;  
    uint8_t buf[MAX_MSG_LEN];
    // uint8_t prev_byte;
    uint8_t chksum;
    bool msg_complete;
    // BAgMsgRxStateT rxState;
} BAgMsgT;

typedef struct SWversionT
{
    uint8_t major;
    uint8_t minor;
    uint8_t build;
} SWversionT;
typedef struct EnergizerInfoT
{
    SWversionT      sw_version;
    uint16_t        hw_version;
    SolarModelInfoT unit_type;
} EnergizerInfoT;
extern EnergizerInfoT energizer_info;

typedef struct EnergizerDataT
{
    uint16_t        temp;
    uint16_t        light_adc;
    uint16_t        load_sense;
    uint16_t        target_cv;
    uint16_t        smooth_vbat;
    uint16_t        smooth_vsol;
    uint16_t        sw_pos;
} EnergizerDataT;
extern EnergizerDataT energizer_data;

typedef struct EnergizerQueriesT
{
    const char      *p_data_name;
    BAgCommandT     command_type;
    uint8_t         command;
    uint8_t         parameter;
    uint8_t         n_args;
    uint16_t*       p_data;
} EnergizerQueriesT;

extern EnergizerQueriesT const energizer_info_queries[];
extern const uint8_t  NUM_OF_DEV_INFO;
extern EnergizerQueriesT const energizer_queries[];
extern const uint8_t NUM_OF_QUERIES;

#define MAX_QUERY_PAYLOAD_SIZE  3


void BAgInit(void);
BAgByteErrT ProcessRxBAgString(uint8_t * rx_buf, uint8_t rx_len);

#endif // __APP_B_AG_H