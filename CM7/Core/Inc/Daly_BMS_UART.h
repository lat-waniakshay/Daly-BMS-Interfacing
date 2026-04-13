/*
 * Daly_BMS_UART.h
 *
 *  Created on: 18-Mar-2026
 *      Author: Akshay
 *
 *  Portable Daly BMS UART Driver
 */

#ifndef INC_DALY_BMS_UART_H_
#define INC_DALY_BMS_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"

#define BMS_PLATFORM_STM32_HAL  1

typedef UART_HandleTypeDef* BMS_UART_Handle_t;

/* ══════════════════════════════════════════════
 *  PROTOCOL CONSTANTS
 * ══════════════════════════════════════════════ */

/* Start Flag */
#define BMS_START_FLAG                  0xA5

/* Module Addresses */
#define BMS_ADDR_BMS_MASTER             0x01
#define BMS_ADDR_GPRS                   0x20
#define BMS_ADDR_UPPER_COMPUTER         0x40
#define BMS_ADDR_BLUETOOTH_APP          0x80

/* Data IDs (Command IDs) */
#define BMS_CMD_SOC_VOLTAGE_CURRENT     0x90
#define BMS_CMD_CELL_VOLTAGE_EXTREMES   0x91
#define BMS_CMD_TEMP_EXTREMES           0x92
#define BMS_CMD_MOS_STATUS              0x93
#define BMS_CMD_STATUS_INFO             0x94
#define BMS_CMD_CELL_VOLTAGES           0x95
#define BMS_CMD_CELL_TEMPERATURES       0x96
#define BMS_CMD_CELL_BALANCE            0x97
#define BMS_CMD_FAULT_STATUS            0x98

/* Frame Structure */
#define BMS_FRAME_HEADER_SIZE           4       /* Start + Addr + ID + Len  */
#define BMS_FRAME_CHECKSUM_SIZE         1
#define BMS_STANDARD_DATA_LEN           8
#define BMS_STANDARD_FRAME_LEN          (BMS_FRAME_HEADER_SIZE + BMS_STANDARD_DATA_LEN + BMS_FRAME_CHECKSUM_SIZE)  /* 13 */

/* Conversion Constants */
#define BMS_CURRENT_OFFSET              30000
#define BMS_TEMP_OFFSET                 40

/* Capacity */
#define BMS_MAX_CELLS                   48
#define BMS_MAX_TEMP_SENSORS            16

/* Timing */
#define BMS_RESPONSE_TIMEOUT_MS         500
#define BMS_RETRY_COUNT                 3
#define BMS_INTER_CMD_DELAY_MS          20

/* RX Buffer */
#define BMS_RX_BUFFER_SIZE              256

/* ══════════════════════════════════════════════
 *  CONVERSION MACROS
 * ══════════════════════════════════════════════ */
#define BMS_BYTES_TO_U16(hi, lo)        ((uint16_t)((uint16_t)(hi) << 8) | (uint16_t)(lo))
#define BMS_BYTES_TO_U32(b3, b2, b1, b0) \
    ((uint32_t)((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) | \
     ((uint32_t)(b1) << 8)  | (uint32_t)(b0))
#define BMS_RAW_TO_VOLTAGE(raw)         ((float)(raw) * 0.1f)
#define BMS_RAW_TO_CURRENT(raw)         (((float)(raw) - (float)BMS_CURRENT_OFFSET) * 0.1f)
#define BMS_RAW_TO_SOC(raw)             ((float)(raw) * 0.1f)
#define BMS_RAW_TO_TEMP(raw)            ((int8_t)((int16_t)(raw) - BMS_TEMP_OFFSET))
#define BMS_GET_BIT(byte, bit)          (((byte) >> (bit)) & 0x01)

/* ══════════════════════════════════════════════
 *  ENUMS
 * ══════════════════════════════════════════════ */

/* Return status for all BMS functions */
typedef enum {
    BMS_OK            = 0,
    BMS_ERR_TIMEOUT   = 1,
    BMS_ERR_CHECKSUM  = 2,
    BMS_ERR_FRAME     = 3,      /* Invalid start flag or address     */
    BMS_ERR_DATA_ID   = 4,      /* Unexpected data ID in response    */
    BMS_ERR_UART      = 5,      /* UART HAL TX/RX failure            */
    BMS_ERR_NULL_PTR  = 6,      /* NULL pointer passed               */
    BMS_ERR_NOT_INIT  = 7       /* BMS_Init() not called yet         */
} BMS_Status_e;

/* 0x93 - BMS operating state */
typedef enum {
    BMS_STATE_STATIONARY = 0,
    BMS_STATE_CHARGE     = 1,
    BMS_STATE_DISCHARGE  = 2
} BMS_Operating_State_e;

/* 0x93 - MOS switch state */
typedef enum {
    MOS_OFF = 0,
    MOS_ON  = 1
} MOS_State_e;

/* 0x94 - Charger/Load connection status */
typedef enum {
    CONNECTION_DISCONNECTED = 0,
    CONNECTION_CONNECTED    = 1
} Connection_Status_e;

/* 0x97 - Cell balance state */
typedef enum {
    BALANCE_CLOSED = 0,
    BALANCE_OPEN   = 1
} Balance_State_e;

/* 0x98 - Fault flag */
typedef enum {
    FAULT_NO_ERROR = 0,
    FAULT_ERROR    = 1
} Fault_Flag_e;

/* ══════════════════════════════════════════════
 *  RAW FRAME STRUCT (for universal TX/RX)
 * ══════════════════════════════════════════════ */
typedef struct {
    uint8_t start;
    uint8_t address;
    uint8_t data_id;
    uint8_t data_len;
    uint8_t data[BMS_RX_BUFFER_SIZE];
    uint8_t checksum;
} BMS_Frame_t;

/* ══════════════════════════════════════════════
 *  DATA STRUCTS (0x90 - 0x98)
 * ══════════════════════════════════════════════ */

/* 0x90 - SOC, Total Voltage & Current */
typedef struct {
    float cumulative_total_voltage;
    float gather_total_voltage;
    float current;
    float soc;
} BMS_SOC_Voltage_Current_t;

/* 0x91 - Max & Min Cell Voltage */
typedef struct {
    uint16_t max_cell_voltage_mV;
    uint8_t  max_voltage_cell_num;
    uint16_t min_cell_voltage_mV;
    uint8_t  min_voltage_cell_num;
} BMS_Cell_Voltage_Extremes_t;

/* 0x92 - Max & Min Temperature */
typedef struct {
    int8_t  max_temperature_degC;
    uint8_t max_temp_sensor_num;
    int8_t  min_temperature_degC;
    uint8_t min_temp_sensor_num;
} BMS_Temperature_Extremes_t;

/* 0x93 - Charge & Discharge MOS Status */
typedef struct {
    BMS_Operating_State_e operating_state;
    MOS_State_e           charge_mos;
    MOS_State_e           discharge_mos;
    uint8_t               bms_life_cycles;
    uint32_t              remaining_capacity_mAh;
} BMS_MOS_Status_t;

/* 0x94 - Digital I/O */
typedef struct {
    bool di1;
    bool di2;
    bool di3;
    bool di4;
    bool do1;
    bool do2;
    bool do3;
    bool do4;
} BMS_Digital_IO_t;

/* 0x94 - Status Information */
typedef struct {
    uint8_t             num_battery_strings;
    uint8_t             num_temperature_sensors;
    Connection_Status_e charger_status;
    Connection_Status_e load_status;
    BMS_Digital_IO_t    digital_io;
} BMS_Status_Info_t;

/* 0x95 - Individual Cell Voltages */
typedef struct {
    uint8_t  num_cells;
    uint16_t cell_voltage_mV[BMS_MAX_CELLS];
} BMS_Cell_Voltages_t;

/* 0x96 - Individual Cell Temperatures */
typedef struct {
    uint8_t num_sensors;
    int8_t  temperature_degC[BMS_MAX_TEMP_SENSORS];
} BMS_Cell_Temperatures_t;

/* 0x97 - Cell Balance States */
typedef struct {
    Balance_State_e cell_balance[BMS_MAX_CELLS];
} BMS_Cell_Balance_t;

/* 0x98 - Fault Sub-Structs */
typedef struct {
    Fault_Flag_e cell_volt_high_lvl1;
    Fault_Flag_e cell_volt_high_lvl2;
    Fault_Flag_e cell_volt_low_lvl1;
    Fault_Flag_e cell_volt_low_lvl2;
    Fault_Flag_e sum_volt_high_lvl1;
    Fault_Flag_e sum_volt_high_lvl2;
    Fault_Flag_e sum_volt_low_lvl1;
    Fault_Flag_e sum_volt_low_lvl2;
} BMS_Fault_Voltage_t;

typedef struct {
    Fault_Flag_e chg_temp_high_lvl1;
    Fault_Flag_e chg_temp_high_lvl2;
    Fault_Flag_e chg_temp_low_lvl1;
    Fault_Flag_e chg_temp_low_lvl2;
    Fault_Flag_e dischg_temp_high_lvl1;
    Fault_Flag_e dischg_temp_high_lvl2;
    Fault_Flag_e dischg_temp_low_lvl1;
    Fault_Flag_e dischg_temp_low_lvl2;
} BMS_Fault_Temperature_t;

typedef struct {
    Fault_Flag_e chg_overcurrent_lvl1;
    Fault_Flag_e chg_overcurrent_lvl2;
    Fault_Flag_e dischg_overcurrent_lvl1;
    Fault_Flag_e dischg_overcurrent_lvl2;
    Fault_Flag_e soc_high_lvl1;
    Fault_Flag_e soc_high_lvl2;
    Fault_Flag_e soc_low_lvl1;
    Fault_Flag_e soc_low_lvl2;
} BMS_Fault_Current_SOC_t;

typedef struct {
    Fault_Flag_e diff_volt_lvl1;
    Fault_Flag_e diff_volt_lvl2;
    Fault_Flag_e diff_temp_lvl1;
    Fault_Flag_e diff_temp_lvl2;
} BMS_Fault_Differential_t;

typedef struct {
    Fault_Flag_e chg_mos_temp_high;
    Fault_Flag_e dischg_mos_temp_high;
    Fault_Flag_e chg_mos_temp_sensor_err;
    Fault_Flag_e dischg_mos_temp_sensor_err;
    Fault_Flag_e chg_mos_adhesion_err;
    Fault_Flag_e dischg_mos_adhesion_err;
    Fault_Flag_e chg_mos_open_circuit_err;
    Fault_Flag_e dischg_mos_open_circuit_err;
} BMS_Fault_MOS_t;

typedef struct {
    Fault_Flag_e afe_collect_chip_err;
    Fault_Flag_e voltage_collect_dropped;
    Fault_Flag_e cell_temp_sensor_err;
    Fault_Flag_e eeprom_err;
    Fault_Flag_e rtc_err;
    Fault_Flag_e precharge_failure;
    Fault_Flag_e communication_failure;
    Fault_Flag_e internal_comm_failure;
} BMS_Fault_System_t;

typedef struct {
    Fault_Flag_e current_module_fault;
    Fault_Flag_e sum_voltage_detect_fault;
    Fault_Flag_e short_circuit_protect_fault;
    Fault_Flag_e low_volt_forbidden_chg_fault;
} BMS_Fault_Hardware_t;

typedef struct {
    BMS_Fault_Voltage_t       voltage;
    BMS_Fault_Temperature_t   temperature;
    BMS_Fault_Current_SOC_t   current_soc;
    BMS_Fault_Differential_t  differential;
    BMS_Fault_MOS_t           mos;
    BMS_Fault_System_t        system;
    BMS_Fault_Hardware_t      hardware;
    uint8_t                   fault_code;
} BMS_Fault_Status_t;

/* ══════════════════════════════════════════════
 *  MASTER DATA STRUCT
 * ══════════════════════════════════════════════ */
typedef struct {
    BMS_SOC_Voltage_Current_t   soc_voltage_current;
    BMS_Cell_Voltage_Extremes_t cell_voltage_extremes;
    BMS_Temperature_Extremes_t  temperature_extremes;
    BMS_MOS_Status_t            mos_status;
    BMS_Status_Info_t           status_info;
    BMS_Cell_Voltages_t         cell_voltages;
    BMS_Cell_Temperatures_t     cell_temperatures;
    BMS_Cell_Balance_t          cell_balance;
    BMS_Fault_Status_t          fault_status;
} BMS_Data_t;

/* ══════════════════════════════════════════════
 *  BMS HANDLE (one per physical BMS)
 *
 *  Usage in main.c:
 *    BMS_Handle_t bms1, bms2;
 *    BMS_Init(&bms1, &huart5);
 *    BMS_Init(&bms2, &huart3);
 *    BMS_GetAllData(&bms1);
 *    float voltage = bms1.data.soc_voltage_current.cumulative_total_voltage;
 * ══════════════════════════════════════════════ */
typedef struct {
    BMS_UART_Handle_t   uart;               /* Platform UART handle             */
    BMS_Data_t          BMS_Data;           /* Parsed BMS data (all 0x90~0x98)  */
    BMS_Frame_t         tx_frame;           /* Reusable TX frame buffer         */
    BMS_Frame_t         rx_frame;           /* Reusable RX frame buffer         */
    uint8_t             tx_buf[BMS_STANDARD_FRAME_LEN]; /* Raw TX byte buffer   */
    uint8_t             rx_buf[BMS_RX_BUFFER_SIZE];     /* Raw RX byte buffer   */
    bool                initialized;        /* true after BMS_Init()            */
} BMS_Handle_t;

/* ══════════════════════════════════════════════
 *  PUBLIC API - FUNCTION PROTOTYPES
 * ══════════════════════════════════════════════ */

/*-----------------------------------------------
 * Initialization
 * Call once per BMS in main() before any other API
 *
 * Example:
 *   BMS_Handle_t bms;
 *   BMS_Init(&bms, &huart5);
 *----------------------------------------------*/
BMS_Status_e BMS_Init(BMS_Handle_t *bms, BMS_UART_Handle_t uart_handle);

/*-----------------------------------------------
 * Universal TX/RX (low-level)
 * You normally don't call these directly.
 * The individual getter functions use them internally.
 *----------------------------------------------*/
BMS_Status_e BMS_SendCommand(BMS_Handle_t *bms, uint8_t data_id);
BMS_Status_e BMS_ReceiveResponse(BMS_Handle_t *bms, uint8_t expected_data_id, uint16_t rx_len);

/*-----------------------------------------------
 * Individual Data Getters (0x90 ~ 0x98)
 * Each sends the request, receives the response,
 * parses it, and stores into bms->data.xxx
 *----------------------------------------------*/
BMS_Status_e BMS_GetSOCVoltageCurrent(BMS_Handle_t *bms);       /* 0x90 */
BMS_Status_e BMS_GetCellVoltageExtremes(BMS_Handle_t *bms);     /* 0x91 */
BMS_Status_e BMS_GetTemperatureExtremes(BMS_Handle_t *bms);     /* 0x92 */
BMS_Status_e BMS_GetMOSStatus(BMS_Handle_t *bms);               /* 0x93 */
BMS_Status_e BMS_GetStatusInfo(BMS_Handle_t *bms);              /* 0x94 */
BMS_Status_e BMS_GetCellVoltages(BMS_Handle_t *bms);            /* 0x95 */
BMS_Status_e BMS_GetCellTemperatures(BMS_Handle_t *bms);        /* 0x96 */
BMS_Status_e BMS_GetCellBalance(BMS_Handle_t *bms);             /* 0x97 */
BMS_Status_e BMS_GetFaultStatus(BMS_Handle_t *bms);             /* 0x98 */

/*-----------------------------------------------
 * Get All BMS Data in one shot
 * Calls all 9 getter functions sequentially.
 * Returns the FIRST error encountered, or BMS_OK
 * if all succeed. Continues polling remaining
 * commands even if one fails.
 *----------------------------------------------*/
BMS_Status_e BMS_GetAllData(BMS_Handle_t *bms);

#endif /* INC_DALY_BMS_UART_H_ */
