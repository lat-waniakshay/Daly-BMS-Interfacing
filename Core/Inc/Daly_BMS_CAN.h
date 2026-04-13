/*
 * Daly_BMS_CAN.h
 *
 *  Created on: 18-Mar-2026
 *      Author: Akshay
 *
 *  Portable Daly BMS CAN Driver
 */
/* ══════════════════════════════════════════════
//.ioc file configuration :
//	mode : normal
//	baud rate : 250000 (250k)
//	Automatic Bus-Off Management: Enable
//	Automatic Wake-Up Mode: Enable
//	Automatic Retransmission: Enable
/* ══════════════════════════════════════════════
 *  BMS CAN HANDLE (one per physical BMS)
 *
 *  Usage in main.c:
 *    BMS_CAN_Handle_t bms_can1;
 *    BMS_CAN_Init(&bms_can1, &hcan1);
 *    BMS_CAN_GetAllData(&bms_can1);
 *    float voltage = bms_can1.BMS_Data.soc_voltage_current.cumulative_total_voltage;
 * ══════════════════════════════════════════════ */

#ifndef INC_DALY_BMS_CAN_H_
#define INC_DALY_BMS_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"

#define BMS_CAN_PLATFORM_STM32_HAL  1

typedef CAN_HandleTypeDef* BMS_CAN_Handle_t;

/* ══════════════════════════════════════════════
 *  PROTOCOL CONSTANTS
 * ══════════════════════════════════════════════ */

/* CAN Baud Rate */
#define BMS_CAN_BAUD_RATE               250000

/* Module Addresses */
#define BMS_CAN_ADDR_BMS_MASTER         0x01
#define BMS_CAN_ADDR_GPRS               0x20
#define BMS_CAN_ADDR_UPPER_COMPUTER     0x40
#define BMS_CAN_ADDR_BLUETOOTH_APP      0x80

/* CAN ID Construction:
 *   TX (PC -> BMS): Priority(0x18) + DataID + BMS_Addr(0x01) + PC_Addr(0x40)
 *     Example for 0x90: 0x18 90 01 40
 *   RX (BMS -> PC): Priority(0x18) + DataID + PC_Addr(0x40) + BMS_Addr(0x01)
 *     Example for 0x90: 0x18 90 40 01
 */
#define BMS_CAN_PRIORITY                0x18
#define BMS_CAN_TX_ID_BASE              ((uint32_t)BMS_CAN_PRIORITY << 24 | \
                                         (uint32_t)BMS_CAN_ADDR_BMS_MASTER << 8 | \
                                         (uint32_t)BMS_CAN_ADDR_UPPER_COMPUTER)
/* TX ID = 0x18XX0140, where XX = Data ID */

#define BMS_CAN_RX_ID_BASE              ((uint32_t)BMS_CAN_PRIORITY << 24 | \
                                         (uint32_t)BMS_CAN_ADDR_UPPER_COMPUTER << 8 | \
                                         (uint32_t)BMS_CAN_ADDR_BMS_MASTER)
/* RX ID = 0x18XX4001, where XX = Data ID */

#define BMS_CAN_BUILD_TX_ID(data_id)    (BMS_CAN_TX_ID_BASE | ((uint32_t)(data_id) << 16))
#define BMS_CAN_BUILD_RX_ID(data_id)    (BMS_CAN_RX_ID_BASE | ((uint32_t)(data_id) << 16))

/* Extract Data ID from a 29-bit CAN ID */
#define BMS_CAN_EXTRACT_DATA_ID(can_id) ((uint8_t)(((can_id) >> 16) & 0xFF))

/* Data IDs (Command IDs) - identical to UART */
#define BMS_CAN_CMD_SOC_VOLTAGE_CURRENT     0x90
#define BMS_CAN_CMD_CELL_VOLTAGE_EXTREMES   0x91
#define BMS_CAN_CMD_TEMP_EXTREMES           0x92
#define BMS_CAN_CMD_MOS_STATUS              0x93
#define BMS_CAN_CMD_STATUS_INFO             0x94
#define BMS_CAN_CMD_CELL_VOLTAGES           0x95
#define BMS_CAN_CMD_CELL_TEMPERATURES       0x96
#define BMS_CAN_CMD_CELL_BALANCE            0x97
#define BMS_CAN_CMD_FAULT_STATUS            0x98

/* CAN Frame Structure */
#define BMS_CAN_DATA_LEN                8       /* Standard CAN data = 8 bytes */

/* Conversion Constants */
#define BMS_CAN_CURRENT_OFFSET          30000
#define BMS_CAN_TEMP_OFFSET             40

/* Capacity */
#define BMS_CAN_MAX_CELLS               48
#define BMS_CAN_MAX_TEMP_SENSORS        16

/* Timing */
#define BMS_CAN_RESPONSE_TIMEOUT_MS     500
#define BMS_CAN_RETRY_COUNT             3
#define BMS_CAN_INTER_CMD_DELAY_MS      25

/* Multi-frame reception max frames */
#define BMS_CAN_MAX_CELL_FRAMES         16      /* Max 48 cells / 3 per frame = 16 */
#define BMS_CAN_MAX_TEMP_FRAMES         3       /* Max 16 sensors / 7 per frame = 3 */

/* ══════════════════════════════════════════════
 *  CONVERSION MACROS
 * ══════════════════════════════════════════════ */
#define BMS_CAN_BYTES_TO_U16(hi, lo)        ((uint16_t)((uint16_t)(hi) << 8) | (uint16_t)(lo))
#define BMS_CAN_BYTES_TO_U32(b3, b2, b1, b0) \
    ((uint32_t)((uint32_t)(b3) << 24) | ((uint32_t)(b2) << 16) | \
     ((uint32_t)(b1) << 8)  | (uint32_t)(b0))
#define BMS_CAN_RAW_TO_VOLTAGE(raw)         ((float)(raw) * 0.1f)
#define BMS_CAN_RAW_TO_CURRENT(raw)         (((float)(raw) - (float)BMS_CAN_CURRENT_OFFSET) * 0.1f)
#define BMS_CAN_RAW_TO_SOC(raw)             ((float)(raw) * 0.1f)
#define BMS_CAN_RAW_TO_TEMP(raw)            ((int8_t)((int16_t)(raw) - BMS_CAN_TEMP_OFFSET))
#define BMS_CAN_GET_BIT(byte, bit)          (((byte) >> (bit)) & 0x01)

/* ══════════════════════════════════════════════
 *  ENUMS
 * ══════════════════════════════════════════════ */

/* Return status for all BMS CAN functions */
typedef enum {
    BMS_CAN_OK            = 0,
    BMS_CAN_ERR_TIMEOUT   = 1,
    BMS_CAN_ERR_CAN_ID    = 2,      /* Unexpected CAN ID in response    */
    BMS_CAN_ERR_DATA_ID   = 3,      /* Unexpected data ID in response   */
    BMS_CAN_ERR_HAL       = 4,      /* CAN HAL TX/RX failure            */
    BMS_CAN_ERR_NULL_PTR  = 5,      /* NULL pointer passed              */
    BMS_CAN_ERR_NOT_INIT  = 6,      /* BMS_CAN_Init() not called yet    */
    BMS_CAN_ERR_NO_MSG    = 7,      /* No message in RX FIFO            */
    BMS_CAN_ERR_FILTER    = 8       /* CAN filter configuration failed  */
} BMS_CAN_Status_e;

/* 0x93 - BMS operating state */
typedef enum {
    BMS_CAN_STATE_STATIONARY = 0,
    BMS_CAN_STATE_CHARGE     = 1,
    BMS_CAN_STATE_DISCHARGE  = 2
} BMS_CAN_Operating_State_e;

/* 0x93 - MOS switch state */
typedef enum {
    BMS_CAN_MOS_OFF = 0,
    BMS_CAN_MOS_ON  = 1
} BMS_CAN_MOS_State_e;

/* 0x94 - Charger/Load connection status */
typedef enum {
    BMS_CAN_CONNECTION_DISCONNECTED = 0,
    BMS_CAN_CONNECTION_CONNECTED    = 1
} BMS_CAN_Connection_Status_e;

/* 0x97 - Cell balance state */
typedef enum {
    BMS_CAN_BALANCE_CLOSED = 0,
    BMS_CAN_BALANCE_OPEN   = 1
} BMS_CAN_Balance_State_e;

/* 0x98 - Fault flag */
typedef enum {
    BMS_CAN_FAULT_NO_ERROR = 0,
    BMS_CAN_FAULT_ERROR    = 1
} BMS_CAN_Fault_Flag_e;

/* ══════════════════════════════════════════════
 *  CAN FRAME STRUCT (for universal TX/RX)
 * ══════════════════════════════════════════════ */
typedef struct {
    uint32_t    id;                             /* 29-bit Extended CAN ID   */
    uint8_t     dlc;                            /* Data Length Code (0~8)   */
    uint8_t     data[BMS_CAN_DATA_LEN];         /* Data payload             */
} BMS_CAN_Frame_t;

/* ══════════════════════════════════════════════
 *  DATA STRUCTS (0x90 - 0x98)
 *  Identical data content to UART protocol
 * ══════════════════════════════════════════════ */

/* 0x90 - SOC, Total Voltage & Current */
typedef struct {
    float cumulative_total_voltage;
    float gather_total_voltage;
    float current;
    float soc;
} BMS_CAN_SOC_Voltage_Current_t;

/* 0x91 - Max & Min Cell Voltage */
typedef struct {
    uint16_t max_cell_voltage_mV;
    uint8_t  max_voltage_cell_num;
    uint16_t min_cell_voltage_mV;
    uint8_t  min_voltage_cell_num;
} BMS_CAN_Cell_Voltage_Extremes_t;

/* 0x92 - Max & Min Temperature */
typedef struct {
    int8_t  max_temperature_degC;
    uint8_t max_temp_sensor_num;
    int8_t  min_temperature_degC;
    uint8_t min_temp_sensor_num;
} BMS_CAN_Temperature_Extremes_t;

/* 0x93 - Charge & Discharge MOS Status */
typedef struct {
    BMS_CAN_Operating_State_e operating_state;
    BMS_CAN_MOS_State_e       charge_mos;
    BMS_CAN_MOS_State_e       discharge_mos;
    uint8_t                   bms_life_cycles;
    uint32_t                  remaining_capacity_mAh;
} BMS_CAN_MOS_Status_t;

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
} BMS_CAN_Digital_IO_t;

/* 0x94 - Status Information */
typedef struct {
    uint8_t                     num_battery_strings;
    uint8_t                     num_temperature_sensors;
    BMS_CAN_Connection_Status_e charger_status;
    BMS_CAN_Connection_Status_e load_status;
    BMS_CAN_Digital_IO_t        digital_io;
} BMS_CAN_Status_Info_t;

/* 0x95 - Individual Cell Voltages */
typedef struct {
    uint8_t  num_cells;
    uint16_t cell_voltage_mV[BMS_CAN_MAX_CELLS];
} BMS_CAN_Cell_Voltages_t;

/* 0x96 - Individual Cell Temperatures */
typedef struct {
    uint8_t num_sensors;
    int8_t  temperature_degC[BMS_CAN_MAX_TEMP_SENSORS];
} BMS_CAN_Cell_Temperatures_t;

/* 0x97 - Cell Balance States */
typedef struct {
    BMS_CAN_Balance_State_e cell_balance[BMS_CAN_MAX_CELLS];
} BMS_CAN_Cell_Balance_t;

/* 0x98 - Fault Sub-Structs */
typedef struct {
    BMS_CAN_Fault_Flag_e cell_volt_high_lvl1;
    BMS_CAN_Fault_Flag_e cell_volt_high_lvl2;
    BMS_CAN_Fault_Flag_e cell_volt_low_lvl1;
    BMS_CAN_Fault_Flag_e cell_volt_low_lvl2;
    BMS_CAN_Fault_Flag_e sum_volt_high_lvl1;
    BMS_CAN_Fault_Flag_e sum_volt_high_lvl2;
    BMS_CAN_Fault_Flag_e sum_volt_low_lvl1;
    BMS_CAN_Fault_Flag_e sum_volt_low_lvl2;
} BMS_CAN_Fault_Voltage_t;

typedef struct {
    BMS_CAN_Fault_Flag_e chg_temp_high_lvl1;
    BMS_CAN_Fault_Flag_e chg_temp_high_lvl2;
    BMS_CAN_Fault_Flag_e chg_temp_low_lvl1;
    BMS_CAN_Fault_Flag_e chg_temp_low_lvl2;
    BMS_CAN_Fault_Flag_e dischg_temp_high_lvl1;
    BMS_CAN_Fault_Flag_e dischg_temp_high_lvl2;
    BMS_CAN_Fault_Flag_e dischg_temp_low_lvl1;
    BMS_CAN_Fault_Flag_e dischg_temp_low_lvl2;
} BMS_CAN_Fault_Temperature_t;

typedef struct {
    BMS_CAN_Fault_Flag_e chg_overcurrent_lvl1;
    BMS_CAN_Fault_Flag_e chg_overcurrent_lvl2;
    BMS_CAN_Fault_Flag_e dischg_overcurrent_lvl1;
    BMS_CAN_Fault_Flag_e dischg_overcurrent_lvl2;
    BMS_CAN_Fault_Flag_e soc_high_lvl1;
    BMS_CAN_Fault_Flag_e soc_high_lvl2;
    BMS_CAN_Fault_Flag_e soc_low_lvl1;
    BMS_CAN_Fault_Flag_e soc_low_lvl2;
} BMS_CAN_Fault_Current_SOC_t;

typedef struct {
    BMS_CAN_Fault_Flag_e diff_volt_lvl1;
    BMS_CAN_Fault_Flag_e diff_volt_lvl2;
    BMS_CAN_Fault_Flag_e diff_temp_lvl1;
    BMS_CAN_Fault_Flag_e diff_temp_lvl2;
} BMS_CAN_Fault_Differential_t;

typedef struct {
    BMS_CAN_Fault_Flag_e chg_mos_temp_high;
    BMS_CAN_Fault_Flag_e dischg_mos_temp_high;
    BMS_CAN_Fault_Flag_e chg_mos_temp_sensor_err;
    BMS_CAN_Fault_Flag_e dischg_mos_temp_sensor_err;
    BMS_CAN_Fault_Flag_e chg_mos_adhesion_err;
    BMS_CAN_Fault_Flag_e dischg_mos_adhesion_err;
    BMS_CAN_Fault_Flag_e chg_mos_open_circuit_err;
    BMS_CAN_Fault_Flag_e dischg_mos_open_circuit_err;
} BMS_CAN_Fault_MOS_t;

typedef struct {
    BMS_CAN_Fault_Flag_e afe_collect_chip_err;
    BMS_CAN_Fault_Flag_e voltage_collect_dropped;
    BMS_CAN_Fault_Flag_e cell_temp_sensor_err;
    BMS_CAN_Fault_Flag_e eeprom_err;
    BMS_CAN_Fault_Flag_e rtc_err;
    BMS_CAN_Fault_Flag_e precharge_failure;
    BMS_CAN_Fault_Flag_e communication_failure;
    BMS_CAN_Fault_Flag_e internal_comm_failure;
} BMS_CAN_Fault_System_t;

typedef struct {
    BMS_CAN_Fault_Flag_e current_module_fault;
    BMS_CAN_Fault_Flag_e sum_voltage_detect_fault;
    BMS_CAN_Fault_Flag_e short_circuit_protect_fault;
    BMS_CAN_Fault_Flag_e low_volt_forbidden_chg_fault;
} BMS_CAN_Fault_Hardware_t;

typedef struct {
    BMS_CAN_Fault_Voltage_t       voltage;
    BMS_CAN_Fault_Temperature_t   temperature;
    BMS_CAN_Fault_Current_SOC_t   current_soc;
    BMS_CAN_Fault_Differential_t  differential;
    BMS_CAN_Fault_MOS_t           mos;
    BMS_CAN_Fault_System_t        system;
    BMS_CAN_Fault_Hardware_t      hardware;
    uint8_t                       fault_code;
} BMS_CAN_Fault_Status_t;

/* ══════════════════════════════════════════════
 *  MASTER DATA STRUCT
 * ══════════════════════════════════════════════ */
typedef struct {
    BMS_CAN_SOC_Voltage_Current_t   soc_voltage_current;
    BMS_CAN_Cell_Voltage_Extremes_t cell_voltage_extremes;
    BMS_CAN_Temperature_Extremes_t  temperature_extremes;
    BMS_CAN_MOS_Status_t            mos_status;
    BMS_CAN_Status_Info_t           status_info;
    BMS_CAN_Cell_Voltages_t         cell_voltages;
    BMS_CAN_Cell_Temperatures_t     cell_temperatures;
    BMS_CAN_Cell_Balance_t          cell_balance;
    BMS_CAN_Fault_Status_t          fault_status;
} BMS_CAN_Data_t;

typedef struct {
    BMS_CAN_Handle_t    can;                /* Platform CAN handle              */
    BMS_CAN_Data_t      BMS_Data;           /* Parsed BMS data (all 0x90~0x98)  */
    BMS_CAN_Frame_t     tx_frame;           /* Reusable TX frame buffer         */
    BMS_CAN_Frame_t     rx_frame;           /* Reusable RX frame buffer         */
    uint32_t            rx_fifo;            /* Which RX FIFO to use (0 or 1)    */
    bool                initialized;        /* true after BMS_CAN_Init()        */
} BMS_CAN_Handle_t_s;

/* ══════════════════════════════════════════════
 *  PUBLIC API - FUNCTION PROTOTYPES
 * ══════════════════════════════════════════════ */

/*-----------------------------------------------
 * Initialization
 * Call once per BMS in main() before any other API.
 * Configures CAN filter for Daly BMS extended IDs.
 *
 * Example:
 *   BMS_CAN_Handle_t_s bms;
 *   BMS_CAN_Init(&bms, &hcan1, CAN_RX_FIFO0);
 *----------------------------------------------*/
BMS_CAN_Status_e BMS_CAN_Init(BMS_CAN_Handle_t_s *bms, BMS_CAN_Handle_t can_handle, uint32_t rx_fifo);

/*-----------------------------------------------
 * CAN Filter Configuration
 * Sets up a hardware filter to accept Daly BMS
 * response IDs (0x18XX4001) on the specified FIFO.
 * Called internally by BMS_CAN_Init().
 *----------------------------------------------*/
BMS_CAN_Status_e BMS_CAN_ConfigFilter(BMS_CAN_Handle_t_s *bms, uint32_t filter_bank);

/*-----------------------------------------------
 * Universal TX/RX (low-level)
 * You normally don't call these directly.
 * The individual getter functions use them internally.
 *----------------------------------------------*/
BMS_CAN_Status_e BMS_CAN_SendCommand(BMS_CAN_Handle_t_s *bms, uint8_t data_id);
BMS_CAN_Status_e BMS_CAN_ReceiveResponse(BMS_CAN_Handle_t_s *bms, uint8_t expected_data_id);

/*-----------------------------------------------
 * Individual Data Getter
 * Each sends the request, receives the response,
 * parses it, and stores into bms->BMS_Data.xxx
 *----------------------------------------------*/
BMS_CAN_Status_e BMS_CAN_GetSOCVoltageCurrent(BMS_CAN_Handle_t_s *bms);       /* 0x90 */
BMS_CAN_Status_e BMS_CAN_GetCellVoltageExtremes(BMS_CAN_Handle_t_s *bms);     /* 0x91 */
BMS_CAN_Status_e BMS_CAN_GetTemperatureExtremes(BMS_CAN_Handle_t_s *bms);     /* 0x92 */
BMS_CAN_Status_e BMS_CAN_GetMOSStatus(BMS_CAN_Handle_t_s *bms);               /* 0x93 */
BMS_CAN_Status_e BMS_CAN_GetStatusInfo(BMS_CAN_Handle_t_s *bms);              /* 0x94 */
BMS_CAN_Status_e BMS_CAN_GetCellVoltages(BMS_CAN_Handle_t_s *bms);            /* 0x95 */
BMS_CAN_Status_e BMS_CAN_GetCellTemperatures(BMS_CAN_Handle_t_s *bms);        /* 0x96 */
BMS_CAN_Status_e BMS_CAN_GetCellBalance(BMS_CAN_Handle_t_s *bms);             /* 0x97 */
BMS_CAN_Status_e BMS_CAN_GetFaultStatus(BMS_CAN_Handle_t_s *bms);             /* 0x98 */

/*-----------------------------------------------
 * Get All BMS Data in one shot
 * Calls all 9 getter functions sequentially.
 * Returns the FIRST error encountered, or BMS_CAN_OK
 * if all succeed. Continues polling remaining
 * commands even if one fails.
 *----------------------------------------------*/
BMS_CAN_Status_e BMS_CAN_GetAllData(BMS_CAN_Handle_t_s *bms);

#endif /* INC_DALY_BMS_CAN_H_ */
