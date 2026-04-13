/*
 * Daly_BMS_UART.c
 *
 *  Created on: 18-Mar-2026
 *      Author: Akshay
 *
 *  Portable Daly BMS UART Driver Implementation
 */

#include "Daly_BMS_UART.h"

/**
 * @brief Platform-specific UART transmit (blocking)
 * @param bms    BMS handle
 * @param buf    Data buffer to send
 * @param len    Number of bytes to send
 * @return BMS_OK on success, BMS_ERR_UART on failure
 */
static BMS_Status_e BMS_UART_Transmit(BMS_Handle_t *bms, uint8_t *buf, uint16_t len)
{
#if BMS_PLATFORM_STM32_HAL
    if (HAL_UART_Transmit(bms->uart, buf, len, BMS_RESPONSE_TIMEOUT_MS) != HAL_OK) {
        return BMS_ERR_UART;
    }
    return BMS_OK;
#else
    /* ── Implement for your platform ── */
    /* Example for ESP-IDF:
     *   if (uart_write_bytes(bms->uart_port, buf, len) < 0)
     *       return BMS_ERR_UART;
     *   return BMS_OK;
     */
    (void)bms; (void)buf; (void)len;
    return BMS_ERR_UART;
#endif
}

/**
 * @brief Platform-specific UART receive (blocking with timeout)
 * @param bms    BMS handle
 * @param buf    Buffer to store received data
 * @param len    Number of bytes expected
 * @return BMS_OK on success, BMS_ERR_TIMEOUT / BMS_ERR_UART on failure
 */
static BMS_Status_e BMS_UART_Receive(BMS_Handle_t *bms, uint8_t *buf, uint16_t len)
{
#if BMS_PLATFORM_STM32_HAL
    HAL_StatusTypeDef status = HAL_UART_Receive(bms->uart, buf, len, BMS_RESPONSE_TIMEOUT_MS);
    if (status == HAL_TIMEOUT) {
        return BMS_ERR_TIMEOUT;
    } else if (status != HAL_OK) {
        return BMS_ERR_UART;
    }
    return BMS_OK;
#else
    /* ── Implement for your platform ── */
    (void)bms; (void)buf; (void)len;
    return BMS_ERR_UART;
#endif
}

/**
 * @brief Platform-specific millisecond delay
 * @param ms   Milliseconds to delay
 */
static void BMS_Delay(uint32_t ms)
{
#if BMS_PLATFORM_STM32_HAL
    HAL_Delay(ms);
#else
    /* ── Implement for your platform ── */
    (void)ms;
#endif
}

/* ══════════════════════════════════════════════
 *  PRIVATE: Checksum Calculation
 * ══════════════════════════════════════════════ */

/**
 * @brief Calculate checksum (sum of all bytes, keep low byte only)
 * @param buf    Buffer of bytes
 * @param len    Number of bytes to sum
 * @return Low byte of sum
 */
static uint8_t BMS_CalcChecksum(const uint8_t *buf, uint16_t len)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum += buf[i];
    }
    return checksum;
}

/* ══════════════════════════════════════════════
 *  PRIVATE: Frame Build & Validate
 * ══════════════════════════════════════════════ */

/**
 * @brief Build a TX frame into bms->tx_buf
 * @param bms      BMS handle
 * @param data_id  Command ID (0x90~0x98)
 * @return Total frame length in bytes
 */
static uint16_t BMS_BuildRequestFrame(BMS_Handle_t *bms, uint8_t data_id)
{
    uint16_t idx = 0;

    bms->tx_buf[idx++] = BMS_START_FLAG;            /* Byte 0: Start      */
    bms->tx_buf[idx++] = BMS_ADDR_UPPER_COMPUTER;   /* Byte 1: Address    */
    bms->tx_buf[idx++] = data_id;                    /* Byte 2: Data ID    */
    bms->tx_buf[idx++] = BMS_STANDARD_DATA_LEN;     /* Byte 3: Length = 8 */

    /* Byte 4~11: Data content = all zeros (request) */
    for (uint8_t i = 0; i < BMS_STANDARD_DATA_LEN; i++) {
        bms->tx_buf[idx++] = 0x00;
    }

    /* Byte 12: Checksum */
    bms->tx_buf[idx] = BMS_CalcChecksum(bms->tx_buf, idx);
    idx++;

    return idx; /* Should be BMS_STANDARD_FRAME_LEN = 13 */
}

/**
 * @brief Validate a received frame in bms->rx_buf
 * @param bms              BMS handle
 * @param expected_data_id Expected Data ID in the response
 * @param rx_len           Number of bytes received
 * @return BMS_OK if valid, appropriate error code otherwise
 */
static BMS_Status_e BMS_ValidateResponseFrame(BMS_Handle_t *bms, uint8_t expected_data_id, uint16_t rx_len)
{
    /* Check start flag */
    if (bms->rx_buf[0] != BMS_START_FLAG) {
        return BMS_ERR_FRAME;
    }

    /* Check BMS address */
    if (bms->rx_buf[1] != BMS_ADDR_BMS_MASTER) {
        return BMS_ERR_FRAME;
    }

    /* Check Data ID matches what we requested */
    if (bms->rx_buf[2] != expected_data_id) {
        return BMS_ERR_DATA_ID;
    }

    /* Verify checksum: sum of all bytes before checksum */
    uint8_t calc_cs = BMS_CalcChecksum(bms->rx_buf, rx_len - 1);
    if (calc_cs != bms->rx_buf[rx_len - 1]) {
        return BMS_ERR_CHECKSUM;
    }

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Initialization
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_Init(BMS_Handle_t *bms, BMS_UART_Handle_t uart_handle)
{
    if (bms == NULL) {
        return BMS_ERR_NULL_PTR;
    }

    /* Zero out entire handle */
    memset(bms, 0, sizeof(BMS_Handle_t));

    /* Store UART handle */
    bms->uart = uart_handle;
    bms->initialized = true;

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Universal Send Command
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_SendCommand(BMS_Handle_t *bms, uint8_t data_id)
{
    if (bms == NULL)         return BMS_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_ERR_NOT_INIT;

    /* Build the 13-byte request frame */
    uint16_t frame_len = BMS_BuildRequestFrame(bms, data_id);

    /* Transmit over UART */
    return BMS_UART_Transmit(bms, bms->tx_buf, frame_len);
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Universal Receive Response
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_ReceiveResponse(BMS_Handle_t *bms, uint8_t expected_data_id, uint16_t rx_len)
{
    if (bms == NULL)         return BMS_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_ERR_NOT_INIT;

    /* Clear RX buffer */
    memset(bms->rx_buf, 0, sizeof(bms->rx_buf));

    /* Receive rx_len bytes from UART */
    BMS_Status_e status = BMS_UART_Receive(bms, bms->rx_buf, rx_len);
    if (status != BMS_OK) {
        return status;
    }

    /* Validate the frame */
    return BMS_ValidateResponseFrame(bms, expected_data_id, rx_len);
}

/* ══════════════════════════════════════════════
 *  PRIVATE: Helper - Send command & receive standard 13-byte response
 * ══════════════════════════════════════════════ */

static BMS_Status_e BMS_RequestStandard(BMS_Handle_t *bms, uint8_t data_id)
{
    BMS_Status_e status;

    status = BMS_SendCommand(bms, data_id);
    if (status != BMS_OK) return status;

    status = BMS_ReceiveResponse(bms, data_id, BMS_STANDARD_FRAME_LEN);
    return status;
}

/* ══════════════════════════════════════════════
 *  Pointer to data payload (skips header)
 *  After receiving, rx_buf[4] is Byte0 of data
 * ══════════════════════════════════════════════ */
#define BMS_RX_DATA(bms)  (&(bms)->rx_buf[BMS_FRAME_HEADER_SIZE])

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x90 - SOC, Voltage & Current
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetSOCVoltageCurrent(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_SOC_VOLTAGE_CURRENT);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    uint16_t raw_cumulative = BMS_BYTES_TO_U16(d[0], d[1]);
    uint16_t raw_gather     = BMS_BYTES_TO_U16(d[2], d[3]);
    uint16_t raw_current    = BMS_BYTES_TO_U16(d[4], d[5]);
    uint16_t raw_soc        = BMS_BYTES_TO_U16(d[6], d[7]);

    bms->BMS_Data.soc_voltage_current.cumulative_total_voltage = BMS_RAW_TO_VOLTAGE(raw_cumulative);
    bms->BMS_Data.soc_voltage_current.gather_total_voltage     = BMS_RAW_TO_VOLTAGE(raw_gather);
    bms->BMS_Data.soc_voltage_current.current                  = BMS_RAW_TO_CURRENT(raw_current);
    bms->BMS_Data.soc_voltage_current.soc                      = BMS_RAW_TO_SOC(raw_soc);

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x91 - Cell Voltage Extremes
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetCellVoltageExtremes(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_CELL_VOLTAGE_EXTREMES);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    bms->BMS_Data.cell_voltage_extremes.max_cell_voltage_mV = BMS_BYTES_TO_U16(d[0], d[1]);
    bms->BMS_Data.cell_voltage_extremes.max_voltage_cell_num = d[2];
    bms->BMS_Data.cell_voltage_extremes.min_cell_voltage_mV = BMS_BYTES_TO_U16(d[3], d[4]);
    bms->BMS_Data.cell_voltage_extremes.min_voltage_cell_num = d[5];

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x92 - Temperature Extremes
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetTemperatureExtremes(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_TEMP_EXTREMES);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    bms->BMS_Data.temperature_extremes.max_temperature_degC = BMS_RAW_TO_TEMP(d[0]);
    bms->BMS_Data.temperature_extremes.max_temp_sensor_num  = d[1];
    bms->BMS_Data.temperature_extremes.min_temperature_degC = BMS_RAW_TO_TEMP(d[2]);
    bms->BMS_Data.temperature_extremes.min_temp_sensor_num  = d[3];

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x93 - MOS Status
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetMOSStatus(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_MOS_STATUS);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    bms->BMS_Data.mos_status.operating_state       = (BMS_Operating_State_e)d[0];
    bms->BMS_Data.mos_status.charge_mos            = (MOS_State_e)d[1];
    bms->BMS_Data.mos_status.discharge_mos         = (MOS_State_e)d[2];
    bms->BMS_Data.mos_status.bms_life_cycles       = d[3];
    bms->BMS_Data.mos_status.remaining_capacity_mAh = BMS_BYTES_TO_U32(d[4], d[5], d[6], d[7]);

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x94 - Status Information
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetStatusInfo(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_STATUS_INFO);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    bms->BMS_Data.status_info.num_battery_strings    = d[0];
    bms->BMS_Data.status_info.num_temperature_sensors = d[1];
    bms->BMS_Data.status_info.charger_status         = (Connection_Status_e)d[2];
    bms->BMS_Data.status_info.load_status            = (Connection_Status_e)d[3];

    /* Parse Digital I/O from Byte4 */
    uint8_t dio = d[4];
    bms->BMS_Data.status_info.digital_io.di1 = BMS_GET_BIT(dio, 0);
    bms->BMS_Data.status_info.digital_io.di2 = BMS_GET_BIT(dio, 1);
    bms->BMS_Data.status_info.digital_io.di3 = BMS_GET_BIT(dio, 2);
    bms->BMS_Data.status_info.digital_io.di4 = BMS_GET_BIT(dio, 3);
    bms->BMS_Data.status_info.digital_io.do1 = BMS_GET_BIT(dio, 4);
    bms->BMS_Data.status_info.digital_io.do2 = BMS_GET_BIT(dio, 5);
    bms->BMS_Data.status_info.digital_io.do3 = BMS_GET_BIT(dio, 6);
    bms->BMS_Data.status_info.digital_io.do4 = BMS_GET_BIT(dio, 7);

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x95 - Individual Cell Voltages
 *
 *  Multi-frame: BMS sends multiple 13-byte frames.
 *  Each frame: Byte0 = frame number (0~15, 0xFF = invalid)
 *              Byte1~Byte6 = 3 cell voltages (2 bytes each)
 *              Byte7 = reserved
 *
 *  We first need to know how many cells exist.
 *  Call BMS_GetStatusInfo() first, or use bms->BMS_Data.status_info.num_battery_strings.
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetCellVoltages(BMS_Handle_t *bms)
{
    if (bms == NULL)         return BMS_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_ERR_NOT_INIT;

    /* Send the request once */
    BMS_Status_e status = BMS_SendCommand(bms, BMS_CMD_CELL_VOLTAGES);
    if (status != BMS_OK) return status;

    /* Determine number of cells (use status_info if already fetched, else default to max) */
    uint8_t num_cells = bms->BMS_Data.status_info.num_battery_strings;
    if (num_cells == 0 || num_cells > BMS_MAX_CELLS) {
        num_cells = BMS_MAX_CELLS;
    }

    /* Calculate how many frames to expect: 3 cells per frame, round up */
    uint8_t num_frames = (num_cells + 2) / 3;
    uint8_t cell_idx = 0;

    for (uint8_t frame = 0; frame < num_frames; frame++) {
        /* Receive one 13-byte frame */
        status = BMS_ReceiveResponse(bms, BMS_CMD_CELL_VOLTAGES, BMS_STANDARD_FRAME_LEN);
        if (status != BMS_OK) return status;

        uint8_t *d = BMS_RX_DATA(bms);

        /* d[0] = frame number (skip 0xFF invalid frames) */
        if (d[0] == 0xFF) continue;

        /* d[1]~d[6] = 3 cell voltages, 2 bytes each */
        for (uint8_t i = 0; i < 3 && cell_idx < num_cells; i++) {
            uint16_t mv = BMS_BYTES_TO_U16(d[1 + i * 2], d[2 + i * 2]);
            bms->BMS_Data.cell_voltages.cell_voltage_mV[cell_idx] = mv;
            cell_idx++;
        }
    }

    bms->BMS_Data.cell_voltages.num_cells = cell_idx;
    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x96 - Individual Cell Temperatures
 *
 *  Multi-frame: BMS sends multiple 13-byte frames.
 *  Each frame: Byte0 = frame number (starting at 0)
 *              Byte1~Byte7 = 7 temperature values (1 byte each, 40°C offset)
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetCellTemperatures(BMS_Handle_t *bms)
{
    if (bms == NULL)         return BMS_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_ERR_NOT_INIT;

    /* Send the request once */
    BMS_Status_e status = BMS_SendCommand(bms, BMS_CMD_CELL_TEMPERATURES);
    if (status != BMS_OK) return status;

    /* Determine number of sensors */
    uint8_t num_sensors = bms->BMS_Data.status_info.num_temperature_sensors;
    if (num_sensors == 0 || num_sensors > BMS_MAX_TEMP_SENSORS) {
        num_sensors = BMS_MAX_TEMP_SENSORS;
    }

    /* Calculate frames: 7 temps per frame, round up */
    uint8_t num_frames = (num_sensors + 6) / 7;
    uint8_t sensor_idx = 0;

    for (uint8_t frame = 0; frame < num_frames; frame++) {
        status = BMS_ReceiveResponse(bms, BMS_CMD_CELL_TEMPERATURES, BMS_STANDARD_FRAME_LEN);
        if (status != BMS_OK) return status;

        uint8_t *d = BMS_RX_DATA(bms);

        /* d[0] = frame number */
        /* d[1]~d[7] = 7 temperature values */
        for (uint8_t i = 0; i < 7 && sensor_idx < num_sensors; i++) {
            bms->BMS_Data.cell_temperatures.temperature_degC[sensor_idx] = BMS_RAW_TO_TEMP(d[1 + i]);
            sensor_idx++;
        }
    }

    bms->BMS_Data.cell_temperatures.num_sensors = sensor_idx;
    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x97 - Cell Balance States
 *
 *  8 bytes of data = 64 bits.
 *  Bit0 = Cell 1, ... Bit47 = Cell 48.
 *  Bit48~63 = reserved.
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetCellBalance(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_CELL_BALANCE);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    for (uint8_t cell = 0; cell < BMS_MAX_CELLS; cell++) {
        uint8_t byte_idx = cell / 8;
        uint8_t bit_idx  = cell % 8;
        bms->BMS_Data.cell_balance.cell_balance[cell] = (Balance_State_e)BMS_GET_BIT(d[byte_idx], bit_idx);
    }

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x98 - Fault Status
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetFaultStatus(BMS_Handle_t *bms)
{
    BMS_Status_e status = BMS_RequestStandard(bms, BMS_CMD_FAULT_STATUS);
    if (status != BMS_OK) return status;

    uint8_t *d = BMS_RX_DATA(bms);

    /* Byte 0 - Voltage faults */
    bms->BMS_Data.fault_status.voltage.cell_volt_high_lvl1 = (Fault_Flag_e)BMS_GET_BIT(d[0], 0);
    bms->BMS_Data.fault_status.voltage.cell_volt_high_lvl2 = (Fault_Flag_e)BMS_GET_BIT(d[0], 1);
    bms->BMS_Data.fault_status.voltage.cell_volt_low_lvl1  = (Fault_Flag_e)BMS_GET_BIT(d[0], 2);
    bms->BMS_Data.fault_status.voltage.cell_volt_low_lvl2  = (Fault_Flag_e)BMS_GET_BIT(d[0], 3);
    bms->BMS_Data.fault_status.voltage.sum_volt_high_lvl1  = (Fault_Flag_e)BMS_GET_BIT(d[0], 4);
    bms->BMS_Data.fault_status.voltage.sum_volt_high_lvl2  = (Fault_Flag_e)BMS_GET_BIT(d[0], 5);
    bms->BMS_Data.fault_status.voltage.sum_volt_low_lvl1   = (Fault_Flag_e)BMS_GET_BIT(d[0], 6);
    bms->BMS_Data.fault_status.voltage.sum_volt_low_lvl2   = (Fault_Flag_e)BMS_GET_BIT(d[0], 7);

    /* Byte 1 - Temperature faults */
    bms->BMS_Data.fault_status.temperature.chg_temp_high_lvl1    = (Fault_Flag_e)BMS_GET_BIT(d[1], 0);
    bms->BMS_Data.fault_status.temperature.chg_temp_high_lvl2    = (Fault_Flag_e)BMS_GET_BIT(d[1], 1);
    bms->BMS_Data.fault_status.temperature.chg_temp_low_lvl1     = (Fault_Flag_e)BMS_GET_BIT(d[1], 2);
    bms->BMS_Data.fault_status.temperature.chg_temp_low_lvl2     = (Fault_Flag_e)BMS_GET_BIT(d[1], 3);
    bms->BMS_Data.fault_status.temperature.dischg_temp_high_lvl1 = (Fault_Flag_e)BMS_GET_BIT(d[1], 4);
    bms->BMS_Data.fault_status.temperature.dischg_temp_high_lvl2 = (Fault_Flag_e)BMS_GET_BIT(d[1], 5);
    bms->BMS_Data.fault_status.temperature.dischg_temp_low_lvl1  = (Fault_Flag_e)BMS_GET_BIT(d[1], 6);
    bms->BMS_Data.fault_status.temperature.dischg_temp_low_lvl2  = (Fault_Flag_e)BMS_GET_BIT(d[1], 7);

    /* Byte 2 - Current & SOC faults */
    bms->BMS_Data.fault_status.current_soc.chg_overcurrent_lvl1    = (Fault_Flag_e)BMS_GET_BIT(d[2], 0);
    bms->BMS_Data.fault_status.current_soc.chg_overcurrent_lvl2    = (Fault_Flag_e)BMS_GET_BIT(d[2], 1);
    bms->BMS_Data.fault_status.current_soc.dischg_overcurrent_lvl1 = (Fault_Flag_e)BMS_GET_BIT(d[2], 2);
    bms->BMS_Data.fault_status.current_soc.dischg_overcurrent_lvl2 = (Fault_Flag_e)BMS_GET_BIT(d[2], 3);
    bms->BMS_Data.fault_status.current_soc.soc_high_lvl1           = (Fault_Flag_e)BMS_GET_BIT(d[2], 4);
    bms->BMS_Data.fault_status.current_soc.soc_high_lvl2           = (Fault_Flag_e)BMS_GET_BIT(d[2], 5);
    bms->BMS_Data.fault_status.current_soc.soc_low_lvl1            = (Fault_Flag_e)BMS_GET_BIT(d[2], 6);
    bms->BMS_Data.fault_status.current_soc.soc_low_lvl2            = (Fault_Flag_e)BMS_GET_BIT(d[2], 7);

    /* Byte 3 - Differential faults */
    bms->BMS_Data.fault_status.differential.diff_volt_lvl1 = (Fault_Flag_e)BMS_GET_BIT(d[3], 0);
    bms->BMS_Data.fault_status.differential.diff_volt_lvl2 = (Fault_Flag_e)BMS_GET_BIT(d[3], 1);
    bms->BMS_Data.fault_status.differential.diff_temp_lvl1 = (Fault_Flag_e)BMS_GET_BIT(d[3], 2);
    bms->BMS_Data.fault_status.differential.diff_temp_lvl2 = (Fault_Flag_e)BMS_GET_BIT(d[3], 3);

    /* Byte 4 - MOS faults */
    bms->BMS_Data.fault_status.mos.chg_mos_temp_high         = (Fault_Flag_e)BMS_GET_BIT(d[4], 0);
    bms->BMS_Data.fault_status.mos.dischg_mos_temp_high      = (Fault_Flag_e)BMS_GET_BIT(d[4], 1);
    bms->BMS_Data.fault_status.mos.chg_mos_temp_sensor_err   = (Fault_Flag_e)BMS_GET_BIT(d[4], 2);
    bms->BMS_Data.fault_status.mos.dischg_mos_temp_sensor_err = (Fault_Flag_e)BMS_GET_BIT(d[4], 3);
    bms->BMS_Data.fault_status.mos.chg_mos_adhesion_err      = (Fault_Flag_e)BMS_GET_BIT(d[4], 4);
    bms->BMS_Data.fault_status.mos.dischg_mos_adhesion_err   = (Fault_Flag_e)BMS_GET_BIT(d[4], 5);
    bms->BMS_Data.fault_status.mos.chg_mos_open_circuit_err  = (Fault_Flag_e)BMS_GET_BIT(d[4], 6);
    bms->BMS_Data.fault_status.mos.dischg_mos_open_circuit_err = (Fault_Flag_e)BMS_GET_BIT(d[4], 7);

    /* Byte 5 - System faults */
    bms->BMS_Data.fault_status.system.afe_collect_chip_err     = (Fault_Flag_e)BMS_GET_BIT(d[5], 0);
    bms->BMS_Data.fault_status.system.voltage_collect_dropped  = (Fault_Flag_e)BMS_GET_BIT(d[5], 1);
    bms->BMS_Data.fault_status.system.cell_temp_sensor_err     = (Fault_Flag_e)BMS_GET_BIT(d[5], 2);
    bms->BMS_Data.fault_status.system.eeprom_err               = (Fault_Flag_e)BMS_GET_BIT(d[5], 3);
    bms->BMS_Data.fault_status.system.rtc_err                  = (Fault_Flag_e)BMS_GET_BIT(d[5], 4);
    bms->BMS_Data.fault_status.system.precharge_failure        = (Fault_Flag_e)BMS_GET_BIT(d[5], 5);
    bms->BMS_Data.fault_status.system.communication_failure    = (Fault_Flag_e)BMS_GET_BIT(d[5], 6);
    bms->BMS_Data.fault_status.system.internal_comm_failure    = (Fault_Flag_e)BMS_GET_BIT(d[5], 7);

    /* Byte 6 - Hardware faults */
    bms->BMS_Data.fault_status.hardware.current_module_fault        = (Fault_Flag_e)BMS_GET_BIT(d[6], 0);
    bms->BMS_Data.fault_status.hardware.sum_voltage_detect_fault    = (Fault_Flag_e)BMS_GET_BIT(d[6], 1);
    bms->BMS_Data.fault_status.hardware.short_circuit_protect_fault = (Fault_Flag_e)BMS_GET_BIT(d[6], 2);
    bms->BMS_Data.fault_status.hardware.low_volt_forbidden_chg_fault = (Fault_Flag_e)BMS_GET_BIT(d[6], 3);

    /* Byte 7 - Fault code */
    bms->BMS_Data.fault_status.fault_code = d[7];

    return BMS_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Get All BMS Data
 *
 *  Polls all 9 commands sequentially.
 *  Note: 0x94 (StatusInfo) is called first because
 *  0x95 and 0x96 need num_cells and num_sensors.
 *  Returns first error encountered but continues
 *  polling remaining commands.
 * ══════════════════════════════════════════════ */

BMS_Status_e BMS_GetAllData(BMS_Handle_t *bms)
{
    if (bms == NULL)         return BMS_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_ERR_NOT_INIT;

    BMS_Status_e first_error = BMS_OK;
    BMS_Status_e status;

    /* Helper macro: call function, record first error, add inter-command delay */
    #define BMS_POLL(func) do {                         \
        status = func(bms);                             \
        if (status != BMS_OK && first_error == BMS_OK)  \
            first_error = status;                       \
        BMS_Delay(BMS_INTER_CMD_DELAY_MS);              \
    } while(0)

    /* 0x94 first — needed by 0x95 and 0x96 for cell/sensor count */
    BMS_POLL(BMS_GetStatusInfo);

    /* Then the rest in order */
    BMS_POLL(BMS_GetSOCVoltageCurrent);
    BMS_POLL(BMS_GetCellVoltageExtremes);
    BMS_POLL(BMS_GetTemperatureExtremes);
    BMS_POLL(BMS_GetMOSStatus);
    BMS_POLL(BMS_GetCellVoltages);
    BMS_POLL(BMS_GetCellTemperatures);
    BMS_POLL(BMS_GetCellBalance);
    BMS_POLL(BMS_GetFaultStatus);

    #undef BMS_POLL

    return first_error;
}

