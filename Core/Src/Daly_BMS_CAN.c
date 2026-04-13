/*
 * Daly_BMS_CAN.c
 *
 *  Created on: 18-Mar-2026
 *      Author: Akshay
 *
 *  Portable Daly BMS CAN Driver
 *
 *  CAN ID Format (29-bit Extended):
 *    TX (PC -> BMS): 0x18 + DataID + 0x01 + 0x40 = 0x18XX0140
 *    RX (BMS -> PC): 0x18 + DataID + 0x40 + 0x01 = 0x18XX4001
 */

#include "Daly_BMS_CAN.h"

/* ══════════════════════════════════════════════
 *  PLATFORM ABSTRACTION: CAN TX/RX
 * ══════════════════════════════════════════════ */

/**
 * @brief Platform-specific CAN transmit (blocking)
 * @param bms      BMS CAN handle
 * @param frame    CAN frame to send
 * @return BMS_CAN_OK on success, BMS_CAN_ERR_HAL on failure
 */
static BMS_CAN_Status_e BMS_CAN_Transmit(BMS_CAN_Handle_t_s *bms, BMS_CAN_Frame_t *frame)
{
#if BMS_CAN_PLATFORM_STM32_HAL
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.ExtId = frame->id;
    tx_header.IDE   = CAN_ID_EXT;           /* 29-bit extended ID           */
    tx_header.RTR   = CAN_RTR_DATA;         /* Data frame, not remote       */
    tx_header.DLC   = frame->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* Wait for a free TX mailbox with timeout */
    uint32_t start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(bms->can) == 0) {
        if ((HAL_GetTick() - start_tick) > BMS_CAN_RESPONSE_TIMEOUT_MS) {
            return BMS_CAN_ERR_TIMEOUT;
        }
    }

    if (HAL_CAN_AddTxMessage(bms->can, &tx_header, frame->data, &tx_mailbox) != HAL_OK) {
        return BMS_CAN_ERR_HAL;
    }

    /* Wait for TX complete */
    start_tick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(bms->can, tx_mailbox)) {
        if ((HAL_GetTick() - start_tick) > BMS_CAN_RESPONSE_TIMEOUT_MS) {
            return BMS_CAN_ERR_TIMEOUT;
        }
    }

    return BMS_CAN_OK;
#else
    /* ── Implement for your platform ── */
    (void)bms; (void)frame;
    return BMS_CAN_ERR_HAL;
#endif
}

/**
 * @brief Platform-specific CAN receive (blocking with timeout)
 * @param bms      BMS CAN handle
 * @param frame    Buffer to store received CAN frame
 * @return BMS_CAN_OK on success, BMS_CAN_ERR_TIMEOUT / BMS_CAN_ERR_HAL on failure
 */
static BMS_CAN_Status_e BMS_CAN_Receive(BMS_CAN_Handle_t_s *bms, BMS_CAN_Frame_t *frame)
{
#if BMS_CAN_PLATFORM_STM32_HAL
    CAN_RxHeaderTypeDef rx_header;

    /* Poll for message in RX FIFO with timeout */
    uint32_t start_tick = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(bms->can, bms->rx_fifo) == 0) {
        if ((HAL_GetTick() - start_tick) > BMS_CAN_RESPONSE_TIMEOUT_MS) {
            return BMS_CAN_ERR_TIMEOUT;
        }
    }

    if (HAL_CAN_GetRxMessage(bms->can, bms->rx_fifo, &rx_header, frame->data) != HAL_OK) {
        return BMS_CAN_ERR_HAL;
    }

    /* Store received ID and DLC */
    if (rx_header.IDE == CAN_ID_EXT) {
        frame->id = rx_header.ExtId;
    } else {
        frame->id = rx_header.StdId;
    }
    frame->dlc = rx_header.DLC;

    return BMS_CAN_OK;
#else
    /* ── Implement for your platform ── */
    (void)bms; (void)frame;
    return BMS_CAN_ERR_HAL;
#endif
}

/**
 * @brief Platform-specific millisecond delay
 * @param ms   Milliseconds to delay
 */
static void BMS_CAN_Delay(uint32_t ms)
{
#if BMS_CAN_PLATFORM_STM32_HAL
    HAL_Delay(ms);
#else
    /* ── Implement for your platform ── */
    (void)ms;
#endif
}

/* ══════════════════════════════════════════════
 *  PRIVATE: Frame Build & Validate
 * ══════════════════════════════════════════════ */

/**
 * @brief Build a TX CAN frame for a request
 * @param frame    Output frame to populate
 * @param data_id  Command ID (0x90~0x98)
 */
static void BMS_CAN_BuildRequestFrame(BMS_CAN_Frame_t *frame, uint8_t data_id)
{
    frame->id  = BMS_CAN_BUILD_TX_ID(data_id);
    frame->dlc = BMS_CAN_DATA_LEN;

    /* Data content = all zeros for request */
    memset(frame->data, 0x00, BMS_CAN_DATA_LEN);
}

/**
 * @brief Validate a received CAN frame
 * @param frame            Received frame
 * @param expected_data_id Expected Data ID in the response
 * @return BMS_CAN_OK if valid, appropriate error code otherwise
 */
static BMS_CAN_Status_e BMS_CAN_ValidateResponseFrame(BMS_CAN_Frame_t *frame, uint8_t expected_data_id)
{
    /* Build expected RX CAN ID */
    uint32_t expected_id = BMS_CAN_BUILD_RX_ID(expected_data_id);

    /* Check CAN ID matches */
    if (frame->id != expected_id) {
        /* Check if at least the data ID portion matches */
        uint8_t rx_data_id = BMS_CAN_EXTRACT_DATA_ID(frame->id);
        if (rx_data_id != expected_data_id) {
            return BMS_CAN_ERR_DATA_ID;
        }
        return BMS_CAN_ERR_CAN_ID;
    }

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PRIVATE: Helper - Send command & receive single response
 * ══════════════════════════════════════════════ */

static BMS_CAN_Status_e BMS_CAN_RequestStandard(BMS_CAN_Handle_t_s *bms, uint8_t data_id)
{
    BMS_CAN_Status_e status;

    status = BMS_CAN_SendCommand(bms, data_id);
    if (status != BMS_CAN_OK) return status;

    status = BMS_CAN_ReceiveResponse(bms, data_id);
    return status;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Initialization
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_Init(BMS_CAN_Handle_t_s *bms, BMS_CAN_Handle_t can_handle, uint32_t rx_fifo)
{
    if (bms == NULL) {
        return BMS_CAN_ERR_NULL_PTR;
    }

    /* Zero out entire handle */
    memset(bms, 0, sizeof(BMS_CAN_Handle_t_s));

    /* Store CAN handle and FIFO selection */
    bms->can     = can_handle;
    bms->rx_fifo = rx_fifo;

    /* Configure CAN filter for Daly BMS responses */
    BMS_CAN_Status_e status = BMS_CAN_ConfigFilter(bms, 0);
    if (status != BMS_CAN_OK) {
        return status;
    }

#if BMS_CAN_PLATFORM_STM32_HAL
    /* Start the CAN peripheral */
    if (HAL_CAN_Start(bms->can) != HAL_OK) {
        return BMS_CAN_ERR_HAL;
    }
#endif

    bms->initialized = true;
    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: CAN Filter Configuration
 *
 *  Accepts all Daly BMS response frames:
 *  ID pattern:  0x18XX4001 (where XX = 0x90~0x98)
 *  Mask:        0x1F00FFFF (match priority + src/dst addr,
 *               allow any data ID in bits 16~23)
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_ConfigFilter(BMS_CAN_Handle_t_s *bms, uint32_t filter_bank)
{
#if BMS_CAN_PLATFORM_STM32_HAL
    CAN_FilterTypeDef filter;

    /*
     * We want to accept IDs of form: 0x18XX4001
     * Filter ID:   0x18004001 (base pattern)
     * Filter Mask: 0x1F00FFFF (ignore bits 16~23 = Data ID byte)
     *
     * STM32 CAN filter for 29-bit IDs:
     *   FilterIdHigh   = (ID >> 13) & 0xFFFF
     *   FilterIdLow    = ((ID << 3) & 0xFFF8) | IDE_bit(0x04)
     *   Same for mask registers
     */
    uint32_t filter_id   = 0x18004001;  /* Base response ID pattern      */
    uint32_t filter_mask = 0x1F00FFFF;  /* Match all except Data ID byte */

    filter.FilterBank           = filter_bank;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((filter_id >> 13) & 0xFFFF);
    filter.FilterIdLow          = (uint16_t)(((filter_id << 3) & 0xFFF8) | 0x04); /* IDE = 1 */
    filter.FilterMaskIdHigh     = (uint16_t)((filter_mask >> 13) & 0xFFFF);
    filter.FilterMaskIdLow      = (uint16_t)(((filter_mask << 3) & 0xFFF8) | 0x04);
    filter.FilterFIFOAssignment = bms->rx_fifo;
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14; /* For dual-CAN MCUs (CAN1 uses 0~13) */

    if (HAL_CAN_ConfigFilter(bms->can, &filter) != HAL_OK) {
        return BMS_CAN_ERR_FILTER;
    }

    return BMS_CAN_OK;
#else
    /* ── Implement for your platform ── */
    (void)bms; (void)filter_bank;
    return BMS_CAN_ERR_FILTER;
#endif
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Universal Send Command
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_SendCommand(BMS_CAN_Handle_t_s *bms, uint8_t data_id)
{
    if (bms == NULL)         return BMS_CAN_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_CAN_ERR_NOT_INIT;

    /* Build the CAN request frame */
    BMS_CAN_BuildRequestFrame(&bms->tx_frame, data_id);

    /* Transmit over CAN */
    return BMS_CAN_Transmit(bms, &bms->tx_frame);
}

/* ══════════════════════════════════════════════
 *  PUBLIC: Universal Receive Response
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_ReceiveResponse(BMS_CAN_Handle_t_s *bms, uint8_t expected_data_id)
{
    if (bms == NULL)         return BMS_CAN_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_CAN_ERR_NOT_INIT;

    /* Clear RX frame */
    memset(&bms->rx_frame, 0, sizeof(BMS_CAN_Frame_t));

    /* Receive a CAN frame */
    BMS_CAN_Status_e status = BMS_CAN_Receive(bms, &bms->rx_frame);
    if (status != BMS_CAN_OK) {
        return status;
    }

    /* Validate the frame */
    return BMS_CAN_ValidateResponseFrame(&bms->rx_frame, expected_data_id);
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x90 - SOC, Voltage & Current
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetSOCVoltageCurrent(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_SOC_VOLTAGE_CURRENT);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    uint16_t raw_cumulative = BMS_CAN_BYTES_TO_U16(d[0], d[1]);
    uint16_t raw_gather     = BMS_CAN_BYTES_TO_U16(d[2], d[3]);
    uint16_t raw_current    = BMS_CAN_BYTES_TO_U16(d[4], d[5]);
    uint16_t raw_soc        = BMS_CAN_BYTES_TO_U16(d[6], d[7]);

    bms->BMS_Data.soc_voltage_current.cumulative_total_voltage = BMS_CAN_RAW_TO_VOLTAGE(raw_cumulative);
    bms->BMS_Data.soc_voltage_current.gather_total_voltage     = BMS_CAN_RAW_TO_VOLTAGE(raw_gather);
    bms->BMS_Data.soc_voltage_current.current                  = BMS_CAN_RAW_TO_CURRENT(raw_current);
    bms->BMS_Data.soc_voltage_current.soc                      = BMS_CAN_RAW_TO_SOC(raw_soc);

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x91 - Cell Voltage Extremes
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetCellVoltageExtremes(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_CELL_VOLTAGE_EXTREMES);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    bms->BMS_Data.cell_voltage_extremes.max_cell_voltage_mV  = BMS_CAN_BYTES_TO_U16(d[0], d[1]);
    bms->BMS_Data.cell_voltage_extremes.max_voltage_cell_num = d[2];
    bms->BMS_Data.cell_voltage_extremes.min_cell_voltage_mV  = BMS_CAN_BYTES_TO_U16(d[3], d[4]);
    bms->BMS_Data.cell_voltage_extremes.min_voltage_cell_num = d[5];

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x92 - Temperature Extremes
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetTemperatureExtremes(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_TEMP_EXTREMES);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    bms->BMS_Data.temperature_extremes.max_temperature_degC = BMS_CAN_RAW_TO_TEMP(d[0]);
    bms->BMS_Data.temperature_extremes.max_temp_sensor_num  = d[1];
    bms->BMS_Data.temperature_extremes.min_temperature_degC = BMS_CAN_RAW_TO_TEMP(d[2]);
    bms->BMS_Data.temperature_extremes.min_temp_sensor_num  = d[3];

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x93 - MOS Status
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetMOSStatus(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_MOS_STATUS);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    bms->BMS_Data.mos_status.operating_state        = (BMS_CAN_Operating_State_e)d[0];
    bms->BMS_Data.mos_status.charge_mos             = (BMS_CAN_MOS_State_e)d[1];
    bms->BMS_Data.mos_status.discharge_mos          = (BMS_CAN_MOS_State_e)d[2];
    bms->BMS_Data.mos_status.bms_life_cycles        = d[3];
    bms->BMS_Data.mos_status.remaining_capacity_mAh = BMS_CAN_BYTES_TO_U32(d[4], d[5], d[6], d[7]);

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x94 - Status Information
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetStatusInfo(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_STATUS_INFO);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    bms->BMS_Data.status_info.num_battery_strings     = d[0];
    bms->BMS_Data.status_info.num_temperature_sensors  = d[1];
    bms->BMS_Data.status_info.charger_status           = (BMS_CAN_Connection_Status_e)d[2];
    bms->BMS_Data.status_info.load_status              = (BMS_CAN_Connection_Status_e)d[3];

    uint8_t dio = d[4];
    bms->BMS_Data.status_info.digital_io.di1 = BMS_CAN_GET_BIT(dio, 0);
    bms->BMS_Data.status_info.digital_io.di2 = BMS_CAN_GET_BIT(dio, 1);
    bms->BMS_Data.status_info.digital_io.di3 = BMS_CAN_GET_BIT(dio, 2);
    bms->BMS_Data.status_info.digital_io.di4 = BMS_CAN_GET_BIT(dio, 3);
    bms->BMS_Data.status_info.digital_io.do1 = BMS_CAN_GET_BIT(dio, 4);
    bms->BMS_Data.status_info.digital_io.do2 = BMS_CAN_GET_BIT(dio, 5);
    bms->BMS_Data.status_info.digital_io.do3 = BMS_CAN_GET_BIT(dio, 6);
    bms->BMS_Data.status_info.digital_io.do4 = BMS_CAN_GET_BIT(dio, 7);

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x95 - Individual Cell Voltages
 *
 *  Multi-frame: BMS sends multiple CAN frames.
 *  Each frame (8 bytes):
 *    Byte0 = frame number (starting from 0, 0xFF = invalid)
 *    Byte1~Byte6 = 3 cell voltages (2 bytes each, 1mV)
 *    Byte7 = reserved
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetCellVoltages(BMS_CAN_Handle_t_s *bms)
{
    if (bms == NULL)         return BMS_CAN_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_CAN_ERR_NOT_INIT;

    /* Send the request once */
    BMS_CAN_Status_e status = BMS_CAN_SendCommand(bms, BMS_CAN_CMD_CELL_VOLTAGES);
    if (status != BMS_CAN_OK) return status;

    /* Determine number of cells */
    uint8_t num_cells = bms->BMS_Data.status_info.num_battery_strings;
    if (num_cells == 0 || num_cells > BMS_CAN_MAX_CELLS) {
        num_cells = BMS_CAN_MAX_CELLS;
    }

    /* Calculate how many frames to expect: 3 cells per frame, round up */
    uint8_t num_frames = (num_cells + 2) / 3;
    uint8_t cell_idx = 0;

    for (uint8_t frame = 0; frame < num_frames; frame++) {
        /* Receive one CAN frame */
        status = BMS_CAN_ReceiveResponse(bms, BMS_CAN_CMD_CELL_VOLTAGES);
        if (status != BMS_CAN_OK) return status;

        uint8_t *d = bms->rx_frame.data;

        /* d[0] = frame number (skip 0xFF invalid frames) */
        if (d[0] == 0xFF) continue;

        /* d[1]~d[6] = 3 cell voltages, 2 bytes each */
        for (uint8_t i = 0; i < 3 && cell_idx < num_cells; i++) {
            uint16_t mv = BMS_CAN_BYTES_TO_U16(d[1 + i * 2], d[2 + i * 2]);
            bms->BMS_Data.cell_voltages.cell_voltage_mV[cell_idx] = mv;
            cell_idx++;
        }
    }

    bms->BMS_Data.cell_voltages.num_cells = cell_idx;
    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x96 - Individual Cell Temperatures
 *
 *  Multi-frame: BMS sends multiple CAN frames.
 *  Each frame (8 bytes):
 *    Byte0 = frame number (starting at 0)
 *    Byte1~Byte7 = 7 temperature values (1 byte each, 40°C offset)
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetCellTemperatures(BMS_CAN_Handle_t_s *bms)
{
    if (bms == NULL)         return BMS_CAN_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_CAN_ERR_NOT_INIT;

    /* Send the request once */
    BMS_CAN_Status_e status = BMS_CAN_SendCommand(bms, BMS_CAN_CMD_CELL_TEMPERATURES);
    if (status != BMS_CAN_OK) return status;

    /* Determine number of sensors */
    uint8_t num_sensors = bms->BMS_Data.status_info.num_temperature_sensors;
    if (num_sensors == 0 || num_sensors > BMS_CAN_MAX_TEMP_SENSORS) {
        num_sensors = BMS_CAN_MAX_TEMP_SENSORS;
    }

    /* Calculate frames: 7 temps per frame, round up */
    uint8_t num_frames = (num_sensors + 6) / 7;
    uint8_t sensor_idx = 0;

    for (uint8_t frame = 0; frame < num_frames; frame++) {
        status = BMS_CAN_ReceiveResponse(bms, BMS_CAN_CMD_CELL_TEMPERATURES);
        if (status != BMS_CAN_OK) return status;

        uint8_t *d = bms->rx_frame.data;

        /* d[0] = frame number */
        /* d[1]~d[7] = 7 temperature values */
        for (uint8_t i = 0; i < 7 && sensor_idx < num_sensors; i++) {
            bms->BMS_Data.cell_temperatures.temperature_degC[sensor_idx] = BMS_CAN_RAW_TO_TEMP(d[1 + i]);
            sensor_idx++;
        }
    }

    bms->BMS_Data.cell_temperatures.num_sensors = sensor_idx;
    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x97 - Cell Balance States
 *
 *  8 bytes of data = 64 bits.
 *  Bit0 = Cell 1, ... Bit47 = Cell 48.
 *  Bit48~63 = reserved.
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetCellBalance(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_CELL_BALANCE);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    for (uint8_t cell = 0; cell < BMS_CAN_MAX_CELLS; cell++) {
        uint8_t byte_idx = cell / 8;
        uint8_t bit_idx  = cell % 8;
        bms->BMS_Data.cell_balance.cell_balance[cell] = (BMS_CAN_Balance_State_e)BMS_CAN_GET_BIT(d[byte_idx], bit_idx);
    }

    return BMS_CAN_OK;
}

/* ══════════════════════════════════════════════
 *  PUBLIC: 0x98 - Fault Status
 * ══════════════════════════════════════════════ */

BMS_CAN_Status_e BMS_CAN_GetFaultStatus(BMS_CAN_Handle_t_s *bms)
{
    BMS_CAN_Status_e status = BMS_CAN_RequestStandard(bms, BMS_CAN_CMD_FAULT_STATUS);
    if (status != BMS_CAN_OK) return status;

    uint8_t *d = bms->rx_frame.data;

    /* Byte 0 - Voltage faults */
    bms->BMS_Data.fault_status.voltage.cell_volt_high_lvl1 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 0);
    bms->BMS_Data.fault_status.voltage.cell_volt_high_lvl2 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 1);
    bms->BMS_Data.fault_status.voltage.cell_volt_low_lvl1  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 2);
    bms->BMS_Data.fault_status.voltage.cell_volt_low_lvl2  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 3);
    bms->BMS_Data.fault_status.voltage.sum_volt_high_lvl1  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 4);
    bms->BMS_Data.fault_status.voltage.sum_volt_high_lvl2  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 5);
    bms->BMS_Data.fault_status.voltage.sum_volt_low_lvl1   = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 6);
    bms->BMS_Data.fault_status.voltage.sum_volt_low_lvl2   = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[0], 7);

    /* Byte 1 - Temperature faults */
    bms->BMS_Data.fault_status.temperature.chg_temp_high_lvl1    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 0);
    bms->BMS_Data.fault_status.temperature.chg_temp_high_lvl2    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 1);
    bms->BMS_Data.fault_status.temperature.chg_temp_low_lvl1     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 2);
    bms->BMS_Data.fault_status.temperature.chg_temp_low_lvl2     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 3);
    bms->BMS_Data.fault_status.temperature.dischg_temp_high_lvl1 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 4);
    bms->BMS_Data.fault_status.temperature.dischg_temp_high_lvl2 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 5);
    bms->BMS_Data.fault_status.temperature.dischg_temp_low_lvl1  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 6);
    bms->BMS_Data.fault_status.temperature.dischg_temp_low_lvl2  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[1], 7);

    /* Byte 2 - Current & SOC faults */
    bms->BMS_Data.fault_status.current_soc.chg_overcurrent_lvl1    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 0);
    bms->BMS_Data.fault_status.current_soc.chg_overcurrent_lvl2    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 1);
    bms->BMS_Data.fault_status.current_soc.dischg_overcurrent_lvl1 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 2);
    bms->BMS_Data.fault_status.current_soc.dischg_overcurrent_lvl2 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 3);
    bms->BMS_Data.fault_status.current_soc.soc_high_lvl1           = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 4);
    bms->BMS_Data.fault_status.current_soc.soc_high_lvl2           = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 5);
    bms->BMS_Data.fault_status.current_soc.soc_low_lvl1            = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 6);
    bms->BMS_Data.fault_status.current_soc.soc_low_lvl2            = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[2], 7);

    /* Byte 3 - Differential faults */
    bms->BMS_Data.fault_status.differential.diff_volt_lvl1 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[3], 0);
    bms->BMS_Data.fault_status.differential.diff_volt_lvl2 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[3], 1);
    bms->BMS_Data.fault_status.differential.diff_temp_lvl1 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[3], 2);
    bms->BMS_Data.fault_status.differential.diff_temp_lvl2 = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[3], 3);

    /* Byte 4 - MOS faults */
    bms->BMS_Data.fault_status.mos.chg_mos_temp_high           = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 0);
    bms->BMS_Data.fault_status.mos.dischg_mos_temp_high        = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 1);
    bms->BMS_Data.fault_status.mos.chg_mos_temp_sensor_err     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 2);
    bms->BMS_Data.fault_status.mos.dischg_mos_temp_sensor_err  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 3);
    bms->BMS_Data.fault_status.mos.chg_mos_adhesion_err        = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 4);
    bms->BMS_Data.fault_status.mos.dischg_mos_adhesion_err     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 5);
    bms->BMS_Data.fault_status.mos.chg_mos_open_circuit_err    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 6);
    bms->BMS_Data.fault_status.mos.dischg_mos_open_circuit_err = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[4], 7);

    /* Byte 5 - System faults */
    bms->BMS_Data.fault_status.system.afe_collect_chip_err     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 0);
    bms->BMS_Data.fault_status.system.voltage_collect_dropped  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 1);
    bms->BMS_Data.fault_status.system.cell_temp_sensor_err     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 2);
    bms->BMS_Data.fault_status.system.eeprom_err               = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 3);
    bms->BMS_Data.fault_status.system.rtc_err                  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 4);
    bms->BMS_Data.fault_status.system.precharge_failure        = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 5);
    bms->BMS_Data.fault_status.system.communication_failure    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 6);
    bms->BMS_Data.fault_status.system.internal_comm_failure    = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[5], 7);

    /* Byte 6 - Hardware faults */
    bms->BMS_Data.fault_status.hardware.current_module_fault         = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[6], 0);
    bms->BMS_Data.fault_status.hardware.sum_voltage_detect_fault     = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[6], 1);
    bms->BMS_Data.fault_status.hardware.short_circuit_protect_fault  = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[6], 2);
    bms->BMS_Data.fault_status.hardware.low_volt_forbidden_chg_fault = (BMS_CAN_Fault_Flag_e)BMS_CAN_GET_BIT(d[6], 3);

    /* Byte 7 - Fault code */
    bms->BMS_Data.fault_status.fault_code = d[7];

    return BMS_CAN_OK;
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

BMS_CAN_Status_e BMS_CAN_GetAllData(BMS_CAN_Handle_t_s *bms)
{
    if (bms == NULL)         return BMS_CAN_ERR_NULL_PTR;
    if (!bms->initialized)   return BMS_CAN_ERR_NOT_INIT;

    BMS_CAN_Status_e first_error = BMS_CAN_OK;
    BMS_CAN_Status_e status;

    /* Helper macro: call function, record first error, add inter-command delay */
    #define BMS_CAN_POLL(func) do {                                 \
        status = func(bms);                                         \
        if (status != BMS_CAN_OK && first_error == BMS_CAN_OK)     \
            first_error = status;                                   \
        BMS_CAN_Delay(BMS_CAN_INTER_CMD_DELAY_MS);                 \
    } while(0)

    /* 0x94 first — needed by 0x95 and 0x96 for cell/sensor count */
    BMS_CAN_POLL(BMS_CAN_GetStatusInfo);

    /* Then the rest in order */
    BMS_CAN_POLL(BMS_CAN_GetSOCVoltageCurrent);
    BMS_CAN_POLL(BMS_CAN_GetCellVoltageExtremes);
    BMS_CAN_POLL(BMS_CAN_GetTemperatureExtremes);
    BMS_CAN_POLL(BMS_CAN_GetMOSStatus);
    BMS_CAN_POLL(BMS_CAN_GetCellVoltages);
    BMS_CAN_POLL(BMS_CAN_GetCellTemperatures);
    BMS_CAN_POLL(BMS_CAN_GetCellBalance);
    BMS_CAN_POLL(BMS_CAN_GetFaultStatus);

    #undef BMS_CAN_POLL

    return first_error;
}
