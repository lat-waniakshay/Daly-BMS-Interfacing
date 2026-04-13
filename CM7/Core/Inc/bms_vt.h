/*
 * bms_vt.h
 *
 *  Created on: Oct 27, 2025
 *      Author: ishan
 */

#ifndef INC_BMS_VT_H_
#define INC_BMS_VT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

/* ---------- Daly BMS Protocol Constants ---------- */
#define DALY_START        0xA5
#define DALY_PC_ADDR      0x40
#define DALY_BMS_ADDR     0x01
#define DALY_DATA_LEN     0x08
#define DALY_ID_90        0x90
#define DALY_ID_92        0x92
#define DALY_PAYLOAD_LEN  8
#define DALY_FRAME_LEN    13

/* ---------- Ring Buffer Configuration ---------- */
#define RB_SIZE           512
#define RB_MASK           (RB_SIZE - 1)

/* ---------- Accumulator Configuration ---------- */
#define ACCUM_MAX         256

/* ---------- External UART Handles ---------- */
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

/* ---------- Global Variables ---------- */
extern uint8_t rb[RB_SIZE];
extern volatile uint16_t rb_head;
extern volatile uint16_t rb_tail;
extern uint8_t rx_byte1;
extern uint8_t accum[ACCUM_MAX];
extern size_t accum_len;
extern int raw_line_count;

/* ---------- Public Functions ---------- */
void uart5_rx_it_start(void);
void pump_rx_and_print_raw(void);
void rb_push(uint8_t b);

int daly_poll(uint8_t id, uint32_t timeout_ms, uint8_t out_payload[DALY_PAYLOAD_LEN]);
float parse_vcum_0x90(const uint8_t *payload);
int8_t parse_tmax_0x92(const uint8_t *payload);

void UART5_IRQHandler(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void dbg_printf(const char *fmt, ...);
void dump_hex(const char *prefix, const uint8_t *p, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* INC_BMS_VT_H_ */
