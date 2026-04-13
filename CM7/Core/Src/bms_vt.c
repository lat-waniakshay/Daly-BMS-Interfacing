/*
 * bms_vt.c
 *
 *  Created on: Oct 27, 2025
 *      Author: ishan
 */

/* Ring buffer */

#include "bms_vt.h"

uint8_t rb[RB_SIZE];
volatile uint16_t rb_head = 0;
volatile uint16_t rb_tail = 0;
uint8_t rx_byte1;

/* Accumulator for frame parsing */
uint8_t accum[ACCUM_MAX];
size_t accum_len = 0;
int raw_line_count = 0;

/* ---------- Debug printf over UART3 ---------- */
 void dbg_printf(const char *fmt, ...) {
	char buf[256];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n > 0) {
//		HAL_UART_Transmit(&huart5, (uint8_t*) buf, (uint16_t) n, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart7, (uint8_t*) buf, (uint16_t) n, HAL_MAX_DELAY);
	}
}

/* Hex dump helper */
 void dump_hex(const char *prefix, const uint8_t *p, size_t n) {
	dbg_printf("%s", prefix);
	for (size_t i = 0; i < n; i++)
		dbg_printf("%02X ", p[i]);
	dbg_printf("\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart5) {
		rb_push(rx_byte1);
		HAL_UART_Receive_IT(&huart5, &rx_byte1, 1); // re-arm
	}
}

void UART5_IRQHandler(void) {
	if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) != RESET) {
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		/* Could set a flag if needed */
	}
	HAL_UART_IRQHandler(&huart5);
}


/* ---------- Ring buffer helpers ---------- */
void rb_push(uint8_t b) {
	uint16_t next = (rb_head + 1) & RB_MASK;
	if (next != rb_tail) {
		rb[rb_head] = b;
		rb_head = next;
	}
	/* else: overflow -> drop byte silently */
}

 int rb_pop(void) {
	if (rb_tail == rb_head)
		return -1;
	int b = rb[rb_tail];
	rb_tail = (rb_tail + 1) & RB_MASK;
	return b;
}

/* Start RX-IT (byte-at-a-time) */
 void uart5_rx_it_start(void) {
	HAL_UART_Receive_IT(&huart5, &rx_byte1, 1);
}

/* ---------- Daly helpers ---------- */
 uint8_t daly_checksum(const uint8_t *b, size_t n) {
	uint32_t s = 0;
	for (size_t i = 0; i < n; i++)
		s += b[i];
	return (uint8_t) (s & 0xFF);
}

 void daly_build(uint8_t id, uint8_t out[DALY_FRAME_LEN]) {
	out[0] = DALY_START;
	out[1] = DALY_PC_ADDR;
	out[2] = id;
	out[3] = DALY_DATA_LEN;
	memset(&out[4], 0, DALY_PAYLOAD_LEN);
	out[12] = daly_checksum(out, 12);
}

 int scan_and_consume_one(uint8_t id, uint8_t out_frame[DALY_FRAME_LEN]) {
	if (accum_len < DALY_FRAME_LEN)
		return 0;
	for (size_t i = 0; i + DALY_FRAME_LEN <= accum_len; i++) {
		if (accum[i] == DALY_START && accum[i + 1] == DALY_BMS_ADDR
				&& accum[i + 2] == id&&
				accum[i + 3] == DALY_DATA_LEN) {

			uint8_t cs = daly_checksum(&accum[i], DALY_FRAME_LEN - 1);
			if (cs == accum[i + DALY_FRAME_LEN - 1]) {
				memcpy(out_frame, &accum[i], DALY_FRAME_LEN);
				size_t consume = i + DALY_FRAME_LEN;
				memmove(accum, &accum[consume], accum_len - consume);
				accum_len -= consume;
				return 1;
			}
		}
	}
	return 0;
}

 float parse_vcum_0x90(const uint8_t *payload) {
	uint16_t V_cum_0p1V = (payload[0] << 8) | payload[1];
	return V_cum_0p1V * 0.1f;
}

 int8_t parse_tmax_0x92(const uint8_t *payload) {
	return (int8_t) payload[0] - 40;
}

 int daly_poll(uint8_t id, uint32_t timeout_ms,
		uint8_t out_payload[DALY_PAYLOAD_LEN]) {
	//accum_len = 0;
	uint8_t tx[DALY_FRAME_LEN];
	daly_build(id, tx);
	//dump_hex("TX: ", tx, sizeof(tx));
	//HAL_UART_Transmit(&huart2, tx, sizeof(tx), 50);
	HAL_UART_Transmit(&huart5, tx, sizeof(tx), 50);

	uint32_t t0 = HAL_GetTick();
	uint8_t frame[DALY_FRAME_LEN];

	for (;;) {
		pump_rx_and_print_raw();
		if (scan_and_consume_one(id, frame)) {
			//dump_hex("RX(frame): ", frame, sizeof(frame)); // optional
			memcpy(out_payload, &frame[4], DALY_PAYLOAD_LEN);
			return 1;
		}
		if ((HAL_GetTick() - t0) > timeout_ms)
			return 0;
		HAL_Delay(1);
	}
}
/* Pump all bytes from ISR ring to:
 1) Raw console stream (prints every byte)
 2) Accumulator used by the parser */
 void pump_rx_and_print_raw(void) {
	int c;
	while ((c = rb_pop()) >= 0) {
		/* 1) Print every byte received */
//		if (raw_line_count == 0)
//			dbg_printf("RX: ");
//		dbg_printf("%02X ", (uint8_t) c);
//		raw_line_count++;
//		if (raw_line_count >= 16) {
//			dbg_printf("\r\n");
//			raw_line_count = 0;
//		}
		/* 2) Append to accumulator for parsing */
		if (accum_len < ACCUM_MAX) {
			accum[accum_len++] = (uint8_t) c;
		} else {
			/* Drop oldest half to make room, then append */
			memmove(accum, &accum[ACCUM_MAX / 2], ACCUM_MAX - ACCUM_MAX / 2);
			accum_len = ACCUM_MAX - ACCUM_MAX / 2;
			accum[accum_len++] = (uint8_t) c;
		}
	}
}
