/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_QDEC_HC32_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_QDEC_HC32_H_

/* tmr filter configuration */
#define TMR_CB_F_1			0 << 13
#define TMR_CB_F_4			1 << 13
#define TMR_CB_F_16			2 << 13
#define TMR_CB_F_64			3 << 13

#define TMR_CA_F_1			0 << 9
#define TMR_CA_F_4			1 << 9
#define TMR_CA_F_16			2 << 9
#define TMR_CA_F_64			3 << 9

#define TMR_CB_F_E			1 << 12
#define TMR_CA_F_E			1 << 8

/*
** bit0.16 When CLKA is low, a rising edge is sampled on CLKB
** bit1.17 When CLKA is low, a falling edge is sampled on CLKB
** bit2.18 When CLKA is high, a rising edge is sampled on CLKB
** bit3.19 When CLKA is high, a falling edge is sampled on CLKB
** bit4.20 When CLKB is low, a rising edge is sampled on CLKA
** bit5.21 When CLKB is low, a falling edge is sampled on CLKA
** bit6.22 When CLKB is high, a rising edge is sampled on CLKA
** bit7.23 When CLKB is high, a falling edge is sampled on CLKA
** bit8.24 When a rising edge occurred on TRIG
** bit9.25 When a falling edge occurred on TRIG
** bit11.27 When the symmetric unit overflow
** bit12.28 When the symmetric unit underflow 
 */

#define TMR_UP_COND_A_L_B_R	(1 << 0)
#define TMR_UP_COND_A_L_B_F	(1 << 1)
#define TMR_UP_COND_A_H_B_R	(1 << 2)
#define TMR_UP_COND_A_H_B_F	(1 << 3)
#define TMR_UP_COND_B_L_A_R	(1 << 4)
#define TMR_UP_COND_B_L_A_F	(1 << 5)
#define TMR_UP_COND_B_H_A_R	(1 << 6)
#define TMR_UP_COND_B_H_A_F	(1 << 7)

#define TMR_DOWN_COND_A_L_B_R	(1 << 16)
#define TMR_DOWN_COND_A_L_B_F	(1 << 17)
#define TMR_DOWN_COND_A_H_B_R	(1 << 18)
#define TMR_DOWN_COND_A_H_B_F	(1 << 19)
#define TMR_DOWN_COND_B_L_A_R	(1 << 20)
#define TMR_DOWN_COND_B_L_A_F	(1 << 21)
#define TMR_DOWN_COND_B_H_A_R	(1 << 22)
#define TMR_DOWN_COND_B_H_A_F	(1 << 23)

#define TMR_CNT_COND_MAX_VAL	(0xFF)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_QDEC_HC32_H_ */
