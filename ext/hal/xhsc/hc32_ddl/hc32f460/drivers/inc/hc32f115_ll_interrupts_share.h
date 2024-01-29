/**
 *******************************************************************************
 * @file  hc32f115_ll_interrupts_share.h
 * @brief This file contains all the functions prototypes of the interrupt driver
 *        library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   xxxx-xx-xx       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __HC32F115_INTERRUPTS_SHARE_H__
#define __HC32F115_INTERRUPTS_SHARE_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

#if defined (HC32F115)
#include "hc32f1xx.h"
#include "hc32f1xx_conf.h"
#else
#error "Please select target device HC32F115."
#endif

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @addtogroup LL_HC32F115_SHARE_INTERRUPTS
 * @{
 */

#if (LL_INTERRUPTS_SHARE_ENABLE == DDL_ON)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup Share_Interrupts_Global_Functions
 * @{
 */

int32_t INTC_ShareIrqCmd(en_int_src_t enIntSrc, en_functional_state_t enNewState);

void IRQ024_Handler(void);
void IRQ025_Handler(void);
void IRQ026_Handler(void);
void IRQ027_Handler(void);
void IRQ028_Handler(void);
void IRQ029_Handler(void);
void IRQ030_Handler(void);
void IRQ031_Handler(void);

void EXTINT08_IrqHandler(void);
void EXTINT09_IrqHandler(void);

void EFM_ProgramEraseError_IrqHandler(void);
void EFM_ColError_IrqHandler(void);
void EFM_OpEnd_IrqHandler(void);

void CLK_XtalStop_IrqHandler(void);

void SWDT_IrqHandler(void);

void TMRB_1_Ovf_IrqHandler(void);
void TMRB_1_Udf_IrqHandler(void);
void TMRB_1_Cmp_IrqHandler(void);
void TMRB_2_Ovf_IrqHandler(void);
void TMRB_2_Udf_IrqHandler(void);
void TMRB_2_Cmp_IrqHandler(void);
void TMRB_3_Ovf_IrqHandler(void);
void TMRB_3_Udf_IrqHandler(void);
void TMRB_3_Cmp_IrqHandler(void);
void TMRB_4_Ovf_IrqHandler(void);
void TMRB_4_Udf_IrqHandler(void);
void TMRB_4_Cmp_IrqHandler(void);
void TMRB_5_Ovf_IrqHandler(void);
void TMRB_5_Udf_IrqHandler(void);
void TMRB_5_Cmp_IrqHandler(void);
void TMRB_6_Ovf_IrqHandler(void);
void TMRB_6_Udf_IrqHandler(void);
void TMRB_6_Cmp_IrqHandler(void);
void TMRB_7_Ovf_IrqHandler(void);
void TMRB_7_Udf_IrqHandler(void);
void TMRB_7_Cmp_IrqHandler(void);
void TMRB_8_Ovf_IrqHandler(void);
void TMRB_8_Udf_IrqHandler(void);
void TMRB_8_Cmp_IrqHandler(void);

void USART1_RxError_IrqHandler(void);
void USART1_RxFull_IrqHandler(void);
void USART1_TxEmpty_IrqHandler(void);
void USART1_TxComplete_IrqHandler(void);
void USART2_RxError_IrqHandler(void);
void USART2_RxFull_IrqHandler(void);
void USART2_TxEmpty_IrqHandler(void);
void USART2_TxComplete_IrqHandler(void);
void USART3_RxError_IrqHandler(void);
void USART3_RxFull_IrqHandler(void);
void USART3_TxEmpty_IrqHandler(void);
void USART3_TxComplete_IrqHandler(void);
void USART4_RxError_IrqHandler(void);
void USART4_RxFull_IrqHandler(void);
void USART4_TxEmpty_IrqHandler(void);
void USART4_TxComplete_IrqHandler(void);

void I2C_RxFull_IrqHandler(void);
void I2C_TxEmpty_IrqHandler(void);
void I2C_TxComplete_IrqHandler(void);
void I2C_EE_IrqHandler(void);

void SPI_RxFull_IrqHandler(void);
void SPI_TxEmpty_IrqHandler(void);
void SPI_Idle_IrqHandler(void);
void SPI_Error_IrqHandler(void);

void CTC_IrqHandler(void);

void EKEY_IrqHandler(void);

void TMR0_CmpA_IrqHandler(void);

void ADC_SeqA_IrqHandler(void);
void ADC_SeqB_IrqHandler(void);
void ADC_Cmp0_IrqHandler(void);
void ADC_Cmp1_IrqHandler(void);

void PWC_LVD_IrqHandler(void);

void RTC_Alarm_IrqHandler(void);
void RTC_Period_IrqHandler(void);

/**
 * @}
 */

#endif /* LL_INTERRUPTS_SHARE_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HC32F115_INTERRUPTS_SHARE_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
