/**
 *******************************************************************************
 * @file  hc32_ll_mau.h
 * @brief This file contains all the functions prototypes of the MAU driver
 *        library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-06-30       CDT             Add API MAU_DeInit() [-HC32F4A0,HC32F472-]
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
#ifndef __HC32_LL_MAU_H__
#define __HC32_LL_MAU_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

#if defined (HC32F4A0) || defined (HC32F472)
#include "hc32f4xx.h"
#include "hc32f4xx_conf.h"
#elif defined (HC32F334)
#include "hc32f3xx.h"
#include "hc32f3xx_conf.h"
#elif defined (HC32F165)
#include "hc32f1xx.h"
#include "hc32f1xx_conf.h"
#else
#error "The currently selected chip does not support MAU peripherals."
#endif
/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @addtogroup LL_MAU
 * @{
 */

#if (LL_MAU_ENABLE == DDL_ON)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/


/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/**
 * @defgroup MAU_Global_Macros MAU Global Macros
 * @{
 */

#define MAU_SQRT_TIMEOUT               (HCLK_VALUE / 10000UL)/* About 1mS timeout */
#define MAU_SQRT_OUTPUT_LSHIFT_MAX     (16U)

#if defined (HC32F4A0) || defined (HC32F472)
#define MAU_SIN_Q15_SCALAR             (0x8000UL)
#define MAU_SIN_ANGIDX_TOTAL           (0x1000UL)
#endif

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/

/**
 * @addtogroup MAU_Global_Functions
 * @{
 */

void MAU_SqrtInit(CM_MAU_TypeDef *MAUx, uint8_t u8ShiftNum, en_functional_state_t enNewState);
void MAU_SqrtDeInit(CM_MAU_TypeDef *MAUx);

void MAU_SqrtResultLShiftConfig(CM_MAU_TypeDef *MAUx, uint8_t u8ShiftNum);
void MAU_SqrtIntCmd(CM_MAU_TypeDef *MAUx, en_functional_state_t enNewState);
void MAU_SqrtWriteData(CM_MAU_TypeDef *MAUx, uint32_t u32Radicand);
en_flag_status_t MAU_SqrtGetStatus(const CM_MAU_TypeDef *MAUx);
uint32_t MAU_SqrtReadData(const CM_MAU_TypeDef *MAUx);
void MAU_SqrtStart(CM_MAU_TypeDef *MAUx);

#if defined (HC32F165)
void MAU_DivIntCmd(CM_MAU_TypeDef *MAUx, en_functional_state_t enNewState);
void MAU_DivWriteData(CM_MAU_TypeDef *MAUx, uint32_t u32dividend, uint32_t u32Divisor);
en_flag_status_t MAU_DivGetStatus(const CM_MAU_TypeDef *MAUx);
uint32_t MAU_DivReadData(const CM_MAU_TypeDef *MAUx);
void MAU_DivStart(CM_MAU_TypeDef *MAUx);
#endif

int32_t MAU_Sqrt(CM_MAU_TypeDef *MAUx, uint32_t u32Radicand, uint32_t *pu32Result);
#if defined (HC32F165)
int32_t MAU_Div(CM_MAU_TypeDef *MAUx, uint32_t u32dividend, uint32_t u32Divisor, uint32_t *pu32Result);
//todo
#endif

#if defined (HC32F4A0) || defined (HC32F472)
int16_t MAU_Sin(CM_MAU_TypeDef *MAUx, uint16_t u16AngleIdx);
#endif

int32_t MAU_DeInit(void);

/**
 * @}
 */

#endif /* LL_MAU_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HC32_LL_MAU_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
