/**
 *******************************************************************************
 * @file  hc32_ll_buz.h
 * @brief This file contains all the functions prototypes of the BUZ driver
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
#ifndef __HC32_LL_BUZ_H__
#define __HC32_LL_BUZ_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

#if defined (HC32F165)
#include "hc32f1xx.h"
#include "hc32f1xx_conf.h"
#else
#error "The currently selected chip does not support BUZ peripherals."
#endif

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @addtogroup LL_BUZ
 * @{
 */

#if (LL_BUZ_ENABLE == DDL_ON)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup BUZ_Global_Types BUZ Global Types
 * @{
 */

/**
 * @brief BUZ initialization structure definition
 */
typedef struct {
    uint8_t u8ClockSrc;       /*!< Specifies the clock source of the BUZ counter.
                                   This parameter can be a value of @ref BUZ_Clock_Source */
    uint8_t u8CountReloadVal; /*!< Specifies buzzer count reload value.
                                   This parameter can be a value between Min_Data = 0 and Max_Data = 0xFF */
} stc_buz_init_t;

/**
 * @}
 */

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup BUZ_Global_Macros BUZ Global Macros
 * @{
 */

/**
 * @defgroup BUZ_Clock_Source BUZ Clock Source
 * @{
 */
#define BUZ_CLK_SRC_HRC_DIV512              (0U)                /*!< HRC/512 */
#define BUZ_CLK_SRC_LRC_DIV4                (BUZ_CTL_CLKSEL)    /*!< LRC/4   */
/**
 * @}
 */

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
 * @addtogroup BUZ_Global_Functions
 * @{
 */
int32_t BUZ_StructInit(stc_buz_init_t *pstcBuzInit);
int32_t BUZ_Init(CM_BUZ_TypeDef *BUZx, const stc_buz_init_t *pstcBuzInit);
int32_t BUZ_DeInit(CM_BUZ_TypeDef *BUZx);

void BUZ_Start(CM_BUZ_TypeDef *BUZx);
void BUZ_Stop(CM_BUZ_TypeDef *BUZx);

uint8_t BUZ_GetClockSrc(const CM_BUZ_TypeDef *BUZx);
void BUZ_SetClockSrc(CM_BUZ_TypeDef *BUZx, uint8_t u8Src);
uint8_t BUZ_GetCountValue(const CM_BUZ_TypeDef *BUZx);
void BUZ_SetCountValue(CM_BUZ_TypeDef *BUZx, uint8_t u8Value);
/**
 * @}
 */

#endif /* LL_BUZ_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HC32_LL_BUZ_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
