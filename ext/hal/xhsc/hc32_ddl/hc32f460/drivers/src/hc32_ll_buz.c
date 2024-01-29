/**
 *******************************************************************************
 * @file  hc32_ll_buz.c
 * @brief This file provides firmware functions to manage the Buzzer(BUZ).
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

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_buz.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @defgroup LL_BUZ BUZ
 * @brief BUZ Driver Library
 * @{
 */

#if (LL_BUZ_ENABLE == DDL_ON)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup BUZ_Local_Macros BUZ Local Macros
 * @{
 */

/**
 * @defgroup BUZ_Check_Parameters_Validity BUZ Check Parameters Validity
 * @{
 */
#define IS_BUZ_UNIT(x)                  ((x) == CM_BUZ)

#define IS_BUZ_CLK_SRC(x)                                                      \
(   ((x) == BUZ_CLK_SRC_HRC_DIV512)     ||                                     \
    ((x) == BUZ_CLK_SRC_LRC_DIV4))
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup BUZ_Global_Functions BUZ Global Functions
 * @{
 */

/**
 * @brief  Set the fields of structure stc_buz_init_t to default values.
 * @param  [out] pstcBuzInit            Pointer to a @ref stc_buz_init_t structure.
 * @retval int32_t:
 *           - LL_OK:                   Initialize successfully.
 *           - LL_ERR_INVD_PARAM:       The pointer pstcBuzInit value is NULL.
 */
int32_t BUZ_StructInit(stc_buz_init_t *pstcBuzInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcBuzInit) {
        pstcBuzInit->u8ClockSrc = BUZ_CLK_SRC_HRC_DIV512;
        pstcBuzInit->u8CountReloadVal = 0U;
        i32Ret = LL_OK;
    }

    return i32Ret;
}

/**
 * @brief  Initialize BUZ function.
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZx:    BUZ instance register base
 * @param  [in] pstcBuzInit             Pointer to a @ref stc_buz_init_t structure.
 * @retval int32_t:
 *           - LL_OK:                   Initialize successfully.
 *           - LL_ERR_INVD_PARAM:       The pointer pstcBuzInit value is NULL.
 */
int32_t BUZ_Init(CM_BUZ_TypeDef *BUZx, const stc_buz_init_t *pstcBuzInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcBuzInit) {
        DDL_ASSERT(IS_BUZ_UNIT(BUZx));
        DDL_ASSERT(IS_BUZ_CLK_SRC(pstcBuzInit->u8ClockSrc));

        MODIFY_REG8(BUZx->CTL, BUZ_CTL_CLKSEL, pstcBuzInit->u8ClockSrc);
        WRITE_REG8(BUZx->CNT, pstcBuzInit->u8CountReloadVal);
        i32Ret = LL_OK;
    }

    return i32Ret;
}

/**
 * @brief  De-Initialize BUZ function.
 * @param [in] BUZx                     Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZx:    BUZ instance register base
 * @retval None
 */
int32_t BUZ_DeInit(CM_BUZ_TypeDef *BUZx)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));

    WRITE_REG8(BUZx->CTL, 0U);
    WRITE_REG8(BUZx->CNT, 0U);
    return LL_OK;
}

/**
 * @brief Start BUZ counter
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @retval None
 */
void BUZ_Start(CM_BUZ_TypeDef *BUZx)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    SET_REG8_BIT(BUZx->CTL, BUZ_CTL_ST);
}

/**
 * @brief Stop BUZ counter
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @retval None
 */
void BUZ_Stop(CM_BUZ_TypeDef *BUZx)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    CLR_REG8_BIT(BUZx->CTL, BUZ_CTL_ST);
}

/**
 * @brief  Get the clock source of the BUZ counter.
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @retval The clock source of the BUZ counter
 */
uint8_t BUZ_GetClockSrc(const CM_BUZ_TypeDef *BUZx)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    return READ_REG8_BIT(BUZx->CTL, BUZ_CTL_CLKSEL);
}

/**
 * @brief  Set the clock source of the BUZ counter.
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @param  [in] u8Src                   The clock source of the BUZ counter
 *         This parameter can be one of the macros group @ref BUZ_Clock_Source
 *           @arg BUZ_CLK_SRC_HRC_DIV512: Uses the HRC/512 as counter's count clock
 *           @arg BUZ_CLK_SRC_LRC_DIV4:   Uses the LRC/4 as counter's count clock
 * @retval None
 */
void BUZ_SetClockSrc(CM_BUZ_TypeDef *BUZx, uint8_t u8Src)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    DDL_ASSERT(IS_BUZ_CLK_SRC(u8Src));

    MODIFY_REG8(BUZx->CTL, BUZ_CTL_CLKSEL, u8Src);
}

/**
 * @brief  Get the count value of the BUZ counter.
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @retval The count value of the BUZ counter
 */
uint8_t BUZ_GetCountValue(const CM_BUZ_TypeDef *BUZx)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    return READ_REG8(BUZx->CNT);
}

/**
 * @brief  Set the count value of the BUZ counter.
 * @param  [in] BUZx                    Pointer to BUZ instance register base
 *         This parameter can be one of the following values:
 *           @arg CM_BUZ or CM_BUZ_x:   BUZ unit instance register base
 * @param  [in] u8Value                 The count value of the BUZ counter
 *           @arg number of 8bit
 * @retval None
 */
void BUZ_SetCountValue(CM_BUZ_TypeDef *BUZx, uint8_t u8Value)
{
    DDL_ASSERT(IS_BUZ_UNIT(BUZx));
    WRITE_REG8(BUZx->CNT, u8Value);
}

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

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
