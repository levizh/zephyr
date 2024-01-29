/**
 *******************************************************************************
 * @file  hc32_ll_tmrb.c
 * @brief This file provides firmware functions to manage the TMRB(TimerB).
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-09-30       CDT             Delete union in stc_tmrb_init_t structure [-HC32F120,HC32M120-]
                                    Split register BCSTR,modify function relate to BCSTR [-HC32F120,HC32M120-]
                                    Refine code due to some of member type of struct stc_tmrb_init_t is modified [-HC32F120,HC32M120-]
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
#include "hc32_ll_tmrb.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @defgroup LL_TMRB TMRB
 * @brief TMRB Driver Library
 * @{
 */

#if (LL_TMRB_ENABLE == DDL_ON)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup TMRB_Local_Macros TMRB Local Macros
 * @{
 */

/* TMRB maximum channel */
#if defined (HC32M423)
#define TMRB_CH_MAX                         (6UL)
#define TMRB_CAPT_CH_MAX                    (4UL)
#elif defined (CD32Z16X)
#define TMRB_CH_MAX                         (4UL)
#define TMRB_CAPT_CH_MAX                    (4UL)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_CH_MAX                         (1UL)
#define TMRB_CAPT_CH_MAX                    (1UL)
#elif defined (HC32F165)
#define TMRB_CH_MAX                         (2UL)
#define TMRB_CAPT_CH_MAX                    (2UL)
#endif

/**
 * @defgroup TMRB_Mask TMRB Mask
 * @{
 */
#define TMRB_CNT_FLAG_MASK                  (TMRB_FLAG_OVF | TMRB_FLAG_UDF)
#define TMRB_CNT_INT_MASK                   (TMRB_INT_OVF  | TMRB_INT_UDF)
#if defined (HC32M423)
#define TMRB_CMP_FLAG_MASK                  (TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2 | TMRB_FLAG_CMP3 | TMRB_FLAG_CMP4 | \
                                             TMRB_FLAG_CMP5 | TMRB_FLAG_CMP6)
#define TMRB_CMP_INT_MASK                   (TMRB_INT_CMP1 | TMRB_INT_CMP2 | TMRB_INT_CMP3 | TMRB_INT_CMP4 | \
                                             TMRB_INT_CMP5 | TMRB_INT_CMP6)
#elif defined (CD32Z16X)
#define TMRB_CMP_FLAG_MASK                  (TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2 | TMRB_FLAG_CMP3 | TMRB_FLAG_CMP4)
#define TMRB_CMP_INT_MASK                   (TMRB_INT_CMP1 | TMRB_INT_CMP2 | TMRB_INT_CMP3 | TMRB_INT_CMP4)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_CMP_FLAG_MASK                  (TMRB_FLAG_CMP1)
#define TMRB_CMP_INT_MASK                   (TMRB_INT_CMP1)
#elif defined (HC32F165)
#define TMRB_CMP_FLAG_MASK                  (TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2)
#define TMRB_CMP_INT_MASK                   (TMRB_INT_CMP1 | TMRB_INT_CMP2)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Calculate_Register_Address TMRB Calculate Register Address
 * @{
 */
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
#define TMRB_BCONR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->BCONR1)) + (((__CH__) >> 1UL) << 3UL))
#define TMRB_CMPAR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->CMPAR1)) + ((__CH__) << 2UL))
#define TMRB_CCONR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->CCONR1)) + ((__CH__) << 2UL))
#define TMRB_PCONR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->PCONR1)) + ((__CH__) << 2UL))
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_CMPAR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->CMPAR)))
#define TMRB_CCONR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->CCONR)))
#define TMRB_PCONR_ADDR(__UNIT__, __CH__)   (__IO uint16_t*)((uint32_t)(&((__UNIT__)->PCONR)))
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Check_Parameters_Validity TMRB Check Parameters Validity
 * @{
 */
#if defined (HC32M423)
#define IS_TMRB_UNIT(x)                         ((x) == CM_TMRB)

#define IS_TMRB_CH(x)                                                          \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2)                           ||                             \
    ((x) == TMRB_CH3)                           ||                             \
    ((x) == TMRB_CH4)                           ||                             \
    ((x) == TMRB_CH5)                           ||                             \
    ((x) == TMRB_CH6))

#define IS_TMRB_CAPT_CH(x)                                                     \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2)                           ||                             \
    ((x) == TMRB_CH3)                           ||                             \
    ((x) == TMRB_CH4))

#define IS_TMRB_BUF_CH(x)                                                      \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH3)                           ||                             \
    ((x) == TMRB_CH5))

#define IS_TMRB_CMP_BUF_COND(x)                                                \
(   ((x) == TMRB_BUF_COND_INVD)                 ||                             \
    ((x) == TMRB_BUF_COND_PEAK)                 ||                             \
    ((x) == TMRB_BUF_COND_VALLEY)               ||                             \
    ((x) == TMRB_BUF_COND_PEAK_VALLEY))
#elif defined (CD32Z16X)
#define IS_TMRB_UNIT(x)                                                        \
(   ((x) == CM_TMRB_1)                          ||                             \
    ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_3))

#define IS_TMRB_CH(x)                                                          \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2)                           ||                             \
    ((x) == TMRB_CH3)                           ||                             \
    ((x) == TMRB_CH4))

#define IS_TMRB_CAPT_CH(x)                                                     \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2)                           ||                             \
    ((x) == TMRB_CH3)                           ||                             \
    ((x) == TMRB_CH4))

#define IS_TMRB_BUF_CH(x)                                                      \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH3))

#define IS_TMRB_CMP_BUF_COND(x)                                                \
(   ((x) == TMRB_BUF_COND_INVD)                 ||                             \
    ((x) == TMRB_BUF_COND_PEAK)                 ||                             \
    ((x) == TMRB_BUF_COND_VALLEY)               ||                             \
    ((x) == TMRB_BUF_COND_PEAK_VALLEY))
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160)
#define IS_TMRB_UNIT(x)                                                        \
(   ((x) == CM_TMRB_1)                          ||                             \
    ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_3)                          ||                             \
    ((x) == CM_TMRB_4)                          ||                             \
    ((x) == CM_TMRB_5)                          ||                             \
    ((x) == CM_TMRB_6)                          ||                             \
    ((x) == CM_TMRB_7)                          ||                             \
    ((x) == CM_TMRB_8))

#define IS_TMRB_SYNC_UNIT(x)                                                   \
(   ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_4)                          ||                             \
    ((x) == CM_TMRB_6)                          ||                             \
    ((x) == CM_TMRB_8))

#define IS_TMRB_CH(x)                           ((x) == TMRB_CH1)
#define IS_TMRB_CAPT_CH(x)                      ((x) == TMRB_CH1)
#elif defined (HC32M120)
#define IS_TMRB_UNIT(x)                                                        \
(   ((x) == CM_TMRB_1)                          ||                             \
    ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_3)                          ||                             \
    ((x) == CM_TMRB_4))

#define IS_TMRB_SYNC_UNIT(x)                                                   \
(   ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_4))

#define IS_TMRB_CH(x)                           ((x) == TMRB_CH1)
#define IS_TMRB_CAPT_CH(x)                      ((x) == TMRB_CH1)
#elif defined (HC32F165)
#define IS_TMRB_UNIT(x)                                                        \
(   ((x) == CM_TMRB_1)                          ||                             \
    ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_3)                          ||                             \
    ((x) == CM_TMRB_4)                          ||                             \
    ((x) == CM_TMRB_5)                          ||                             \
    ((x) == CM_TMRB_6))

#define IS_TMRB_CH(x)                                                          \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2))

#define IS_TMRB_CAPT_CH(x)                                                     \
(   ((x) == TMRB_CH1)                           ||                             \
    ((x) == TMRB_CH2))

#define IS_TMRB_BUF_CH(x)                       ((x) == TMRB_CH1)

#define IS_TMRB_CMP_BUF_COND(x)                                                \
(   ((x) == TMRB_BUF_COND_INVD)                 ||                             \
    ((x) == TMRB_BUF_COND_PEAK)                 ||                             \
    ((x) == TMRB_BUF_COND_VALLEY)               ||                             \
    ((x) == TMRB_BUF_COND_PEAK_VALLEY))

#define IS_TMRB_SYNC_UNIT(x)                                                   \
(   ((x) == CM_TMRB_2)                          ||                             \
    ((x) == CM_TMRB_4)                          ||                             \
    ((x) == CM_TMRB_6))
#endif

#define IS_TMRB_CNT_SRC(x)                                                     \
(   ((x) == TMRB_CNT_SRC_HW)                    ||                             \
    ((x) == TMRB_CNT_SRC_SW))

#define IS_TMRB_CLK_DIV(x)                                                     \
(   ((x) == TMRB_CLK_DIV1)                      ||                             \
    ((x) == TMRB_CLK_DIV2)                      ||                             \
    ((x) == TMRB_CLK_DIV4)                      ||                             \
    ((x) == TMRB_CLK_DIV8)                      ||                             \
    ((x) == TMRB_CLK_DIV16)                     ||                             \
    ((x) == TMRB_CLK_DIV32)                     ||                             \
    ((x) == TMRB_CLK_DIV64)                     ||                             \
    ((x) == TMRB_CLK_DIV128)                    ||                             \
    ((x) == TMRB_CLK_DIV256)                    ||                             \
    ((x) == TMRB_CLK_DIV512)                    ||                             \
    ((x) == TMRB_CLK_DIV1024))

#define IS_TMRB_CNT_MD(x)                                                      \
(   ((x) == TMRB_MD_SAWTOOTH)                   ||                             \
    ((x) == TMRB_MD_TRIANGLE))

#define IS_TMRB_CNT_DIR(x)                                                     \
(   ((x) == TMRB_DIR_UP)                        ||                             \
    ((x) == TMRB_DIR_DOWN))

#define IS_TMRB_CNT_RELOAD(x)                                                  \
(   ((x) == TMRB_CNT_RELOAD_DISABLE)            ||                             \
    ((x) == TMRB_CNT_RELOAD_ENABLE))

#define IS_TMRB_START_COND(x)                                                  \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_START_COND_ALL) == TMRB_START_COND_ALL))

#define IS_TMRB_STOP_COND(x)                                                   \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_STOP_COND_ALL) == TMRB_STOP_COND_ALL))

#define IS_TMRB_CLR_COND(x)                                                    \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_CLR_COND_ALL) == TMRB_CLR_COND_ALL))

#define IS_TMRB_CNT_UP_COND(x)                                                 \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_CNT_UP_COND_ALL) == TMRB_CNT_UP_COND_ALL))

#define IS_TMRB_CNT_DOWN_COND(x)                                               \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_CNT_DOWN_COND_ALL) == TMRB_CNT_DOWN_COND_ALL))

#define IS_TMRB_HW_CNT_UP_COND(x)               (((x) | TMRB_CNT_UP_COND_ALL) == TMRB_CNT_UP_COND_ALL)

#define IS_TMRB_HW_CNT_DOWN_COND(x)             (((x) | TMRB_CNT_DOWN_COND_ALL) == TMRB_CNT_DOWN_COND_ALL)

#define IS_TMRB_PWM_CNT_STAT(x)                                                \
(   ((x) == TMRB_PWM_CNT_START)                 ||                             \
    ((x) == TMRB_PWM_CNT_STOP)                  ||                             \
    ((x) == TMRB_PWM_CNT_MATCH)                 ||                             \
    ((x) == TMRB_PWM_CNT_PERIOD))

#define IS_TMRB_PWM_START_STOP_POLARITY(x)                                     \
(   ((x) == TMRB_PWM_LOW)                       ||                             \
    ((x) == TMRB_PWM_HIGH)                      ||                             \
    ((x) == TMRB_PWM_HOLD))

#define IS_TMRB_PWM_MATCH_POLARITY(x)                                          \
(   ((x) == TMRB_PWM_LOW)                       ||                             \
    ((x) == TMRB_PWM_HIGH)                      ||                             \
    ((x) == TMRB_PWM_HOLD)                      ||                             \
    ((x) == TMRB_PWM_INVT))

#define IS_TMRB_PWM_POLARITY(state, polarity)                                  \
(   ((((state) == TMRB_PWM_CNT_START) || ((state) == TMRB_PWM_CNT_STOP))    && \
     (IS_TMRB_PWM_START_STOP_POLARITY(polarity)))                           || \
    ((((state) == TMRB_PWM_CNT_MATCH) || ((state) == TMRB_PWM_CNT_PERIOD))  && \
    (IS_TMRB_PWM_MATCH_POLARITY(polarity))))

#define IS_TMRB_PWM_FORCE_POLARITY(x)                                          \
(   ((x) == TMRB_PWM_FORCE_INVD)                ||                             \
    ((x) == TMRB_PWM_FORCE_LOW)                 ||                             \
    ((x) == TMRB_PWM_FORCE_HIGH))

#define IS_TMRB_FUNC_MD(x)                                                     \
(   ((x) == TMRB_FUNC_CMP)                      ||                             \
    ((x) == TMRB_FUNC_CAPT))

#define IS_TMRB_FILTER_CLK_DIV(x)                                              \
(   ((x) == TMRB_FILTER_CLK_DIV1)               ||                             \
    ((x) == TMRB_FILTER_CLK_DIV4)               ||                             \
    ((x) == TMRB_FILTER_CLK_DIV16)              ||                             \
    ((x) == TMRB_FILTER_CLK_DIV64))

#define IS_TMRB_CAPT_COND(x)                                                   \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_CAPT_COND_ALL) == TMRB_CAPT_COND_ALL))

#define IS_TMRB_FLAG(x)                                                        \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_FLAG_ALL) == TMRB_FLAG_ALL))

#define IS_TMRB_INT(x)                                                         \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_INT_ALL) == TMRB_INT_ALL))

#define IS_TMRB_EVT(x)                                                         \
(   ((x) != 0U)                                 &&                             \
    (((x) | TMRB_EVT_ALL) == TMRB_EVT_ALL))
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
 * @defgroup TMRB_Global_Functions TMRB Global Functions
 * @{
 */

/**
 * @brief  De-Initialize TMRB unit base function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval None
 */
void TMRB_DeInit(CM_TMRB_TypeDef *TMRBx)
{
    uint32_t u32Count;
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
    __IO uint16_t *BCONR;
#endif
    __IO uint16_t *PCONR;
    __IO uint16_t *CMPAR;
    __IO uint16_t *CCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    /* Configures the registers to reset value. */
    WRITE_REG8(TMRBx->BCSTRL, 0x02U);
    WRITE_REG8(TMRBx->BCSTRH, 0x00U);
    WRITE_REG16(TMRBx->CNTER, 0x0000U);
    WRITE_REG16(TMRBx->PERAR, 0xFFFFU);
    WRITE_REG16(TMRBx->HCONR, 0x0000U);
    WRITE_REG16(TMRBx->HCUPR, 0x0000U);
    WRITE_REG16(TMRBx->HCDOR, 0x0000U);
    for (u32Count = 0U; u32Count < TMRB_CH_MAX; u32Count++) {
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
        if (0U == (u32Count % 2U)) {
            BCONR = TMRB_BCONR_ADDR(TMRBx, u32Count);
            WRITE_REG16(*BCONR, 0x0000U);
        }
#endif
        PCONR = TMRB_PCONR_ADDR(TMRBx, u32Count);
        WRITE_REG16(*PCONR, 0x0000U);
        CMPAR = TMRB_CMPAR_ADDR(TMRBx, u32Count);
        WRITE_REG16(*CMPAR, 0xFFFFU);
    }
    for (u32Count = 0U; u32Count < TMRB_CAPT_CH_MAX; u32Count++) {
        CCONR = TMRB_CCONR_ADDR(TMRBx, u32Count);
        WRITE_REG16(*CCONR, 0x0000U);
    }
    WRITE_REG16(TMRBx->ICONR, 0x0000U);
    WRITE_REG16(TMRBx->ECONR, 0x0000U);
    WRITE_REG16(TMRBx->STFLR, 0x0000U);
}

/**
 * @brief  Initialize TMRB base function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] pstcTmrbInit            Pointer to a @ref stc_tmrb_init_t structure.
 * @retval int32_t:
 *           - LL_OK: Initialize success
 *           - LL_ERR_INVD_PARAM: pstcTmrbInit is NULL
 */
int32_t TMRB_Init(CM_TMRB_TypeDef *TMRBx, const stc_tmrb_init_t *pstcTmrbInit)
{
    int32_t i32Ret = LL_OK;

    if (NULL == pstcTmrbInit) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        /* Check parameters */
        DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
        DDL_ASSERT(IS_TMRB_CNT_SRC(pstcTmrbInit->u8CountSrc));
        DDL_ASSERT(IS_TMRB_CNT_RELOAD(pstcTmrbInit->u8CountReload));

        if (TMRB_CNT_SRC_SW == pstcTmrbInit->u8CountSrc) {
            DDL_ASSERT(IS_TMRB_CLK_DIV(pstcTmrbInit->sw_count.u8ClockDiv));
            DDL_ASSERT(IS_TMRB_CNT_MD(pstcTmrbInit->sw_count.u8CountMode));
            DDL_ASSERT(IS_TMRB_CNT_DIR(pstcTmrbInit->sw_count.u8CountDir));

            MODIFY_REG8(TMRBx->BCSTRL, (TMRB_BCSTRL_CKDIV | TMRB_BCSTRL_MODE | TMRB_BCSTRL_DIR),
                        (pstcTmrbInit->sw_count.u8ClockDiv | pstcTmrbInit->sw_count.u8CountMode |
                         pstcTmrbInit->sw_count.u8CountDir));
            MODIFY_REG8(TMRBx->BCSTRH, TMRB_BCSTRH_OVSTP, pstcTmrbInit->u8CountReload);
        } else {
            DDL_ASSERT(IS_TMRB_HW_CNT_UP_COND(pstcTmrbInit->hw_count.u16CountUpCond));
            DDL_ASSERT(IS_TMRB_HW_CNT_DOWN_COND(pstcTmrbInit->hw_count.u16CountDownCond));

            MODIFY_REG8(TMRBx->BCSTRH, TMRB_BCSTRH_OVSTP, pstcTmrbInit->u8CountReload);
            WRITE_REG16(TMRBx->HCUPR, pstcTmrbInit->hw_count.u16CountUpCond);
            WRITE_REG16(TMRBx->HCDOR, pstcTmrbInit->hw_count.u16CountDownCond);
        }
        WRITE_REG16(TMRBx->PERAR, pstcTmrbInit->u16PeriodValue);
        WRITE_REG16(TMRBx->CNTER, 0U);
    }

    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_tmrb_init_t to default values.
 * @param  [out] pstcTmrbInit           Pointer to a @ref stc_tmrb_init_t structure.
 * @retval int32_t:
 *           - LL_OK: Initialize success
 *           - LL_ERR_INVD_PARAM: pstcTmrbInit is NULL
 */
int32_t TMRB_StructInit(stc_tmrb_init_t *pstcTmrbInit)
{
    int32_t i32Ret = LL_OK;

    if (NULL == pstcTmrbInit) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        pstcTmrbInit->u8CountSrc           = TMRB_CNT_SRC_SW;
        pstcTmrbInit->sw_count.u8ClockDiv  = TMRB_CLK_DIV1;
        pstcTmrbInit->sw_count.u8CountMode = TMRB_MD_SAWTOOTH;
        pstcTmrbInit->sw_count.u8CountDir  = TMRB_DIR_DOWN;
        pstcTmrbInit->hw_count.u16CountUpCond   = TMRB_CNT_UP_COND_INVD;
        pstcTmrbInit->hw_count.u16CountDownCond = TMRB_CNT_DOWN_COND_INVD;
        pstcTmrbInit->u16PeriodValue        = 0xFFFFU;
        pstcTmrbInit->u8CountReload        = TMRB_CNT_RELOAD_ENABLE;
    }

    return i32Ret;
}

/**
 * @brief  Start TMRB counter.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval None
 */
void TMRB_Start(CM_TMRB_TypeDef *TMRBx)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    SET_REG8_BIT(TMRBx->BCSTRL, TMRB_BCSTRL_START);
}

/**
 * @brief  Stop TMRB counter.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval None
 */
void TMRB_Stop(CM_TMRB_TypeDef *TMRBx)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    CLR_REG8_BIT(TMRBx->BCSTRL, TMRB_BCSTRL_START);
}

/**
 * @brief  Set the TMRB counter count value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Value                Counter count value (between Min_Data=0 and Max_Data=0xFFFF)
 * @retval None
 */
void TMRB_SetCountValue(CM_TMRB_TypeDef *TMRBx, uint16_t u16Value)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    WRITE_REG16(TMRBx->CNTER, u16Value);
}

/**
 * @brief  Get the TMRB counter count value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval uint16_t                     Counter count value
 */
uint16_t TMRB_GetCountValue(const CM_TMRB_TypeDef *TMRBx)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    return READ_REG16(TMRBx->CNTER);
}

/**
 * @brief  Set TMRB counter period value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Value                Counter period value (between Min_Data=0 and Max_Data=0xFFFF)
 * @retval None
 */
void TMRB_SetPeriodValue(CM_TMRB_TypeDef *TMRBx, uint16_t u16Value)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    WRITE_REG16(TMRBx->PERAR, u16Value);
}

/**
 * @brief  Get TMRB counter period value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval uint16_t                     Counter period value
 */
uint16_t TMRB_GetPeriodValue(const CM_TMRB_TypeDef *TMRBx)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    return READ_REG16(TMRBx->PERAR);
}

/**
 * @brief  Set TMRB compare register value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16Value                Compare value (between Min_Data=0 and Max_Data=0xFFFF)
 * @retval None
 */
void TMRB_SetCompareValue(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Value)
{
    __IO uint16_t *CMPAR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));

    CMPAR = TMRB_CMPAR_ADDR(TMRBx, u32Ch);
    WRITE_REG16(*CMPAR, u16Value);
}

/**
 * @brief  Get TMRB compare register value.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @retval uint16_t                     Compare value
 */
uint16_t TMRB_GetCompareValue(const CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch)
{
    __I uint16_t *CMPAR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));

    CMPAR = TMRB_CMPAR_ADDR(TMRBx, u32Ch);
    return READ_REG16(*CMPAR);
}

/**
 * @brief  Enable or disable the specified hardware count up condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Cond                 TMRB hardware up counter condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Count_Up_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWCountUpCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CNT_UP_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->HCUPR, u16Cond);
    } else {
        CLR_REG16_BIT(TMRBx->HCUPR, u16Cond);
    }
}

/**
 * @brief  Enable or disable the specified hardware count down condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Cond                 TMRB hardware down counter condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Count_Down_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWCountDownCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CNT_DOWN_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->HCDOR, u16Cond);
    } else {
        CLR_REG16_BIT(TMRBx->HCDOR, u16Cond);
    }
}

/**
 * @brief  Enable or disable the specified hardware start condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Cond                 TMRB hardware start counter condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Start_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWStartCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_START_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->HCONR, u16Cond);
    } else {
        CLR_REG16_BIT(TMRBx->HCONR, u16Cond);
    }
}

/**
 * @brief  Enable or disable the specified hardware stop condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Cond                 TMRB hardware start counter condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Stop_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWStopCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_STOP_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->HCONR, u16Cond);
    } else {
        CLR_REG16_BIT(TMRBx->HCONR, u16Cond);
    }
}

/**
 * @brief  Enable or disable the specified hardware clear condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Cond                 TMRB hardware start counter condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Clear_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWClearCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CLR_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->HCONR, u16Cond);
    } else {
        CLR_REG16_BIT(TMRBx->HCONR, u16Cond);
    }
}

/**
 * @brief  Enable or disable the specified hardware capture condition of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB compare channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16Cond                 TMRB hardware capture condition
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Capture_Condition
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_HWCaptureCondCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Cond, en_functional_state_t enNewState)
{
    __IO uint16_t *CCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CAPT_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_CAPT_COND(u16Cond));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CCONR = TMRB_CCONR_ADDR(TMRBx, u32Ch);
    if (ENABLE == enNewState) {
        SET_REG16_BIT(*CCONR, u16Cond);
    } else {
        CLR_REG16_BIT(*CCONR, u16Cond);
    }
}

/**
 * @brief  Set TMRB clock division.
 * @note   The CLK division function is valid when clock source is HCLK/PCLK.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u8Div                   TMRB clock division
 *         This parameter can be one of the following values:
 *           @arg TMRB_CLK_DIV1:        CLK
 *           @arg TMRB_CLK_DIV2:        CLK/2
 *           @arg TMRB_CLK_DIV4:        CLK/4
 *           @arg TMRB_CLK_DIV8:        CLK/8
 *           @arg TMRB_CLK_DIV16:       CLK/16
 *           @arg TMRB_CLK_DIV32:       CLK/32
 *           @arg TMRB_CLK_DIV64:       CLK/64
 *           @arg TMRB_CLK_DIV128:      CLK/128
 *           @arg TMRB_CLK_DIV256:      CLK/256
 *           @arg TMRB_CLK_DIV512:      CLK/512
 *           @arg TMRB_CLK_DIV1024:     CLK/1024
 * @retval None
 */
void TMRB_SetClockDiv(CM_TMRB_TypeDef *TMRBx, uint8_t u8Div)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CLK_DIV(u8Div));

    MODIFY_REG8(TMRBx->BCSTRL, TMRB_BCSTRL_CKDIV, u8Div);
}

/**
 * @brief  Set the TMRB count mode.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u8Mode                  The TMRB count mode
 *         This parameter can be one of the following values:
 *           @arg TMRB_MD_SAWTOOTH:     Sawtooth wave mode
 *           @arg TMRB_MD_TRIANGLE:     Triangle wave mode
 * @retval None
 */
void TMRB_SetCountMode(CM_TMRB_TypeDef *TMRBx, uint8_t u8Mode)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CNT_MD(u8Mode));

    MODIFY_REG8(TMRBx->BCSTRL, TMRB_BCSTRL_MODE, u8Mode);
}

/**
 * @brief  Set the TMRB count direction.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u8Dir                   The TMRB count direction
 *         This parameter can be one of the following values:
 *           @arg TMRB_DIR_DOWN:        Count down
 *           @arg TMRB_DIR_UP:          Count up
 * @retval None
 */
void TMRB_SetCountDir(CM_TMRB_TypeDef *TMRBx, uint8_t u8Dir)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CNT_DIR(u8Dir));

    MODIFY_REG8(TMRBx->BCSTRL, TMRB_BCSTRL_DIR, u8Dir);
}

/**
 * @brief  Get the TMRB count direction.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @retval uint8_t                      Count direction
 *           - TMRB_DIR_DOWN:           Count down
 *           - TMRB_DIR_UP:             Count up
 */
uint8_t TMRB_GetCountDir(const CM_TMRB_TypeDef *TMRBx)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));

    return READ_REG8_BIT(TMRBx->BCSTRL, TMRB_BCSTRL_DIR);
}

/**
 * @brief  Set TMRB function mode.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB compare channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16Func                 Function mode
 *         This parameter can be one of the following values:
 *           @arg TMRB_FUNC_CMP:        Compare output function
 *           @arg TMRB_FUNC_CAPT:       Capture input function
 * @retval None
 */
void TMRB_SetFunc(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Func)
{
    __IO uint16_t *CCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CAPT_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_FUNC_MD(u16Func));

    CCONR = TMRB_CCONR_ADDR(TMRBx, u32Ch);
    MODIFY_REG16(*CCONR, TMRB_CCONR_CAPMD, u16Func);
}

/**
 * @brief  Enable or disable TMRB reload function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_CountReloadCmd(CM_TMRB_TypeDef *TMRBx, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        CLR_REG8_BIT(TMRBx->BCSTRH, TMRB_BCSTRH_OVSTP);
    } else {
        SET_REG8_BIT(TMRBx->BCSTRH, TMRB_BCSTRH_OVSTP);
    }
}

#if defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120) || defined (HC32F165)
/**
 * @brief  Enable or disable TMRB sync start.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
           This parameter can be one of the following values:
 *           @arg CM_TMRB_x:            TMRB unit instance
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_SyncStartCmd(CM_TMRB_TypeDef *TMRBx, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_SYNC_UNIT(TMRBx));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG8_BIT(TMRBx->BCSTRL, TMRB_BCSTRL_SYNST);
    } else {
        CLR_REG8_BIT(TMRBx->BCSTRL, TMRB_BCSTRL_SYNST);
    }
}
#endif

/**
 * @brief  Set TMRB input capture noise filter clock division.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB compare channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16Div                  TMRB input capture filter clock division
 *         This parameter can be one of the following values:
 *           @arg TMRB_FILTER_CLK_DIV1:     CLK
 *           @arg TMRB_FILTER_CLK_DIV4:     CLK/4
 *           @arg TMRB_FILTER_CLK_DIV16:    CLK/16
 *           @arg TMRB_FILTER_CLK_DIV64:    CLK/64
 * @retval None
 */
void TMRB_SetFilterClockDiv(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Div)
{
    __IO uint16_t *CCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CAPT_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_FILTER_CLK_DIV(u16Div));

    CCONR = TMRB_CCONR_ADDR(TMRBx, u32Ch);
    MODIFY_REG16(*CCONR, TMRB_CCONR_NOFICKCP, u16Div);
}

/**
 * @brief  Enable or disable the TMRB input capture noise filter function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB compare channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_FilterCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint16_t *CCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CAPT_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CCONR = TMRB_CCONR_ADDR(TMRBx, u32Ch);
    if (ENABLE == enNewState) {
        SET_REG16_BIT(*CCONR, TMRB_CCONR_NOFIENCP);
    } else {
        CLR_REG16_BIT(*CCONR, TMRB_CCONR_NOFIENCP);
    }
}

/**
 * @brief  De-Initialize the TMRB PWM.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @retval None
 */
void TMRB_PWM_DeInit(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch)
{
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
    __IO uint16_t *BCONR;
#endif
    __IO uint16_t *PCONR;
    __IO uint16_t *CMPAR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));

    /* Configures the registers to reset value. */
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
    if (IS_TMRB_BUF_CH(u32Ch)) {
        BCONR = TMRB_BCONR_ADDR(TMRBx, u32Ch);
        WRITE_REG16(*BCONR, 0x0000U);
    }
#endif
    PCONR = TMRB_PCONR_ADDR(TMRBx, u32Ch);
    WRITE_REG16(*PCONR, 0x0000U);
    CMPAR = TMRB_CMPAR_ADDR(TMRBx, u32Ch);
    WRITE_REG16(*CMPAR, 0xFFFFU);
    CLR_REG16_BIT(TMRBx->ICONR, (1UL << u32Ch));
    CLR_REG16_BIT(TMRBx->ECONR, (1UL << u32Ch));
    CLR_REG16_BIT(TMRBx->STFLR, (1UL << u32Ch));
}

/**
 * @brief  Initialize TMRB PWM function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] pstcTmrbPwmInit         Pointer to a @ref stc_tmrb_pwm_init_t.
 * @retval int32_t:
 *           - LL_OK: Initialize success
 *           - LL_ERR_INVD_PARAM: pstcTmrbPwmInit is NULL
 */
int32_t TMRB_PWM_Init(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, const stc_tmrb_pwm_init_t *pstcTmrbPwmInit)
{
    __IO uint16_t *PCONR;
    __IO uint16_t *CMPAR;
    __IO uint16_t *CCONR;
    int32_t i32Ret = LL_OK;

    if (NULL == pstcTmrbPwmInit) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        /* Check parameters */
        DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
        DDL_ASSERT(IS_TMRB_CH(u32Ch));
        DDL_ASSERT(IS_TMRB_PWM_START_STOP_POLARITY(pstcTmrbPwmInit->u16StartPolarity));
        DDL_ASSERT(IS_TMRB_PWM_START_STOP_POLARITY(pstcTmrbPwmInit->u16StopPolarity));
        DDL_ASSERT(IS_TMRB_PWM_MATCH_POLARITY(pstcTmrbPwmInit->u16CompareMatchPolarity));
        DDL_ASSERT(IS_TMRB_PWM_MATCH_POLARITY(pstcTmrbPwmInit->u16PeriodMatchPolarity));

        CMPAR = TMRB_CMPAR_ADDR(TMRBx, u32Ch);
        WRITE_REG16(*CMPAR, pstcTmrbPwmInit->u16CompareValue);
        PCONR = TMRB_PCONR_ADDR(TMRBx, u32Ch);
        MODIFY_REG16(*PCONR, (TMRB_PCONR_STAC | TMRB_PCONR_STPC | TMRB_PCONR_CMPC | TMRB_PCONR_PERC),
                     ((uint16_t)(pstcTmrbPwmInit->u16StartPolarity        << TMRB_PCONR_STAC_POS) | \
                      (uint16_t)(pstcTmrbPwmInit->u16StopPolarity         << TMRB_PCONR_STPC_POS) | \
                      (uint16_t)(pstcTmrbPwmInit->u16CompareMatchPolarity << TMRB_PCONR_CMPC_POS) | \
                      (uint16_t)(pstcTmrbPwmInit->u16PeriodMatchPolarity  << TMRB_PCONR_PERC_POS)));
        /* Set TMRB compare output function mode */
        if (IS_TMRB_CAPT_CH(u32Ch)) {
            CCONR = TMRB_CCONR_ADDR(TMRBx, u32Ch);
            CLR_REG16_BIT(*CCONR, TMRB_CCONR_CAPMD);
        }
    }

    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_tmrb_pwm_init_t to default values.
 * @param  [out] pstcTmrbPwmInit            Pointer to a @ref stc_tmrb_pwm_init_t structure.
 * @retval int32_t:
 *           - LL_OK: Initialize success
 *           - LL_ERR_INVD_PARAM: pstcTmrbPwmInit is NULL
 */
int32_t TMRB_PWM_StructInit(stc_tmrb_pwm_init_t *pstcTmrbPwmInit)
{
    int32_t i32Ret = LL_OK;

    if (NULL == pstcTmrbPwmInit) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        pstcTmrbPwmInit->u16CompareValue         = 0xFFFFU;
        pstcTmrbPwmInit->u16StartPolarity        = TMRB_PWM_LOW;
        pstcTmrbPwmInit->u16StopPolarity         = TMRB_PWM_LOW;
        pstcTmrbPwmInit->u16CompareMatchPolarity = TMRB_PWM_LOW;
        pstcTmrbPwmInit->u16PeriodMatchPolarity  = TMRB_PWM_LOW;
    }

    return i32Ret;
}

/**
 * @brief  Set TIM_<t>_PWMn port output polarity.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16CountState           TMRB count state
 *         This parameter can be one of the following values:
 *           @arg TMRB_PWM_CNT_START:   The TMRB start count
 *           @arg TMRB_PWM_CNT_STOP:    The TMRB stop count
 *           @arg TMRB_PWM_CNT_MATCH:   The TMRB compare match
 *           @arg TMRB_PWM_CNT_PERIOD:  The TMRB period match
 * @param  [in] u16Polarity             TMRB TIM_<t>_PWMn port output polarity
 *         This parameter can be one of the following values:
 *           @arg TMRB_PWM_LOW:         TIM_<t>_PWMn output low level
 *           @arg TMRB_PWM_HIGH:        TIM_<t>_PWMn output high level
 *           @arg TMRB_PWM_HOLD:        TIM_<t>_PWMn output hold level
 *           @arg TMRB_PWM_INVT:        TIM_<t>_PWMn output inverted level
 * @retval None
 */
void TMRB_PWM_SetPolarity(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16CountState, uint16_t u16Polarity)
{
    __IO uint16_t *PCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_PWM_POLARITY(u16CountState, u16Polarity));

    PCONR = TMRB_PCONR_ADDR(TMRBx, u32Ch);
    MODIFY_REG16(*PCONR, ((uint16_t)TMRB_PCONR_STAC << u16CountState), ((uint16_t)u16Polarity << u16CountState));
}

/**
 * @brief  Set TIM_<t>_PWMn port force output polarity.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] u16Polarity             TMRB TIM_<t>_PWMn port output polarity
 *         This parameter can be one of the following values:
 *           @arg TMRB_PWM_FORCE_INVD:  Invalid
 *           @arg TMRB_PWM_FORCE_LOW:   Force TIM_<t>_PWMn output low level
 *           @arg TMRB_PWM_FORCE_HIGH:  Force TIM_<t>_PWMn force output high level
 * @retval None
 */
void TMRB_PWM_SetForcePolarity(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Polarity)
{
    __IO uint16_t *PCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_PWM_FORCE_POLARITY(u16Polarity));

    PCONR = TMRB_PCONR_ADDR(TMRBx, u32Ch);
    MODIFY_REG16(*PCONR, TMRB_PCONR_FORC, u16Polarity);
}

/**
 * @brief  Enable or disable the TMRB PWM TIM_<t>_PWMn port function.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_PWM_OutputCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint16_t *PCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    PCONR = TMRB_PCONR_ADDR(TMRBx, u32Ch);
    if (ENABLE == enNewState) {
        SET_REG16_BIT(*PCONR, TMRB_PCONR_OUTEN);
    } else {
        CLR_REG16_BIT(*PCONR, TMRB_PCONR_OUTEN);
    }
}

#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
/**
 * @brief  Set TMRB triangle wave buffer mode.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel     TMRB channel 1,3,..
 * @param  [in] u16BufCond              The buffer transfer condition
 *         This parameter can be one of the following values:
 *           @arg TMRB_BUF_COND_INVD:           Don't transfer buffer value
 *           @arg TMRB_BUF_COND_PEAK:           Transfer buffer value when TMRB count peak
 *           @arg TMRB_BUF_COND_VALLEY:         Transfer buffer value when TMRB count valley
 *           @arg TMRB_BUF_COND_PEAK_VALLEY:    Transfer buffer value when TMRB count peak or valley
 * @retval None
 */
void TMRB_SetCompareBufCond(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16BufCond)
{
    __IO uint16_t *BCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_BUF_CH(u32Ch));
    DDL_ASSERT(IS_TMRB_CMP_BUF_COND(u16BufCond));

    BCONR = TMRB_BCONR_ADDR(TMRBx, u32Ch);
    MODIFY_REG16(*BCONR, (TMRB_BCONR_BSE0 | TMRB_BCONR_BSE1), u16BufCond);
}

/**
 * @brief  Enable or disable the specified channel buffer function of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u32Ch                   TMRB channel
 *         This parameter can be one of the following values:
 *           @arg @ref TMRB_Channel     TMRB channel 1,3,..
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_CompareBufCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint16_t *BCONR;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_BUF_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    BCONR = TMRB_BCONR_ADDR(TMRBx, u32Ch);
    if (ENABLE == enNewState) {
        SET_REG16_BIT(*BCONR, TMRB_BCONR_BEN);
    } else {
        CLR_REG16_BIT(*BCONR, TMRB_BCONR_BEN);
    }
}
#endif

/**
 * @brief  Enable or disable the specified TMRB interrupt.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16IntType              TMRB interrupt type
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Interrupt
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_IntCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16IntType, en_functional_state_t enNewState)
{
    const uint8_t u8CountIntType = (uint8_t)((u16IntType & TMRB_CNT_INT_MASK) >> 8U);
    const uint16_t u16CmpIntType = (u16IntType & TMRB_CMP_INT_MASK);

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_INT(u16IntType));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG8_BIT(TMRBx->BCSTRH, u8CountIntType);
        SET_REG16_BIT(TMRBx->ICONR, u16CmpIntType);
    } else {
        CLR_REG8_BIT(TMRBx->BCSTRH, u8CountIntType);
        CLR_REG16_BIT(TMRBx->ICONR, u16CmpIntType);
    }
}

/**
 * @brief  Enable or disable the specified compare channel event of TMRB.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16EventType            TMRB event
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Event
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void TMRB_EventCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16EventType, en_functional_state_t enNewState)
{
    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_EVT(u16EventType));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    if (ENABLE == enNewState) {
        SET_REG16_BIT(TMRBx->ECONR, u16EventType);
    } else {
        CLR_REG16_BIT(TMRBx->ECONR, u16EventType);
    }
}

/**
 * @brief  Get TMRB flag status.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Flag                 TMRB flag type
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Flag
 * @retval An @ref en_flag_status_t enumeration type value.
 */
en_flag_status_t TMRB_GetStatus(const CM_TMRB_TypeDef *TMRBx, uint16_t u16Flag)
{
    en_flag_status_t enCntStatus = RESET;
    en_flag_status_t enCmpStatus = RESET;
    const uint8_t u8CountFlag = (uint8_t)((u16Flag & TMRB_CNT_FLAG_MASK) >> 8U);
    const uint16_t u16CmpFlag = (u16Flag & TMRB_CMP_FLAG_MASK);

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_FLAG(u16Flag));

    if (0U != READ_REG8_BIT(TMRBx->BCSTRH, u8CountFlag)) {
        enCntStatus = SET;
    }
    if (0U != READ_REG16_BIT(TMRBx->STFLR, u16CmpFlag)) {
        enCmpStatus = SET;
    }

    return ((SET == enCntStatus) || (SET == enCmpStatus)) ? SET : RESET;
}

/**
 * @brief  Clear TMRB flag.
 * @param  [in] TMRBx                   Pointer to TMRB unit instance
 *         This parameter can be one of the following values:
 *           @arg CM_TMRB or CM_TMRB_x: TMRB unit instance
 * @param  [in] u16Flag                 TMRB flag type
 *         This parameter can be any combination value of the following values:
 *           @arg @ref TMRB_Flag
 * @retval None
 */
void TMRB_ClearStatus(CM_TMRB_TypeDef *TMRBx, uint16_t u16Flag)
{
    uint16_t u16FlagTemp;

    /* Check parameters */
    DDL_ASSERT(IS_TMRB_UNIT(TMRBx));
    DDL_ASSERT(IS_TMRB_FLAG(u16Flag));

    u16FlagTemp = u16Flag & TMRB_CNT_FLAG_MASK;
    if (0U != u16FlagTemp) {
        CLR_REG8_BIT(TMRBx->BCSTRH, (uint8_t)(u16FlagTemp >> 8U));
    }
    u16FlagTemp = u16Flag & TMRB_CMP_FLAG_MASK;
    if (0U != u16FlagTemp) {
        CLR_REG16_BIT(TMRBx->STFLR, u16FlagTemp);
    }
}

/**
 * @}
 */

#endif /* LL_TMRB_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
