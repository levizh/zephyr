/**
 *******************************************************************************
 * @file  hc32_ll_hrpwm.c
 * @brief This file provides firmware functions to manage the High Resolution
 *        Pulse-Width Modulation(HRPWM).
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-09-30       CDT             Modify typo [-HC32F4A0-]
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
#include "hc32_ll_hrpwm.h"


/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @defgroup LL_HRPWM HRPWM
 * @brief HRPWM Driver Library
 * @{
 */

#if (LL_HRPWM_ENABLE == DDL_ON)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup HRPWM_Local_Macros HRPWM Local Macros
 * @{
 */

#if defined (HC32F4A0)
/* About 1mS timeout */
#define HRPWM_CAL_TIMEOUT               (HCLK_VALUE/1000UL)
#define HRPWM_PCLK0_MIN                 (120000000UL)

#define HRPWM_SYSCLKSRC_HRC             (0x00U)
#define HRPWM_SYSCLKSRC_MRC             (0x01U)
#define HRPWM_SYSCLKSRC_LRC             (0x02U)
#define HRPWM_SYSCLKSRC_XTAL            (0x03U)
#define HRPWM_SYSCLKSRC_XTAL32          (0x04U)
#define HRPWM_SYSCLKSRC_PLL             (0x05U)

#define HRPWM_PLLSRC_XTAL               (0x00UL)
#define HRPWM_PLLSRC_HRC                (0x01UL)

/**
 * @defgroup HRPWM_Check_Param_Validity HRPWM Check Parameters Validity
 * @{
 */

/*! Parameter valid check for HRPWM output channel */
#define IS_VALID_HRPWM_CH(x)                                                   \
(   ((x) >= HRPWM_CH_MIN)                       &&                             \
    ((x) <= HRPWM_CH_MAX))

/*! Parameter valid check for HRPWM calibration unit */
#define IS_VALID_HRPWM_CAL_UNIT(x)                                             \
(   (HRPWM_CAL_UNIT0 == (x))                    ||                             \
    (HRPWM_CAL_UNIT1 == (x)))
#endif

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
 * @defgroup HRPWM_Global_Functions HRPWM Global Functions
 * @{
 */

#if defined (HC32F4A0)
/**
 * @brief  Process for getting HRPWM Calibrate function code
 * @param  [in] u32Unit             Calibrate unit, the parameter should be HRPWM_CAL_UNIT0 or HRPWM_CAL_UNIT1
 * @param  [out] pu8Code            The pointer to get calibrate code.
 * @retval int32_t:
 *         - LL_OK:                 Success
 *         - LL_ERR_TIMEOUT:        Time out
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_CalibrateProcess(uint32_t u32Unit, uint8_t *pu8Code)
{
    __IO uint32_t u32Timeout = HRPWM_CAL_TIMEOUT;
    int32_t i32Ret = LL_OK;

    if (NULL != pu8Code) {
        /* Enable calibrate */
        HRPWM_CalibrateCmd(u32Unit, ENABLE);
        /* Wait calibrate finish flag */
        while (DISABLE == HRPWM_GetCalibrateState(u32Unit)) {
            if (0UL == u32Timeout--) {
                i32Ret = LL_ERR_TIMEOUT;
                break;
            }
        }

        if (LL_OK == i32Ret) {
            /* Get calibrate code */
            *pu8Code = HRPWM_GetCalibrateCode(u32Unit);
        }
    } else {
        i32Ret = LL_ERR_INVD_PARAM;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM Calibrate function enable or disable for specified unit
 * @param  [in] u32Unit             Calibrate unit, the parameter should be HRPWM_CAL_UNIT0 or HRPWM_CAL_UNIT1
 * @param  [in] enNewState          An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void HRPWM_CalibrateCmd(uint32_t u32Unit, en_functional_state_t enNewState)
{
    __IO uint32_t *CALCRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CAL_UNIT(u32Unit));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CALCRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM_COMMON->CALCR0) + 4UL * u32Unit);

    if (ENABLE == enNewState) {
        SET_REG32_BIT(*CALCRx, HRPWM_COMMON_CALCR_CALEN);
    } else {
        CLR_REG32_BIT(*CALCRx, HRPWM_COMMON_CALCR_CALEN);
    }
}

/**
 * @brief  HRPWM Calibrate function status get for specified unit
 * @param  [in] u32Unit             Calibrate unit, the parameter should be HRPWM_CAL_UNIT0 or HRPWM_CAL_UNIT1
 * @retval An @ref en_functional_state_t enumeration value.
 */
en_functional_state_t HRPWM_GetCalibrateState(uint32_t u32Unit)
{
    en_functional_state_t enRet;
    __IO uint32_t *CALCRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CAL_UNIT(u32Unit));

    CALCRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM_COMMON->CALCR0) + 4UL * u32Unit);

    if (0UL != READ_REG32_BIT(*CALCRx, HRPWM_COMMON_CALCR_ENDF)) {
        enRet = ENABLE;
    } else {
        enRet = DISABLE;
    }
    return enRet;
}

/**
 * @brief  HRPWM Calibrate code get for specified unit
 * @param  [in] u32Unit             Calibrate unit, the parameter should be HRPWM_CAL_UNIT0 or HRPWM_CAL_UNIT1
 * @retval uint8_t:                 The calibration code.
 */
uint8_t HRPWM_GetCalibrateCode(uint32_t u32Unit)
{
    __IO uint32_t *CALCRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CAL_UNIT(u32Unit));

    CALCRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM_COMMON->CALCR0) + 4UL * u32Unit);

    return ((uint8_t)(READ_REG32(*CALCRx)));
}

/**
 * @brief  HRPWM function enable or disable for specified channel
 * @param  [in] u32Ch               Channel, the parameter should range from HRPWM_CH_MIN to HRPWM_CH_MAX
 * @param  [in] enNewState          An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void HRPWM_ChCmd(uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint32_t *CRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM->CR1) + 4UL * (u32Ch - 1UL));
    if (ENABLE == enNewState) {
        SET_REG32_BIT(*CRx, HRPWM_CR_EN);
    } else {
        CLR_REG32_BIT(*CRx, HRPWM_CR_EN);
    }
}

/**
 * @brief  HRPWM positive edge adjust enable or disable for specified channel
 * @param  [in] u32Ch               Channel, the parameter should range from HRPWM_CH_MIN to HRPWM_CH_MAX
 * @param  [in] enNewState          An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void HRPWM_ChPositiveAdjustCmd(uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint32_t *CRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM->CR1) + 4UL * (u32Ch - 1UL));
    if (ENABLE == enNewState) {
        SET_REG32_BIT(*CRx, HRPWM_CR_PE);
    } else {
        CLR_REG32_BIT(*CRx, HRPWM_CR_PE);
    }
}

/**
 * @brief  HRPWM negative edge adjust enable or disable for specified channel
 * @param  [in] u32Ch               Channel, the parameter should range from HRPWM_CH_MIN to HRPWM_CH_MAX
 * @param  [in] enNewState          An @ref en_functional_state_t enumeration value.
 * @retval None
 */
void HRPWM_ChNegativeAdjustCmd(uint32_t u32Ch, en_functional_state_t enNewState)
{
    __IO uint32_t *CRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CH(u32Ch));
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    CRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM->CR1) + 4UL * (u32Ch - 1UL));
    if (ENABLE == enNewState) {
        SET_REG32_BIT(*CRx, HRPWM_CR_NE);
    } else {
        CLR_REG32_BIT(*CRx, HRPWM_CR_NE);
    }
}

/**
 * @brief  HRPWM positive edge adjust delay counts configuration for specified channel
 * @param  [in] u32Ch               Channel, the parameter should range from HRPWM_CH_MIN to HRPWM_CH_MAX
 * @param  [in] u8DelayNum          Delay counts of minimum delay time.
 * @retval None
 */
void HRPWM_ChPositiveAdjustConfig(uint32_t u32Ch, uint8_t u8DelayNum)
{
    __IO uint32_t *CRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CH(u32Ch));

    CRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM->CR1) + 4UL * (u32Ch - 1UL));
    MODIFY_REG32(*CRx, HRPWM_CR_PSEL, ((uint32_t)u8DelayNum - 1UL) << HRPWM_CR_PSEL_POS);
}

/**
 * @brief  HRPWM negative edge adjust delay counts configuration for specified channel
 * @param  [in] u32Ch               Channel, the parameter should range from HRPWM_CH_MIN to HRPWM_CH_MAX
 * @param  [in] u8DelayNum          Delay counts of minimum delay time.
 * @retval None
 */
void HRPWM_ChNegativeAdjustConfig(uint32_t u32Ch, uint8_t u8DelayNum)
{
    __IO uint32_t *CRx;
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_CH(u32Ch));

    CRx = (__IO uint32_t *)(((uint32_t)&CM_HRPWM->CR1) + 4UL * (u32Ch - 1UL));
    MODIFY_REG32(*CRx, HRPWM_CR_NSEL, ((uint32_t)u8DelayNum - 1UL) << HRPWM_CR_NSEL_POS);
}

/**
 * @brief  HRPWM Judge the condition of calibration function.
 * @param  None
 * @retval An @ref en_functional_state_t enumeration value.
 */
en_functional_state_t HRPWM_CondConfirm(void)
{
    en_functional_state_t enRet = ENABLE;
    uint32_t plln;
    uint32_t pllp;
    uint32_t pllm;
    uint32_t u32SysclkFreq;
    uint32_t u32Pclk0Freq;

    switch (READ_REG8_BIT(CM_CMU->CKSWR, CMU_CKSWR_CKSW)) {
        case HRPWM_SYSCLKSRC_HRC:
            /* HRC is used to system clock */
            u32SysclkFreq = HRC_VALUE;
            break;
        case HRPWM_SYSCLKSRC_MRC:
            /* MRC is used to system clock */
            u32SysclkFreq = MRC_VALUE;
            break;
        case HRPWM_SYSCLKSRC_LRC:
            /* LRC is used to system clock */
            u32SysclkFreq = LRC_VALUE;
            break;
        case HRPWM_SYSCLKSRC_XTAL:
            /* XTAL is used to system clock */
            u32SysclkFreq = XTAL_VALUE;
            break;
        case HRPWM_SYSCLKSRC_XTAL32:
            /* XTAL32 is used to system clock */
            u32SysclkFreq = XTAL32_VALUE;
            break;
        case HRPWM_SYSCLKSRC_PLL:
            /* PLLHP is used as system clock. */
            pllp = (uint32_t)((CM_CMU->PLLHCFGR >> CMU_PLLHCFGR_PLLHP_POS) & 0x0FUL);
            plln = (uint32_t)((CM_CMU->PLLHCFGR >> CMU_PLLHCFGR_PLLHN_POS) & 0xFFUL);
            pllm = (uint32_t)((CM_CMU->PLLHCFGR >> CMU_PLLHCFGR_PLLHM_POS) & 0x03UL);

            /* fpll = ((pllin / pllm) * plln) / pllp */
            if (HRPWM_PLLSRC_XTAL == READ_REG32_BIT(CM_CMU->PLLHCFGR, CMU_PLLHCFGR_PLLSRC)) {
                u32SysclkFreq = ((XTAL_VALUE / (pllm + 1UL)) * (plln + 1UL)) / (pllp + 1UL);
            } else {
                u32SysclkFreq = ((HRC_VALUE / (pllm + 1UL)) * (plln + 1UL)) / (pllp + 1UL);
            }
            break;
        default:
            u32SysclkFreq = HRC_VALUE;
            enRet = DISABLE;
            break;
    }

    if (ENABLE == enRet) {
        /* Get pclk0. */
        u32Pclk0Freq = u32SysclkFreq >> (READ_REG32_BIT(CM_CMU->SCFGR, CMU_SCFGR_PCLK0S) >> CMU_SCFGR_PCLK0S_POS);

        if (u32Pclk0Freq < HRPWM_PCLK0_MIN) {
            enRet = DISABLE;
        }
    }
    return enRet;
}

#elif defined (HC32F334)
/**
 * @brief  Calibrate process for single mode
 * @param  None
 * @retval int32_t:
 *         - LL_OK:                 Success
 *         - LL_ERR_TIMEOUT:        Time out
 *         - LL_ERR:                Failed
 * @note  Please confirm that the pclk0 is ready for the function
 */
int32_t HRPWM_Calibrate_ProcessSingle(void)
{
    __IO uint32_t u32Timeout = HRPWM_CAL_TIMEOUT;
    int32_t i32Ret = LL_OK;

    HRPWM_Calibrate_SingleDisable();
    HRPWM_Calibrate_PeriodDisable();
    HRPWM_Calibrate_ClearFlag(HRPWM_CAL_FLAG_END | HRPWM_CAL_FLAG_ERR);
    HRPWM_Calibrate_SingleEnable();

    while (RESET == HRPWM_Calibrate_GetStatus(HRPWM_CAL_FLAG_END)) {
        if (0UL == u32Timeout--) {
            i32Ret = LL_ERR_TIMEOUT;
            break;
        }
    }
    if (LL_OK == i32Ret) {
        if (SET == HRPWM_Calibrate_GetStatus(HRPWM_CAL_FLAG_ERR)) {
            i32Ret = LL_ERR;
        }
    }
    return i32Ret;
}

/**
 * @brief  Calibrate initialize for period mode
 * @param  [in] u32Period           Calibrate period, @ref HRPWM_Calibrate_Period_Define
 * @retval None
 * @note  Please confirm that the pclk0 is ready for the function
 */
void HRPWM_Calibrate_PeriodInit(uint32_t u32Period)
{
    HRPWM_Calibrate_SingleDisable();
    HRPWM_Calibrate_PeriodDisable();
    HRPWM_Calibrate_ClearFlag(HRPWM_CAL_FLAG_END | HRPWM_CAL_FLAG_ERR);

    HRPWM_Calibrate_SetPeriod(u32Period);
    HRPWM_Calibrate_PeriodEnable();
}

/**
 * @brief  Set the fields of structure stc_hrpwm_init_t to default values
 * @param  [out] pstcHrpwmInit      Pointer to a @ref stc_hrpwm_init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_StructInit(stc_hrpwm_init_t *pstcHrpwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcHrpwmInit) {
        pstcHrpwmInit->u32CountMode = HRPWM_MD_SAWTOOTH;
        pstcHrpwmInit->u32CountReload = HRPWM_CNT_RELOAD_ON;
        pstcHrpwmInit->u32PeriodValue = HRPWM_REG_VALUE_3FFFC0;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Initialize the HRPWM count function
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcHrpwmInit       Pointer of configuration structure @ref stc_hrpwm_init_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 * @note  Make sure the HRPWM had been calibrated before calling this function
 */
int32_t HRPWM_Init(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_init_t *pstcHrpwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcHrpwmInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_CNT_MD(pstcHrpwmInit->u32CountMode));
        DDL_ASSERT(IS_VALID_CNT_RELOAD_MD(pstcHrpwmInit->u32CountReload));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE1(pstcHrpwmInit->u32PeriodValue));

        WRITE_REG32(HRPWMx->HRPERAR, pstcHrpwmInit->u32PeriodValue);
        WRITE_REG32(HRPWMx->GCONR, pstcHrpwmInit->u32CountMode | pstcHrpwmInit->u32CountReload);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  De-initialize the HRPWM unit
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @retval None
 */
void HRPWM_DeInit(CM_HRPWM_TypeDef *HRPWMx)
{
    /* Check parameters */
    DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));

    WRITE_REG32(HRPWMx->GCONR, 0UL);

    WRITE_REG32(HRPWMx->CNTER, 0UL);
    WRITE_REG32(HRPWMx->UPDAR, 0UL);
    WRITE_REG32(HRPWMx->HRPERAR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRPERBR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMAR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMBR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMCR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMDR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMER, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMFR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMGR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRGCMHR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->SCMAR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->SCMBR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->SCMCR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->SCMDR, HRPWM_REG_VALUE_3FFFC0);

    WRITE_REG32(HRPWMx->HRDTUAR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRDTDAR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRDTUBR, HRPWM_REG_VALUE_3FFFC0);
    WRITE_REG32(HRPWMx->HRDTDBR, HRPWM_REG_VALUE_3FFFC0);

    WRITE_REG32(HRPWMx->ICONR, 0UL);
    WRITE_REG32(HRPWMx->BCONR1, 0UL);
    WRITE_REG32(HRPWMx->DCONR, 0UL);
    WRITE_REG32(HRPWMx->PCNAR1, 0UL);
    WRITE_REG32(HRPWMx->PCNBR1, 0UL);
    WRITE_REG32(HRPWMx->VPERR, 0UL);
    WRITE_REG32(HRPWMx->STFLR1, 0x80000000UL);
    WRITE_REG32(HRPWMx->STFLR2, 0UL);
    WRITE_REG32(HRPWMx->HSTAR1, 0UL);
    WRITE_REG32(HRPWMx->HCLRR1, 0UL);
    WRITE_REG32(HRPWMx->HCPAR1, 0UL);
    WRITE_REG32(HRPWMx->HCPBR1, 0UL);
    WRITE_REG32(HRPWMx->HCPAR2, 0UL);
    WRITE_REG32(HRPWMx->HCPBR2, 0UL);
    WRITE_REG32(HRPWMx->BCONR2, 0UL);

    WRITE_REG32(HRPWMx->PCNAR2, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->PCNBR2, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->HSTAR2, 0UL);
    WRITE_REG32(HRPWMx->HCLRR2, 0UL);

    WRITE_REG32(HRPWMx->EEFOFFSETAR, 0x00400000UL);
    WRITE_REG32(HRPWMx->EEFOFFSETBR, 0x00400000UL);
    WRITE_REG32(HRPWMx->EEFWINAR, 0x00400000UL);
    WRITE_REG32(HRPWMx->EEFWINBR, 0x00400000UL);
    WRITE_REG32(HRPWMx->EEFLTCR1, 0UL);
    WRITE_REG32(HRPWMx->EEFLTCR2, 0UL);
    WRITE_REG32(HRPWMx->PCNAR3, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->PCNBR3, 0x000AAAAAUL);

    WRITE_REG32(HRPWMx->IDLECR, 0UL);
    WRITE_REG32(HRPWMx->GCONR1, 0UL);

    WRITE_REG32(HRPWMx->BICONR, 0UL);
    WRITE_REG32(HRPWMx->BPCNAR1, 0UL);
    WRITE_REG32(HRPWMx->BPCNBR1, 0UL);
    WRITE_REG32(HRPWMx->BPCNAR2, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->BPCNBR2, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->BPCNAR3, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->BPCNBR3, 0x000AAAAAUL);
    WRITE_REG32(HRPWMx->BGCONR1, 0UL);

    WRITE_REG32(HRPWMx->SCMASELR, 0UL);
    WRITE_REG32(HRPWMx->CR, 0UL);
    WRITE_REG32(HRPWMx->PHSCTL, 0UL);
    WRITE_REG32(HRPWMx->PHSCMP1A, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP1B, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP2A, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP2B, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP3A, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP3B, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP4A, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP4B, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP5A, 0x007FFFC0UL);
    WRITE_REG32(HRPWMx->PHSCMP5B, 0x007FFFC0UL);
}

/**
 * @brief  De-initialize the HRPWM COMMON register
 * @param  None
 * @retval None
 */
void HRPWM_COMMON_DeInit(void)
{
    WRITE_REG32(CM_HRPWM_COMMON->CALCR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SCAPR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SSTAIDLR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SSTARUNR1, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SSTADIDLR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->GCTLR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->GBCONR, 0x0000003FUL);
    WRITE_REG32(CM_HRPWM_COMMON->GBSFLR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->BMCR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->BMSTRG1, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->BMSTRG2, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->BMPERAR, 0x0000FFFFUL);
    WRITE_REG32(CM_HRPWM_COMMON->BMPERBR, 0x0000FFFFUL);
    WRITE_REG32(CM_HRPWM_COMMON->BMCMAR, 0x0000FFFFUL);
    WRITE_REG32(CM_HRPWM_COMMON->BMCMBR, 0x0000FFFFUL);
    WRITE_REG32(CM_HRPWM_COMMON->EECR1, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->EECR2, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->EECR3, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SYNOCR, 0x00001000UL);
    WRITE_REG32(CM_HRPWM_COMMON->EEDSELR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->FCNTR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SSTAR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SSTPR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SCLRR, 0UL);
    WRITE_REG32(CM_HRPWM_COMMON->SUPDR, 0UL);
}

/**
 * @brief  Set the fields of structure stc_hrpwm_pwm_init_t to default values
 * @param  [out] pstcPwmInit        Pointer to a @ref stc_hrpwm_pwm_init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_StructInit(stc_hrpwm_pwm_init_t *pstcPwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcPwmInit) {
        pstcPwmInit->u32CompareValue = HRPWM_REG_VALUE_3FFFC0;
        pstcPwmInit->u32StartPolarity = HRPWM_PWM_START_LOW;
        pstcPwmInit->u32StopPolarity = HRPWM_PWM_STOP_LOW;
        pstcPwmInit->u32PeakPolarity = HRPWM_PWM_PEAK_LOW;
        pstcPwmInit->u32ValleyPolarity = HRPWM_PWM_VALLEY_LOW;
        pstcPwmInit->u32UpMatchAPolarity = HRPWM_PWM_UP_MATCH_A_LOW;
        pstcPwmInit->u32DownMatchAPolarity = HRPWM_PWM_DOWN_MATCH_A_LOW;
        pstcPwmInit->u32UpMatchBPolarity = HRPWM_PWM_UP_MATCH_B_LOW;
        pstcPwmInit->u32DownMatchBPolarity = HRPWM_PWM_DOWN_MATCH_B_LOW;
        pstcPwmInit->u32UpMatchEPolarity = HRPWM_PWM_UP_MATCH_E_LOW;
        pstcPwmInit->u32DownMatchEPolarity = HRPWM_PWM_DOWN_MATCH_E_LOW;
        pstcPwmInit->u32UpMatchFPolarity = HRPWM_PWM_UP_MATCH_F_LOW;
        pstcPwmInit->u32DownMatchFPolarity = HRPWM_PWM_DOWN_MATCH_F_LOW;
        pstcPwmInit->u32UpMatchSpecialAPolarity = HRPWM_PWM_UP_MATCH_SPECIAL_A_LOW;
        pstcPwmInit->u32DownMatchSpecialAPolarity = HRPWM_PWM_DOWN_MATCH_SPECIAL_A_LOW;
        pstcPwmInit->u32UpMatchSpecialBPolarity = HRPWM_PWM_UP_MATCH_SPECIAL_B_LOW;
        pstcPwmInit->u32DownMatchSpecialBPolarity = HRPWM_PWM_DOWN_MATCH_SPECIAL_B_LOW;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM Channel A function
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmInit         Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_ChAInit(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_init_t *pstcPwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE1(pstcPwmInit->u32CompareValue));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_START(pstcPwmInit->u32StartPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_STOP(pstcPwmInit->u32StopPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_PEAK(pstcPwmInit->u32PeakPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_VALLEY(pstcPwmInit->u32ValleyPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_A(pstcPwmInit->u32UpMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_A(pstcPwmInit->u32DownMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_B(pstcPwmInit->u32UpMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_B(pstcPwmInit->u32DownMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_E(pstcPwmInit->u32UpMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_E(pstcPwmInit->u32DownMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_F(pstcPwmInit->u32UpMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_F(pstcPwmInit->u32DownMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_A(pstcPwmInit->u32UpMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_A(pstcPwmInit->u32DownMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_B(pstcPwmInit->u32UpMatchSpecialBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_B(pstcPwmInit->u32DownMatchSpecialBPolarity));

        WRITE_REG32(HRPWMx->HRGCMAR, pstcPwmInit->u32CompareValue);
        MODIFY_REG32(HRPWMx->PCNAR1, PCNAR1_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32StartPolarity
                     | pstcPwmInit->u32StopPolarity
                     | pstcPwmInit->u32PeakPolarity
                     | pstcPwmInit->u32ValleyPolarity
                     | pstcPwmInit->u32UpMatchAPolarity
                     | pstcPwmInit->u32DownMatchAPolarity
                     | pstcPwmInit->u32UpMatchBPolarity
                     | pstcPwmInit->u32DownMatchBPolarity);
        MODIFY_REG32(HRPWMx->PCNAR2, PCNAR2_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32UpMatchEPolarity
                     | pstcPwmInit->u32UpMatchFPolarity
                     | pstcPwmInit->u32UpMatchSpecialAPolarity
                     | pstcPwmInit->u32UpMatchSpecialBPolarity);
        MODIFY_REG32(HRPWMx->PCNAR3, PCNAR3_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32DownMatchEPolarity
                     | pstcPwmInit->u32DownMatchFPolarity
                     | pstcPwmInit->u32DownMatchSpecialAPolarity
                     | pstcPwmInit->u32DownMatchSpecialBPolarity);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM Channel A function (buffer)
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmInit         Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_ChAInit_Buf(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_init_t *pstcPwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE(pstcPwmInit->u32CompareValue));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_START(pstcPwmInit->u32StartPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_STOP(pstcPwmInit->u32StopPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_PEAK(pstcPwmInit->u32PeakPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_VALLEY(pstcPwmInit->u32ValleyPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_A(pstcPwmInit->u32UpMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_A(pstcPwmInit->u32DownMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_B(pstcPwmInit->u32UpMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_B(pstcPwmInit->u32DownMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_E(pstcPwmInit->u32UpMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_E(pstcPwmInit->u32DownMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_F(pstcPwmInit->u32UpMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_F(pstcPwmInit->u32DownMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_A(pstcPwmInit->u32UpMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_A(pstcPwmInit->u32DownMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_B(pstcPwmInit->u32UpMatchSpecialBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_B(pstcPwmInit->u32DownMatchSpecialBPolarity));

        WRITE_REG32(HRPWMx->HRGCMCR, pstcPwmInit->u32CompareValue);
        MODIFY_REG32(HRPWMx->BPCNAR1, PCNAR1_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32StartPolarity
                     | pstcPwmInit->u32StopPolarity
                     | pstcPwmInit->u32PeakPolarity
                     | pstcPwmInit->u32ValleyPolarity
                     | pstcPwmInit->u32UpMatchAPolarity
                     | pstcPwmInit->u32DownMatchAPolarity
                     | pstcPwmInit->u32UpMatchBPolarity
                     | pstcPwmInit->u32DownMatchBPolarity);
        MODIFY_REG32(HRPWMx->BPCNAR2, PCNAR2_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32UpMatchEPolarity
                     | pstcPwmInit->u32UpMatchFPolarity
                     | pstcPwmInit->u32UpMatchSpecialAPolarity
                     | pstcPwmInit->u32UpMatchSpecialBPolarity);
        MODIFY_REG32(HRPWMx->BPCNAR3, PCNAR3_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32DownMatchEPolarity
                     | pstcPwmInit->u32DownMatchFPolarity
                     | pstcPwmInit->u32DownMatchSpecialAPolarity
                     | pstcPwmInit->u32DownMatchSpecialBPolarity);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM Channel B function
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmInit         Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_ChBInit(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_init_t *pstcPwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE(pstcPwmInit->u32CompareValue));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_START(pstcPwmInit->u32StartPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_STOP(pstcPwmInit->u32StopPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_PEAK(pstcPwmInit->u32PeakPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_VALLEY(pstcPwmInit->u32ValleyPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_A(pstcPwmInit->u32UpMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_A(pstcPwmInit->u32DownMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_B(pstcPwmInit->u32UpMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_B(pstcPwmInit->u32DownMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_E(pstcPwmInit->u32UpMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_E(pstcPwmInit->u32DownMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_F(pstcPwmInit->u32UpMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_F(pstcPwmInit->u32DownMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_A(pstcPwmInit->u32UpMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_A(pstcPwmInit->u32DownMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_B(pstcPwmInit->u32UpMatchSpecialBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_B(pstcPwmInit->u32DownMatchSpecialBPolarity));

        WRITE_REG32(HRPWMx->HRGCMBR, pstcPwmInit->u32CompareValue);
        MODIFY_REG32(HRPWMx->PCNBR1, PCNBR1_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32StartPolarity
                     | pstcPwmInit->u32StopPolarity
                     | pstcPwmInit->u32PeakPolarity
                     | pstcPwmInit->u32ValleyPolarity
                     | pstcPwmInit->u32UpMatchAPolarity
                     | pstcPwmInit->u32DownMatchAPolarity
                     | pstcPwmInit->u32UpMatchBPolarity
                     | pstcPwmInit->u32DownMatchBPolarity);
        MODIFY_REG32(HRPWMx->PCNBR2, PCNBR2_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32UpMatchEPolarity
                     | pstcPwmInit->u32UpMatchFPolarity
                     | pstcPwmInit->u32UpMatchSpecialAPolarity
                     | pstcPwmInit->u32UpMatchSpecialBPolarity);
        MODIFY_REG32(HRPWMx->PCNBR3, PCNBR3_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32DownMatchEPolarity
                     | pstcPwmInit->u32DownMatchFPolarity
                     | pstcPwmInit->u32DownMatchSpecialAPolarity
                     | pstcPwmInit->u32DownMatchSpecialBPolarity);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM Channel B function (buffer)
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmInit         Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_ChBInit_Buf(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_init_t *pstcPwmInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE(pstcPwmInit->u32CompareValue));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_START(pstcPwmInit->u32StartPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_STOP(pstcPwmInit->u32StopPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_PEAK(pstcPwmInit->u32PeakPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_VALLEY(pstcPwmInit->u32ValleyPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_A(pstcPwmInit->u32UpMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_A(pstcPwmInit->u32DownMatchAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_B(pstcPwmInit->u32UpMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_B(pstcPwmInit->u32DownMatchBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_E(pstcPwmInit->u32UpMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_E(pstcPwmInit->u32DownMatchEPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_F(pstcPwmInit->u32UpMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_F(pstcPwmInit->u32DownMatchFPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_A(pstcPwmInit->u32UpMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_A(pstcPwmInit->u32DownMatchSpecialAPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_UP_MATCH_SPECIAL_B(pstcPwmInit->u32UpMatchSpecialBPolarity));
        DDL_ASSERT(IS_VALID_PWM_POLARITY_DOWN_MATCH_SPECIAL_B(pstcPwmInit->u32DownMatchSpecialBPolarity));

        WRITE_REG32(HRPWMx->HRGCMDR, pstcPwmInit->u32CompareValue);
        MODIFY_REG32(HRPWMx->BPCNBR1, PCNBR1_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32StartPolarity
                     | pstcPwmInit->u32StopPolarity
                     | pstcPwmInit->u32PeakPolarity
                     | pstcPwmInit->u32ValleyPolarity
                     | pstcPwmInit->u32UpMatchAPolarity
                     | pstcPwmInit->u32DownMatchAPolarity
                     | pstcPwmInit->u32UpMatchBPolarity
                     | pstcPwmInit->u32DownMatchBPolarity);
        MODIFY_REG32(HRPWMx->BPCNBR2, PCNBR2_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32UpMatchEPolarity
                     | pstcPwmInit->u32UpMatchFPolarity
                     | pstcPwmInit->u32UpMatchSpecialAPolarity
                     | pstcPwmInit->u32UpMatchSpecialBPolarity);
        MODIFY_REG32(HRPWMx->BPCNBR3, PCNBR3_REG_POLARITY_CFG_MASK,
                     pstcPwmInit->u32DownMatchEPolarity
                     | pstcPwmInit->u32DownMatchFPolarity
                     | pstcPwmInit->u32DownMatchSpecialAPolarity
                     | pstcPwmInit->u32DownMatchSpecialBPolarity);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_pwm_output_init_t to default values
 * @param  [out] pstcPwmOutputInit  Pointer to a @ref stc_hrpwm_pwm_output_init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_OutputStructInit(stc_hrpwm_pwm_output_init_t *pstcPwmOutputInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcPwmOutputInit) {
        pstcPwmOutputInit->u32ChSwapMode = HRPWM_PWM_CH_SWAP_MD_NOT_IMMED;
        pstcPwmOutputInit->u32ChSwap = HRPWM_PWM_CH_SWAP_OFF;
        pstcPwmOutputInit->u32ChAReverse = HRPWM_PWM_CH_A_REVS_OFF;
        pstcPwmOutputInit->u32ChBReverse = HRPWM_PWM_CH_B_REVS_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM output function
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmOutputInit   Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_OutputInit(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_output_init_t *pstcPwmOutputInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmOutputInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_PWM_SWAP_MD(pstcPwmOutputInit->u32ChSwapMode));
        DDL_ASSERT(IS_VALID_PWM_SWAP(pstcPwmOutputInit->u32ChSwap));
        DDL_ASSERT(IS_VALID_PWM_CH_A_REVS(pstcPwmOutputInit->u32ChAReverse));
        DDL_ASSERT(IS_VALID_PWM_CH_B_REVS(pstcPwmOutputInit->u32ChBReverse));

        MODIFY_REG32(HRPWMx->GCONR1,
                     HRPWM_GCONR1_SWAPMD | HRPWM_GCONR1_SWAPEN | HRPWM_GCONR1_INVCAEN | HRPWM_GCONR1_INVCBEN,
                     pstcPwmOutputInit->u32ChSwapMode
                     | pstcPwmOutputInit->u32ChSwap
                     | pstcPwmOutputInit->u32ChAReverse
                     | pstcPwmOutputInit->u32ChBReverse);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize PWM output function (buffer)
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcPwmOutputInit   Pointer of initialize structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PWM_OutputInit_Buf(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_pwm_output_init_t *pstcPwmOutputInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    if (NULL != pstcPwmOutputInit) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_PWM_SWAP_MD(pstcPwmOutputInit->u32ChSwapMode));
        DDL_ASSERT(IS_VALID_PWM_SWAP(pstcPwmOutputInit->u32ChSwap));
        DDL_ASSERT(IS_VALID_PWM_CH_A_REVS(pstcPwmOutputInit->u32ChAReverse));
        DDL_ASSERT(IS_VALID_PWM_CH_B_REVS(pstcPwmOutputInit->u32ChBReverse));

        MODIFY_REG32(HRPWMx->BGCONR1,
                     HRPWM_GCONR1_SWAPMD | HRPWM_GCONR1_SWAPEN | HRPWM_GCONR1_INVCAEN | HRPWM_GCONR1_INVCBEN,
                     pstcPwmOutputInit->u32ChSwapMode
                     | pstcPwmOutputInit->u32ChSwap
                     | pstcPwmOutputInit->u32ChAReverse
                     | pstcPwmOutputInit->u32ChBReverse);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRWPM trigger pin input filter clock configuration
 * @param  [in] u32Pin              Pin to be configured
 *   @arg  HRPWM_INPUT_TRIGA
 *   @arg  HRPWM_INPUT_TRIGB
 *   @arg  HRPWM_INPUT_TRIGC
 *   @arg  HRPWM_INPUT_TRIGD
 * @param  [in] u32Div              Filter clock @ref HRPWM_Input_Filter_Clock_Define
 * @retval None
 */
void HRPWM_TriggerPinSetFilterClock(uint32_t u32Pin, uint32_t u32Div)
{
    DDL_ASSERT(IS_VALID_TRIG_PIN(u32Pin));
    DDL_ASSERT(IS_VALID_FILTER_CLK(u32Div));

    switch (u32Pin) {
        case HRPWM_INPUT_TRIGA:
            MODIFY_REG32(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFICKTA, u32Div << HRPWM_COMMON_FCNTR_NOFICKTA_POS);
            break;
        case HRPWM_INPUT_TRIGB:
            MODIFY_REG32(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFICKTB, u32Div << HRPWM_COMMON_FCNTR_NOFICKTB_POS);
            break;
        case HRPWM_INPUT_TRIGC:
            MODIFY_REG32(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFICKTC, u32Div << HRPWM_COMMON_FCNTR_NOFICKTC_POS);
            break;
        case HRPWM_INPUT_TRIGD:
            MODIFY_REG32(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFICKTD, u32Div << HRPWM_COMMON_FCNTR_NOFICKTD_POS);
            break;
        default:
            break;
    }
}

/**
 * @brief  HRWPM trigger pin filter function enable
 * @param  [in] u32Pin              Pin to be configured
 *   @arg  HRPWM_INPUT_TRIGA
 *   @arg  HRPWM_INPUT_TRIGB
 *   @arg  HRPWM_INPUT_TRIGC
 *   @arg  HRPWM_INPUT_TRIGD
 * @retval None
 */
void HRPWM_TriggerPinFilterEnable(uint32_t u32Pin)
{
    DDL_ASSERT(IS_VALID_TRIG_PIN(u32Pin));
    switch (u32Pin) {
        case HRPWM_INPUT_TRIGA:
            SET_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTA);
            break;
        case HRPWM_INPUT_TRIGB:
            SET_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTB);
            break;
        case HRPWM_INPUT_TRIGC:
            SET_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTC);
            break;
        case HRPWM_INPUT_TRIGD:
            SET_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTD);
            break;
        default:
            break;
    }
}

/**
 * @brief  HRWPM trigger pin filter function disable
 * @param  [in] u32Pin              Pin to be configured
 *   @arg  HRPWM_INPUT_TRIGA
 *   @arg  HRPWM_INPUT_TRIGB
 *   @arg  HRPWM_INPUT_TRIGC
 *   @arg  HRPWM_INPUT_TRIGD
 * @retval None
 */
void HRPWM_TriggerPinFilterDisable(uint32_t u32Pin)
{
    DDL_ASSERT(IS_VALID_TRIG_PIN(u32Pin));
    switch (u32Pin) {
        case HRPWM_INPUT_TRIGA:
            CLR_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTA);
            break;
        case HRPWM_INPUT_TRIGB:
            CLR_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTB);
            break;
        case HRPWM_INPUT_TRIGC:
            CLR_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTC);
            break;
        case HRPWM_INPUT_TRIGD:
            CLR_REG32_BIT(CM_HRPWM_COMMON->FCNTR, HRPWM_COMMON_FCNTR_NOFIENTD);
            break;
        default:
            break;
    }
}

/**
 * @brief  Set the fields of structure stc_hrpwm_deadtime_config_t to default values
 * @param  [out] pstcDeadTimeConfig Pointer to a @ref stc_hrpwm_deadtime_config_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_DeadTimeStructInit(stc_hrpwm_deadtime_config_t *pstcDeadTimeConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcDeadTimeConfig) {
        pstcDeadTimeConfig->u32EqualUpDown = HRPWM_DEADTIME_EQUAL_OFF;
        pstcDeadTimeConfig->u32BufUp = HRPWM_DEADTIME_CNT_UP_BUF_OFF;
        pstcDeadTimeConfig->u32BufDown = HRPWM_DEADTIME_CNT_DOWN_BUF_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Dead time function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in]  pstcDeadTimeConfig Pointer to a @ref pstcDeadTimeConfig structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 * @note  Please notice that HRPWM_Buf_DeadtimeConfig() function should be excuted before call this function
 */
int32_t HRPWM_DeadTimeConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_deadtime_config_t *pstcDeadTimeConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcDeadTimeConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_DEADTIME_EQUAL_FUNC(pstcDeadTimeConfig->u32EqualUpDown));
        DDL_ASSERT(IS_VALID_DEADTIME_BUF_FUNC_DTUAR(pstcDeadTimeConfig->u32BufUp));
        DDL_ASSERT(IS_VALID_DEADTIME_BUF_FUNC_DTDAR(pstcDeadTimeConfig->u32BufDown));

        MODIFY_REG32(HRPWMx->DCONR, HRPWM_DCONR_SEPA | HRPWM_DCONR_DTBENU | HRPWM_DCONR_DTBEND,
                     pstcDeadTimeConfig->u32EqualUpDown | pstcDeadTimeConfig->u32BufUp | pstcDeadTimeConfig->u32BufDown);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_buf_config_t to default values
 * @param  [out] pstcBufConfig      Pointer to a @ref stc_hrpwm_buf_config_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_StructInit(stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBufConfig) {
        pstcBufConfig->u32BufTransCond = HRPWM_BUF_TRANS_INVD;
        pstcBufConfig->enBufTransU1Single = DISABLE;
        pstcBufConfig->enBufTransAfterU1Single = DISABLE;

        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM general compare register A and E buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_GeneralAEConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR1, HRPWM_BCONR1_BTRUAE | HRPWM_BCONR1_BTRDAE,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR1_BTRUAE_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PAE | HRPWM_BCONR2_BTRU0AE,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PAE_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0AE_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM general compare register B and F buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_GeneralBFConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR1, HRPWM_BCONR1_BTRUBF | HRPWM_BCONR1_BTRDBF,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR1_BTRUBF_POS);

        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PBF | HRPWM_BCONR2_BTRU0BF,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PBF_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0BF_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM period buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_PeriodConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR1, HRPWM_BCONR1_BTRUP | HRPWM_BCONR1_BTRDP,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR1_BTRUP_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PP | HRPWM_BCONR2_BTRU0P,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PP_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0P_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM special compare register A buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_SpecialAConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR1, HRPWM_BCONR1_BTRUSPA | HRPWM_BCONR1_BTRDSPA,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR1_BTRUSPA_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PSPA | HRPWM_BCONR2_BTRU0SPA,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PSPA_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0SPA_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM special compare register B buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_SpecialBConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR1, HRPWM_BCONR1_BTRUSPB | HRPWM_BCONR1_BTRDSPB,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR1_BTRUSPB_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PSPB | HRPWM_BCONR2_BTRU0SPB,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PSPB_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0SPB_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM external event filter window register buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_EVTWindowConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRUEEFWIN | HRPWM_BCONR2_BTRDEEFWIN,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR2_BTRUEEFWIN_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PEEFWIN | HRPWM_BCONR2_BTRU0EEFWIN,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PEEFWIN_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0EEFWIN_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM external event filter offset register buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_EVTOffsetConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRUEEFOFF | HRPWM_BCONR2_BTRDEEFOFF,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR2_BTRUEEFOFF_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PEEFOFF | HRPWM_BCONR2_BTRU0EEFOFF,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PEEFOFF_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0EEFOFF_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM control register buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_ControlRegConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRUCTL | HRPWM_BCONR2_BTRDCTL,
                     pstcBufConfig->u32BufTransCond << HRPWM_BCONR2_BTRUCTL_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PCTL | HRPWM_BCONR2_BTRU0CTL,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PCTL_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0CTL_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM dead time register buffer function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBufConfig       Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Buf_DeadtimeConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_buf_config_t *pstcBufConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBufConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_BUF_TRANS_COND1(pstcBufConfig->u32BufTransCond));
        if (CM_HRPWM1 != HRPWMx) {
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransU1Single));
            DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcBufConfig->enBufTransAfterU1Single));
        }
        /* Config normal buffer transfer condition */
        MODIFY_REG32(HRPWMx->DCONR, HRPWM_DCONR_DTBTRU | HRPWM_DCONR_DTBTRD,
                     pstcBufConfig->u32BufTransCond << HRPWM_DCONR_DTBTRU_POS);
        if (CM_HRPWM1 != HRPWMx) {
            /* Config HRPWM2~6 buffer single transfer function */
            MODIFY_REG32(HRPWMx->BCONR2, HRPWM_BCONR2_BTRU0PDT | HRPWM_BCONR2_BTRU0DT,
                         (uint32_t)pstcBufConfig->enBufTransU1Single << HRPWM_BCONR2_BTRU0PDT_POS |
                         (uint32_t)pstcBufConfig->enBufTransAfterU1Single << HRPWM_BCONR2_BTRU0DT_POS);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_idle_delay_Init_t to default values
 * @param  [out] pstcIdleDelayInit  Pointer to a @ref stc_hrpwm_idle_delay_Init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_Delay_StructInit(stc_hrpwm_idle_delay_Init_t *pstcIdleDelayInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcIdleDelayInit) {
        pstcIdleDelayInit->u32TriggerSrc = HRPWM_IDLE_DLY_TRIG_EVT6;
        pstcIdleDelayInit->u32PeriodPoint = HRPWM_CPLT_PERIOD_SAWTOOTH_PEAK_TRIANGLE_VALLEY;
        pstcIdleDelayInit->u32IdleChALevel = HRPWM_IDLE_CHA_LVL_LOW;
        pstcIdleDelayInit->u32IdleChBLevel = HRPWM_IDLE_CHB_LVL_LOW;
        pstcIdleDelayInit->u32IdleOutputCHAStatus = HRPWM_IDLE_OUTPUT_CHA_OFF;
        pstcIdleDelayInit->u32IdleOutputCHBStatus = HRPWM_IDLE_OUTPUT_CHB_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM initialize the idle delay output function
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcIdleDelayInit   Pointer to a @ref stc_hrpwm_idle_delay_Init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_Delay_Init(CM_HRPWM_TypeDef *HRPWMx, stc_hrpwm_idle_delay_Init_t *pstcIdleDelayInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
    /* Check structure pointer */
    if (NULL != pstcIdleDelayInit) {
        DDL_ASSERT(IS_VALID_IDLE_DLY_TRIG_SRC(pstcIdleDelayInit->u32TriggerSrc));
        DDL_ASSERT(IS_VALID_CPLT_PERIOD_POINT(pstcIdleDelayInit->u32PeriodPoint));
        DDL_ASSERT(IS_VALID_IDLE_CHA_LVL(pstcIdleDelayInit->u32IdleChALevel));
        DDL_ASSERT(IS_VALID_IDLE_CHB_LVL(pstcIdleDelayInit->u32IdleChBLevel));
        DDL_ASSERT(IS_VALID_IDLE_OUTPUT_CHA_STAT(pstcIdleDelayInit->u32IdleOutputCHAStatus));
        DDL_ASSERT(IS_VALID_IDLE_OUTPUT_CHB_STAT(pstcIdleDelayInit->u32IdleOutputCHBStatus));

        MODIFY_REG32(HRPWMx->IDLECR,
                     HRPWM_IDLECR_DLYEVSEL | HRPWM_IDLECR_IDLESA | HRPWM_IDLECR_IDLESB |
                     HRPWM_IDLECR_DLYCHA | HRPWM_IDLECR_DLYCHB,
                     pstcIdleDelayInit->u32TriggerSrc | pstcIdleDelayInit->u32IdleChALevel |
                     pstcIdleDelayInit->u32IdleChBLevel | pstcIdleDelayInit->u32IdleOutputCHAStatus |
                     pstcIdleDelayInit->u32IdleOutputCHBStatus);
        MODIFY_REG32(HRPWMx->GCONR1, HRPWM_GCONR1_PRDSEL, pstcIdleDelayInit->u32PeriodPoint);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_idle_BM_Init_t to default values
 * @param  [out] pstcBMInit       Pointer to a @ref stc_hrpwm_idle_BM_Init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_BM_StructInit(stc_hrpwm_idle_BM_Init_t *pstcBMInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBMInit) {
        pstcBMInit->u32Mode = HRPWM_BM_ACTION_MD1;
        pstcBMInit->u32CountSrc = HRPWM_BM_CNT_SRC_PEROID_POINT_U1;
        pstcBMInit->u32CountPclk0Div = HRPWM_BM_CNT_SRC_PCLK0_DIV1;
        pstcBMInit->u32PeriodValue = 0x00UL;
        pstcBMInit->u32CompareValue = 0x00UL;
        pstcBMInit->u32CountReload = HRPWM_BM_CNT_RELOAD_OFF;
        pstcBMInit->u64TriggerSrc = HRPWM_BM_TRIG_NONE;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Idle BM function configuration
 * @param  [in] pstcBMInit          Point a structure @ref stc_hrpwm_idle_BM_Init_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_BM_Init(stc_hrpwm_idle_BM_Init_t *pstcBMInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBMInit) {
        DDL_ASSERT(IS_VALID_BM_ACTION_MD(pstcBMInit->u32Mode));
        DDL_ASSERT(IS_VALID_BM_CNT_SRC(pstcBMInit->u32CountSrc));
        DDL_ASSERT(IS_VALID_BM_CNT_PCLK0_DIV(pstcBMInit->u32CountPclk0Div));
        DDL_ASSERT(IS_VALID_BM_CNT_REG_RANGE(pstcBMInit->u32PeriodValue));
        DDL_ASSERT(IS_VALID_BM_CNT_REG_RANGE(pstcBMInit->u32CompareValue));
        DDL_ASSERT(IS_VALID_BM_CNT_RELOAD(pstcBMInit->u32CountReload));
        DDL_ASSERT(IS_VALID_BM_TRIG_SRC(pstcBMInit->u64TriggerSrc));

        MODIFY_REG32(CM_HRPWM_COMMON->BMCR,
                     HRPWM_COMMON_BMCR_BMMD | HRPWM_COMMON_BMCR_BMCLKS | HRPWM_COMMON_BMCR_BMPSC | HRPWM_COMMON_BMCR_BMCTN,
                     pstcBMInit->u32Mode | pstcBMInit->u32CountSrc | pstcBMInit->u32CountPclk0Div |
                     pstcBMInit->u32CountReload);
        WRITE_REG32(CM_HRPWM_COMMON->BMPERAR, pstcBMInit->u32PeriodValue);
        WRITE_REG32(CM_HRPWM_COMMON->BMCMAR, pstcBMInit->u32CompareValue);
        WRITE_REG32(CM_HRPWM_COMMON->BMSTRG1, pstcBMInit->u64TriggerSrc & 0xFFFFFFFFUL);
        WRITE_REG32(CM_HRPWM_COMMON->BMSTRG2, (pstcBMInit->u64TriggerSrc >> 32U) & 0xFFFFFFFFUL);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_idle_BM_Output_Init_t to default values
 * @param  [out] pstcBMOutputInit   Pointer to a @ref stc_hrpwm_idle_BM_Output_Init_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_BM_OutputStructInit(stc_hrpwm_idle_BM_Output_Init_t *pstcBMOutputInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcBMOutputInit) {
        pstcBMOutputInit->u32IdleChALevel = HRPWM_IDLE_CHA_LVL_LOW;
        pstcBMOutputInit->u32IdleChBLevel = HRPWM_IDLE_CHB_LVL_LOW;
        pstcBMOutputInit->u32IdleOutputCHAStatus = HRPWM_BM_OUTPUT_CHA_OFF;
        pstcBMOutputInit->u32IdleOutputCHBStatus = HRPWM_BM_OUTPUT_CHB_OFF;
        pstcBMOutputInit->u32IdleChAEnterDelay = HRPWM_BM_DLY_ENTER_CHA_OFF;
        pstcBMOutputInit->u32IdleChBEnterDelay = HRPWM_BM_DLY_ENTER_CHB_OFF;
        pstcBMOutputInit->u32Follow = HRPWM_BM_FOLLOW_FUNC_OFF;
        pstcBMOutputInit->u32UnitCountReset = HRPWM_BM_UNIT_CNT_CONTINUE;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Idle BM output function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcBMOutputInit    Point a structure @ref stc_hrpwm_idle_BM_Output_Init_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Idle_BM_OutputInit(CM_HRPWM_TypeDef *HRPWMx, stc_hrpwm_idle_BM_Output_Init_t *pstcBMOutputInit)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    uint32_t u32UnitNum;
    /* Check structure pointer */
    if (NULL != pstcBMOutputInit) {
        DDL_ASSERT(IS_VALID_IDLE_CHA_LVL(pstcBMOutputInit->u32IdleChALevel));
        DDL_ASSERT(IS_VALID_IDLE_CHB_LVL(pstcBMOutputInit->u32IdleChBLevel));
        DDL_ASSERT(IS_VALID_BM_OUTPUT_CHA_STAT(pstcBMOutputInit->u32IdleOutputCHAStatus));
        DDL_ASSERT(IS_VALID_BM_OUTPUT_CHB_STAT(pstcBMOutputInit->u32IdleOutputCHBStatus));
        DDL_ASSERT(IS_VALID_BM_DLY_ENTER_CHA_STAT(pstcBMOutputInit->u32IdleChAEnterDelay));
        DDL_ASSERT(IS_VALID_BM_DLY_ENTER_CHB_STAT(pstcBMOutputInit->u32IdleChBEnterDelay));
        DDL_ASSERT(IS_VALID_BM_FOLLOW_FUNC(pstcBMOutputInit->u32Follow));
        DDL_ASSERT(IS_VALID_BM_UNIT_CNT_RST_FUNC(pstcBMOutputInit->u32UnitCountReset));

        MODIFY_REG32(HRPWMx->IDLECR,
                     HRPWM_IDLECR_IDLEBMA | HRPWM_IDLECR_IDLEBMB | HRPWM_IDLECR_IDLESA | HRPWM_IDLECR_IDLESB |
                     HRPWM_IDLECR_DIDLA | HRPWM_IDLECR_DIDLB | HRPWM_IDLECR_FOLLOW,
                     pstcBMOutputInit->u32IdleChALevel | pstcBMOutputInit->u32IdleChBLevel |
                     pstcBMOutputInit->u32IdleOutputCHAStatus | pstcBMOutputInit->u32IdleOutputCHBStatus |
                     pstcBMOutputInit->u32IdleChAEnterDelay | pstcBMOutputInit->u32IdleChBEnterDelay |
                     pstcBMOutputInit->u32Follow);
        u32UnitNum = ((uint32_t)&HRPWMx - CM_HRPWM1_BASE) / (CM_HRPWM2_BASE - CM_HRPWM1_BASE);
        if (HRPWM_BM_UNIT_CNT_CONTINUE == pstcBMOutputInit->u32UnitCountReset) {
            CLR_REG32_BIT(CM_HRPWM_COMMON->BMCR, HRPWM_COMMON_BMCR_BMTMR1 << u32UnitNum);
        } else {
            SET_REG32_BIT(CM_HRPWM_COMMON->BMCR, HRPWM_COMMON_BMCR_BMTMR1 << u32UnitNum);
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_valid_period_config_t to default values
 * @param  [out] pstcValidperiodConfig  Pointer to a @ref stc_hrpwm_valid_period_config_t structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_ValidPeriod_StructInit(stc_hrpwm_valid_period_config_t *pstcValidperiodConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcValidperiodConfig) {
        pstcValidperiodConfig->u32CountCond = HRPWM_VALID_PERIOD_INVD;
        pstcValidperiodConfig->u32Interval = 0x00UL;
        pstcValidperiodConfig->u32SpecialA = HRPWM_VALID_PERIOD_SPECIAL_A_OFF;
        pstcValidperiodConfig->u32SpecialB = HRPWM_VALID_PERIOD_SPECIAL_B_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM valid period function configuration for special compare function
 * @param  [in] HRPWMx                  HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcValidperiodConfig   Pointer of configuration structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_ValidPeriod_Config(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_valid_period_config_t *pstcValidperiodConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcValidperiodConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_PERIOD_CNT_COND(pstcValidperiodConfig->u32CountCond));
        DDL_ASSERT(IS_VALID_PERIOD_INTERVAL(pstcValidperiodConfig->u32Interval));
        DDL_ASSERT(IS_VALID_PERIOD_SPECIAL_A_FUNC(pstcValidperiodConfig->u32SpecialA));
        DDL_ASSERT(IS_VALID_PERIOD_SPECIAL_B_FUNC(pstcValidperiodConfig->u32SpecialB));

        MODIFY_REG32(HRPWMx->VPERR, HRPWM_VPERR_PCNTS | HRPWM_VPERR_PCNTE | HRPWM_VPERR_SPPERIA | HRPWM_VPERR_SPPERIB,
                     pstcValidperiodConfig->u32CountCond |
                     pstcValidperiodConfig->u32Interval << HRPWM_VPERR_PCNTS_POS |
                     pstcValidperiodConfig->u32SpecialA |
                     pstcValidperiodConfig->u32SpecialB);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_evt_config_t to default values
 * @param  [out] pstcEventConfig        Pointer to a @ref stc_hrpwm_evt_config_t structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_EVT_StructInit(stc_hrpwm_evt_config_t *pstcEventConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcEventConfig) {
        pstcEventConfig->u32EventSrc = HRPWM_EVT_SRC1;
        pstcEventConfig->u32EventSrc2 = HRPWM_EVT_SRC2_CMP1;
        pstcEventConfig->u32ValidLevel = HRPWM_EVT_VALID_LVL_HIGH;
        pstcEventConfig->u32ValidAction = HRPWM_EVT_VALID_LVL;
        pstcEventConfig->u32FastAsyncMode = HRPWM_EVT_FAST_ASYNC_OFF;
        pstcEventConfig->u32FilterClock = HRPWM_EVT_FILTER_NONE;
        pstcEventConfig->u32ExtEvtDetect = HRPWM_EVT_GEN_DETECT_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  External event configuration
 * @param  [in] u32EventNum         External event number @ref HRPWM_EVT_Num_Define
 * @param  [in] pstcEventConfig     Point External event source config pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EVT_Config(uint32_t u32EventNum, const stc_hrpwm_evt_config_t *pstcEventConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcEventConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_EVT_NUM(u32EventNum));
        DDL_ASSERT(IS_VALID_EVT_SRC(pstcEventConfig->u32EventSrc));
        DDL_ASSERT(IS_VALID_EVT_VALID_ACTION(pstcEventConfig->u32ValidAction));
        DDL_ASSERT(IS_VALID_EVT_VALID_LVL(pstcEventConfig->u32ValidLevel));
        if (HRPWM_EVT_SRC2 == pstcEventConfig->u32EventSrc) {
            DDL_ASSERT(IS_VALID_EVT_SRC2(pstcEventConfig->u32EventSrc2));
        }
        if (u32EventNum <= HRPWM_EVT5) {
            DDL_ASSERT(IS_VALID_EVT_FAST_ASYNC_MD(pstcEventConfig->u32FastAsyncMode));
        } else {
            DDL_ASSERT(HRPWM_EVT_FAST_ASYNC_OFF == pstcEventConfig->u32FastAsyncMode);
            DDL_ASSERT(IS_VALID_EVT_FILTER_CLK(pstcEventConfig->u32FilterClock));
        }
        DDL_ASSERT(IS_VALID_EVT_GEN_DETECT_EVT_FUNC(pstcEventConfig->u32ExtEvtDetect));

        if (u32EventNum <= HRPWM_EVT5) {
            /* Event1 ~ Event5 */
            MODIFY_REG32(CM_HRPWM_COMMON->EECR1,
                         HRPWM_EXTEVT1_CONFIG_MASK << (HRPWM_COMMON_EECR1_EE2SRC_POS * u32EventNum),
                         (pstcEventConfig->u32EventSrc | pstcEventConfig->u32ValidLevel |
                          pstcEventConfig->u32ValidAction | pstcEventConfig->u32FastAsyncMode) << (HRPWM_COMMON_EECR1_EE2SRC_POS * u32EventNum));
        } else {
            /* Event6 ~ Event10 */
            MODIFY_REG32(CM_HRPWM_COMMON->EECR2,
                         HRPWM_EXTEVT6_CONFIG_MASK << (HRPWM_COMMON_EECR2_EE7SRC_POS * (u32EventNum - HRPWM_EVT6)),
                         (pstcEventConfig->u32EventSrc | pstcEventConfig->u32ValidLevel |
                          pstcEventConfig->u32ValidAction) << (HRPWM_COMMON_EECR2_EE7SRC_POS * (u32EventNum - HRPWM_EVT6)));
            MODIFY_REG32(CM_HRPWM_COMMON->EECR3, HRPWM_COMMON_EECR3_EE6F << (HRPWM_COMMON_EECR3_EE7F_POS * (u32EventNum - HRPWM_EVT6)),
                         pstcEventConfig->u32FilterClock << (HRPWM_COMMON_EECR3_EE7F_POS * (u32EventNum - HRPWM_EVT6)));
        }
        if (HRPWM_EVT_SRC2 == pstcEventConfig->u32EventSrc) {
            if (u32EventNum <= HRPWM_EVT2) {
                MODIFY_REG32(CM_HRPWM_COMMON->GCTLR, HRPWM_COMMON_GCTLR_EE1SRC2 << (HRPWM_COMMON_GCTLR_EE2SRC2_POS * u32EventNum),
                             pstcEventConfig->u32EventSrc2 << (HRPWM_COMMON_GCTLR_EE2SRC2_POS * u32EventNum));
            } else {
                MODIFY_REG32(CM_HRPWM_COMMON->GCTLR, HRPWM_COMMON_GCTLR_EE1SRC2 << (HRPWM_COMMON_GCTLR_EE3SRC2_POS + HRPWM_COMMON_GCTLR_EE2SRC2_POS * (u32EventNum - HRPWM_EVT3)),
                             pstcEventConfig->u32EventSrc2 << (HRPWM_COMMON_GCTLR_EE3SRC2_POS + HRPWM_COMMON_GCTLR_EE2SRC2_POS * (u32EventNum - HRPWM_EVT3)));
            }
        }
        MODIFY_REG32(CM_HRPWM_COMMON->EEDSELR, HRPWM_EVT_GEN_DETECT_ON << u32EventNum, pstcEventConfig->u32ExtEvtDetect << u32EventNum);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_evt_filter_config_t to default values
 * @param  [out] pstcFilterConfig       Pointer to a @ref stc_hrpwm_evt_filter_config_t structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_EVT_FilterStructInit(stc_hrpwm_evt_filter_config_t *pstcFilterConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcFilterConfig) {
        pstcFilterConfig->u32Mode = HRPWM_EVT_FILTER_OFF;
        pstcFilterConfig->u32Latch = HRPWM_EVT_FILTER_LATCH_OFF;
        pstcFilterConfig->u32WindowTimeOut = HRPWM_EVT_FILTER_TIMEOUT_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  External event filter configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] u32EventNum         External event number @ref HRPWM_EVT_Num_Define
 * @param  [in] pstcFilterConfig    Point External filter source config pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EVT_FilterConfig(CM_HRPWM_TypeDef *HRPWMx, uint32_t u32EventNum, const stc_hrpwm_evt_filter_config_t *pstcFilterConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcFilterConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EVT_NUM(u32EventNum));
        DDL_ASSERT(IS_VALID_EVT_FILTER_MD(pstcFilterConfig->u32Mode));
        DDL_ASSERT(IS_VALID_EVT_LATCH_FUNC(pstcFilterConfig->u32Latch));
        DDL_ASSERT(IS_VALID_EVT_TIMEOUT_FUNC(pstcFilterConfig->u32WindowTimeOut));

        if (u32EventNum <= HRPWM_EVT5) {
            /* Event1 ~ Event5 */
            MODIFY_REG32(HRPWMx->EEFLTCR1,
                         HRPWM_EXTEVT1_FILTER_CONFIG_MASK << (HRPWM_EEFLTCR1_EE2LAT_POS * u32EventNum),
                         (pstcFilterConfig->u32Mode | pstcFilterConfig->u32Latch | pstcFilterConfig->u32WindowTimeOut)
                         << (HRPWM_EEFLTCR1_EE2LAT_POS * u32EventNum));
        } else {
            /* Event6 ~ Event10 */
            MODIFY_REG32(HRPWMx->EEFLTCR2,
                         HRPWM_EXTEVT1_FILTER_CONFIG_MASK << (HRPWM_EEFLTCR1_EE2LAT_POS * (u32EventNum - HRPWM_EVT6)),
                         (pstcFilterConfig->u32Mode | pstcFilterConfig->u32Latch | pstcFilterConfig->u32WindowTimeOut)
                         << (HRPWM_EEFLTCR1_EE2LAT_POS * (u32EventNum - HRPWM_EVT6)));
        }
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_evt_filter_signal_config_t to default values
 * @param  [out] pstcSignalConfig   Pointer to a @ref stc_hrpwm_evt_filter_signal_config_t structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_EVT_FilterSignalStructInit(stc_hrpwm_evt_filter_signal_config_t *pstcSignalConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcSignalConfig) {
        pstcSignalConfig->u32InitPolarity = HRPWM_EVT_FILTER_INIT_POLARITY_LOW;
        pstcSignalConfig->u32Offset = 0x00UL;
        pstcSignalConfig->u32OffsetDir = HRPWM_EVT_FILTER_OFS_DIR_DOWN;
        pstcSignalConfig->u32Window = 0x00UL;
        pstcSignalConfig->u32WindowDir = HRPWM_EVT_FILTER_WIN_DIR_DOWN;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM external event filter signal configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcSignalConfig    Point External filter source config pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EVT_FilterSignalConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_evt_filter_signal_config_t *pstcSignalConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcSignalConfig) {
        /* Check parameters */
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EVT_FILTER_INIT_POLARITY(pstcSignalConfig->u32InitPolarity));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE(pstcSignalConfig->u32Offset));
        DDL_ASSERT(IS_VALID_EVT_FILTER_OFS_DIR(pstcSignalConfig->u32OffsetDir));
        DDL_ASSERT(IS_VALID_DATA_REG_RANGE(pstcSignalConfig->u32Window));
        DDL_ASSERT(IS_VALID_EVT_FILTER_WIN_DIR(pstcSignalConfig->u32WindowDir));

        MODIFY_REG32(HRPWMx->EEFLTCR1, HRPWM_EEFLTCR1_EEINTPOL, pstcSignalConfig->u32InitPolarity);
        WRITE_REG32(HRPWMx->EEFOFFSETAR, pstcSignalConfig->u32Offset | pstcSignalConfig->u32OffsetDir);
        WRITE_REG32(HRPWMx->EEFWINAR, pstcSignalConfig->u32Window | pstcSignalConfig->u32WindowDir);
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_syn_output_config_t to default values
 * @param  [out] pstcSignalConfig   Pointer to a @ref stc_hrpwm_syn_output_config_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Sync_StructInit(stc_hrpwm_syn_output_config_t *pstcSyncConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcSyncConfig) {
        pstcSyncConfig->u32Src = HRPWM_SYNC_SRC_U1_CNT_VALLEY;
        pstcSyncConfig->u32MatchBDir = HRPWM_SYNC_MATCH_B_DIR_DOWN;
        pstcSyncConfig->u32Pulse = HRPWM_SYNC_PULSE_OFF;
        pstcSyncConfig->u32PulseWidth = 0x01UL;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM synchronous output configure
 * @param  [in] pstcSyncConfig      Point a structure @ref stc_hrpwm_syn_output_config_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_Sync_Config(stc_hrpwm_syn_output_config_t *pstcSyncConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcSyncConfig) {
        DDL_ASSERT(IS_VALID_SYNC_SRC(pstcSyncConfig->u32Src));
        DDL_ASSERT(IS_VALID_SYNC_MATCH_B_DIR(pstcSyncConfig->u32MatchBDir));
        DDL_ASSERT(IS_VALID_SYNC_PULSE(pstcSyncConfig->u32Pulse));
        DDL_ASSERT(IS_VALID_SYNC_PULSE_WIDTH(pstcSyncConfig->u32PulseWidth));
        WRITE_REG32(CM_HRPWM_COMMON->SYNOCR, pstcSyncConfig->u32Src | pstcSyncConfig->u32MatchBDir |
                    pstcSyncConfig->u32Pulse | pstcSyncConfig->u32PulseWidth << HRPWM_COMMON_SYNOCR_SYNCMP_POS);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_dac_trigger_config_t to default values
 * @param  [out] pstcDACTriggerConfig   Pointer to a @ref stc_hrpwm_dac_trigger_config_t structure
 * @retval int32_t:
 *         - LL_OK:                     Successfully done
 *         - LL_ERR_INVD_PARAM:         Parameter error
 */
int32_t HRPWM_DAC_TriggerStructInit(stc_hrpwm_dac_trigger_config_t *pstcDACTriggerConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;

    /* Check structure pointer */
    if (NULL != pstcDACTriggerConfig) {
        pstcDACTriggerConfig->u32Src = HRPWM_DAC_TRIG_SRC_CNT_VALLEY;
        pstcDACTriggerConfig->u32DACCh1Dest = HRPWM_DAC_CH1_TRIG_DEST_NONE;
        pstcDACTriggerConfig->u32DACCh2Dest = HRPWM_DAC_CH2_TRIG_DEST_NONE;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM DAC synchronous trigger configure
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcSyncConfig      Point a structure @ref stc_hrpwm_dac_trigger_config_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_DAC_TriggerConfig(CM_HRPWM_TypeDef *HRPWMx, stc_hrpwm_dac_trigger_config_t *pstcDACTriggerConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcDACTriggerConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_HRPWM_DAC_TRIG_SRC(pstcDACTriggerConfig->u32Src));
        DDL_ASSERT(IS_VALID_HRPWM_DAC_CH1_TRIG_DEST(pstcDACTriggerConfig->u32DACCh1Dest));
        DDL_ASSERT(IS_VALID_HRPWM_DAC_CH2_TRIG_DEST(pstcDACTriggerConfig->u32DACCh2Dest));

        MODIFY_REG32(HRPWMx->CR, HRPWM_CR_DACSRC | HRPWM_CR_DACSYNC1 | HRPWM_CR_DACSYNC2,
                     pstcDACTriggerConfig->u32Src |
                     pstcDACTriggerConfig->u32DACCh1Dest |
                     pstcDACTriggerConfig->u32DACCh2Dest);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_ph_config_t to default values
 * @param  [out] pstcPHConfig       Pointer to a @ref stc_hrpwm_ph_config_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PH_StructInit(stc_hrpwm_ph_config_t *pstcPHConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcPHConfig) {
        pstcPHConfig->u32PhaseIndex = HRPWM_PH_MATCH_IDX1;
        pstcPHConfig->u32ForceChA = HRPWM_PH_MATCH_FORCE_CHA_OFF;
        pstcPHConfig->u32ForceChB = HRPWM_PH_MATCH_FORCE_CHB_OFF;
        pstcPHConfig->u32PeriodLink = HRPWM_PH_PERIOD_LINK_OFF;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM Phase configuration for specified HRPWM unit
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWM2 ~ CM_HRPWM6
 * @param  [in] pstcPHConfig     Point a structure @ref stc_hrpwm_ph_config_t
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_PH_Config(CM_HRPWM_TypeDef *HRPWMx, stc_hrpwm_ph_config_t *pstcPHConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcPHConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_PH_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_PH_MATCH_IDX(pstcPHConfig->u32PhaseIndex));
        DDL_ASSERT(IS_VALID_PH_MATCH_FORCE_CHA_FUNC(pstcPHConfig->u32ForceChA));
        DDL_ASSERT(IS_VALID_PH_MATCH_FORCE_CHB_FUNC(pstcPHConfig->u32ForceChB));
        DDL_ASSERT(IS_VALID_PH_PERIOD_LINK(pstcPHConfig->u32PeriodLink));

        WRITE_REG32(HRPWMx->PHSCTL,
                    (pstcPHConfig->u32PhaseIndex << HRPWM_PHSCTL_PHCMPSEL_POS) |
                    pstcPHConfig->u32ForceChA | pstcPHConfig->u32ForceChB);
        MODIFY_REG32(HRPWMx->GCONR1, HRPWM_GCONR1_PRDLK, pstcPHConfig->u32PeriodLink);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  Set the fields of structure stc_hrpwm_emb_config_t to default values
 * @param  [out] pstcEmbConfig      Pointer to a @ref stc_hrpwm_emb_config_t structure
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EMB_StructInit(stc_hrpwm_emb_config_t *pstcEmbConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    /* Check structure pointer */
    if (NULL != pstcEmbConfig) {
        pstcEmbConfig->u32ValidCh = HRPWM_EMB_EVT_CH0;
        pstcEmbConfig->u32ReleaseMode = HRPWM_EMB_RELEASE_IMMED;
        pstcEmbConfig->u32PinStatus = HRPWM_EMB_PIN_NORMAL;
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM channel A EMB function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcEmbConfig       Point EMB function Config Pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EMB_ChAConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_emb_config_t *pstcEmbConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcEmbConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EMB_CH(pstcEmbConfig->u32ValidCh));
        DDL_ASSERT(IS_VALID_EMB_RELEASE_MD(pstcEmbConfig->u32ReleaseMode));
        DDL_ASSERT(IS_VALID_EMB_VALID_PIN_STAT(pstcEmbConfig->u32PinStatus));

        MODIFY_REG32(HRPWMx->PCNAR1, HRPWM_PCNAR1_EMBSA | HRPWM_PCNAR1_EMBRA | HRPWM_PCNAR1_EMBCA,
                     pstcEmbConfig->u32ValidCh | pstcEmbConfig->u32ReleaseMode | pstcEmbConfig->u32PinStatus);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM channel A EMB function configuration (buffer)
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcEmbConfig       Point EMB function Config Pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EMB_ChAConfig_Buf(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_emb_config_t *pstcEmbConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcEmbConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EMB_CH(pstcEmbConfig->u32ValidCh));
        DDL_ASSERT(IS_VALID_EMB_RELEASE_MD(pstcEmbConfig->u32ReleaseMode));
        DDL_ASSERT(IS_VALID_EMB_VALID_PIN_STAT(pstcEmbConfig->u32PinStatus));

        MODIFY_REG32(HRPWMx->BPCNAR1, HRPWM_PCNAR1_EMBSA | HRPWM_PCNAR1_EMBRA | HRPWM_PCNAR1_EMBCA,
                     pstcEmbConfig->u32ValidCh | pstcEmbConfig->u32ReleaseMode | pstcEmbConfig->u32PinStatus);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM channel B EMB function configuration
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcEmbConfig       Point EMB function Config Pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EMB_ChBConfig(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_emb_config_t *pstcEmbConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcEmbConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EMB_CH(pstcEmbConfig->u32ValidCh));
        DDL_ASSERT(IS_VALID_EMB_RELEASE_MD(pstcEmbConfig->u32ReleaseMode));
        DDL_ASSERT(IS_VALID_EMB_VALID_PIN_STAT(pstcEmbConfig->u32PinStatus));

        MODIFY_REG32(HRPWMx->PCNBR1, HRPWM_PCNBR1_EMBSB | HRPWM_PCNBR1_EMBRB | HRPWM_PCNBR1_EMBCB,
                     pstcEmbConfig->u32ValidCh | pstcEmbConfig->u32ReleaseMode | pstcEmbConfig->u32PinStatus);
        i32Ret = LL_OK;
    }
    return i32Ret;
}

/**
 * @brief  HRPWM channel B EMB function configuration (buffer)
 * @param  [in] HRPWMx              HRPWM unit
 *  @arg CM_HRPWMx
 * @param  [in] pstcEmbConfig       Point EMB function Config Pointer
 * @retval int32_t:
 *         - LL_OK:                 Successfully done
 *         - LL_ERR_INVD_PARAM:     Parameter error
 */
int32_t HRPWM_EMB_ChBConfig_Buf(CM_HRPWM_TypeDef *HRPWMx, const stc_hrpwm_emb_config_t *pstcEmbConfig)
{
    int32_t i32Ret = LL_ERR_INVD_PARAM;
    if (NULL != pstcEmbConfig) {
        DDL_ASSERT(IS_VALID_HRPWM_UNIT(HRPWMx));
        DDL_ASSERT(IS_VALID_EMB_CH(pstcEmbConfig->u32ValidCh));
        DDL_ASSERT(IS_VALID_EMB_RELEASE_MD(pstcEmbConfig->u32ReleaseMode));
        DDL_ASSERT(IS_VALID_EMB_VALID_PIN_STAT(pstcEmbConfig->u32PinStatus));

        MODIFY_REG32(HRPWMx->BPCNBR1, HRPWM_PCNBR1_EMBSB | HRPWM_PCNBR1_EMBRB | HRPWM_PCNBR1_EMBCB,
                     pstcEmbConfig->u32ValidCh | pstcEmbConfig->u32ReleaseMode | pstcEmbConfig->u32PinStatus);
        i32Ret = LL_OK;
    }
    return i32Ret;
}
#endif

/**
 * @}
 */

#endif /* LL_HRPWM_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
