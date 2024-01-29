/**
 *******************************************************************************
 * @file  hc32f160_ll_interrupts_share.c
 * @brief This file provides firmware functions to manage the Share Interrupt
 *        Controller (SHARE_INTERRUPTS).
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
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
#include "hc32f160_ll_interrupts_share.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @defgroup LL_HC32F160_SHARE_INTERRUPTS SHARE_INTERRUPTS
 * @brief Share Interrupts Driver Library
 * @{
 */

#if (LL_INTERRUPTS_SHARE_ENABLE == DDL_ON)

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup Share_Interrupts_Local_Macros Share Interrupts Local Macros
 * @{
 */
/**
 * @defgroup INTC_Register_Protect_Key INTC Registers Protect Key
 * @{
 */
#ifndef INTC_REG_UNLOCK_KEY
#define INTC_REG_UNLOCK_KEY          (0xA5U)
#endif
#ifndef INTC_REG_LOCK_KEY
#define INTC_REG_LOCK_KEY            (0x00U)
#endif

/**
 * @}
 */

/**
 * @defgroup Share_Interrupts_Check_Parameters_Validity Share Interrupts Check Parameters Validity
 * @{
 */
/* Parameter validity check for INTC register lock status. */
#define IS_INTC_S_UNLOCK()            (INTC_REG_UNLOCK_KEY == (CM_INTC->FPRCR))

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
 * @defgroup Share_Interrupts_Global_Functions Share Interrupts Global Functions
 * @{
 */
/**
* @brief  Share IRQ configure
* @param  [in] enIntSrc: Peripheral interrupt source @ref en_int_src_t
* @param  [in] enNewState: An @ref en_functional_state_t enumeration value.
* @retval int32_t:
*           - LL_OK: Share IRQ configure successfully
*           - LL_ERR_INVD_PARAM: EXTINT00~07 cannot be configured into share IRQ handler
*/
int32_t INTC_ShareIrqCmd(en_int_src_t enIntSrc, en_functional_state_t enNewState)
{
    __IO uint32_t *ISELRx;
    int32_t i32Ret = LL_OK;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    /* EXTINT0~7 cannot be configured into share IRQ */
    if (0U == (uint32_t)enIntSrc % 0x10U) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        DDL_ASSERT(IS_INTC_S_UNLOCK());

        ISELRx = (__IO uint32_t *)(((uint32_t)&CM_INTC->ISELBR24) + (4U * ((uint32_t)enIntSrc / 0x10U)));
        if (ENABLE == enNewState) {
            SET_REG32_BIT(*ISELRx, (1UL << ((uint32_t)enIntSrc % 0x10UL)));
        } else {
            CLR_REG32_BIT(*ISELRx, (1UL << ((uint32_t)enIntSrc % 0x10UL)));
        }
    }
    return i32Ret;
}

/**
 * @brief  Interrupt No.024 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ024_Handler(void)
{
    uint32_t u32Tmp1;
    uint32_t u32Tmp2;
    /* External interrupt 08 */
    u32Tmp1 = bCM_INTC->EIRQFR_b.EIRQF8;
    if ((1UL == bCM_INTC->ISELBR24_b.ISEL1) && (0UL != u32Tmp1)) {
        EXTINT08_IrqHandler();
    }
    /* DMA Ch.0 transfer complete */
    if (0UL == bCM_DMA->INTMASK1_b.MSKTC0) {
        u32Tmp1 = bCM_DMA->INTSTAT1_b.TC0;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL2) && (0UL != u32Tmp1)) {
            DMA_TC0_IrqHandler();
        }
    }
    /* DMA Ch.0 block transfer complete */
    if (0UL == bCM_DMA->INTMASK1_b.MSKBTC0) {
        u32Tmp1 = bCM_DMA->INTSTAT1_b.BTC0;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL3) && (0UL != u32Tmp1)) {
            DMA_BTC0_IrqHandler();
        }
    }
    /* EFM program/erase error */
    if (1UL == bCM_EFM->FITE_b.PEERRITE) {
        u32Tmp1 = CM_EFM->FSR & (EFM_FSR_PEWERR | EFM_FSR_PEPRTERR | EFM_FSR_PGMISMTCH);
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL4) && (0UL != u32Tmp1)) {
            EFM_ProgramEraseError_IrqHandler();
        }
    }
    /* EFM read collision */
    if (1UL == bCM_EFM->FITE_b.COLERRITE) {
        u32Tmp1 = bCM_EFM->FSR_b.COLERR;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL5) && (0UL != u32Tmp1)) {
            EFM_ColError_IrqHandler();
        }
    }
    /* XTAL stop */
    if (1UL == bCM_CMU->XTALSTDCR_b.XTALSTDIE) {
        u32Tmp1 = bCM_CMU->XTALSTDSR_b.XTALSTDF;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL6) && (0UL != u32Tmp1)) {
            CLK_XtalStop_IrqHandler();
        }
    }
    /* SWDT underflow or fresh error */
    u32Tmp1 = CM_SWDT->SR & (SWDT_SR_UDF | SWDT_SR_REF);
    if ((1UL == bCM_INTC->ISELBR24_b.ISEL7) && (0UL != u32Tmp1)) {
        SWDT_IrqHandler();
    }
    /* TimerB7 overflow */
    if (1UL == bCM_TMRB_7->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_7->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_7_Ovf_IrqHandler();
        }
    }
    /* TimerB7 underflow */
    if (1UL == bCM_TMRB_7->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_7->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_7_Udf_IrqHandler();
        }
    }
    /* TimerB8 compare match */
    if (1UL == bCM_TMRB_8->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_8->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL10) && (1UL == u32Tmp1)) {
            TMRB_8_Cmp_IrqHandler();
        }
    }
    /* USART5 Rx ORE/FE/PE error */
    if (1UL == bCM_USART5->CR1_b.RIE) {
        u32Tmp1 = CM_USART5->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL11) && (0UL != u32Tmp1)) {
            USART5_RxError_IrqHandler();
        }
    }
    /* USART1 Rx ORE/FE/PE error */
    if (1UL == bCM_USART1->CR1_b.RIE) {
        u32Tmp1 = CM_USART1->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL12) && (0UL != u32Tmp1)) {
            USART1_RxError_IrqHandler();
        }
    }
    /* I2c error and event */
    u32Tmp1 = CM_I2C->SR & (I2C_SR_STARTF   | I2C_SR_SLADDR0F       |           \
                            I2C_SR_SLADDR1F | I2C_SR_STOPF          |           \
                            I2C_SR_ARLOF    | I2C_SR_NACKF          |           \
                            I2C_SR_GENCALLF | I2C_SR_SMBDEFAULTF    |           \
                            I2C_SR_SMBHOSTF | I2C_SR_SMBALRTF);

    u32Tmp2 = CM_I2C->CR2 & (I2C_CR2_STARTIE  | I2C_CR2_SLADDR0IE    |           \
                             I2C_CR2_SLADDR1IE | I2C_CR2_STOPIE       |           \
                             I2C_CR2_ARLOIE   | I2C_CR2_NACKIE       |           \
                             I2C_CR2_GENCALLIE | I2C_CR2_SMBDEFAULTIE |           \
                             I2C_CR2_SMBHOSTIE | I2C_CR2_SMBALRTIE);
    if ((1UL == bCM_INTC->ISELBR24_b.ISEL13) && (0UL != (u32Tmp1 & u32Tmp2))) {
        I2C_EE_IrqHandler();
    }
    /* SPI parity/overflow/underflow/mode error */
    if (1UL == bCM_SPI->CR1_b.EIE) {
        u32Tmp1 = CM_SPI->SR & (SPI_SR_UDRERF | SPI_SR_PERF | SPI_SR_MODFERF | SPI_SR_OVRERF);
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL14) && (0UL != u32Tmp1)) {
            SPI_Error_IrqHandler();
        }
    }
    /* USART4 Rx ORE/FE/PE error */
    if (1UL == bCM_USART4->CR1_b.RIE) {
        u32Tmp1 = CM_USART4->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR24_b.ISEL15) && (0UL != u32Tmp1)) {
            USART4_RxError_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.025 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ025_Handler(void)
{
    uint32_t u32Tmp1;
    /* External interrupt 09 */
    u32Tmp1 = bCM_INTC->EIRQFR_b.EIRQF9;
    if ((1UL == bCM_INTC->ISELBR25_b.ISEL1) && (0UL != u32Tmp1)) {
        EXTINT09_IrqHandler();
    }
    /* DMA Ch.1 transfer complete */
    if (0UL == bCM_DMA->INTMASK1_b.MSKTC1) {
        u32Tmp1 = bCM_DMA->INTSTAT1_b.TC1;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL2) && (0UL != u32Tmp1)) {
            DMA_TC1_IrqHandler();
        }
    }
    /* DMA Ch.1 block transfer complete */
    if (0UL == bCM_DMA->INTMASK1_b.MSKBTC1) {
        u32Tmp1 = bCM_DMA->INTSTAT1_b.BTC1;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL3) && (0UL != u32Tmp1)) {
            DMA_BTC1_IrqHandler();
        }
    }
    /* Clock trimming error */
    if (1UL == bCM_CTC->CR1_b.ERRIE) {
        u32Tmp1 = CM_CTC->STR & (CTC_STR_TRMOVF | CTC_STR_TRMUDF);
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL4) && (0UL != u32Tmp1)) {
            CTC_IrqHandler();
        }
    }
    /* TimerB8 overflow */
    if (1UL == bCM_TMRB_8->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_8->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_8_Ovf_IrqHandler();
        }
    }
    /* TimerB8 underflow */
    if (1UL == bCM_TMRB_8->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_8->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_8_Udf_IrqHandler();
        }
    }
    /* TimerB7 compare match */
    if (1UL == bCM_TMRB_7->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_7->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_7_Cmp_IrqHandler();
        }
    }
    /* USART6 Rx ORE/FE/PE error */
    if (1UL == bCM_USART6->CR1_b.RIE) {
        u32Tmp1 = CM_USART6->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL11) && (0UL != u32Tmp1)) {
            USART6_RxError_IrqHandler();
        }
    }
    /* USART1 Rx end */
    if (1UL == bCM_USART1->CR1_b.RIE) {
        u32Tmp1 = bCM_USART1->SR_b.RXNE;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL12) && (0UL != u32Tmp1)) {
            USART1_RxFull_IrqHandler();
        }
    }
    /* USART4 Rx end */
    if (1UL == bCM_USART4->CR1_b.RIE) {
        u32Tmp1 = bCM_USART4->SR_b.RXNE;
        if ((1UL == bCM_INTC->ISELBR25_b.ISEL15) && (0UL != u32Tmp1)) {
            USART4_RxFull_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.026 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ026_Handler(void)
{
    uint32_t u32Tmp1;
    uint32_t u32Tmp2;
    /* DMA request or transfer error */
    u32Tmp1 = CM_DMA->INTSTAT0 & (DMA_INTSTAT0_TRNERR | DMA_INTSTAT0_REQERR);
    u32Tmp2 = (uint32_t)(~(CM_DMA->INTMASK0) & (DMA_INTMASK0_MSKTRNERR | DMA_INTMASK0_MSKREQERR));
    if ((1UL == bCM_INTC->ISELBR26_b.ISEL1) && (0UL != (u32Tmp1 & u32Tmp2))) {
        DMA_Error_IrqHandler();
    }
    /* TimerB5 overflow */
    if (1UL == bCM_TMRB_5->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_5->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_5_Ovf_IrqHandler();
        }
    }
    /* TimerB5 underflow */
    if (1UL == bCM_TMRB_5->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_5->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_5_Udf_IrqHandler();
        }
    }
    /* TimerB6 compare match */
    if (1UL == bCM_TMRB_6->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_6->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_6_Cmp_IrqHandler();
        }
    }
    /* USART5 Rx end */
    if (1UL == bCM_USART5->CR1_b.RIE) {
        u32Tmp1 = bCM_USART5->SR_b.RXNE;
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL11) && (0UL != u32Tmp1)) {
            USART5_RxFull_IrqHandler();
        }
    }
    /* USART1 Tx buffer empty */
    if (1UL == bCM_USART1->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART1->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL12) && (0UL != u32Tmp1)) {
            USART1_TxEmpty_IrqHandler();
        }
    }
    /* USART3 Rx ORE/FE/PE error */
    if (1UL == bCM_USART3->CR1_b.RIE) {
        u32Tmp1 = CM_USART3->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR26_b.ISEL13) && (0UL != u32Tmp1)) {
            USART3_RxError_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.027 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ027_Handler(void)
{
    uint32_t u32Tmp1;
    /* EKEY and other Interrupt source are exclusive */
    if (1UL == bCM_INTC->ISELBR27_b.ISEL1) {
        EKEY_IrqHandler();
    } else {
        /* Timer 0 compare match */
        if (1UL == bCM_TMR0->BCONR_b.INTENA) {
            u32Tmp1 = bCM_TMR0->STFLR_b.CMFA;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL2) && (0UL != u32Tmp1)) {
                TMR0_CmpA_IrqHandler();
            }
        }
        /* USART6 Rx end */
        if (1UL == bCM_USART6->CR1_b.RIE) {
            u32Tmp1 = bCM_USART6->SR_b.RXNE;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL3) && (0UL != u32Tmp1)) {
                USART6_RxFull_IrqHandler();
            }
        }
        /* TimerB6 overflow */
        if (1UL == bCM_TMRB_6->BCSTRH_b.ITENOVF) {
            u32Tmp1 = bCM_TMRB_6->BCSTRH_b.OVFF;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL8) && (0UL != u32Tmp1)) {
                TMRB_6_Ovf_IrqHandler();
            }
        }
        /* TimerB6 underflow */
        if (1UL == bCM_TMRB_6->BCSTRH_b.ITENUDF) {
            u32Tmp1 = bCM_TMRB_6->BCSTRH_b.UDFF;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL9) && (0UL != u32Tmp1)) {
                TMRB_6_Udf_IrqHandler();
            }
        }
        /* TimerB5 compare match */
        if (1UL == bCM_TMRB_5->ICONR_b.ITEN1) {
            u32Tmp1 = bCM_TMRB_5->STFLR_b.CMPF1;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL10) && (0UL != u32Tmp1)) {
                TMRB_5_Cmp_IrqHandler();
            }
        }
        /* USART1 Tx end */
        if (1UL == bCM_USART1->CR1_b.TCIE) {
            u32Tmp1 = bCM_USART1->SR_b.TC;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL12) && (0UL != u32Tmp1)) {
                USART1_TxComplete_IrqHandler();
            }
        }
        /* USART3 Rx end */
        if (1UL == bCM_USART3->CR1_b.RIE) {
            u32Tmp1 = bCM_USART3->SR_b.RXNE;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL13) && (0UL != u32Tmp1)) {
                USART3_RxFull_IrqHandler();
            }
        }
        /* SPI Rx end */
        if (1UL == bCM_SPI->CR1_b.RXIE) {
            u32Tmp1 = bCM_SPI->SR_b.RDFF;
            if ((1UL == bCM_INTC->ISELBR27_b.ISEL14) && (0UL != u32Tmp1)) {
                SPI_RxFull_IrqHandler();
            }
        }
    }
}

/**
 * @brief  Interrupt No.028 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ028_Handler(void)
{
    uint32_t u32Tmp1;
    /* USART5 Tx buffer empty */
    if (1UL == bCM_USART5->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART5->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL7) && (0UL != u32Tmp1)) {
            USART5_TxEmpty_IrqHandler();
        }
    }
    /* TimerB3 overflow */
    if (1UL == bCM_TMRB_3->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_3->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_3_Ovf_IrqHandler();
        }
    }
    /* TimerB3 underflow */
    if (1UL == bCM_TMRB_3->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_3->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_3_Udf_IrqHandler();
        }
    }
    /* TimerB4 compare match */
    if (1UL == bCM_TMRB_4->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_4->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_4_Cmp_IrqHandler();
        }
    }
    /* ADC seq.A convert complete */
    if (1UL == bCM_ADC->ICR_b.EOCAIEN) {
        u32Tmp1 = bCM_ADC->ISR_b.EOCAF;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL11) && (0UL != u32Tmp1)) {
            ADC_SeqA_IrqHandler();
        }
    }
    /* USART2 Rx ORE/FE/PE error */
    if (1UL == bCM_USART2->CR1_b.RIE) {
        u32Tmp1 = CM_USART2->SR & (USART_SR_PE | USART_SR_FE | USART_SR_ORE);
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL12) && (0UL != u32Tmp1)) {
            USART2_RxError_IrqHandler();
        }
    }
    /* USART3 Tx buffer empty */
    if (1UL == bCM_USART3->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART3->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL13) && (0UL != u32Tmp1)) {
            USART3_TxEmpty_IrqHandler();
        }
    }
    /* I2c Tx end */
    if (1UL == bCM_I2C->CR2_b.TENDIE) {
        u32Tmp1 = bCM_I2C->SR_b.TENDF;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL14) && (0UL != u32Tmp1)) {
            I2C_TxComplete_IrqHandler();
        }
    }
    /* SPI bus idle */
    if (1UL == bCM_SPI->CR1_b.IDIE) {
        u32Tmp1 = bCM_SPI->SR_b.IDLNF;
        if ((1UL == bCM_INTC->ISELBR28_b.ISEL15) && (0UL == u32Tmp1)) {
            SPI_Idle_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.029 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ029_Handler(void)
{
    uint32_t u32Tmp1;
    /* External interrupt 10 */
    u32Tmp1 = bCM_INTC->EIRQFR_b.EIRQF10;
    if ((1UL == bCM_INTC->ISELBR29_b.ISEL5) && (0UL != u32Tmp1)) {
        EXTINT10_IrqHandler();
    }
    /* USART6 Tx buffer empty */
    if (1UL == bCM_USART6->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART6->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL7) && (0UL != u32Tmp1)) {
            USART6_TxEmpty_IrqHandler();
        }
    }
    /* TimerB4 overflow */
    if (1UL == bCM_TMRB_4->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_4->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_4_Ovf_IrqHandler();
        }
    }
    /* TimerB4 underflow */
    if (1UL == bCM_TMRB_4->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_4->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_4_Udf_IrqHandler();
        }
    }
    /* TimerB3 compare match */
    if (1UL == bCM_TMRB_3->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_3->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_3_Cmp_IrqHandler();
        }
    }
    /* ADC seq.B convert complete */
    if (1UL == bCM_ADC->ICR_b.EOCBIEN) {
        u32Tmp1 = bCM_ADC->ISR_b.EOCBF;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL11) && (0UL != u32Tmp1)) {
            ADC_SeqB_IrqHandler();
        }
    }
    /* USART2 Rx end */
    if (1UL == bCM_USART2->CR1_b.RIE) {
        u32Tmp1 = bCM_USART2->SR_b.RXNE;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL12) && (0UL != u32Tmp1)) {
            USART2_RxFull_IrqHandler();
        }
    }
    /* USART3 Tx end */
    if (1UL == bCM_USART3->CR1_b.TCIE) {
        u32Tmp1 = bCM_USART3->SR_b.TC;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL13) && (0UL != u32Tmp1)) {
            USART3_TxComplete_IrqHandler();
        }
    }
    /* I2c Rx end */
    if (1UL == bCM_I2C->CR2_b.RFULLIE) {
        u32Tmp1 = bCM_I2C->SR_b.RFULLF;
        if ((1UL == bCM_INTC->ISELBR29_b.ISEL14) && (0UL != u32Tmp1)) {
            I2C_RxFull_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.030 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ030_Handler(void)
{
    uint32_t u32Tmp1;
    /* External interrupt 11 */
    u32Tmp1 = bCM_INTC->EIRQFR_b.EIRQF11;
    if ((1UL == bCM_INTC->ISELBR30_b.ISEL5) && (0UL != u32Tmp1)) {
        EXTINT11_IrqHandler();
    }
    /* USART5 Tx end */
    if (1UL == bCM_USART5->CR1_b.TCIE) {
        u32Tmp1 = bCM_USART5->SR_b.TC;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL7) && (0UL != u32Tmp1)) {
            USART5_TxComplete_IrqHandler();
        }
    }
    /* TimerB1 overflow */
    if (1UL == bCM_TMRB_1->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_1->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_1_Ovf_IrqHandler();
        }
    }
    /* TimerB1 underflow */
    if (1UL == bCM_TMRB_1->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_1->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_1_Udf_IrqHandler();
        }
    }
    /* TimerB2 compare match */
    if (1UL == bCM_TMRB_2->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_2->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_2_Cmp_IrqHandler();
        }
    }
    /* ADC convert result in range of window 0 setting */
    if (1UL == bCM_ADC->AWDCR_b.AWD0IEN) {
        u32Tmp1 = bCM_ADC->AWDSR_b.AWD0F;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL11) && (0UL != u32Tmp1)) {
            ADC_Cmp0_IrqHandler();
        }
    }
    /* USART2 Tx buffer empty */
    if (1UL == bCM_USART2->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART2->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL12) && (0UL != u32Tmp1)) {
            USART2_TxEmpty_IrqHandler();
        }
    }
    /* I2c Tx buffer empty */
    if (1UL == bCM_I2C->CR2_b.TEMPTYIE) {
        u32Tmp1 = bCM_I2C->SR_b.TEMPTYF;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL14) && (0UL != u32Tmp1)) {
            I2C_TxEmpty_IrqHandler();
        }
    }
    /* USART4 Tx buffer empty */
    if (1UL == bCM_USART4->CR1_b.TXEIE) {
        u32Tmp1 = bCM_USART4->SR_b.TXE;
        if ((1UL == bCM_INTC->ISELBR30_b.ISEL15) && (0UL != u32Tmp1)) {
            USART4_TxEmpty_IrqHandler();
        }
    }
}

/**
 * @brief  Interrupt No.031 share IRQ handler
 * @param  None
 * @retval None
 */
void IRQ031_Handler(void)
{
    uint32_t u32Tmp1;
    uint32_t u32Tmp2;
    /* LVD detected */
    if (0UL == bCM_EFM->LVDICGCR_b.LVDDIS) {
        u32Tmp1 = bCM_PWC->LVDCSR_b.DETF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL2) && (0UL != u32Tmp1)) {
            PWC_LVD_IrqHandler();
        }
    }
    /* USART6 Tx end */
    if (1UL == bCM_USART6->CR1_b.TCIE) {
        u32Tmp1 = bCM_USART6->SR_b.TC;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL3) && (0UL != u32Tmp1)) {
            USART6_TxComplete_IrqHandler();
        }
    }
    /* EFM operate end */
    if (1UL == bCM_EFM->FITE_b.OPTENDITE) {
        u32Tmp1 = bCM_EFM->FSR_b.OPTEND;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL4) && (0UL != u32Tmp1)) {
            EFM_OpEnd_IrqHandler();
        }
    }
    /* RTC alarm */
    if (1UL == bCM_RTC->CR2_b.ALMIE) {
        u32Tmp1 = bCM_RTC->CR2_b.ALMF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL5) && (0UL != u32Tmp1)) {
            RTC_Alarm_IrqHandler();
        }
    }
    /* RTC period */
    if (1UL == bCM_RTC->CR2_b.PRDIE) {
        u32Tmp1 = bCM_RTC->CR2_b.PRDF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL6) && (0UL != u32Tmp1)) {
            RTC_Period_IrqHandler();
        }
    }
    /* TimerB2 overflow */
    if (1UL == bCM_TMRB_2->BCSTRH_b.ITENOVF) {
        u32Tmp1 = bCM_TMRB_2->BCSTRH_b.OVFF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL8) && (0UL != u32Tmp1)) {
            TMRB_2_Ovf_IrqHandler();
        }
    }
    /* TimerB2 underflow */
    if (1UL == bCM_TMRB_2->BCSTRH_b.ITENUDF) {
        u32Tmp1 = bCM_TMRB_2->BCSTRH_b.UDFF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL9) && (0UL != u32Tmp1)) {
            TMRB_2_Udf_IrqHandler();
        }
    }
    /* TimerB1 compare match */
    if (1UL == bCM_TMRB_1->ICONR_b.ITEN1) {
        u32Tmp1 = bCM_TMRB_1->STFLR_b.CMPF1;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL10) && (0UL != u32Tmp1)) {
            TMRB_1_Cmp_IrqHandler();
        }
    }
    if (1UL == bCM_ADC->AWDCR_b.AWD1IEN) {
        /* ADC convert result in range of window 1 if independence use */
        u32Tmp1 = bCM_ADC->AWDSR_b.AWD1F;
        u32Tmp2 = (uint32_t)CM_ADC->AWDCR & (uint32_t)ADC_AWDCR_AWDCM;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL11) && (0UL != u32Tmp1) && (0UL == u32Tmp2)) {
            ADC_Cmp1_IrqHandler();
        }
        /* ADC convert result combination use of window 0 & 1 */
        u32Tmp1 = bCM_ADC->AWDSR_b.AWDCMF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL11) && (0UL != u32Tmp1) && (0UL != u32Tmp2)) {
            ADC_Cmp1_IrqHandler();
        }
    }
    /* USART2 Tx end */
    if (1UL == bCM_USART2->CR1_b.TCIE) {
        u32Tmp1 = bCM_USART2->SR_b.TC;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL12) && (0UL != u32Tmp1)) {
            USART2_TxComplete_IrqHandler();
        }
    }
    /* SPI Tx buffer empty */
    if (1UL == bCM_SPI->CR1_b.TXIE) {
        u32Tmp1 = bCM_SPI->SR_b.TDEF;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL13) && (0UL != u32Tmp1)) {
            SPI_TxEmpty_IrqHandler();
        }
    }
    /* USART4 Tx end */
    if (1UL == bCM_USART4->CR1_b.TCIE) {
        u32Tmp1 = bCM_USART4->SR_b.TC;
        if ((1UL == bCM_INTC->ISELBR31_b.ISEL15) && (0UL != u32Tmp1)) {
            USART4_TxComplete_IrqHandler();
        }
    }
}
/**
 * @}
 */

/**
 * @defgroup Share_Interrupts_Weakdef_Prototypes Share Interrupts weak function prototypes
 * @{
 */
__WEAKDEF void EXTINT08_IrqHandler(void)
{
}

__WEAKDEF void EXTINT09_IrqHandler(void)
{
}

__WEAKDEF void EXTINT10_IrqHandler(void)
{
}

__WEAKDEF void EXTINT11_IrqHandler(void)
{
}

__WEAKDEF void DMA_TC0_IrqHandler(void)
{
}

__WEAKDEF void DMA_BTC0_IrqHandler(void)
{
}

__WEAKDEF void DMA_TC1_IrqHandler(void)
{
}

__WEAKDEF void DMA_BTC1_IrqHandler(void)
{
}

__WEAKDEF void DMA_Error_IrqHandler(void)
{
}

__WEAKDEF void EFM_ProgramEraseError_IrqHandler(void)
{
}

__WEAKDEF void EFM_ColError_IrqHandler(void)
{
}

__WEAKDEF void EFM_OpEnd_IrqHandler(void)
{
}

__WEAKDEF void CLK_XtalStop_IrqHandler(void)
{
}

__WEAKDEF void SWDT_IrqHandler(void)
{
}

__WEAKDEF void TMRB_1_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_1_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_1_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_2_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_2_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_2_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_3_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_3_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_3_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_4_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_4_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_4_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_5_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_5_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_5_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_6_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_6_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_6_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_7_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_7_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_7_Cmp_IrqHandler(void)
{
}

__WEAKDEF void TMRB_8_Ovf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_8_Udf_IrqHandler(void)
{
}

__WEAKDEF void TMRB_8_Cmp_IrqHandler(void)
{
}

__WEAKDEF void USART1_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART1_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART1_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART1_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void USART2_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART2_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART2_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART2_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void USART3_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART3_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART3_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART3_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void USART4_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART4_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART4_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART4_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void USART5_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART5_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART5_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART5_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void USART6_RxError_IrqHandler(void)
{
}

__WEAKDEF void USART6_RxFull_IrqHandler(void)
{
}

__WEAKDEF void USART6_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void USART6_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void I2C_RxFull_IrqHandler(void)
{
}

__WEAKDEF void I2C_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void I2C_TxComplete_IrqHandler(void)
{
}

__WEAKDEF void I2C_EE_IrqHandler(void)
{
}

__WEAKDEF void SPI_RxFull_IrqHandler(void)
{
}

__WEAKDEF void SPI_TxEmpty_IrqHandler(void)
{
}

__WEAKDEF void SPI_Idle_IrqHandler(void)
{
}

__WEAKDEF void SPI_Error_IrqHandler(void)
{
}

__WEAKDEF void CTC_IrqHandler(void)
{
}

__WEAKDEF void EKEY_IrqHandler(void)
{
}

__WEAKDEF void TMR0_CmpA_IrqHandler(void)
{
}

__WEAKDEF void ADC_SeqA_IrqHandler(void)
{
}

__WEAKDEF void ADC_SeqB_IrqHandler(void)
{
}

__WEAKDEF void ADC_Cmp0_IrqHandler(void)
{
}

__WEAKDEF void ADC_Cmp1_IrqHandler(void)
{
}

__WEAKDEF void PWC_LVD_IrqHandler(void)
{
}

__WEAKDEF void RTC_Alarm_IrqHandler(void)
{
}

__WEAKDEF void RTC_Period_IrqHandler(void)
{
}

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

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
