/**
 *******************************************************************************
 * @file  hc32_ll_tmrb.h
 * @brief This file contains all the functions prototypes of the TimerB driver
 *        library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-09-30       CDT             Modify typo [-HC32F120,HC32M120-]
                                    Delete union in stc_tmrb_init_t structure [-HC32F120,HC32M120-]
                                    Split register BCSTR,modify definition relate to Bit of BCSTR [-HC32F120,HC32M120-]
                                    Modify some of member type of struct stc_tmrb_init_t and relate function about these member [-HC32F120,HC32M120-]
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
#ifndef __HC32_LL_TMRB_H__
#define __HC32_LL_TMRB_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

#if defined (HC32M423) || defined (CD32Z16X)
#include "hc32m4xx.h"
#include "hc32m4xx_conf.h"
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32F165)
#include "hc32f1xx.h"
#include "hc32f1xx_conf.h"
#elif defined (HC32M120)
#include "hc32m1xx.h"
#include "hc32m1xx_conf.h"
#else
#error "The currently selected chip does not support TMRB peripherals."
#endif

/**
 * @addtogroup LL_Driver
 * @{
 */

/**
 * @addtogroup LL_TMRB
 * @{
 */

#if (LL_TMRB_ENABLE == DDL_ON)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup TMRB_Global_Types TMRB Global Types
 * @{
 */

/**
 * @brief TMRB initialization structure definition
 */
typedef struct {
    uint8_t u8CountSrc;                 /*!< TMRB Count source select.
                                             This parameter can be a value of @ref TMRB_Count_Source */
    struct {
        uint8_t u8ClockDiv;             /*!< TMRB Count clock division select.
                                             This parameter can be a value of @ref TMRB_Clock_Division */
        uint8_t u8CountMode;            /*!< TMRB count mode.
                                             This parameter can be a value of @ref TMRB_Count_Mode */
        uint8_t u8CountDir;             /*!< TMRB count direction.
                                             This parameter can be a value of @ref TMRB_Count_Direction */
    } sw_count;
    struct {
        uint16_t u16CountUpCond;        /*!< TMRB hardware count up condition.
                                             This parameter can be a value of @ref TMRB_Count_Up_Condition */
        uint16_t u16CountDownCond;      /*!< TMRB hardware count down condition.
                                             This parameter can be a value of @ref TMRB_Count_Down_Condition */
    } hw_count;
    uint16_t u16PeriodValue;            /*!< TMRB period value.
                                             This parameter can be a value of half-word */
    uint8_t u8CountReload;              /*!< TMRB count reload function.
                                             This parameter can be a value of @ref TMRB_Count_Reload */
} stc_tmrb_init_t;

/**
 * @brief TMRB PWM initialization structure definition
 * @note
 */
typedef struct {
    uint16_t u16CompareValue;           /*!< TMRB compare value.
                                             This parameter can be a value of half-word */
    uint16_t u16StartPolarity;          /*!< TMRB output polarity when start count.
                                             This parameter can be a value of @ref TMRB_PWM_Polarity */
    uint16_t u16StopPolarity;           /*!< TMRB output polarity when stop count.
                                             This parameter can be a value of @ref TMRB_PWM_Polarity */
    uint16_t u16CompareMatchPolarity;   /*!< TMRB output polarity when compare value match.
                                             This parameter can be a value of @ref TMRB_PWM_Polarity */
    uint16_t u16PeriodMatchPolarity;    /*!< TMRB output polarity when period value match.
                                             This parameter can be a value of @ref TMRB_PWM_Polarity */
} stc_tmrb_pwm_init_t;

/**
 * @}
 */

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup TMRB_Global_Macros TMRB Global Macros
 * @{
 */

/**
 * @defgroup TMRB_Channel TMRB Channel
 * @note TMRB has six channels, but channels 5&6 don't support PWM output to Port and input capture.[-HC32M423-]
 * @{
 */
#define TMRB_CH1                            (0UL)
#if defined (HC32M423)
#define TMRB_CH2                            (1UL)
#define TMRB_CH3                            (2UL)
#define TMRB_CH4                            (3UL)
#define TMRB_CH5                            (4UL)
#define TMRB_CH6                            (5UL)
#elif defined (CD32Z16X)
#define TMRB_CH2                            (1UL)
#define TMRB_CH3                            (2UL)
#define TMRB_CH4                            (3UL)
#elif defined (HC32F165)
#define TMRB_CH2                            (1UL)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Source TMRB Count Source
 * @{
 */
#define TMRB_CNT_SRC_SW                     (0U)    /*!< Software count source  */
#define TMRB_CNT_SRC_HW                     (1U)    /*!< Hardware count source  */
/**
 * @}
 */

/**
 * @defgroup TMRB_Clock_Division TMRB Clock Division
 * @{
 */
#define TMRB_CLK_DIV1                       (0U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK      */
#define TMRB_CLK_DIV2                       (1U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/2    */
#define TMRB_CLK_DIV4                       (2U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/4    */
#define TMRB_CLK_DIV8                       (3U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/8    */
#define TMRB_CLK_DIV16                      (4U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/16   */
#define TMRB_CLK_DIV32                      (5U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/32   */
#define TMRB_CLK_DIV64                      (6U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/64   */
#define TMRB_CLK_DIV128                     (7U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/128  */
#define TMRB_CLK_DIV256                     (8U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/256  */
#define TMRB_CLK_DIV512                     (9U  << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/512  */
#define TMRB_CLK_DIV1024                    (10U << TMRB_BCSTRL_CKDIV_POS)  /*!< CLK/1024 */
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Mode TMRB Count Mode
 * @{
 */
#define TMRB_MD_SAWTOOTH                    (0U)
#define TMRB_MD_TRIANGLE                    (TMRB_BCSTRL_MODE)
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Direction TMRB Count Direction
 * @{
 */
#define TMRB_DIR_DOWN                       (0U)
#define TMRB_DIR_UP                         (TMRB_BCSTRL_DIR)
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Reload TMRB Count Reload
 * @{
 */
#define TMRB_CNT_RELOAD_DISABLE             (TMRB_BCSTRH_OVSTP) /*!< Disable the auto reload function */
#define TMRB_CNT_RELOAD_ENABLE              (0U)                /*!< Enable the auto reload function  */
/**
 * @}
 */

/**
 * @defgroup TMRB_Start_Condition TMRB Start Condition
 * @{
 */
#define TMRB_START_COND_PWM_RISING          (TMRB_HCONR_HSTA0)  /*!< Start count when detect the rising edge on TIM_<t>_PWM1      */
#define TMRB_START_COND_PWM_FALLING         (TMRB_HCONR_HSTA1)  /*!< Start count when detect the falling edge on TIM_<t>_PWM1     */
#define TMRB_START_COND_EVT                 (TMRB_HCONR_HSTA2)  /*!< Start count when detect the timer TMRB_HTSSR specified event */
#define TMRB_START_COND_ALL                 (TMRB_START_COND_PWM_RISING | TMRB_START_COND_PWM_FALLING | \
                                             TMRB_START_COND_EVT)
/**
 * @}
 */

/**
 * @defgroup TMRB_Stop_Condition TMRB Stop Condition
 * @{
 */
#define TMRB_STOP_COND_PWM_RISING           (TMRB_HCONR_HSTP0)  /*!< Stop count when detect the rising edge on TIM_<t>_PWM1      */
#define TMRB_STOP_COND_PWM_FALLING          (TMRB_HCONR_HSTP1)  /*!< Stop count when detect the falling edge on TIM_<t>_PWM1     */
#define TMRB_STOP_COND_EVT                  (TMRB_HCONR_HSTP2)  /*!< Stop count when detect the timer TMRB_HTSSR specified event */
#define TMRB_STOP_COND_ALL                  (TMRB_STOP_COND_PWM_RISING | TMRB_STOP_COND_PWM_FALLING | \
                                             TMRB_STOP_COND_EVT)
/**
 * @}
 */

/**
 * @defgroup TMRB_Clear_Condition TMRB Clear Condition
 * @{
 */
#define TMRB_CLR_COND_PWM_RISING            (TMRB_HCONR_HCLE0)  /*!< Clear count value when detect the rising edge on TIM_<t>_PWM1      */
#define TMRB_CLR_COND_PWM_FALLING           (TMRB_HCONR_HCLE1)  /*!< Clear count value when detect the falling edge on TIM_<t>_PWM1     */
#define TMRB_CLR_COND_EVT                   (TMRB_HCONR_HCLE2)  /*!< Clear count value when detect the timer TMRB_HTSSR specified event */
#if defined (HC32M423)
#define TMRB_CLR_COND_PWM3_RISING           (TMRB_HCONR_HCLE5)  /*!< Clear count value when detect the rising edge on TIM_<t>_PWM3  */
#define TMRB_CLR_COND_PWM3_FALLING          (TMRB_HCONR_HCLE6)  /*!< Clear count value when detect the falling edge on TIM_<t>_PWM3 */
#define TMRB_CLR_COND_ALL                   (TMRB_CLR_COND_PWM_RISING  | TMRB_CLR_COND_PWM_FALLING  | \
                                             TMRB_CLR_COND_PWM3_RISING | TMRB_CLR_COND_PWM3_FALLING | \
                                             TMRB_CLR_COND_EVT)
#elif defined (CD32Z16X)
#define TMRB_CLR_COND_SYM_PWM_RISING        (TMRB_HCONR_HCLE3)  /*!< Clear count value when detect the rising edge on TIM_<n>_PWM1, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/..  */
#define TMRB_CLR_COND_SYM_PWM_FALLING       (TMRB_HCONR_HCLE4)  /*!< Clear count value when detect the falling edge on TIM_<n>_PWM1, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/.. */
#define TMRB_CLR_COND_PWM3_RISING           (TMRB_HCONR_HCLE5)  /*!< Clear count value when detect the rising edge on TIM_<t>_PWM3  */
#define TMRB_CLR_COND_PWM3_FALLING          (TMRB_HCONR_HCLE6)  /*!< Clear count value when detect the falling edge on TIM_<t>_PWM3 */
#define TMRB_CLR_COND_ALL                   (TMRB_CLR_COND_PWM_RISING     | TMRB_CLR_COND_PWM_FALLING     | \
                                             TMRB_CLR_COND_SYM_PWM_RISING | TMRB_CLR_COND_SYM_PWM_FALLING | \
                                             TMRB_CLR_COND_PWM3_RISING    | TMRB_CLR_COND_PWM3_FALLING    | \
                                             TMRB_CLR_COND_EVT)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120) || defined (HC32F165)
#define TMRB_CLR_COND_SYM_PWM_RISING        (TMRB_HCONR_HCLE3)  /*!< Clear count value when detect the rising edge on TIM_<n>_PWM1, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/..  */
#define TMRB_CLR_COND_SYM_PWM_FALLING       (TMRB_HCONR_HCLE4)  /*!< Clear count value when detect the falling edge on TIM_<n>_PWM1, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/.. */
#define TMRB_CLR_COND_ALL                   (TMRB_CLR_COND_PWM_RISING     | TMRB_CLR_COND_PWM_FALLING     | \
                                             TMRB_CLR_COND_SYM_PWM_RISING | TMRB_CLR_COND_SYM_PWM_FALLING | \
                                             TMRB_CLR_COND_EVT)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Up_Condition TMRB Count Up Condition
 * @{
 */
#define TMRB_CNT_UP_COND_INVD               (0U)                /*!< The condition of count up is invalid                            */
#define TMRB_CNT_UP_COND_PWM_RISING         (TMRB_HCUPR_HCUP8)  /*!< Count up when detect the rising edge on TIM_<t>_PWM1            */
#define TMRB_CNT_UP_COND_PWM_FALLING        (TMRB_HCUPR_HCUP9)  /*!< Count up when detect the falling edge on TIM_<t>_PWM1           */
#define TMRB_CNT_UP_COND_EVT                (TMRB_HCUPR_HCUP10) /*!< Count up value when detect the timer TMRB_HTSSR specified event */
#if defined (HC32M423) || defined (CD32Z16X)
#define TMRB_CNT_UP_COND_ALL                (TMRB_CNT_UP_COND_PWM_RISING | TMRB_CNT_UP_COND_PWM_FALLING | \
                                             TMRB_CNT_UP_COND_EVT)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120) || defined (HC32F165)
#define TMRB_CNT_UP_COND_SYM_OVF            (TMRB_HCUPR_HCUP11) /*!< Count up by overflow of TIM_<n>, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/..  */
#define TMRB_CNT_UP_COND_SYM_UDF            (TMRB_HCUPR_HCUP12) /*!< Count up by underflow of TIM_<n>, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/.. */
#define TMRB_CNT_UP_COND_ALL                (TMRB_CNT_UP_COND_PWM_RISING | TMRB_CNT_UP_COND_PWM_FALLING | \
                                             TMRB_CNT_UP_COND_SYM_OVF    | TMRB_CNT_UP_COND_SYM_UDF     | \
                                             TMRB_CNT_UP_COND_EVT)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Count_Down_Condition TMRB Count Down Condition
 * @{
 */
#define TMRB_CNT_DOWN_COND_INVD             (0U)                /*!< The condition of count down is invalid                            */
#define TMRB_CNT_DOWN_COND_PWM_RISING       (TMRB_HCDOR_HCDO8)  /*!< Count down when detect the rising edge on TIM_<t>_PWM1            */
#define TMRB_CNT_DOWN_COND_PWM_FALLING      (TMRB_HCDOR_HCDO9)  /*!< Count down when detect the falling edge on TIM_<t>_PWM1           */
#define TMRB_CNT_DOWN_COND_EVT              (TMRB_HCDOR_HCDO10) /*!< Count down value when detect the timer TMRB_HTSSR specified event */
#if defined (HC32M423) || defined (CD32Z16X)
#define TMRB_CNT_DOWN_COND_ALL              (TMRB_CNT_DOWN_COND_PWM_RISING | TMRB_CNT_DOWN_COND_PWM_FALLING | \
                                             TMRB_CNT_DOWN_COND_EVT)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120) || defined (HC32F165)
#define TMRB_CNT_DOWN_COND_SYM_OVF          (TMRB_HCUPR_HCUP11) /*!< Count down by overflow of TIM_<n>, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/..  */
#define TMRB_CNT_DOWN_COND_SYM_UDF          (TMRB_HCUPR_HCUP12) /*!< Count down by underflow of TIM_<n>, n=2/4/.. when current unit m=1/3/.. or n=1/3/.. when current unit m=2/4/.. */
#define TMRB_CNT_DOWN_COND_ALL              (TMRB_CNT_DOWN_COND_PWM_RISING | TMRB_CNT_DOWN_COND_PWM_FALLING | \
                                             TMRB_CNT_DOWN_COND_SYM_UDF    | TMRB_CNT_DOWN_COND_SYM_OVF     | \
                                             TMRB_CNT_DOWN_COND_EVT)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_PWM_Count_State TMRB PWM Count State
 * @{
 */
#define TMRB_PWM_CNT_START                  (TMRB_PCONR_STAC_POS)
#define TMRB_PWM_CNT_STOP                   (TMRB_PCONR_STPC_POS)
#define TMRB_PWM_CNT_MATCH                  (TMRB_PCONR_CMPC_POS)
#define TMRB_PWM_CNT_PERIOD                 (TMRB_PCONR_PERC_POS)
/**
 * @}
 */

/**
 * @defgroup TMRB_PWM_Polarity TMRB PWM Polarity
 * @note     The variables 'u16StartPolarity' and 'u16StopPolarity' don't support "TMRB_PWM_INVT".
 * @{
 */
#define TMRB_PWM_LOW                        (0U)                /*!< TIM_<t>_PWMn output low level      */
#define TMRB_PWM_HIGH                       (TMRB_PCONR_STAC_0) /*!< TIM_<t>_PWMn output high level     */
#define TMRB_PWM_HOLD                       (TMRB_PCONR_STAC_1) /*!< TIM_<t>_PWMn output hold level     */
#define TMRB_PWM_INVT                       (TMRB_PCONR_STAC)   /*!< TIM_<t>_PWMn output inverted level */
/**
 * @}
 */

/**
 * @defgroup TMRB_PWM_Force_Output_Polarity TMRB Output Compare Force Output Polarity
 * @{
 */
#define TMRB_PWM_FORCE_INVD                 (0x0000U)               /*!< Invalid                                    */
#define TMRB_PWM_FORCE_LOW                  (TMRB_PCONR_FORC_1)     /*!< Force TIM_<t>_PWMn output low level        */
#define TMRB_PWM_FORCE_HIGH                 (TMRB_PCONR_FORC)       /*!< Force TIM_<t>_PWMn force output high level */
/**
 * @}
 */

/**
 * @defgroup TMRB_Function_Mode TMRB Function Mode
 * @{
 */
#define TMRB_FUNC_CMP                       (0U)                /*!< Output compare function */
#define TMRB_FUNC_CAPT                      (TMRB_CCONR_CAPMD)  /*!< Input capture function */
/**
 * @}
 */

/**
 * @defgroup TMRB_Filter_Clock_Division TMRB Filter Clock Division
 * @{
 */
#define TMRB_FILTER_CLK_DIV1                (0U)                        /*!< CLK          */
#define TMRB_FILTER_CLK_DIV4                (TMRB_CCONR_NOFICKCP_0)     /*!< CLK/4        */
#define TMRB_FILTER_CLK_DIV16               (TMRB_CCONR_NOFICKCP_1)     /*!< CLK/16       */
#define TMRB_FILTER_CLK_DIV64               (TMRB_CCONR_NOFIENCP)       /*!< CLK/64       */
/**
 * @}
 */

/**
 * @defgroup TMRB_Capture_Condition TMRB Capture Condition
 * @{
 */
#define TMRB_CAPT_COND_PWM_RISING           (TMRB_CCONR_HICP0)  /*!< Trigger capture when detect the rising edge on TIM_<t>_PWMn      */
#define TMRB_CAPT_COND_PWM_FALLING          (TMRB_CCONR_HICP1)  /*!< Trigger capture when detect the falling edge on TIM_<t>_PWMn     */
#define TMRB_CAPT_COND_EVT                  (TMRB_CCONR_HICP2)  /*!< Trigger capture when detect the timer TMRB_HTSSR specified event */
#define TMRB_CAPT_COND_ALL                  (TMRB_CAPT_COND_PWM_RISING | TMRB_CAPT_COND_PWM_FALLING | TMRB_CAPT_COND_EVT)
/**
 * @}
 */

#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
/**
 * @defgroup TMRB_Compare_Buffer_Condition TMRB Compare Buffer Condition
 * @note     An even channel(m=2/4/..) is the cache value for an odd channel(n=1/3/..).
 * @{
 */
#define TMRB_BUF_COND_INVD                  (0U)                                            /* In triangle wave count mode, Don't transfer buffer value                          */
#define TMRB_BUF_COND_PEAK                  (TMRB_BCONR_BSE0)                               /* In triangle wave count mode, Transfer buffer value when TMRB count peak           */
#define TMRB_BUF_COND_VALLEY                (TMRB_BCONR_BSE1)                               /* In triangle wave count mode, Transfer buffer value when TMRB count valley         */
#define TMRB_BUF_COND_PEAK_VALLEY           (TMRB_BUF_COND_PEAK | TMRB_BUF_COND_VALLEY)     /* In triangle wave count mode, Transfer buffer value when TMRB count peak or valley */
/**
 * @}
 */
#endif

/**
 * @defgroup TMRB_Flag TMRB Flag
 * @{
 */
#define TMRB_FLAG_OVF                       ((uint16_t)TMRB_BCSTRH_OVFF << 8U)
#define TMRB_FLAG_UDF                       ((uint16_t)TMRB_BCSTRH_UDFF << 8U)
#define TMRB_FLAG_CMP1                      (TMRB_STFLR_CMPF1)
#if defined (HC32M423)
#define TMRB_FLAG_CMP2                      (TMRB_STFLR_CMPF2)
#define TMRB_FLAG_CMP3                      (TMRB_STFLR_CMPF3)
#define TMRB_FLAG_CMP4                      (TMRB_STFLR_CMPF4)
#define TMRB_FLAG_CMP5                      (TMRB_STFLR_CMPF5)
#define TMRB_FLAG_CMP6                      (TMRB_STFLR_CMPF6)
#define TMRB_FLAG_ALL                       (TMRB_FLAG_OVF  | TMRB_FLAG_UDF  | TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2 | \
                                             TMRB_FLAG_CMP3 | TMRB_FLAG_CMP4 | TMRB_FLAG_CMP5 | TMRB_FLAG_CMP6)
#elif defined (CD32Z16X)
#define TMRB_FLAG_CMP2                      (TMRB_STFLR_CMPF2)
#define TMRB_FLAG_CMP3                      (TMRB_STFLR_CMPF3)
#define TMRB_FLAG_CMP4                      (TMRB_STFLR_CMPF4)
#define TMRB_FLAG_ALL                       (TMRB_FLAG_OVF  | TMRB_FLAG_UDF  | TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2 | \
                                             TMRB_FLAG_CMP3 | TMRB_FLAG_CMP4)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_FLAG_ALL                       (TMRB_FLAG_OVF  | TMRB_FLAG_UDF  | TMRB_FLAG_CMP1)
#elif defined (HC32F165)
#define TMRB_FLAG_CMP2                      (TMRB_STFLR_CMPF2)
#define TMRB_FLAG_ALL                       (TMRB_FLAG_OVF  | TMRB_FLAG_UDF  | TMRB_FLAG_CMP1 | TMRB_FLAG_CMP2)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Interrupt TMRB Interrupts
 * @{
 */
#define TMRB_INT_OVF                        ((uint16_t)TMRB_BCSTRH_ITENOVF << 8U)
#define TMRB_INT_UDF                        ((uint16_t)TMRB_BCSTRH_ITENUDF << 8U)
#define TMRB_INT_CMP1                       (TMRB_ICONR_ITEN1)
#if defined (HC32M423)
#define TMRB_INT_CMP2                       (TMRB_ICONR_ITEN2)
#define TMRB_INT_CMP3                       (TMRB_ICONR_ITEN3)
#define TMRB_INT_CMP4                       (TMRB_ICONR_ITEN4)
#define TMRB_INT_CMP5                       (TMRB_ICONR_ITEN5)
#define TMRB_INT_CMP6                       (TMRB_ICONR_ITEN6)
#define TMRB_INT_ALL                        (TMRB_INT_OVF  | TMRB_INT_UDF  | TMRB_INT_CMP1 | TMRB_INT_CMP2 | \
                                             TMRB_INT_CMP3 | TMRB_INT_CMP4 | TMRB_INT_CMP5 | TMRB_INT_CMP6)
#elif defined (CD32Z16X)
#define TMRB_INT_CMP2                       (TMRB_ICONR_ITEN2)
#define TMRB_INT_CMP3                       (TMRB_ICONR_ITEN3)
#define TMRB_INT_CMP4                       (TMRB_ICONR_ITEN4)
#define TMRB_INT_ALL                        (TMRB_INT_OVF  | TMRB_INT_UDF  | TMRB_INT_CMP1 | TMRB_INT_CMP2 | \
                                             TMRB_INT_CMP3 | TMRB_INT_CMP4)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_INT_ALL                        (TMRB_INT_OVF  | TMRB_INT_UDF  | TMRB_INT_CMP1)
#elif defined (HC32F165)
#define TMRB_INT_CMP2                       (TMRB_ICONR_ITEN2)
#define TMRB_INT_ALL                        (TMRB_INT_OVF  | TMRB_INT_UDF  | TMRB_INT_CMP1 | TMRB_INT_CMP2)
#endif
/**
 * @}
 */

/**
 * @defgroup TMRB_Event TMRB Event
 * @{
 */
#define TMRB_EVT_CMP1                       (TMRB_ECONR_ETEN1)
#if defined (HC32M423)
#define TMRB_EVT_CMP2                       (TMRB_ECONR_ETEN2)
#define TMRB_EVT_CMP3                       (TMRB_ECONR_ETEN3)
#define TMRB_EVT_CMP4                       (TMRB_ECONR_ETEN4)
#define TMRB_EVT_CMP5                       (TMRB_ECONR_ETEN5)
#define TMRB_EVT_CMP6                       (TMRB_ECONR_ETEN6)
#define TMRB_EVT_ALL                        (TMRB_EVT_CMP1 | TMRB_EVT_CMP2 | TMRB_EVT_CMP3 | TMRB_EVT_CMP4 | \
                                             TMRB_EVT_CMP5 | TMRB_EVT_CMP6)
#elif defined (CD32Z16X)
#define TMRB_EVT_CMP2                       (TMRB_ECONR_ETEN2)
#define TMRB_EVT_CMP3                       (TMRB_ECONR_ETEN3)
#define TMRB_EVT_CMP4                       (TMRB_ECONR_ETEN4)
#define TMRB_EVT_ALL                        (TMRB_EVT_CMP1 | TMRB_EVT_CMP2 | TMRB_EVT_CMP3 | TMRB_EVT_CMP4)
#elif defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120)
#define TMRB_EVT_ALL                        (TMRB_EVT_CMP1)
#elif defined (HC32F165)
#define TMRB_EVT_CMP2                       (TMRB_ECONR_ETEN2)
#define TMRB_EVT_ALL                        (TMRB_EVT_CMP1 | TMRB_EVT_CMP2)
#endif
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
 * @addtogroup TMRB_Global_Functions
 * @{
 */

/* Base count initialization */
void TMRB_DeInit(CM_TMRB_TypeDef *TMRBx);
int32_t TMRB_Init(CM_TMRB_TypeDef *TMRBx, const stc_tmrb_init_t *pstcTmrbInit);
int32_t TMRB_StructInit(stc_tmrb_init_t *pstcTmrbInit);
void TMRB_Start(CM_TMRB_TypeDef *TMRBx);
void TMRB_Stop(CM_TMRB_TypeDef *TMRBx);

/* Set/Get count value functions */
void TMRB_SetCountValue(CM_TMRB_TypeDef *TMRBx, uint16_t u16Value);
uint16_t TMRB_GetCountValue(const CM_TMRB_TypeDef *TMRBx);
void TMRB_SetPeriodValue(CM_TMRB_TypeDef *TMRBx, uint16_t u16Value);
uint16_t TMRB_GetPeriodValue(const CM_TMRB_TypeDef *TMRBx);
void TMRB_SetCompareValue(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Value);
uint16_t TMRB_GetCompareValue(const CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch);

/* Hardware trigger functions */
void TMRB_HWCountUpCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState);
void TMRB_HWCountDownCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState);
void TMRB_HWStartCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState);
void TMRB_HWStopCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState);
void TMRB_HWClearCondCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16Cond, en_functional_state_t enNewState);
void TMRB_HWCaptureCondCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Cond, en_functional_state_t enNewState);

/* Control configuration functions */
void TMRB_SetClockDiv(CM_TMRB_TypeDef *TMRBx, uint8_t u8Div);
void TMRB_SetCountMode(CM_TMRB_TypeDef *TMRBx, uint8_t u8Mode);
void TMRB_SetCountDir(CM_TMRB_TypeDef *TMRBx, uint8_t u8Dir);
uint8_t TMRB_GetCountDir(const CM_TMRB_TypeDef *TMRBx);
void TMRB_SetFunc(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Func);
void TMRB_CountReloadCmd(CM_TMRB_TypeDef *TMRBx, en_functional_state_t enNewState);
#if defined (HC32F115) || defined (HC32F120) || defined (HC32F160) || defined (HC32M120) || defined (HC32F165)
void TMRB_SyncStartCmd(CM_TMRB_TypeDef *TMRBx, en_functional_state_t enNewState);
#endif
void TMRB_SetFilterClockDiv(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Div);
void TMRB_FilterCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState);

/* PWM functions */
void TMRB_PWM_DeInit(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch);
int32_t TMRB_PWM_Init(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, const stc_tmrb_pwm_init_t *pstcTmrbPwmInit);
int32_t TMRB_PWM_StructInit(stc_tmrb_pwm_init_t *pstcTmrbPwmInit);
void TMRB_PWM_SetPolarity(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16CountState, uint16_t u16Polarity);
void TMRB_PWM_SetForcePolarity(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16Polarity);
void TMRB_PWM_OutputCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState);
#if defined (HC32M423) || defined (CD32Z16X) || defined (HC32F165)
void TMRB_SetCompareBufCond(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, uint16_t u16BufCond);
void TMRB_CompareBufCmd(CM_TMRB_TypeDef *TMRBx, uint32_t u32Ch, en_functional_state_t enNewState);
#endif

/* Interrupt and flag management functions */
void TMRB_IntCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16IntType, en_functional_state_t enNewState);
void TMRB_EventCmd(CM_TMRB_TypeDef *TMRBx, uint16_t u16EventType, en_functional_state_t enNewState);
en_flag_status_t TMRB_GetStatus(const CM_TMRB_TypeDef *TMRBx, uint16_t u16Flag);
void TMRB_ClearStatus(CM_TMRB_TypeDef *TMRBx, uint16_t u16Flag);

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

#ifdef __cplusplus
}
#endif

#endif /* LL_Driver */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
