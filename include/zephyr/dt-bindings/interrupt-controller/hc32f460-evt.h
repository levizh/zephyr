/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_HC32F460_EVT_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_HC32F460_EVT_H_


#define    EVT_SRC_SWI_IRQ0             0       /* SWI_IRQ0  */
#define    EVT_SRC_SWI_IRQ1             1       /* SWI_IRQ1  */
#define    EVT_SRC_SWI_IRQ2             2       /* SWI_IRQ2  */
#define    EVT_SRC_SWI_IRQ3             3       /* SWI_IRQ3  */
#define    EVT_SRC_SWI_IRQ4             4       /* SWI_IRQ4  */
#define    EVT_SRC_SWI_IRQ5             5       /* SWI_IRQ5  */
#define    EVT_SRC_SWI_IRQ6             6       /* SWI_IRQ6  */
#define    EVT_SRC_SWI_IRQ7             7       /* SWI_IRQ7  */
#define    EVT_SRC_SWI_IRQ8             8       /* SWI_IRQ8  */
#define    EVT_SRC_SWI_IRQ9             9       /* SWI_IRQ9  */
#define    EVT_SRC_SWI_IRQ10            10      /* SWI_IRQ10 */
#define    EVT_SRC_SWI_IRQ11            11      /* SWI_IRQ11 */
#define    EVT_SRC_SWI_IRQ12            12      /* SWI_IRQ12 */
#define    EVT_SRC_SWI_IRQ13            13      /* SWI_IRQ13 */
#define    EVT_SRC_SWI_IRQ14            14      /* SWI_IRQ14 */
#define    EVT_SRC_SWI_IRQ15            15      /* SWI_IRQ15 */
#define    EVT_SRC_SWI_IRQ16            16      /* SWI_IRQ16 */
#define    EVT_SRC_SWI_IRQ17            17      /* SWI_IRQ17 */
#define    EVT_SRC_SWI_IRQ18            18      /* SWI_IRQ18 */
#define    EVT_SRC_SWI_IRQ19            19      /* SWI_IRQ19 */
#define    EVT_SRC_SWI_IRQ20            20      /* SWI_IRQ20 */
#define    EVT_SRC_SWI_IRQ21            21      /* SWI_IRQ21 */
#define    EVT_SRC_SWI_IRQ22            22      /* SWI_IRQ22 */
#define    EVT_SRC_SWI_IRQ23            23      /* SWI_IRQ23 */
#define    EVT_SRC_SWI_IRQ24            24      /* SWI_IRQ24 */
#define    EVT_SRC_SWI_IRQ25            25      /* SWI_IRQ25 */
#define    EVT_SRC_SWI_IRQ26            26      /* SWI_IRQ26 */
#define    EVT_SRC_SWI_IRQ27            27      /* SWI_IRQ27 */
#define    EVT_SRC_SWI_IRQ28            28      /* SWI_IRQ28 */
#define    EVT_SRC_SWI_IRQ29            29      /* SWI_IRQ29 */
#define    EVT_SRC_SWI_IRQ30            30      /* SWI_IRQ30 */
#define    EVT_SRC_SWI_IRQ31            31      /* SWI_IRQ31 */

           /* External Interrupt. */
#define    EVT_SRC_PORT_EIRQ0           0       /* PORT_EIRQ0  */
#define    EVT_SRC_PORT_EIRQ1           1       /* PORT_EIRQ1  */
#define    EVT_SRC_PORT_EIRQ2           2       /* PORT_EIRQ2  */
#define    EVT_SRC_PORT_EIRQ3           3       /* PORT_EIRQ3  */
#define    EVT_SRC_PORT_EIRQ4           4       /* PORT_EIRQ4  */
#define    EVT_SRC_PORT_EIRQ5           5       /* PORT_EIRQ5  */
#define    EVT_SRC_PORT_EIRQ6           6       /* PORT_EIRQ6  */
#define    EVT_SRC_PORT_EIRQ7           7       /* PORT_EIRQ7  */
#define    EVT_SRC_PORT_EIRQ8           8       /* PORT_EIRQ8  */
#define    EVT_SRC_PORT_EIRQ9           9       /* PORT_EIRQ9  */
#define    EVT_SRC_PORT_EIRQ10          10      /* PORT_EIRQ10 */
#define    EVT_SRC_PORT_EIRQ11          11      /* PORT_EIRQ11 */
#define    EVT_SRC_PORT_EIRQ12          12      /* PORT_EIRQ12 */
#define    EVT_SRC_PORT_EIRQ13          13      /* PORT_EIRQ13 */
#define    EVT_SRC_PORT_EIRQ14          14      /* PORT_EIRQ14 */
#define    EVT_SRC_PORT_EIRQ15          15      /* PORT_EIRQ15 */

           /* DMAC */
#define    EVT_SRC_DMA1_TC0             32      /* DMA1_TC0  */
#define    EVT_SRC_DMA1_TC1             33      /* DMA1_TC1  */
#define    EVT_SRC_DMA1_TC2             34      /* DMA1_TC2  */
#define    EVT_SRC_DMA1_TC3             35      /* DMA1_TC3  */
#define    EVT_SRC_DMA2_TC0             36      /* DMA2_TC0  */
#define    EVT_SRC_DMA2_TC1             37      /* DMA2_TC1  */
#define    EVT_SRC_DMA2_TC2             38      /* DMA2_TC2  */
#define    EVT_SRC_DMA2_TC3             39      /* DMA2_TC3  */
#define    EVT_SRC_DMA1_BTC0            40      /* DMA1_BTC0 */
#define    EVT_SRC_DMA1_BTC1            41      /* DMA1_BTC1 */
#define    EVT_SRC_DMA1_BTC2            42      /* DMA1_BTC2 */
#define    EVT_SRC_DMA1_BTC3            43      /* DMA1_BTC3 */
#define    EVT_SRC_DMA2_BTC0            44      /* DMA2_BTC0 */
#define    EVT_SRC_DMA2_BTC1            45      /* DMA2_BTC1 */
#define    EVT_SRC_DMA2_BTC2            46      /* DMA2_BTC2 */
#define    EVT_SRC_DMA2_BTC3            47      /* DMA2_BTC3 */

           /* EFM */
#define    EVT_SRC_EFM_OPTEND           52      /* EFM_OPTEND */

           /* USB SOF */
#define    EVT_SRC_USBFS_SOF            53      /* USBFS_SOF */

           /* DCU */
#define    EVT_SRC_DCU1                 55      /* DCU1 */
#define    EVT_SRC_DCU2                 56      /* DCU2 */
#define    EVT_SRC_DCU3                 57      /* DCU3 */
#define    EVT_SRC_DCU4                 58      /* DCU4 */

           /* TIMER 0 */
#define    EVT_SRC_TMR0_1_CMP_A         64      /* TMR01_GCMA */
#define    EVT_SRC_TMR0_1_CMP_B         65      /* TMR01_GCMB */
#define    EVT_SRC_TMR0_2_CMP_A         66      /* TMR02_GCMA */
#define    EVT_SRC_TMR0_2_CMP_B         67      /* TMR02_GCMB */

           /* RTC */
#define    EVT_SRC_RTC_ALM              81      /* RTC_ALM */
#define    EVT_SRC_RTC_PRD              82      /* RTC_PRD */

           /* TIMER 6 */
#define    EVT_SRC_TMR6_1_GCMP_A        96      /* TMR61_GCMA */
#define    EVT_SRC_TMR6_1_GCMP_B        97      /* TMR61_GCMB */
#define    EVT_SRC_TMR6_1_GCMP_C        98      /* TMR61_GCMC */
#define    EVT_SRC_TMR6_1_GCMP_D        99      /* TMR61_GCMD */
#define    EVT_SRC_TMR6_1_GCMP_E        100     /* TMR61_GCME */
#define    EVT_SRC_TMR6_1_GCMP_F        101     /* TMR61_GCMF */
#define    EVT_SRC_TMR6_1_OVF           102     /* TMR61_GOVF */
#define    EVT_SRC_TMR6_1_UDF           103     /* TMR61_GUDF */
#define    EVT_SRC_TMR6_1_SCMP_A        107     /* TMR61_SCMA */
#define    EVT_SRC_TMR6_1_SCMP_B        108     /* TMR61_SCMB */
#define    EVT_SRC_TMR6_2_GCMP_A        112     /* TMR62_GCMA */
#define    EVT_SRC_TMR6_2_GCMP_B        113     /* TMR62_GCMB */
#define    EVT_SRC_TMR6_2_GCMP_C        114     /* TMR62_GCMC */
#define    EVT_SRC_TMR6_2_GCMP_D        115     /* TMR62_GCMD */
#define    EVT_SRC_TMR6_2_GCMP_E        116     /* TMR62_GCME */
#define    EVT_SRC_TMR6_2_GCMP_F        117     /* TMR62_GCMF */
#define    EVT_SRC_TMR6_2_OVF           118     /* TMR62_GOVF */
#define    EVT_SRC_TMR6_2_UDF           119     /* TMR62_GUDF */
#define    EVT_SRC_TMR6_2_SCMP_A        123     /* TMR62_SCMA */
#define    EVT_SRC_TMR6_2_SCMP_B        124     /* TMR62_SCMB */
#define    EVT_SRC_TMR6_3_GCMP_A        128     /* TMR63_GCMA */
#define    EVT_SRC_TMR6_3_GCMP_B        129     /* TMR63_GCMB */
#define    EVT_SRC_TMR6_3_GCMP_C        130     /* TMR63_GCMC */
#define    EVT_SRC_TMR6_3_GCMP_D        131     /* TMR63_GCMD */
#define    EVT_SRC_TMR6_3_GCMP_E        132     /* TMR63_GCME */
#define    EVT_SRC_TMR6_3_GCMP_F        133     /* TMR63_GCMF */
#define    EVT_SRC_TMR6_3_OVF           134     /* TMR63_GOVF */
#define    EVT_SRC_TMR6_3_UDF           135     /* TMR63_GUDF */
#define    EVT_SRC_TMR6_3_SCMP_A        139     /* TMR63_SCMA */
#define    EVT_SRC_TMR6_3_SCMP_B        140     /* TMR63_SCMB */

           /* TIMER A */
#define    EVT_SRC_TMRA_1_OVF           256     /* TMRA1_OVF */
#define    EVT_SRC_TMRA_1_UDF           257     /* TMRA1_UDF */
#define    EVT_SRC_TMRA_1_CMP           258     /* TMRA1_CMP */
#define    EVT_SRC_TMRA_2_OVF           259     /* TMRA2_OVF */
#define    EVT_SRC_TMRA_2_UDF           260     /* TMRA2_UDF */
#define    EVT_SRC_TMRA_2_CMP           261     /* TMRA2_CMP */
#define    EVT_SRC_TMRA_3_OVF           262     /* TMRA3_OVF */
#define    EVT_SRC_TMRA_3_UDF           263     /* TMRA3_UDF */
#define    EVT_SRC_TMRA_3_CMP           264     /* TMRA3_CMP */
#define    EVT_SRC_TMRA_4_OVF           265     /* TMRA4_OVF */
#define    EVT_SRC_TMRA_4_UDF           266     /* TMRA4_UDF */
#define    EVT_SRC_TMRA_4_CMP           267     /* TMRA4_CMP */
#define    EVT_SRC_TMRA_5_OVF           268     /* TMRA5_OVF */
#define    EVT_SRC_TMRA_5_UDF           269     /* TMRA5_UDF */
#define    EVT_SRC_TMRA_5_CMP           270     /* TMRA5_CMP */
#define    EVT_SRC_TMRA_6_OVF           272     /* TMRA6_OVF */
#define    EVT_SRC_TMRA_6_UDF           273     /* TMRA6_UDF */
#define    EVT_SRC_TMRA_6_CMP           274     /* TMRA6_CMP */

           /* USART */
#define    EVT_SRC_USART1_EI            278     /* USART1_EI  */
#define    EVT_SRC_USART1_RI            279     /* USART1_RI  */
#define    EVT_SRC_USART1_TI            280     /* USART1_TI  */
#define    EVT_SRC_USART1_TCI           281     /* USART1_TCI */
#define    EVT_SRC_USART1_RTO           282     /* USART1_RTO */
#define    EVT_SRC_USART2_EI            283     /* USART2_EI  */
#define    EVT_SRC_USART2_RI            284     /* USART2_RI  */
#define    EVT_SRC_USART2_TI            285     /* USART2_TI  */
#define    EVT_SRC_USART2_TCI           286     /* USART2_TCI */
#define    EVT_SRC_USART2_RTO           287     /* USART2_RTO */
#define    EVT_SRC_USART3_EI            288     /* USART3_EI  */
#define    EVT_SRC_USART3_RI            289     /* USART3_RI  */
#define    EVT_SRC_USART3_TI            290     /* USART3_TI  */
#define    EVT_SRC_USART3_TCI           291     /* USART3_TCI */
#define    EVT_SRC_USART3_RTO           292     /* USART3_RTO */
#define    EVT_SRC_USART4_EI            293     /* USART4_EI  */
#define    EVT_SRC_USART4_RI            294     /* USART4_RI  */
#define    EVT_SRC_USART4_TI            295     /* USART4_TI  */
#define    EVT_SRC_USART4_TCI           296     /* USART4_TCI */
#define    EVT_SRC_USART4_RTO           297     /* USART4_RTO */

           /* SPI */
#define    EVT_SRC_SPI1_SPRI            299     /* SPI1_SPRI   */
#define    EVT_SRC_SPI1_SPTI            300     /* SPI1_SPTI   */
#define    EVT_SRC_SPI1_SPII            301     /* SPI1_SPII   */
#define    EVT_SRC_SPI1_SPEI            302     /* SPI1_SPEI   */
#define    EVT_SRC_SPI1_SPTEND          303     /* SPI1_SPTEND */
#define    EVT_SRC_SPI2_SPRI            304     /* SPI2_SPRI   */
#define    EVT_SRC_SPI2_SPTI            305     /* SPI2_SPTI   */
#define    EVT_SRC_SPI2_SPII            306     /* SPI2_SPII   */
#define    EVT_SRC_SPI2_SPEI            307     /* SPI2_SPEI   */
#define    EVT_SRC_SPI2_SPTEND          308     /* SPI2_SPTEND */
#define    EVT_SRC_SPI3_SPRI            309     /* SPI3_SPRI   */
#define    EVT_SRC_SPI3_SPTI            310     /* SPI3_SPTI   */
#define    EVT_SRC_SPI3_SPII            311     /* SPI3_SPII   */
#define    EVT_SRC_SPI3_SPEI            312     /* SPI3_SPEI   */
#define    EVT_SRC_SPI3_SPTEND          313     /* SPI3_SPTEND */
#define    EVT_SRC_SPI4_SPRI            314     /* SPI4_SPRI   */
#define    EVT_SRC_SPI4_SPTI            315     /* SPI4_SPTI   */
#define    EVT_SRC_SPI4_SPII            316     /* SPI4_SPII   */
#define    EVT_SRC_SPI4_SPEI            317     /* SPI4_SPEI   */
#define    EVT_SRC_SPI4_SPTEND          318     /* SPI4_SPTEND */

           /* AOS */
#define    EVT_SRC_AOS_STRG             319     /* AOS_STRG */

           /* TIMER 4 */
#define    EVT_SRC_TMR4_1_SCMP0         368     /* TMR41_SCM0 */
#define    EVT_SRC_TMR4_1_SCMP1         369     /* TMR41_SCM1 */
#define    EVT_SRC_TMR4_1_SCMP2         370     /* TMR41_SCM2 */
#define    EVT_SRC_TMR4_1_SCMP3         371     /* TMR41_SCM3 */
#define    EVT_SRC_TMR4_1_SCMP4         372     /* TMR41_SCM4 */
#define    EVT_SRC_TMR4_1_SCMP5         373     /* TMR41_SCM5 */
#define    EVT_SRC_TMR4_2_SCMP0         374     /* TMR42_SCM0 */
#define    EVT_SRC_TMR4_2_SCMP1         375     /* TMR42_SCM1 */
#define    EVT_SRC_TMR4_2_SCMP2         376     /* TMR42_SCM2 */
#define    EVT_SRC_TMR4_2_SCMP3         377     /* TMR42_SCM3 */
#define    EVT_SRC_TMR4_2_SCMP4         378     /* TMR42_SCM4 */
#define    EVT_SRC_TMR4_2_SCMP5         379     /* TMR42_SCM5 */
#define    EVT_SRC_TMR4_3_SCMP0         384     /* TMR43_SCM0 */
#define    EVT_SRC_TMR4_3_SCMP1         385     /* TMR43_SCM1 */
#define    EVT_SRC_TMR4_3_SCMP2         386     /* TMR43_SCM2 */
#define    EVT_SRC_TMR4_3_SCMP3         387     /* TMR43_SCM3 */
#define    EVT_SRC_TMR4_3_SCMP4         388     /* TMR43_SCM4 */
#define    EVT_SRC_TMR4_3_SCMP5         389     /* TMR43_SCM5 */

           /* EVENT PORT */
#define    EVT_SRC_EVENT_PORT1          394     /* EVENT_PORT1 */
#define    EVT_SRC_EVENT_PORT2          395     /* EVENT_PORT2 */
#define    EVT_SRC_EVENT_PORT3          396     /* EVENT_PORT3 */
#define    EVT_SRC_EVENT_PORT4          397     /* EVENT_PORT4 */

           /* I2S */
#define    EVT_SRC_I2S1_TXIRQOUT        400     /* I2S1_TXIRQOUT */
#define    EVT_SRC_I2S1_RXIRQOUT        401     /* I2S1_RXIRQOUT */
#define    EVT_SRC_I2S2_TXIRQOUT        403     /* I2S2_TXIRQOUT */
#define    EVT_SRC_I2S2_RXIRQOUT        404     /* I2S2_RXIRQOUT */
#define    EVT_SRC_I2S3_TXIRQOUT        406     /* I2S3_TXIRQOUT */
#define    EVT_SRC_I2S3_RXIRQOUT        407     /* I2S3_RXIRQOUT */
#define    EVT_SRC_I2S4_TXIRQOUT        409     /* I2S4_TXIRQOUT */
#define    EVT_SRC_I2S4_RXIRQOUT        410     /* I2S4_RXIRQOUT */

           /* COMPARATOR */
#define    EVT_SRC_CMP1                 416     /* ACMP1 */
#define    EVT_SRC_CMP2                 417     /* ACMP1 */
#define    EVT_SRC_CMP3                 418     /* ACMP1 */

           /* I2C */
#define    EVT_SRC_I2C1_RXI             420     /* I2C1_RXI */
#define    EVT_SRC_I2C1_TXI             421     /* I2C1_TXI */
#define    EVT_SRC_I2C1_TEI             422     /* I2C1_TEI */
#define    EVT_SRC_I2C1_EEI             423     /* I2C1_EEI */
#define    EVT_SRC_I2C2_RXI             424     /* I2C2_RXI */
#define    EVT_SRC_I2C2_TXI             425     /* I2C2_TXI */
#define    EVT_SRC_I2C2_TEI             426     /* I2C2_TEI */
#define    EVT_SRC_I2C2_EEI             427     /* I2C2_EEI */
#define    EVT_SRC_I2C3_RXI             428     /* I2C3_RXI */
#define    EVT_SRC_I2C3_TXI             429     /* I2C3_TXI */
#define    EVT_SRC_I2C3_TEI             430     /* I2C3_TEI */
#define    EVT_SRC_I2C3_EEI             431     /* I2C3_EEI */

           /* LVD */
#define    EVT_SRC_LVD1                 433     /* LVD1 */
#define    EVT_SRC_LVD2                 434     /* LVD2 */

           /* OTS */
#define    EVT_SRC_OTS                  435     /* OTS */

           /* WDT */
#define    EVT_SRC_WDT_REFUDF           439     /* WDT_REFUDF */

           /* ADC */
#define    EVT_SRC_ADC1_EOCA            448     /* ADC1_EOCA   */
#define    EVT_SRC_ADC1_EOCB            449     /* ADC1_EOCB   */
#define    EVT_SRC_ADC1_CHCMP           450     /* ADC1_CHCMP  */
#define    EVT_SRC_ADC1_SEQCMP          451     /* ADC1_SEQCMP */
#define    EVT_SRC_ADC2_EOCA            452     /* ADC2_EOCA   */
#define    EVT_SRC_ADC2_EOCB            453     /* ADC2_EOCB   */
#define    EVT_SRC_ADC2_CHCMP           454     /* ADC2_CHCMP  */
#define    EVT_SRC_ADC2_SEQCMP          455     /* ADC2_SEQCMP */

           /* TRNG */
#define    EVT_SRC_TRNG_END             456     /* TRNG_END */

           /* SDIO */
#define    EVT_SRC_SDIOC1_DMAR          480     /* SDIOC1_DMAR */
#define    EVT_SRC_SDIOC1_DMAW          481     /* SDIOC1_DMAW */
#define    EVT_SRC_SDIOC2_DMAR          483     /* SDIOC2_DMAR */
#define    EVT_SRC_SDIOC2_DMAW          484     /* SDIOC2_DMAW */
#define    EVT_SRC_MAX                  511


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_HC32F460_EVT_H_ */
