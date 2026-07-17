/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_RIGHT */
#define PWM_RIGHT_INST                                                     TIMA0
#define PWM_RIGHT_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_RIGHT_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_RIGHT_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_RIGHT_C1_PORT                                             GPIOA
#define GPIO_PWM_RIGHT_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_RIGHT_C1_IOMUX                                  (IOMUX_PINCM14)
#define GPIO_PWM_RIGHT_C1_IOMUX_FUNC                 IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_PWM_RIGHT_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_RIGHT_C3_PORT                                             GPIOA
#define GPIO_PWM_RIGHT_C3_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_RIGHT_C3_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_RIGHT_C3_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMA0_CCP3
#define GPIO_PWM_RIGHT_C3_IDX                                DL_TIMER_CC_3_INDEX

/* Defines for PWM_LEFT */
#define PWM_LEFT_INST                                                      TIMA1
#define PWM_LEFT_INST_IRQHandler                                TIMA1_IRQHandler
#define PWM_LEFT_INST_INT_IRQN                                  (TIMA1_INT_IRQn)
#define PWM_LEFT_INST_CLK_FREQ                                          32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_LEFT_C0_PORT                                              GPIOA
#define GPIO_PWM_LEFT_C0_PIN                                      DL_GPIO_PIN_10
#define GPIO_PWM_LEFT_C0_IOMUX                                   (IOMUX_PINCM21)
#define GPIO_PWM_LEFT_C0_IOMUX_FUNC                  IOMUX_PINCM21_PF_TIMA1_CCP0
#define GPIO_PWM_LEFT_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_LEFT_C1_PORT                                              GPIOA
#define GPIO_PWM_LEFT_C1_PIN                                      DL_GPIO_PIN_11
#define GPIO_PWM_LEFT_C1_IOMUX                                   (IOMUX_PINCM22)
#define GPIO_PWM_LEFT_C1_IOMUX_FUNC                  IOMUX_PINCM22_PF_TIMA1_CCP1
#define GPIO_PWM_LEFT_C1_IDX                                 DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (15999U)



/* Defines for UART_GYRO */
#define UART_GYRO_INST                                                     UART0
#define UART_GYRO_INST_FREQUENCY                                        32000000
#define UART_GYRO_INST_IRQHandler                               UART0_IRQHandler
#define UART_GYRO_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_UART_GYRO_RX_PORT                                             GPIOA
#define GPIO_UART_GYRO_TX_PORT                                             GPIOA
#define GPIO_UART_GYRO_RX_PIN                                     DL_GPIO_PIN_31
#define GPIO_UART_GYRO_TX_PIN                                     DL_GPIO_PIN_28
#define GPIO_UART_GYRO_IOMUX_RX                                   (IOMUX_PINCM6)
#define GPIO_UART_GYRO_IOMUX_TX                                   (IOMUX_PINCM3)
#define GPIO_UART_GYRO_IOMUX_RX_FUNC                    IOMUX_PINCM6_PF_UART0_RX
#define GPIO_UART_GYRO_IOMUX_TX_FUNC                    IOMUX_PINCM3_PF_UART0_TX
#define UART_GYRO_BAUD_RATE                                             (115200)
#define UART_GYRO_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_GYRO_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for UART_BT */
#define UART_BT_INST                                                       UART2
#define UART_BT_INST_FREQUENCY                                          32000000
#define UART_BT_INST_IRQHandler                                 UART2_IRQHandler
#define UART_BT_INST_INT_IRQN                                     UART2_INT_IRQn
#define GPIO_UART_BT_RX_PORT                                               GPIOA
#define GPIO_UART_BT_TX_PORT                                               GPIOA
#define GPIO_UART_BT_RX_PIN                                       DL_GPIO_PIN_22
#define GPIO_UART_BT_TX_PIN                                       DL_GPIO_PIN_21
#define GPIO_UART_BT_IOMUX_RX                                    (IOMUX_PINCM47)
#define GPIO_UART_BT_IOMUX_TX                                    (IOMUX_PINCM46)
#define GPIO_UART_BT_IOMUX_RX_FUNC                     IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_BT_IOMUX_TX_FUNC                     IOMUX_PINCM46_PF_UART2_TX
#define UART_BT_BAUD_RATE                                               (115200)
#define UART_BT_IBRD_32_MHZ_115200_BAUD                                     (17)
#define UART_BT_FBRD_32_MHZ_115200_BAUD                                     (23)





/* Port definition for Pin Group GRP_GRAY_OUT */
#define GRP_GRAY_OUT_PORT                                                (GPIOB)

/* Defines for OUT: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GRP_GRAY_OUT_OUT_PIN                                    (DL_GPIO_PIN_20)
#define GRP_GRAY_OUT_OUT_IOMUX                                   (IOMUX_PINCM48)
/* Defines for AD0: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GRP_GRAY_AD0_PORT                                                (GPIOB)
#define GRP_GRAY_AD0_PIN                                        (DL_GPIO_PIN_25)
#define GRP_GRAY_AD0_IOMUX                                       (IOMUX_PINCM56)
/* Defines for AD1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GRP_GRAY_AD1_PORT                                                (GPIOA)
#define GRP_GRAY_AD1_PIN                                        (DL_GPIO_PIN_25)
#define GRP_GRAY_AD1_IOMUX                                       (IOMUX_PINCM55)
/* Defines for AD2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GRP_GRAY_AD2_PORT                                                (GPIOA)
#define GRP_GRAY_AD2_PIN                                        (DL_GPIO_PIN_27)
#define GRP_GRAY_AD2_IOMUX                                       (IOMUX_PINCM60)
/* Defines for LEFT_A: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GRP_ENCODER_LEFT_A_PORT                                          (GPIOB)
// pins affected by this interrupt request:["LEFT_A","LEFT_B","RIGHT_B"]
#define GRP_ENCODER_GPIOB_INT_IRQN                              (GPIOB_INT_IRQn)
#define GRP_ENCODER_GPIOB_INT_IIDX              (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GRP_ENCODER_LEFT_A_IIDX                             (DL_GPIO_IIDX_DIO23)
#define GRP_ENCODER_LEFT_A_PIN                                  (DL_GPIO_PIN_23)
#define GRP_ENCODER_LEFT_A_IOMUX                                 (IOMUX_PINCM51)
/* Defines for LEFT_B: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GRP_ENCODER_LEFT_B_PORT                                          (GPIOB)
#define GRP_ENCODER_LEFT_B_IIDX                             (DL_GPIO_IIDX_DIO27)
#define GRP_ENCODER_LEFT_B_PIN                                  (DL_GPIO_PIN_27)
#define GRP_ENCODER_LEFT_B_IOMUX                                 (IOMUX_PINCM58)
/* Defines for RIGHT_A: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GRP_ENCODER_RIGHT_A_PORT                                         (GPIOA)
// pins affected by this interrupt request:["RIGHT_A"]
#define GRP_ENCODER_GPIOA_INT_IRQN                              (GPIOA_INT_IRQn)
#define GRP_ENCODER_GPIOA_INT_IIDX              (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GRP_ENCODER_RIGHT_A_IIDX                            (DL_GPIO_IIDX_DIO18)
#define GRP_ENCODER_RIGHT_A_PIN                                 (DL_GPIO_PIN_18)
#define GRP_ENCODER_RIGHT_A_IOMUX                                (IOMUX_PINCM40)
/* Defines for RIGHT_B: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GRP_ENCODER_RIGHT_B_PORT                                         (GPIOB)
#define GRP_ENCODER_RIGHT_B_IIDX                            (DL_GPIO_IIDX_DIO14)
#define GRP_ENCODER_RIGHT_B_PIN                                 (DL_GPIO_PIN_14)
#define GRP_ENCODER_RIGHT_B_IOMUX                                (IOMUX_PINCM31)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_RIGHT_init(void);
void SYSCFG_DL_PWM_LEFT_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_GYRO_init(void);
void SYSCFG_DL_UART_BT_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
