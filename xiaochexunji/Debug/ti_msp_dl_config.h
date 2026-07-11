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



/* Defines for PWM_DRIVE */
#define PWM_DRIVE_INST                                                     TIMA0
#define PWM_DRIVE_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_DRIVE_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_DRIVE_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_DRIVE_C0_PORT                                             GPIOB
#define GPIO_PWM_DRIVE_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_PWM_DRIVE_C0_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_PWM_DRIVE_C0_IOMUX_FUNC                 IOMUX_PINCM31_PF_TIMA0_CCP0
#define GPIO_PWM_DRIVE_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_DRIVE_C1_PORT                                             GPIOA
#define GPIO_PWM_DRIVE_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_DRIVE_C1_IOMUX                                  (IOMUX_PINCM14)
#define GPIO_PWM_DRIVE_C1_IOMUX_FUNC                 IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_PWM_DRIVE_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_DRIVE_C2_PORT                                             GPIOA
#define GPIO_PWM_DRIVE_C2_PIN                                     DL_GPIO_PIN_15
#define GPIO_PWM_DRIVE_C2_IOMUX                                  (IOMUX_PINCM37)
#define GPIO_PWM_DRIVE_C2_IOMUX_FUNC                 IOMUX_PINCM37_PF_TIMA0_CCP2
#define GPIO_PWM_DRIVE_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_DRIVE_C3_PORT                                             GPIOA
#define GPIO_PWM_DRIVE_C3_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_DRIVE_C3_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_DRIVE_C3_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMA0_CCP3
#define GPIO_PWM_DRIVE_C3_IDX                                DL_TIMER_CC_3_INDEX



/* Defines for TIMER_PID */
#define TIMER_PID_INST                                                   (TIMG0)
#define TIMER_PID_INST_IRQHandler                               TIMG0_IRQHandler
#define TIMER_PID_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define TIMER_PID_INST_LOAD_VALUE                                       (39999U)



/* Defines for UART_JDY31 */
#define UART_JDY31_INST                                                    UART0
#define UART_JDY31_INST_FREQUENCY                                       32000000
#define UART_JDY31_INST_IRQHandler                              UART0_IRQHandler
#define UART_JDY31_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_JDY31_RX_PORT                                            GPIOA
#define GPIO_UART_JDY31_TX_PORT                                            GPIOA
#define GPIO_UART_JDY31_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_UART_JDY31_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_UART_JDY31_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_UART_JDY31_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_UART_JDY31_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_JDY31_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define UART_JDY31_BAUD_RATE                                            (115200)
#define UART_JDY31_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_JDY31_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group GRP_GRAY */
#define GRP_GRAY_PORT                                                    (GPIOA)

/* Defines for AD0: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GRP_GRAY_AD0_PIN                                        (DL_GPIO_PIN_24)
#define GRP_GRAY_AD0_IOMUX                                       (IOMUX_PINCM54)
/* Defines for AD1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GRP_GRAY_AD1_PIN                                        (DL_GPIO_PIN_25)
#define GRP_GRAY_AD1_IOMUX                                       (IOMUX_PINCM55)
/* Defines for AD2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GRP_GRAY_AD2_PIN                                        (DL_GPIO_PIN_26)
#define GRP_GRAY_AD2_IOMUX                                       (IOMUX_PINCM59)
/* Port definition for Pin Group GRP_GRAY_OUT */
#define GRP_GRAY_OUT_PORT                                                (GPIOA)

/* Defines for OUT: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GRP_GRAY_OUT_OUT_PIN                                    (DL_GPIO_PIN_27)
#define GRP_GRAY_OUT_OUT_IOMUX                                   (IOMUX_PINCM60)
/* Port definition for Pin Group GRP_RELAY */
#define GRP_RELAY_PORT                                                   (GPIOB)

/* Defines for MOTOR: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GRP_RELAY_MOTOR_PIN                                     (DL_GPIO_PIN_16)
#define GRP_RELAY_MOTOR_IOMUX                                    (IOMUX_PINCM33)
/* Port definition for Pin Group GRP_ENC_LEFT */
#define GRP_ENC_LEFT_PORT                                                (GPIOA)

/* Defines for ENC_L_A: GPIOA.23 with pinCMx 53 on package pin 24 */
// pins affected by this interrupt request:["ENC_L_A","ENC_L_B"]
#define GRP_ENC_LEFT_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GRP_ENC_LEFT_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GRP_ENC_LEFT_ENC_L_A_IIDX                           (DL_GPIO_IIDX_DIO23)
#define GRP_ENC_LEFT_ENC_L_A_PIN                                (DL_GPIO_PIN_23)
#define GRP_ENC_LEFT_ENC_L_A_IOMUX                               (IOMUX_PINCM53)
/* Defines for ENC_L_B: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GRP_ENC_LEFT_ENC_L_B_IIDX                            (DL_GPIO_IIDX_DIO0)
#define GRP_ENC_LEFT_ENC_L_B_PIN                                 (DL_GPIO_PIN_0)
#define GRP_ENC_LEFT_ENC_L_B_IOMUX                                (IOMUX_PINCM1)
/* Port definition for Pin Group GRP_ENC_RIGHT */
#define GRP_ENC_RIGHT_PORT                                               (GPIOB)

/* Defines for ENC_R_A: GPIOB.24 with pinCMx 52 on package pin 23 */
// pins affected by this interrupt request:["ENC_R_A","ENC_R_B"]
#define GRP_ENC_RIGHT_INT_IRQN                                  (GPIOB_INT_IRQn)
#define GRP_ENC_RIGHT_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GRP_ENC_RIGHT_ENC_R_A_IIDX                          (DL_GPIO_IIDX_DIO24)
#define GRP_ENC_RIGHT_ENC_R_A_PIN                               (DL_GPIO_PIN_24)
#define GRP_ENC_RIGHT_ENC_R_A_IOMUX                              (IOMUX_PINCM52)
/* Defines for ENC_R_B: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GRP_ENC_RIGHT_ENC_R_B_IIDX                          (DL_GPIO_IIDX_DIO25)
#define GRP_ENC_RIGHT_ENC_R_B_PIN                               (DL_GPIO_PIN_25)
#define GRP_ENC_RIGHT_ENC_R_B_IOMUX                              (IOMUX_PINCM56)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_DRIVE_init(void);
void SYSCFG_DL_TIMER_PID_init(void);
void SYSCFG_DL_UART_JDY31_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
