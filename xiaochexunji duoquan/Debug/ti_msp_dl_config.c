/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_RIGHTBackup;
DL_TimerA_backupConfig gPWM_LEFTBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_RIGHT_init();
    SYSCFG_DL_PWM_LEFT_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_UART_GYRO_init();
    SYSCFG_DL_UART_BT_init();
    /* Ensure backup structures have no valid state */
	gPWM_RIGHTBackup.backupRdy 	= false;
	gPWM_LEFTBackup.backupRdy 	= false;



}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup);
	retStatus &= DL_TimerA_saveConfiguration(PWM_LEFT_INST, &gPWM_LEFTBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(PWM_LEFT_INST, &gPWM_LEFTBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_RIGHT_INST);
    DL_TimerA_reset(PWM_LEFT_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_UART_Main_reset(UART_GYRO_INST);
    DL_UART_Main_reset(UART_BT_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_RIGHT_INST);
    DL_TimerA_enablePower(PWM_LEFT_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_UART_Main_enablePower(UART_GYRO_INST);
    DL_UART_Main_enablePower(UART_BT_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_RIGHT_C0_IOMUX,GPIO_PWM_RIGHT_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_RIGHT_C0_PORT, GPIO_PWM_RIGHT_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_RIGHT_C1_IOMUX,GPIO_PWM_RIGHT_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_RIGHT_C1_PORT, GPIO_PWM_RIGHT_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_LEFT_C0_IOMUX,GPIO_PWM_LEFT_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_LEFT_C0_PORT, GPIO_PWM_LEFT_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_LEFT_C1_IOMUX,GPIO_PWM_LEFT_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_LEFT_C1_PORT, GPIO_PWM_LEFT_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_GYRO_IOMUX_TX, GPIO_UART_GYRO_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_GYRO_IOMUX_RX, GPIO_UART_GYRO_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_BT_IOMUX_TX, GPIO_UART_BT_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_BT_IOMUX_RX, GPIO_UART_BT_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalInputFeatures(GRP_GRAY_OUT_OUT_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GRP_GRAY_AD0_IOMUX);

    DL_GPIO_initDigitalOutput(GRP_GRAY_AD1_IOMUX);

    DL_GPIO_initDigitalOutput(GRP_GRAY_AD2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GRP_ENCODER_LEFT_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GRP_ENCODER_LEFT_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GRP_ENCODER_RIGHT_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GRP_ENCODER_RIGHT_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GRP_GRAY_AD1_PIN |
		GRP_GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GPIOA, GRP_GRAY_AD1_PIN |
		GRP_GRAY_AD2_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_18_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsInputFilter(GPIOA, DL_GPIO_PIN_18_INPUT_FILTER_3_CYCLES);
    DL_GPIO_clearInterruptStatus(GPIOA, GRP_ENCODER_RIGHT_A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, GRP_ENCODER_RIGHT_A_PIN);
    DL_GPIO_clearPins(GPIOB, GRP_GRAY_AD0_PIN);
    DL_GPIO_enableOutput(GPIOB, GRP_GRAY_AD0_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_14_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_23_EDGE_RISE_FALL |
		DL_GPIO_PIN_27_EDGE_RISE_FALL);
    DL_GPIO_setLowerPinsInputFilter(GPIOB, DL_GPIO_PIN_14_INPUT_FILTER_3_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(GPIOB, DL_GPIO_PIN_23_INPUT_FILTER_3_CYCLES |
		DL_GPIO_PIN_27_INPUT_FILTER_3_CYCLES);
    DL_GPIO_clearInterruptStatus(GPIOB, GRP_ENCODER_LEFT_A_PIN |
		GRP_ENCODER_LEFT_B_PIN |
		GRP_ENCODER_RIGHT_B_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GRP_ENCODER_LEFT_A_PIN |
		GRP_ENCODER_LEFT_B_PIN |
		GRP_ENCODER_RIGHT_B_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_RIGHTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_RIGHTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 3200,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_RIGHT_init(void) {

    DL_TimerA_setClockConfig(
        PWM_RIGHT_INST, (DL_TimerA_ClockConfig *) &gPWM_RIGHTClockConfig);

    DL_TimerA_initPWMMode(
        PWM_RIGHT_INST, (DL_TimerA_PWMConfig *) &gPWM_RIGHTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_RIGHT_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_RIGHT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_RIGHT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, 3200, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_RIGHT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_RIGHT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, 3200, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_RIGHT_INST);


    
    DL_TimerA_setCCPDirection(PWM_RIGHT_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_LEFTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_LEFTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 3200,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_LEFT_init(void) {

    DL_TimerA_setClockConfig(
        PWM_LEFT_INST, (DL_TimerA_ClockConfig *) &gPWM_LEFTClockConfig);

    DL_TimerA_initPWMMode(
        PWM_LEFT_INST, (DL_TimerA_PWMConfig *) &gPWM_LEFTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_LEFT_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_LEFT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_LEFT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, 3200, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_LEFT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_LEFT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, 3200, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_LEFT_INST);


    
    DL_TimerA_setCCPDirection(PWM_LEFT_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (0.5 ms * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_0_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_0_INST);





}


static const DL_UART_Main_ClockConfig gUART_GYROClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_GYROConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_GYRO_init(void)
{
    DL_UART_Main_setClockConfig(UART_GYRO_INST, (DL_UART_Main_ClockConfig *) &gUART_GYROClockConfig);

    DL_UART_Main_init(UART_GYRO_INST, (DL_UART_Main_Config *) &gUART_GYROConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_GYRO_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_GYRO_INST, UART_GYRO_IBRD_32_MHZ_115200_BAUD, UART_GYRO_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_GYRO_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_GYRO_INST);
}
static const DL_UART_Main_ClockConfig gUART_BTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_BTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_BT_init(void)
{
    DL_UART_Main_setClockConfig(UART_BT_INST, (DL_UART_Main_ClockConfig *) &gUART_BTClockConfig);

    DL_UART_Main_init(UART_BT_INST, (DL_UART_Main_Config *) &gUART_BTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_BT_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_BT_INST, UART_BT_IBRD_32_MHZ_115200_BAUD, UART_BT_FBRD_32_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_BT_INST);
}

