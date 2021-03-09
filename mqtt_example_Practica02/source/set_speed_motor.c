/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_ftm.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer instance/channel used for board */
#define BOARD_FTM_BASEADDR       FTM0
#define BOARD_FIRST_FTM_CHANNEL  0U
#define BOARD_SECOND_FTM_CHANNEL 1U

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
extern QueueHandle_t speed_por_queue;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t updatedDutycycle = 0U;
/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0U;
    for (i = 0U; i < 800000U; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*!
 * @brief Main function
 */
void Init_set_motor_speed(void)
{

    ftm_config_t ftmInfo;

    ftm_chnl_pwm_signal_param_t ftmParam;

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber            = (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL;
    ftmParam.level                 = kFTM_LowTrue;
    ftmParam.dutyCyclePercent      = 0U;
    ftmParam.firstEdgeDelayPercent = 0U;
    ftmParam.enableDeadtime        = false;



    /* Board pin, clock, debug console init */
    /*BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();*/

    /* Print a note to terminal */
    /*PRINTF("\r\nFTM example to output PWM on 2 channels\r\n");
    PRINTF("\r\nYou will see a change in LED brightness if an LED is connected to the FTM pin");
    PRINTF("\r\nIf no LED is connected to the FTM pin, then probe the signal using an oscilloscope");*/

    /*
     * ftmInfo.prescale = kFTM_Prescale_Divide_1;
     * ftmInfo.bdmMode = kFTM_BdmMode_0;
     * ftmInfo.pwmSyncMode = kFTM_SoftwareTrigger;
     * ftmInfo.reloadPoints = 0;
     * ftmInfo.faultMode = kFTM_Fault_Disable;
     * ftmInfo.faultFilterValue = 0;
     * ftmInfo.deadTimePrescale = kFTM_Deadtime_Prescale_1;
     * ftmInfo.deadTimeValue = 0;
     * ftmInfo.extTriggers = 0;
     * ftmInfo.chnlInitState = 0;
     * ftmInfo.chnlPolarity = 0;
     * ftmInfo.useGlobalTimeBase = false;
     */
    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 24000U, FTM_SOURCE_CLOCK);
    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
}
static void set_motor_speed(char *value)
    {
        /* Delay to see the change of LEDs brightness */
       // delay();
        /* Start PWM mode with updated duty cycle */

		int duty = atoi(value);
		duty = 100 -duty;
		updatedDutycycle = (uint8_t)duty;

       /* FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, (ftm_chnl_t)BOARD_FIRST_FTM_CHANNEL, kFTM_EdgeAlignedPwm,
                               updatedDutycycle);*/
        FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL, kFTM_EdgeAlignedPwm,
        		updatedDutycycle);
        /* Software trigger to update registers */
        FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);
    }

void setting_speed(void)
{
	char cmd[2+1];


		if (xQueueReceive(speed_por_queue, cmd, portMAX_DELAY) != errQUEUE_EMPTY)
		{
			set_motor_speed(cmd);
		}

}
void turn_on_pwm(void){

	if(updatedDutycycle<30){
		updatedDutycycle = 30;
	}
	  FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL, kFTM_EdgeAlignedPwm,
	        updatedDutycycle);
	   /* Software trigger to update registers */
	   FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);

}
void turn_off_pwm(void){
	uint8_t turn_off = 100U;
	FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, (ftm_chnl_t)BOARD_SECOND_FTM_CHANNEL, kFTM_EdgeAlignedPwm,
			turn_off);
	/* Software trigger to update registers */
	FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);
}


