/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
#define BOARD_FTM_BASEADDR FTM0

/* Interrupt number and interrupt handler for the FTM instance used */
#define BOARD_FTM_IRQ_NUM FTM0_IRQn
#define BOARD_FTM_HANDLER FTM0_IRQHandler

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk) / 4)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag           = false;
volatile uint32_t milisecondCounts = 0U;
volatile bool g_encoder = false;
volatile bool stop = false;
int contador = 0;
int resol= 20;


/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_ENCODER_IRQ_HANDLER(void)
{
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_ENCODER_GPIO, 1U << BOARD_ENCODER_GPIO_PIN);
#else
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_ENCODER_GPIO, 1U << BOARD_ENCODER_GPIO_PIN);
#endif
    /* Change state of button. */
    g_encoder = true;
    contador++;
    SDK_ISR_EXIT_BARRIER;
}
void init_motor_calculate_speed(void)
{

    ftm_config_t ftmInfo;
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };


    /*Encoder*/
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    GPIO_SetPinInterruptConfig(BOARD_ENCODER_GPIO, BOARD_ENCODER_GPIO_PIN, kGPIO_InterruptFallingEdge);
#else
    PORT_SetPinInterruptConfig(BOARD_ENCODER_PORT, BOARD_ENCODER_GPIO_PIN, kPORT_InterruptEitherEdge);
#endif
    EnableIRQ(BOARD_ENCODER_IRQ);
    GPIO_PinInit(BOARD_ENCODER_GPIO, BOARD_ENCODER_GPIO_PIN, &sw_config);

    /* Print a note to terminal */
    PRINTF("\r\nFTM example to simulate a timer\r\n");
    PRINTF("\r\nYou will see a \"-\" or \"|\" in terminal every 1 second:\r\n");

    FTM_GetDefaultConfig(&ftmInfo);

    /* Divide FTM clock by 4 */
    ftmInfo.prescale = kFTM_Prescale_Divide_4;

    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    /*
     * Set timer period.
     */
    FTM_SetTimerPeriod(BOARD_FTM_BASEADDR, USEC_TO_COUNT(100000000000000U, FTM_SOURCE_CLOCK));

    FTM_EnableInterrupts(BOARD_FTM_BASEADDR, kFTM_TimeOverflowInterruptEnable);

    EnableIRQ(BOARD_FTM_IRQ_NUM);

    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);

}
int motor_speed (void){
	double rpm = 0;
    if (g_encoder == true && stop == false)
    {
    /* Reset state of button. */
    g_encoder = false;
    PRINTF("Contador : %d\n",contador);
    	}
    else if(ftmIsrFlag == true && stop == false){
    contador= contador/2;
    rpm = ((double)contador/resol);
    rpm = rpm*60;
    printf("\nvelocidad angular : %f",rpm);
    contador = 0;
    ftmIsrFlag = false;
    stop = true;
    }
return   contador;
}


void BOARD_FTM_HANDLER(void)
{
    /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, kFTM_TimeOverflowFlag);
    ftmIsrFlag = true;
    __DSB();
}
