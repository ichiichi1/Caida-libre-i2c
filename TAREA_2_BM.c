/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    TAREA_2_BM.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

volatile bool g_MasterCompletionFlag = false;
volatile bool pitIsrFlag = false;
volatile bool toggle = false;
float ejez;
float ejey;
float ejex;


static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

void PIT0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
}
/*
 * @brief   Application entry point.
 */
int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	CLOCK_EnableClock(kCLOCK_PortE);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

		pit_config_t pitConfig;
		PIT_GetDefaultConfig(&pitConfig);
		PIT_Init(PIT, &pitConfig);
		PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, (CLOCK_GetBusClkFreq()/4));


		PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
		EnableIRQ(PIT0_IRQn);

		PIT_StartTimer(PIT, kPIT_Chnl_0);

	port_pin_config_t config_led =
		{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
				kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
				kPORT_UnlockRegister, };

		//Led Azul CAIDA LIBRE
		PORT_SetPinConfig(PORTB, 21, &config_led);
		//Led Rojo MOVIMIENTO
		PORT_SetPinConfig(PORTB, 22, &config_led);
		//Led Verde INMOVIL
		PORT_SetPinConfig(PORTE, 26, &config_led);

		gpio_pin_config_t led_config_gpio =
			{ kGPIO_DigitalOutput, 1 };

			GPIO_PinInit(GPIOB, 21, &led_config_gpio);
			GPIO_PinInit(GPIOB, 22, &led_config_gpio);
			GPIO_PinInit(GPIOE, 26, &led_config_gpio);


	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt5,
	        kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTE, 24, &config_i2c);
	PORT_SetPinConfig(PORTE, 25, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle,
	        i2c_master_callback, NULL);

	i2c_master_transfer_t masterXfer;


	uint8_t data_buffer = 0x01;

#if 0
	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferNoStopFlag;

	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

	uint8_t read_data;

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = &read_data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferRepeatedStartFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;
#else

	masterXfer.slaveAddress = 0x1D;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x2A;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

#endif

	/* Force the counter to be placed into memory. */
	volatile static int i = 0;
	uint8_t buffer[6];
	int16_t accelerometer[3];
	/* Enter an infinite loop, just incrementing a counter. */
	while (1)
	{
		masterXfer.slaveAddress = 0x1D;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = 0x01;
		masterXfer.subaddressSize = 1;
		masterXfer.data = buffer;
		masterXfer.dataSize = 6;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
				&masterXfer);
		while (!g_MasterCompletionFlag){}
		g_MasterCompletionFlag = false;

		accelerometer[0] = buffer[0]<<8 | buffer[1];
		accelerometer[1] = buffer[2]<<8 | buffer[3];
		//eje Z
		accelerometer[2] = buffer[4]<<8 | buffer[5];

		ejez = (accelerometer[2]*.000244)/4;
		ejey = (accelerometer[2]*.000244)/4;
		ejex = (accelerometer[2]*.000244)/4;

		//Falta colocar lo parametros para determinar ne que stado esta

		if((ejez > 0 ) && (ejez < .2) && (pitIsrFlag == true) )
		{
         pitIsrFlag = false;
         if(false == toggle)
         {
         GPIO_WritePinOutput(GPIOB,22,0);
         toggle = true;
         }
         else
         {
         GPIO_WritePinOutput(GPIOB,22,1);
         toggle = false;
         }

		}

	}
	return 0;
}
