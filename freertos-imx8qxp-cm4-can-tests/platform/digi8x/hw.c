/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "pin_mux.h"
#include "common.h"

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"
#include "fsl_gpio.h"

flexcan_data_t can_handler[] = {
	/* CAN0 */
	{
		.addr	= LOCAL_EPT_ADDR + 1,
		.name	= "can0_task",
		.base	= ADMA__CAN0,
		.active	= false,

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},
	},
	/* CAN1 */
	{
		.addr	= LOCAL_EPT_ADDR + 2,
		.name	= "can1_task",
		.base	= ADMA__CAN1,
		.active	= false,

		.led = {
			.present = false,
		},

		.stb = {
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO1,
			.pin = 0U,
		},
	},
	/* CAN2 */
	{
		.addr	= LOCAL_EPT_ADDR + 3,
		.name	= "can2_task",
		.base	= ADMA__CAN2,
		.active	= false,

		.led = {
			.present = false,
		},

		.stb = {
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO1,
			.pin = 2U,
		},
	},

};

int flexcan_count(void)
{
	return ARRAY_SIZE(can_handler);
}

void board_hw_init(void)
{
	gpio_pin_config_t H = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
	sc_ipc_t ipc = BOARD_InitRpc();

	BOARD_InitPins(ipc);
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitMemory();

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN#0\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN#1\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_2, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN#2\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_5B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on MU_5B!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on IRQSTEER!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_8B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on MU_8B!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_0!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_1!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_2, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_2!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_3, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_3!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_4, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_4!\r\n");
	}

	IRQSTEER_Init(IRQSTEER);
	NVIC_EnableIRQ(IRQSTEER_4_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, LSIO_MU8_INT_B_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN0_INT_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN1_INT_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN2_INT_IRQn);

	/* CAN3/RS485 switch: enable CAN3 */
	GPIO_PinInit(LSIO__GPIO3, 23, &H);
}