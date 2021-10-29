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
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO0,
			.pin = 15U,
		},

		.stb = {
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO4,
			.pin = 20U,
		},
	},
	/* CAN1 */
	{
		.addr	= LOCAL_EPT_ADDR + 2,
		.name	= "can1_task",
		.base	= ADMA__CAN1,
		.active	= false,

		.led = {
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO3,
			.pin = 1U,
		},

		.stb = {
			.present = true,
			.active_low = false,
			.base = LSIO__GPIO1,
			.pin = 25U,
		},
	},
};

int flexcan_count(void)
{
	return ARRAY_SIZE(can_handler);
}

void board_hw_init(void)
{
	sc_ipc_t ipc = BOARD_InitRpc();

	BOARD_InitPins(ipc);
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitMemory();

	/* power on peripherals */

	/* Note : If other CAN instances are used, SC_R_CAN_0 should still be powered because
	 * all CAN instances share the same clock source from CAN0 in 8QX
	 *
	 */

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN#0\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN#1\r\n");
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
}
