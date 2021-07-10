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
#include "fsl_lpspi.h"

#include "FreeRTOS.h"
#include "task.h"

can_handler_data_t can_handler[] = {
	/* CAN0 */
	{
		.type	= TYPE_FLEXCAN,
		.addr	= LOCAL_EPT_ADDR + 1,
		.name	= "flexcan0_task",
		.active	= false,
		.is_canfd = true,

		.flexcan = {
			.base	= ADMA__CAN0,
		},

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},

	},
	/* CAN1 */
	{
		.type	= TYPE_FLEXCAN,
		.addr	= LOCAL_EPT_ADDR + 2,
		.name	= "flexcan1_task",
		.active	= false,
		.is_canfd = true,

		.flexcan = {
			.base	= ADMA__CAN1,
		},

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
		.type	= TYPE_FLEXCAN,
		.addr	= LOCAL_EPT_ADDR + 3,
		.name	= "flexcan2_task",
		.active	= false,
		.is_canfd = true,

		.flexcan = {
			.base	= ADMA__CAN2,
		},

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
	/* CAN3 */
	{
		.type	= TYPE_MCP2517FD,
		.addr	= LOCAL_EPT_ADDR + 4,
		.name	= "mcpcan0_task",
		.active	= false,
		.is_canfd = true,

		.mcp = {
			.base = ADMA__LPSPI0,
			.ncs = {
				.present = true,
				.active_low = true,
				.base = LSIO__GPIO1,
				.pin = 8U,
			},
		},

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},
	},
	/* CAN4 */
	{
		.type	= TYPE_MCP2517FD,
		.addr	= LOCAL_EPT_ADDR + 5,
		.name	= "mcpcan1_task",
		.active	= false,
		.is_canfd = true,

		.mcp = {
			.base = ADMA__LPSPI0,
			.ncs = {
				.present = true,
				.active_low = true,
				.base = LSIO__GPIO1,
				.pin = 7U,
			},
		},

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},
	},
	/* CAN5 */
	{
		.type	= TYPE_MCP2517FD,
		.addr	= LOCAL_EPT_ADDR + 6,
		.name	= "mcpcan2_task",
		.active	= false,
		.is_canfd = true,

		.mcp = {
			.base = ADMA__LPSPI0,
			.ncs = {
				.present = true,
				.active_low = true,
				.base = LSIO__GPIO4,
				.pin = 20U,
			},
		},

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},
	},
	/* CAN6 */
	{
		.type	= TYPE_MCP2517FD,
		.addr	= LOCAL_EPT_ADDR + 7,
		.name	= "mcpcan3_task",
		.active	= false,
		.is_canfd = true,

		.mcp = {
			.base = ADMA__LPSPI0,
			.ncs = {
				.present = true,
				.active_low = true,
				.base = LSIO__GPIO4,
				.pin = 19U,
			},
		},

		.led = {
			.present = false,
		},

		.stb = {
			.present = false,
		},
	}
};

int can_count(void)
{
	return ARRAY_SIZE(can_handler);
}

static void spi_init(LPSPI_Type *base)
{
	lpspi_master_config_t masterConfig;

	/*
	 * masterConfig.baudRate     = 500000;
	 * masterConfig.bitsPerFrame = 8;
	 * masterConfig.cpol         = kLPSPI_ClockPolarityActiveHigh;
	 * masterConfig.cpha         = kLPSPI_ClockPhaseFirstEdge;
	 * masterConfig.direction    = kLPSPI_MsbFirst;

	 * masterConfig.pcsToSckDelayInNanoSec        = 1000000000U / masterConfig->baudRate * 2U;
	 * masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / masterConfig->baudRate * 2U;
	 * masterConfig.betweenTransferDelayInNanoSec = 1000000000U / masterConfig->baudRate * 2U;

	 * masterConfig.whichPcs           = kLPSPI_Pcs0;
	 * masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

	 * masterConfig.pinCfg        = kLPSPI_SdiInSdoOut;
	 * masterConfig.dataOutConfig = kLpspiDataOutRetained;
	*/
	LPSPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate = 10000000;

	LPSPI_MasterInit(base, &masterConfig, EXAMPLE_SPI_CLK_SOURCE);
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

	if (sc_pm_set_resource_power_mode(ipc, SC_R_SPI_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on SPI0\r\n");
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

	/* SPI interface accessible through expansion connector */
	spi_init(ADMA__LPSPI0);
}


int8_t DRV_SPI_TransferData(uint8_t id, uint8_t *txd, uint8_t *rxd, uint16_t size)
{
	can_handler_data_t *handler = &can_handler[id];
	lpspi_transfer_t xfer;
	uint32_t wait = 0;
	status_t status;

	if (handler->type != TYPE_MCP2517FD)
	{
		PRINTF("%s: invalid can device: %d\r\n", __func__, handler->type);
		return MAKE_STATUS(kStatusGroup_Generic, 4);
	}

	xfer.configFlags = 0x0;
	xfer.dataSize = size;
	xfer.txData = txd;
	xfer.rxData = rxd;

	while ((LPSPI_GetStatusFlags(handler->mcp.base) & (uint32_t)kLPSPI_ModuleBusyFlag))
	{
		if (wait++ > 0xFF)
		{
			PRINTF("%s: warn: SPI busy...\r\n", __func__);
			break;
		}
	}

	GPIO_PinWrite(handler->mcp.ncs.base, handler->mcp.ncs.pin,
			handler->mcp.ncs.active_low ? 0U : 1U);

	status = LPSPI_MasterTransferBlocking(handler->mcp.base, &xfer);

	GPIO_PinWrite(handler->mcp.ncs.base, handler->mcp.ncs.pin,
			handler->mcp.ncs.active_low ? 1U : 0U);

	if (status != kStatus_Success)
	{
		PRINTF("%s: error: %d\r\n", __func__, status);
		return 1;
	}

	return 0;
}
