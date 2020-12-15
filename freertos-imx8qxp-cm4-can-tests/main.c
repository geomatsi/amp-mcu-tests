/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "rsc_table.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "compat_linux.h"
#include "can_linux.h"
#include "can-rpmsg-ipc.h"

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"
#include "fsl_gpio.h"

/* rpmsg definitions */

#define VIRTIO_CONFIG_S_DRIVER_OK	(4)
#define RPMSG_LITE_LINK_ID		(0)
#define RPMSG_LITE_SHMEM_BASE		(VDEV0_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING	"can-rpmsg-imx"
#define APP_TASK_STACK_SIZE		(256U)

#define CAN_RPMSG_MAXDEV		10

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30U)
#endif

/* flexcan definitions */

/*
 * When CLK_SRC=1, the protocol engine works at fixed frequency of 160M.
 * If other frequency wanted, please use CLK_SRC=0 and set the working frequency for SC_R_CAN_0.
 */
#define EXAMPLE_CAN_CLK_SOURCE (kFLEXCAN_ClkSrc1)
#define EXAMPLE_CAN_CLK_FREQ   (SC_160MHZ)

/* Considering that the first valid MB must be used as Reserved TX MB for ERR005641,
 * if RX FIFO enables (RFEN bit in MCE set as 1) and RFFN in CTRL2 is set default as zero,
 * the first valid TX MB Number shall be 8;
 * if RX FIFO enables (RFEN bit in MCE set as 1) and RFFN in CTRL2 is set by other values (0x1~0xF),
 * the user should consider to detail the first valid MB number;
 * if RX FIFO disables (RFEN bit in MCE set as 0) , the first valid MB number would be zero.
 */
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)

/* To get most precise baud rate under some circumstances, users need to set
   quantum which is composed of PSEG1/PSEG2/PROPSEG. Because CAN clock prescaler
   = source clock/(baud rate * quantum), for e.g. 84M clock and 1M baud rate, the
   quantum should be .e.g 14=(6+3+1)+4, so prescaler is 6. By default, quantum
   is set to 10=(3+2+1)+4, because for most platforms e.g. 120M source clock/(1M
   baud rate * 10) is an integer. Remember users must ensure the calculated
   prescaler an integer thus to get precise baud rate. */
#define SET_CAN_QUANTUM 1
#define PSEG1           6
#define PSEG2           4
#define PROPSEG         6
#define FPSEG1          6
#define FPSEG2          4
#define FPROPSEG        7

/* mgmt task data */

typedef struct mgmt_data {
	char name[32];
	uint32_t addr;
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	TaskHandle_t task;
} mgmt_data_t;

/* flexcan task data */

typedef struct gpio_out_pin {
	GPIO_Type *base;
	uint32_t pin;
} gpio_out_pin_t;

typedef struct flexcan_cb_t {
	bool txdone;
	bool rxdone;
	bool wakeup;
	bool failed;
	uint32_t errors;
	uint32_t rxflag;
	uint32_t txflag;
} flexcan_cb_t;

typedef struct flexcan_data {
	CAN_Type *base;
	char name[32];
	bool active;

	/* NXP FlexCAN HAL */
	flexcan_mb_transfer_t tx;
	flexcan_mb_transfer_t rx;
	flexcan_frame_t txframe; 
	flexcan_frame_t rxframe; 
	flexcan_handle_t handle;
	flexcan_cb_t cb;

	/* misc hardware controls */
	gpio_out_pin_t phy;
	gpio_out_pin_t led;

	/* RPMsg-Lite */
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	uint32_t addr;

	/* FreeRTOS */
	SemaphoreHandle_t runsem;
	TaskHandle_t task;

	/*
	 * Note that rx/tx are swapped for FlexCAN and rpmsg:
	 *   - rpmsg Rx becomes FlexCAN Tx
	 *   - FlexCAN Rx becomes rpmsg Tx
	 */
	void *rxbuf;
	void *txbuf;
} flexcan_data_t;

/* globals */ 

char mgmt_rx[RL_BUFFER_PAYLOAD_SIZE];
char mgmt_tx[RL_BUFFER_PAYLOAD_SIZE];
uint32_t remote_addr;

mgmt_data_t mgmt_handler = {
	.addr	= LOCAL_EPT_ADDR,
	.name = "mgmt_task",	
};

flexcan_data_t can_handler[] = {
	/* CAN0 */
	{
		.addr	= LOCAL_EPT_ADDR + 1,
		.name	= "can0_task",
		.base	= ADMA__CAN0,
		.active	= false,

		.led	= {
			.base	= LSIO__GPIO0,
			.pin	= 15U,
		},

		.phy	= {
			.base	= LSIO__GPIO4,
			.pin	= 20U,
		},
	},
	/* CAN1 */
	{
		.addr	= LOCAL_EPT_ADDR + 2,
		.name	= "can1_task",
		.base	= ADMA__CAN1,
		.active	= false,

		.led	= {
			.base	= LSIO__GPIO3,
			.pin	= 1U,
		},

		.phy	= {
			.base	= LSIO__GPIO1,
			.pin	= 25U,
		},
	},
};

/* callbacks */

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	struct flexcan_cb_t *data = (struct flexcan_cb_t *)userData;

	switch (status)
	{
		case kStatus_FLEXCAN_RxIdle:
			if (RX_MESSAGE_BUFFER_NUM == result)
			{
				data->rxdone = true;
			}
			break;

		case kStatus_FLEXCAN_RxOverflow:
			switch (result)
			{
				case RX_MESSAGE_BUFFER_NUM:
					data->rxflag = kStatus_FLEXCAN_RxOverflow;
					data->rxdone = true;
					break;
				default:
					(void)PRINTF("%s: overflow: status: %d result %u\r\n",
							__func__, status, result);
					break;
			}
			break;

		case kStatus_FLEXCAN_TxIdle:
			if (TX_MESSAGE_BUFFER_NUM == result)
			{
				data->txdone = true;
			}
			break;

		case kStatus_FLEXCAN_WakeUp:
			data->wakeup = true;
			break;

		case kStatus_FLEXCAN_ErrorStatus:
			data->errors = result;
			data->failed = true;
			break;

		case kStatus_Fail:
			switch (result)
			{
				case RX_MESSAGE_BUFFER_NUM:
					data->rxflag = kStatus_FLEXCAN_RxBusy;
					data->rxdone = true;
					break;
				default:
					(void)PRINTF("%s: failed: status: %d result %u\r\n",
							__func__, status, result);
					break;
			}
			break;

		default:
			(void)PRINTF("%s: FIXME: unknown status: %d\r\n", __func__, status);
			break;
	}
}

static void flexcan_init(CAN_Type *canbase, flexcan_handle_t *handle, flexcan_cb_t *cb)
{
	flexcan_rx_mb_config_t mbConfig;
	flexcan_config_t flexcanConfig;

	/* configure flexcan */

	/*
	 * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
	 * flexcanConfig.baudRate               = 1000000U;
	 * flexcanConfig.baudRateFD             = 2000000U;
	 * flexcanConfig.maxMbNum               = 16;
	 * flexcanConfig.enableLoopBack         = false;
	 * flexcanConfig.enableSelfWakeup       = false;
	 * flexcanConfig.enableIndividMask      = false;
	 * flexcanConfig.disableSelfReception   = false;
	 * flexcanConfig.enableListenOnlyMode   = false;
	 * flexcanConfig.enableDoze             = false;
	 */
	FLEXCAN_GetDefaultConfig(&flexcanConfig);

	flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
	flexcanConfig.disableSelfReception = true;

	/* If special quantum setting is needed, set the timing parameters. */
	flexcanConfig.timingConfig.propSeg   = PROPSEG;
	flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
	flexcanConfig.timingConfig.phaseSeg2 = PSEG2;

	FLEXCAN_Init(canbase, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(canbase, handle, flexcan_callback, (void *)cb);

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type   = kFLEXCAN_FrameTypeData;
	mbConfig.id     = FLEXCAN_ID_STD(0x0);

	FLEXCAN_SetRxMbConfig(canbase, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

	/* setup Rx global mask: accept all packets */
	FLEXCAN_SetRxMbGlobalMask(canbase, 0x0);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetTxMbConfig(canbase, TX_MESSAGE_BUFFER_NUM, true);
}

/* freertos tasks */

static void mgmt_task(void *param)
{
	const struct remote_resource_table *volatile rsc_table = get_rsc_table();
	const struct fw_rsc_vdev *volatile user_vdev = &rsc_table->user_vdev;
	const struct can_rpmsg_cmd *cmd = (struct can_rpmsg_cmd *)mgmt_rx;
	struct can_rpmsg_rsp *rsp = (struct can_rpmsg_rsp *)mgmt_tx;
	struct mgmt_data *priv = (struct mgmt_data *)param;
	flexcan_data_t *handler;
	uint32_t addr;
	uint32_t size;
	int32_t ret;

	(void)PRINTF("%s: start...\r\n", priv->name);

	while (0 == rpmsg_lite_is_link_up(priv->rpmsg))
	{
		if (user_vdev->status & VIRTIO_CONFIG_S_DRIVER_OK)
		{
			priv->rpmsg->link_state = 1U;
		}
	}

	(void)PRINTF("%s: link is up...\r\n", priv->name);

	/* announce remote to master */

	(void)rpmsg_ns_bind(priv->rpmsg, app_nameservice_isr_cb, ((void *)0));
	SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	(void)rpmsg_ns_announce(priv->rpmsg, priv->ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
	(void)PRINTF("%s: nameservice announce sent...\r\n", priv->name);

	/* process mgmt tasks */

	while (1)
	{
		memset(mgmt_rx, 0x0, sizeof(mgmt_rx));
		memset(mgmt_tx, 0x0, sizeof(mgmt_tx));

		(void)rpmsg_queue_recv(priv->rpmsg, priv->queue, &addr, mgmt_rx, sizeof(mgmt_rx), ((void *)0), RL_BLOCK);

		if (!remote_addr)
		{
			remote_addr = addr;
		}

		if (addr != remote_addr)
		{
			(void)PRINTF("%s: unexpected remote addr: %u != %u\r\n", priv->name, addr, remote_addr);
			continue;
		}

		if (cmd->hdr.type != CAN_RPMSG_CTRL_CMD)
		{
			(void)PRINTF("%s: unexpected ctrl type: 0x%x\r\n", priv->name, cmd->hdr.type);
			continue;
		}

		(void)PRINTF("%s: cmd 0x%x seq %u\r\n", priv->name, cmd->id, cmd->seq);

		switch (cmd->id)
		{
			case CAN_RPMSG_CMD_INIT:
				{
					const struct can_rpmsg_cmd_init *c = (const struct can_rpmsg_cmd_init *)cmd;
					struct can_rpmsg_cmd_init_rsp *r = (struct can_rpmsg_cmd_init_rsp *)rsp;
					size = sizeof(struct can_rpmsg_cmd_init_rsp);

					(void)PRINTF("%s: init: major(%u) minor(%u)\r\n", priv->name, c->major, c->minor);

					r->hdr.hdr.type = CAN_RPMSG_CTRL_RSP;
					r->hdr.hdr.len = sizeof(struct can_rpmsg_cmd_init_rsp);
					r->hdr.seq = c->hdr.seq;
					r->hdr.id = c->hdr.id;
					r->hdr.result = 0x0;

					r->bitrate = EXAMPLE_CAN_CLK_FREQ;
					r->major = CM4_MAJOR_VER;
					r->minor = CM4_MINOR_VER;
					r->devnum = 0x2;
				}

				break;
			case CAN_RPMSG_CMD_UP:
				{
					const struct can_rpmsg_cmd_up *c = (const struct can_rpmsg_cmd_up *)cmd;
					size = sizeof(struct can_rpmsg_rsp);

					(void)PRINTF("%s: up: index(%u)\r\n", priv->name, c->index);

					rsp->hdr.type = CAN_RPMSG_CTRL_RSP;
					rsp->hdr.len = sizeof(struct can_rpmsg_rsp);
					rsp->seq = c->hdr.seq;
					rsp->id = c->hdr.id;

					if (c->index >= sizeof(can_handler))
					{
						rsp->result = -ENODEV;
						break;
					}

					handler = &can_handler[c->index];

					if (handler->active)
					{
						rsp->result = -EALREADY;
						break;
					}

					if (xSemaphoreGive(handler->runsem) != pdTRUE )
					{
						(void)PRINTF("%s: failed to give semaphore\r\n", __func__);
					}

					flexcan_init(handler->base, (flexcan_handle_t *)&handler->handle,
							(flexcan_cb_t *)&handler->cb);
					GPIO_PinWrite(handler->phy.base, handler->phy.pin, 0U);
					handler->active = true;
					rsp->result = 0x0;
				}

				break;
			case CAN_RPMSG_CMD_DOWN:
				{
					const struct can_rpmsg_cmd_down *c = (const struct can_rpmsg_cmd_down *)cmd;
					size = sizeof(struct can_rpmsg_rsp);

					(void)PRINTF("%s: down: index(%u)\r\n", priv->name, c->index);

					rsp->hdr.type = CAN_RPMSG_CTRL_RSP;
					rsp->hdr.len = sizeof(struct can_rpmsg_rsp);
					rsp->seq = c->hdr.seq;
					rsp->id = c->hdr.id;

					if (c->index >= CAN_RPMSG_MAXDEV)
					{
						rsp->result = -ENODEV;
						break;
					}

					handler = &can_handler[c->index];

					if (!handler->active)
					{
						rsp->result = -EALREADY;
						break;
					}

					if (xSemaphoreTake(handler->runsem, portMAX_DELAY) != pdTRUE)
					{
						(void)PRINTF("%s: failed to give semaphore\r\n", __func__);
					}

					GPIO_PinWrite(handler->phy.base, handler->phy.pin, 1U);
					FLEXCAN_Deinit(handler->base);
					handler->active = false;
					rsp->result = 0x0;
				}

				break;
			default:
				{
					size = sizeof(struct can_rpmsg_rsp);

					rsp->hdr.type = CAN_RPMSG_CTRL_RSP;
					rsp->hdr.len = sizeof(struct can_rpmsg_rsp);
					rsp->seq = cmd->seq;
					rsp->id = cmd->id;
					rsp->result = -ENOSYS;

					(void)PRINTF("%s: unkonwn cmd: %d\r\n", priv->name, cmd->id);
				}

				break;
		}

		ret = rpmsg_lite_send(priv->rpmsg, priv->ept, remote_addr, (char *)rsp, size, 10);
		if (ret != RL_SUCCESS)
		{
			(void)PRINTF("%s: failed to send response: %d\r\n", priv->name, ret);
		}
	}

	(void)PRINTF("%s: done...\r\n", priv->name);

	while (1)
	{
	}
}

static void can_task(void *param)
{
	struct flexcan_data *priv = (struct flexcan_data *)param;
	status_t  status;
	uint32_t txlen;
	uint32_t rxlen;
	uint32_t addr;

	(void)PRINTF("%s: start...\r\n", priv->name);

	while (1)
	{
		if (xSemaphoreTake(priv->runsem, 0) != pdTRUE )
		{
			// Failed to take semaphore: host attempts to close CAN device

			(void)PRINTF("%s: ready to wait\r\n", priv->name);

			// Tx cleanup:

			priv->cb.txdone = false;
			priv->tx.frame = NULL;
			priv->rxbuf = NULL;

			// Rx cleanup:

			priv->cb.rxdone = false;
			priv->rx.frame = NULL;
			priv->cb.rxflag = 0;

			// Block until CAN device is re-opened

			if (xSemaphoreTake(priv->runsem, portMAX_DELAY) != pdTRUE )
			{
				(void)PRINTF("%s: failed to take semaphore\r\n", priv->name);
			}
		}

		if (priv->cb.failed)
		{
			(void)PRINTF("%s: errors (%u)\r\n", priv->cb.errors);
			priv->cb.failed = false;
			priv->cb.errors = 0x0;
		}

		if (priv->cb.txdone)
		{
			if (priv->rxbuf)
			{
				GPIO_PinWrite(priv->led.base, priv->led.pin, 0U);
			}

			priv->cb.txdone = false;
			priv->tx.frame = NULL;
			priv->rxbuf = NULL;
		}
		else
		{
			if (!priv->tx.frame)
			{
				if (!priv->rxbuf)
				{
					(void)rpmsg_queue_recv_nocopy(priv->rpmsg, priv->queue, (uint32_t *)&addr, (char **)&priv->rxbuf, &rxlen, 0);
					if (priv->rxbuf)
					{
						m2r(&priv->txframe, (struct can_frame *)priv->rxbuf);
						(void)rpmsg_lite_release_rx_buffer(priv->rpmsg, priv->rxbuf);
					}
				}

				if (priv->rxbuf)
				{
					priv->tx.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
					priv->tx.frame = &priv->txframe;
					priv->cb.txdone = false;

					status = FLEXCAN_TransferSendNonBlocking(priv->base, &priv->handle, &priv->tx);
					if (status != kStatus_Success)
					{
						(void)PRINTF("%s: failed to prepare xmit len %d: %d\r\n", priv->name, rxlen, status);
						priv->tx.frame = NULL;
					} else {
						GPIO_PinWrite(priv->led.base, priv->led.pin, 1U);
					}
				}
			}
		}

		if (priv->cb.rxdone)
		{
			switch (priv->cb.rxflag)
			{
				case kStatus_FLEXCAN_RxOverflow:
					FLEXCAN_ClearMbStatusFlags(priv->base,
								kFLEXCAN_RxFifoOverflowFlag);
					/* drop fame */
					priv->rx.frame = NULL;
					break;
				case kStatus_FLEXCAN_RxBusy:
					/* drop fame */
					priv->rx.frame = NULL;
					break;
				default:
					/* normal reception */
					r2m((struct can_frame *)priv->txbuf, priv->rx.frame);
					rpmsg_lite_send_nocopy(priv->rpmsg, priv->ept,
								remote_addr, priv->txbuf,
								sizeof(struct can_frame));
					priv->rx.frame = NULL;
					priv->txbuf = NULL;
					break;
			}

			priv->cb.rxdone = false;
			priv->cb.rxflag = 0;
		}
		else
		{
			if (!priv->rx.frame)
			{
				if (!priv->txbuf)
				{
					priv->txbuf = rpmsg_lite_alloc_tx_buffer(priv->rpmsg, &txlen, 0);
					if (!priv->txbuf)
					{
						(void)PRINTF("%s: failed to alloc rx buffer...\r\n", priv->name);
					}
				}

				if (priv->txbuf)
				{
					priv->rx.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
					priv->rx.frame = &priv->rxframe;
					priv->cb.rxdone = false;

					status = FLEXCAN_TransferReceiveNonBlocking(priv->base, &priv->handle, &priv->rx);
					if (status != kStatus_Success) {
						(void)PRINTF("%s: failed to prepare rx len %d: %d\r\n", priv->name, txlen, status);
						priv->rx.frame = NULL;
					}
				}
			}
		}

		if (xSemaphoreGive(priv->runsem) != pdTRUE )
		{
			(void)PRINTF("%s: failed to give semaphore\r\n", priv->name);
		}

		taskYIELD();
	}

	(void)PRINTF("%s: done...\r\n", priv->name);

	while (1)
	{
	}
}

/* main */

int main(void)
{
	gpio_pin_config_t phy = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
	gpio_pin_config_t led = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
	sc_ipc_t ipc = BOARD_InitRpc();
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	SemaphoreHandle_t mutex;
	int i;

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
		PRINTF("Error: Failed to power on FLEXCAN\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_CAN_1, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on FLEXCAN\r\n");
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
		PRINTF("Error: Failed to power on GPIO_3!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_3, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_3!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_GPIO_4, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on GPIO_3!\r\n");
	}

	IRQSTEER_Init(IRQSTEER);
	NVIC_EnableIRQ(IRQSTEER_4_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, LSIO_MU8_INT_B_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN0_INT_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN1_INT_IRQn);

	/*
	 * rpmsg init
	 *
	 */

	(void)PRINTF("RPMSG shared base addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
	rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	if (rpmsg == RL_NULL)
	{
		(void)PRINTF("failed to init rpmsg...\r\n");
		while (1)
		{
		}
	}

	/* mgmt */

	queue = rpmsg_queue_create(rpmsg);
	if (queue == RL_NULL)
	{
		(void)PRINTF("%s: failed to allocate queue...\r\n", mgmt_handler.name);
		while (1)
		{
		}
	}

	ept = rpmsg_lite_create_ept(rpmsg, mgmt_handler.addr, rpmsg_queue_rx_cb, queue);
	if (ept == RL_NULL)
	{
		(void)PRINTF("%s: failed to allocate ept...\r\n", mgmt_handler.name);
		while (1)
		{
		}
	}

	mgmt_handler.rpmsg = rpmsg;
	mgmt_handler.queue = queue;
	mgmt_handler.ept = ept;

	if (xTaskCreate(mgmt_task, "mgmt_task", APP_TASK_STACK_SIZE, &mgmt_handler, tskIDLE_PRIORITY + 2U, &mgmt_handler.task) != pdPASS)
	{
		(void)PRINTF("failed to create mgmt task\r\n");
		while (1)
		{
		}
	}

	/* can handlers */

	for (i = 0; i < ARRAY_SIZE(can_handler); i++)
	{
		queue = rpmsg_queue_create(rpmsg);
		if (queue == RL_NULL)
		{
			(void)PRINTF("%s: failed to allocate queue...\r\n", can_handler[i].name);
			while (1)
			{
			}
		}

		ept = rpmsg_lite_create_ept(rpmsg, can_handler[i].addr, rpmsg_queue_rx_cb, queue);
		if (ept == RL_NULL)
		{
			(void)PRINTF("%s: failed to allocate ept...\r\n", can_handler[i].name);
			while (1)
			{
			}
		}

		mutex = xSemaphoreCreateMutex();
		if(mutex == NULL)
		{
			(void)PRINTF("%s: failed to allocate semaphore...\r\n", can_handler[i].name);
			while (1)
			{
			}
		}

		if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		{
			(void)PRINTF("%s: failed to take semaphore\r\n", can_handler[i].name);
			while (1)
			{
			}
		}


		can_handler[i].runsem = mutex;
		can_handler[i].rpmsg = rpmsg;
		can_handler[i].queue = queue;
		can_handler[i].ept = ept;

		flexcan_init(can_handler[i].base, (flexcan_handle_t *)&can_handler[i].handle,
				(flexcan_cb_t *)&can_handler[i].cb);
		GPIO_PinInit(can_handler[i].phy.base, can_handler[i].phy.pin, &phy);
		GPIO_PinInit(can_handler[i].led.base, can_handler[i].led.pin, &led);

		if (xTaskCreate(can_task, can_handler[i].name, APP_TASK_STACK_SIZE, &can_handler[i], tskIDLE_PRIORITY + 1U, &can_handler[i].task) != pdPASS)
		{
			(void)PRINTF("%s: failed to create task\r\n", can_handler[i].name);
			while (1)
			{
			}
		}
	}

	/* */

	vTaskStartScheduler();

	(void)PRINTF("Failed to start FreeRTOS\r\n");
	while (1)
	{
	}
}
