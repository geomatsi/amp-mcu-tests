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
#include "timers.h"
#include "task.h"

#include "common.h"
#include "rsc_table.h"

#include "compat_linux.h"
#include "can-rpmsg-ipc.h"

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"
#include "fsl_gpio.h"

/* globals */ 

char mgmt_rx[RL_BUFFER_PAYLOAD_SIZE];
char mgmt_tx[RL_BUFFER_PAYLOAD_SIZE];
char stats[512];

uint32_t remote_addr;

mgmt_data_t mgmt_handler = {
	.addr	= LOCAL_EPT_ADDR,
	.name = "mgmt_task",	
};

extern can_handler_data_t can_handler[];

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
	can_handler_data_t *handler;
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

					(void)PRINTF("%s: master: major(%u) minor(%u)\r\n", priv->name, c->major, c->minor);

					r->hdr.hdr.type = CAN_RPMSG_CTRL_RSP;
					r->hdr.hdr.len = sizeof(struct can_rpmsg_cmd_init_rsp);
					r->hdr.seq = c->hdr.seq;
					r->hdr.id = c->hdr.id;
					r->hdr.result = 0x0;

					r->devnum = can_count();
					r->bitrate = EXAMPLE_CAN_BITRATE;
					r->major = CM4_MAJOR_VER;
					r->minor = CM4_MINOR_VER;

					(void)PRINTF("%s: remote: major(%u) minor(%u) devices(%u)\r\n",
							priv->name, r->major, r->minor, r->devnum);
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

					if (c->index >= can_count())
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

					/*
					flexcan_init(handler->base, (flexcan_handle_t *)&handler->handle,
							(flexcan_cb_t *)&handler->cb);
					*/

					if (handler->stb.present)
					{
						GPIO_PinWrite(handler->stb.base, handler->stb.pin,
								handler->stb.active_low ? 1U : 0U);
					}

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

					if (c->index >= can_count())
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

					if (handler->stb.present)
					{
						GPIO_PinWrite(handler->stb.base, handler->stb.pin,
								handler->stb.active_low ? 0U : 1U);
					}

					/*
					FLEXCAN_Deinit(handler->base);
					*/
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

static void flexcan_task(void *param)
{
	can_handler_data_t *priv = (struct can_handler_data *)param;
	status_t  status;
	uint32_t txlen;
	uint32_t rxlen;
	uint32_t addr;

	if (priv->type != TYPE_FLEXCAN)
	{
		PRINTF("%s: invalid can device type: %d\r\n", priv->name, priv->type);
		while (1)
		{
		}
	}

	(void)PRINTF("%s (%d): init...\r\n", priv->name, priv->id);

	while (1)
	{
		if (xSemaphoreTake(priv->runsem, 0) != pdTRUE )
		{
			// Failed to take semaphore: host attempts to close CAN device

			(void)PRINTF("%s: ready to wait\r\n", priv->name);

			// Tx cleanup:

			priv->flexcan.cb.txdone = false;
			priv->flexcan.tx.frame = NULL;
			priv->rxbuf = NULL;

			// Rx cleanup:

			priv->flexcan.cb.rxdone = false;
			priv->flexcan.rx.frame = NULL;
			priv->flexcan.cb.rxflag = 0;

			// Block until CAN device is re-opened

			if (xSemaphoreTake(priv->runsem, portMAX_DELAY) != pdTRUE )
			{
				(void)PRINTF("%s: failed to take semaphore\r\n", priv->name);
			}

			(void)PRINTF("%s: start...\r\n", priv->name);
		}

		if (priv->flexcan.cb.failed)
		{
			(void)PRINTF("%s: errors (%u)\r\n", priv->name, priv->flexcan.cb.errors);
			priv->flexcan.cb.failed = false;
			priv->flexcan.cb.errors = 0x0;
		}

		if (priv->flexcan.cb.txdone)
		{
			if (priv->rxbuf)
			{
				if (priv->led.present)
				{
					GPIO_PinWrite(priv->led.base, priv->led.pin,
							priv->led.active_low ? 1U : 0U);
				}
			}

			priv->flexcan.cb.txdone = false;
			priv->flexcan.tx.frame = NULL;
			priv->rxbuf = NULL;
		}
		else
		{
			if (!priv->flexcan.tx.frame)
			{
				if (!priv->rxbuf)
				{
					(void)rpmsg_queue_recv_nocopy(priv->rpmsg, priv->queue, (uint32_t *)&addr, (char **)&priv->rxbuf, &rxlen, 0);
					if (priv->rxbuf)
					{
						to_flexcan(&priv->flexcan.txframe, (struct can_frame *)priv->rxbuf);
						(void)rpmsg_lite_release_rx_buffer(priv->rpmsg, priv->rxbuf);
					}
				}

				if (priv->rxbuf)
				{
					priv->flexcan.tx.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
					priv->flexcan.tx.frame = &priv->flexcan.txframe;
					priv->flexcan.cb.txdone = false;

					status = FLEXCAN_TransferSendNonBlocking(priv->flexcan.base, &priv->flexcan.handle, &priv->flexcan.tx);
					if (status != kStatus_Success)
					{
						(void)PRINTF("%s: failed to prepare xmit len %d: %d\r\n", priv->name, rxlen, status);
						priv->flexcan.tx.frame = NULL;
					} else {
						if (priv->led.present)
						{
							GPIO_PinWrite(priv->led.base, priv->led.pin,
									priv->led.active_low ? 0U : 1U);
						}
					}
				}
			}
		}

		if (priv->flexcan.cb.rxdone)
		{
			switch (priv->flexcan.cb.rxflag)
			{
				case kStatus_FLEXCAN_RxOverflow:
					FLEXCAN_ClearMbStatusFlags(priv->flexcan.base,
								kFLEXCAN_RxFifoOverflowFlag);
					/* drop fame */
					priv->flexcan.rx.frame = NULL;
					break;
				case kStatus_FLEXCAN_RxBusy:
					/* drop fame */
					priv->flexcan.rx.frame = NULL;
					break;
				default:
					/* normal reception */
					from_flexcan((struct can_frame *)priv->txbuf, priv->flexcan.rx.frame);
					rpmsg_lite_send_nocopy(priv->rpmsg, priv->ept,
								remote_addr, priv->txbuf,
								sizeof(struct can_frame));
					priv->flexcan.rx.frame = NULL;
					priv->txbuf = NULL;
					break;
			}

			priv->flexcan.cb.rxdone = false;
			priv->flexcan.cb.rxflag = 0;
		}
		else
		{
			if (!priv->flexcan.rx.frame)
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
					priv->flexcan.rx.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
					priv->flexcan.rx.frame = &priv->flexcan.rxframe;
					priv->flexcan.cb.rxdone = false;

					status = FLEXCAN_TransferReceiveNonBlocking(priv->flexcan.base, &priv->flexcan.handle, &priv->flexcan.rx);
					if (status != kStatus_Success) {
						(void)PRINTF("%s: failed to prepare rx len %d: %d\r\n", priv->name, txlen, status);
						priv->flexcan.rx.frame = NULL;
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

void stats_task(TimerHandle_t xTimer)
{
	 vTaskGetRunTimeStats(stats);
	 (void)PRINTF("%s\n", stats);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
	(void)PRINTF("%s: stack overflow in %s task...\r\n", __func__, pcTaskName);
	while (1)
	{
	}
}

/* main */

int main(void)
{
	gpio_pin_config_t H = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
	gpio_pin_config_t L = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	SemaphoreHandle_t mutex;
	TimerHandle_t timer;
	int i;

	/* hardware init */

	board_hw_init();

	/* rpmsg init */

	(void)PRINTF("RPMSG shared base addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
	rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	if (rpmsg == RL_NULL)
	{
		(void)PRINTF("failed to init rpmsg...\r\n");
		while (1)
		{
		}
	}

	/* stats timer */

	timer = xTimerCreate("stats_timer", 10000, pdTRUE, (void *)0, stats_task);
	if (timer == NULL)
	{
		(void)PRINTF("failed to create stats timer\r\n");
		while (1)
		{
		}
	}

	if( xTimerStart(timer, 0) != pdPASS )
	{
		(void)PRINTF("failed to start stats timer\r\n");
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

	for (i = 0; i < can_count(); i++)
	{
		can_handler[i].id = i;

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

		if (can_handler[i].stb.present)
		{
			/* init and _enable_ xciever stand-by pin */
			GPIO_PinInit(can_handler[i].stb.base, can_handler[i].stb.pin,
					can_handler[i].stb.active_low ? &L : &H);
		}

		if (can_handler[i].led.present)
		{
			/* init and _disable_ activity LED */
			GPIO_PinInit(can_handler[i].led.base, can_handler[i].led.pin,
					can_handler[i].led.active_low ? &H : &L);
		}

		switch (can_handler[i].type)
		{
		case TYPE_FLEXCAN:
			flexcan_init(can_handler[i].flexcan.base, (flexcan_handle_t *)&can_handler[i].flexcan.handle,
				(flexcan_cb_t *)&can_handler[i].flexcan.cb);

			if (xTaskCreate(flexcan_task, can_handler[i].name, APP_TASK_STACK_SIZE, &can_handler[i], tskIDLE_PRIORITY + 1U, &can_handler[i].task) != pdPASS)
			{
				(void)PRINTF("%s: failed to create task\r\n", can_handler[i].name);
				while (1)
				{
				}
			}
			break;
		default:
			(void)PRINTF("%s: unexpected can_handler type: %d\r\n", can_handler[i].name, can_handler[i].type);
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
