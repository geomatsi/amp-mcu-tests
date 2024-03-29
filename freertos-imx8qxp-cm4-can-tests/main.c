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
#include "compat_linux.h"
#include "rsc_table.h"
#include "can-rpmsg-ipc.h"

#include "flexcan_compat.h"
#include "mcpcan_compat.h"

/* globals */ 

extern can_handler_data_t can_handler[];
 
gpio_pin_config_t H = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
gpio_pin_config_t L = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
uint32_t remote_addr = 0xbeef;

/* */

static char stats[512];

static mgmt_data_t cmd_handler = {
	.name = "cmd_task",
};

static mgmt_data_t evt_handler = {
	.name = "evt_task",
};
 
/* callbacks */

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

/* control path interrupt */

int32_t LSIO_MU13_INT_B_IRQHandler(void)
{
	uint32_t signal;
	uint32_t type;
	size_t size;

	if ((((1UL << 27U) >> CTRL_MU_CHAN) & MU_GetStatusFlags(LSIO__MU13_B)) == 0UL)
	{
		return 0;
	}

	signal = MU_ReceiveMsgNonBlocking(LSIO__MU13_B, CTRL_MU_CHAN);
	can_rpmsg_from_sig(signal, &type, &size);
	(void)PRINTF("%s: signal: type %u len %u\r\n", __func__, type, size);

	switch (type)
	{
	case CAN_RPMSG_CTRL_CMD:
		if (cmd_handler.cmd.cmdq)
		{
			if ( pdTRUE != xQueueSendFromISR( cmd_handler.cmd.cmdq, &size, NULL ))
			{
				(void)PRINTF("%s: failed to handle signal 0x%x\r\n", __func__, signal);
			}
		}
		break;
	case CAN_RPMSG_CTRL_ACK:
		if (evt_handler.evt.ackq)
		{
			if ( pdTRUE != xQueueSendFromISR( evt_handler.evt.ackq, &size, NULL ))
			{
				(void)PRINTF("%s: failed to handle signal 0x%x\r\n", __func__, signal);
			}
		}
		break;
	default:
		(void)PRINTF("%s: unexpected signal: 0x%x\r\n", __func__, signal);
		break;
	}

	return 0;
}

/* mgmt: commands */

static void cmd_task(void *param)
{
	const struct can_rpmsg_cmd *cmd = (struct can_rpmsg_cmd *)CTRL_MEM_REQ;
	struct can_rpmsg_rsp *rsp = (struct can_rpmsg_rsp *)CTRL_MEM_RSP;
	struct mgmt_data *priv = (struct mgmt_data *)param;
	can_handler_data_t *handler;
	uint32_t size;
	uint32_t msg;
	int32_t ret;

	(void)PRINTF("%s: start...\r\n", priv->name);

	while (1)
	{
		if (xQueueReceive(priv->cmd.cmdq, &msg, portMAX_DELAY) == pdFALSE)
		{
			(void)PRINTF("%s: failed to recieve command...\r\n", priv->name);
			continue;
		}

		memset((void *)rsp, 0x0, CTRL_MEM_SIZE);

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

					(void)PRINTF("%s: master: major(%u) minor(%u) addr(0x%x)\r\n",
						priv->name, c->major, c->minor, c->addr);
					remote_addr = c->addr;

					r->hdr.hdr.type = CAN_RPMSG_CTRL_RSP;
					r->hdr.hdr.len = sizeof(struct can_rpmsg_cmd_init_rsp);
					r->hdr.seq = c->hdr.seq;
					r->hdr.id = c->hdr.id;
					r->hdr.result = 0x0;

					r->devnum = can_count();
					r->major = CM4_MAJOR_VER;
					r->minor = CM4_MINOR_VER;

					(void)PRINTF("%s: remote: major(%u) minor(%u) devices(%u)\r\n",
							priv->name, r->major, r->minor, r->devnum);
				}

				break;
			case CAN_RPMSG_CMD_GET_CFG:
				{
					const struct can_rpmsg_cmd_get_cfg *c = (const struct can_rpmsg_cmd_get_cfg *)cmd;
					struct can_rpmsg_cmd_get_cfg_rsp *r = (struct can_rpmsg_cmd_get_cfg_rsp *)rsp;
					size = sizeof(struct can_rpmsg_cmd_get_cfg_rsp);

					(void)PRINTF("%s: get_cfg: can(%u)\r\n", priv->name, c->index);

					r->hdr.hdr.type = CAN_RPMSG_CTRL_RSP;
					r->hdr.hdr.len = sizeof(struct can_rpmsg_cmd_get_cfg_rsp);
					r->hdr.seq = c->hdr.seq;
					r->hdr.id = c->hdr.id;
					r->hdr.result = 0x0;

					if (c->index >= can_count())
					{
						r->hdr.result = -ENODEV;
						break;
					}

					handler = &can_handler[c->index];

					r->index = c->index;
					r->canfd = handler->is_canfd;
					r->bitrate = handler->bitrate;
					r->bitrate_mask = handler->bitrate_mask;
					r->dbitrate = handler->dbitrate;
					r->dbitrate_mask = handler->dbitrate_mask;

					(void)PRINTF("%s: can(%u): bitrate(%u) dbitrate(%u)\r\n",
							priv->name, r->index, r->bitrate, r->dbitrate);
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

					if (handler->ops.ifup)
					{
						ret = handler->ops.ifup(handler);
						if (ret)
						{
							rsp->result = -EIO;
							break;
						}
					}

					if (handler->stb.present)
					{
						GPIO_PinWrite(handler->stb.base, handler->stb.pin,
								handler->stb.active_low ? 1U : 0U);
					}

					if (xSemaphoreGive(handler->runsem) != pdTRUE )
					{
						(void)PRINTF("%s: failed to give semaphore\r\n", __func__);
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
						(void)PRINTF("%s: failed to take semaphore\r\n", __func__);
					}

					if (handler->stb.present)
					{
						GPIO_PinWrite(handler->stb.base, handler->stb.pin,
								handler->stb.active_low ? 0U : 1U);
					}

					if (handler->ops.ifdown)
					{
						ret = handler->ops.ifdown(handler);
						if (ret)
						{
							rsp->result = -EIO;
							break;
						}
					}

					handler->active = false;
					rsp->result = 0x0;
				}

				break;
			case CAN_RPMSG_CMD_SET_RATE:
				{
					const struct can_rpmsg_cmd_set_rate *c = (const struct can_rpmsg_cmd_set_rate *)cmd;
					struct can_rpmsg_cmd_set_rate_rsp *r = (struct can_rpmsg_cmd_set_rate_rsp *)rsp;
					size = sizeof(struct can_rpmsg_cmd_set_rate_rsp);
					uint32_t bitrate;
					uint32_t dbitrate;

					(void)PRINTF("%s: get_cfg: can(%u) bitrate(%u) dbitrate(%u)\r\n",
							priv->name, c->index, c->bitrate, c->dbitrate);

					bitrate = c->bitrate;
					dbitrate = c->dbitrate;

					r->hdr.hdr.type = CAN_RPMSG_CTRL_RSP;
					r->hdr.hdr.len = sizeof(struct can_rpmsg_cmd_set_rate_rsp);
					r->hdr.seq = c->hdr.seq;
					r->hdr.id = c->hdr.id;
					r->hdr.result = 0x0;

					if (c->index >= can_count())
					{
						r->hdr.result = -ENODEV;
						break;
					}

					r->index = c->index;

					handler = &can_handler[c->index];

					if (handler->active)
					{
						r->hdr.result = -EBUSY;
						break;
					}

					if (handler->ops.bitrate)
					{
						ret = handler->ops.bitrate(handler, &bitrate, &dbitrate);
						r->hdr.result = ret;
						r->bitrate = bitrate;
						r->dbitrate = dbitrate;
					}
					else
					{
						r->hdr.result = -ENOSYS;
						break;
					}

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

					(void)PRINTF("%s: unknown cmd: %d\r\n", priv->name, cmd->id);
				}

				break;
		}

		__DSB();
		MU_SendMsg(LSIO__MU13_B, CTRL_MU_CHAN, can_rpmsg_to_sig(CAN_RPMSG_CTRL_RSP, size));
	}

	(void)PRINTF("%s: done...\r\n", priv->name);

	while (1)
	{
	}
}

/* mgmt: events */

void evt_send(unsigned int event_id)
{
	if (evt_handler.evt.evtq)
	{
		if ( pdTRUE != xQueueSend( evt_handler.evt.evtq, &event_id, 0 ) )
		{
			(void)PRINTF("%s: failed send HB event\r\n", __func__);
		}
	}
}

static void evt_task(void *param)
{
	struct can_rpmsg_evt *evt = (struct can_rpmsg_evt *)CTRL_MEM_EVT;
	struct mgmt_data *priv = (struct mgmt_data *)param;
	uint32_t beat = 0;
	unsigned int event_id;
	unsigned int ack;
	uint32_t size;

	(void)PRINTF("%s: start...\r\n", priv->name);

	while (1)
	{
		if (xQueueReceive(priv->evt.evtq, &event_id, portMAX_DELAY) == pdFALSE)
		{
			(void)PRINTF("%s: recv failed...\r\n", priv->name);
			continue;
		}

		memset((void *)evt, 0x0, CTRL_MEM_SIZE);

		switch (event_id)
		{
			case CAN_RPMSG_EVT_HB:
				{
					struct can_rpmsg_evt_hb *e = (struct can_rpmsg_evt_hb *)evt;
					size = sizeof(struct can_rpmsg_evt_hb);

					e->hdr.hdr.type = CAN_RPMSG_CTRL_EVT;
					e->hdr.hdr.len = sizeof(struct can_rpmsg_evt_hb);
					e->hdr.id = event_id;
					e->beat = beat++;
				}

				__DSB();
				break;
			default:
				(void)PRINTF("%s: unknown evt: %u\r\n", priv->name, event_id);
				continue;
		}

		MU_SendMsg(LSIO__MU13_B, CTRL_MU_CHAN, can_rpmsg_to_sig(CAN_RPMSG_CTRL_EVT, size));

		if (xQueueReceive(priv->evt.ackq, &ack, portTICK_PERIOD_MS * 1000) == pdFALSE)
		{
			(void)PRINTF("%s: event %u ack timeout...\r\n", priv->name, event_id);
		}
	}

	while (1)
	{
	}
}

void stats_task(TimerHandle_t xTimer)
{
	evt_send(CAN_RPMSG_EVT_HB);
	vTaskGetRunTimeStats(stats);
	(void)PRINTF("%s\n", stats);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
	(void)PRINTF("%s: stack overflow in task %s...\r\n", __func__, pcTaskName);
	while (1)
	{
	}
}

void vApplicationMallocFailedHook(void)
{
	(void)PRINTF("%s: malloc failed...\r\n", __func__);
	while (1)
	{
	}
}

/* main */

int main(void)
{
	const struct remote_resource_table *volatile rsc_table = get_rsc_table();
	const struct fw_rsc_vdev *volatile user_vdev = &rsc_table->user_vdev;
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	SemaphoreHandle_t mutex;
	TimerHandle_t timer;
	int i;

	/* hardware init */

	board_hw_init();

	/* stats timer */

	timer = xTimerCreate("stats_timer", 60000, pdTRUE, (void *)0, stats_task);
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

	/* cmd handler */

	cmd_handler.cmd.cmdq = xQueueCreate(1, sizeof( unsigned int ) );
	if( cmd_handler.cmd.cmdq == NULL )
	{
		(void)PRINTF("failed to create cmd queue\r\n");
		while (1)
		{
		}
	}

	if (xTaskCreate(cmd_task, "cmd_task", APP_TASK_STACK_SIZE, &cmd_handler, tskIDLE_PRIORITY + 2U, &cmd_handler.task) != pdPASS)
	{
		(void)PRINTF("failed to create cmd task\r\n");
		while (1)
		{
		}
	}

	/* evt handler */

	evt_handler.evt.evtq = xQueueCreate(5, sizeof( unsigned int ) );
	if( evt_handler.evt.evtq == NULL )
	{
		(void)PRINTF("failed to create evt queue\r\n");
		while (1)
		{
		}
	}

	evt_handler.evt.ackq = xQueueCreate(1, sizeof( unsigned int ) );
	if( evt_handler.evt.ackq == NULL )
	{
		(void)PRINTF("failed to create evt ack queue\r\n");
		while (1)
		{
		}
	}

	if (xTaskCreate(evt_task, "evt_task", APP_TASK_STACK_SIZE, &evt_handler, tskIDLE_PRIORITY + 2U, &evt_handler.task) != pdPASS)
	{
		(void)PRINTF("failed to create evt task\r\n");
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

		/* The semaphore is created in the 'empty' state, no need to 'take' it. */
		mutex = xSemaphoreCreateBinary();
		if (mutex == NULL)
		{
			(void)PRINTF("%s: failed to allocate semaphore...\r\n", can_handler[i].name);
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
			can_handler[i].flexcan.cb.priv = &can_handler[i];
			can_handler[i].ops.ifup = &flexcan_up;
			can_handler[i].ops.ifdown = &flexcan_down;
			can_handler[i].ops.bitrate = &flexcan_bitrate;

			if (flexcan_init(&can_handler[i]))
			{
				(void)PRINTF("%s: failed to init interface\r\n", can_handler[i].name);
				while (1)
				{
				}
			}

			if (xTaskCreate(flexcan_task, can_handler[i].name, APP_TASK_STACK_SIZE, &can_handler[i], tskIDLE_PRIORITY + 1U, &can_handler[i].task) != pdPASS)
			{
				(void)PRINTF("%s: failed to create task\r\n", can_handler[i].name);
				while (1)
				{
				}
			}
			break;
		case TYPE_MCP2517FD:
			can_handler[i].ops.ifup = &mcp_up;
			can_handler[i].ops.ifdown = &mcp_down;
			can_handler[i].ops.bitrate = &mcp_bitrate;

			if (mcp_init(&can_handler[i]))
			{
				(void)PRINTF("%s: failed to init interface\r\n", can_handler[i].name);
				while (1)
				{
				}
			}

			if (xTaskCreate(mcpcan_task, can_handler[i].name, APP_TASK_STACK_SIZE, &can_handler[i], tskIDLE_PRIORITY + 1U, &can_handler[i].task) != pdPASS)
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

	/* announce remote to master */

	while (0 == rpmsg_lite_is_link_up(rpmsg))
	{
		if (user_vdev->status & VIRTIO_CONFIG_S_DRIVER_OK)
		{
			rpmsg->link_state = 1U;
		}
	}

	(void)PRINTF("RMSG link is up...\r\n");

	(void)rpmsg_ns_bind(rpmsg, app_nameservice_isr_cb, ((void *)0));
	SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	(void)rpmsg_ns_announce(rpmsg, can_handler[0].ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
	(void)PRINTF("RPMSG nameservice announce sent...\r\n");

	/* */

	vTaskStartScheduler();

	(void)PRINTF("Failed to start FreeRTOS\r\n");
	while (1)
	{
	}
}
