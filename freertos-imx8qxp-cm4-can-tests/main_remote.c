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
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "rsc_table.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID            (0)
#define RPMSG_LITE_SHMEM_BASE         (VDEV0_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "can-rpmsg-imx"
#define APP_TASK_STACK_SIZE (256U)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30U)
#endif

typedef struct canfd_frame {
	uint32_t can_id;	/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t len;		/* frame payload length in byte */
	uint8_t flags;		/* additional flags for CAN FD */
	uint8_t __res0;		/* reserved / padding */
	uint8_t __res1;		/* reserved / padding */
	uint8_t data[8] __attribute__((aligned(8)));
} THE_MESSAGE, *THE_MESSAGE_PTR;

#define VIRTIO_CONFIG_S_DRIVER_OK       4

static struct rpmsg_lite_instance *volatile rpmsg;

struct rpmsg_lite_endpoint *volatile ept_can0;
struct rpmsg_lite_endpoint *volatile ept_can1;

volatile rpmsg_queue_handle queue_can0;
volatile rpmsg_queue_handle queue_can1;

static TaskHandle_t mgmt_task_handle = NULL;
static TaskHandle_t can0_task_handle = NULL;
static TaskHandle_t can1_task_handle = NULL;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

static void mgmt_task(void *param)
{
	const struct remote_resource_table *volatile rsc_table = get_rsc_table();
	const struct fw_rsc_vdev *volatile user_vdev = &rsc_table->user_vdev;
	struct rpmsg_lite_endpoint *volatile ept_mgmt;
	volatile rpmsg_queue_handle queue_mgmt;
	volatile uint32_t remote_addr;
	static char handshake[16];

	(void)PRINTF("\r\nmgmt task...\r\n");

	while (0 == rpmsg_lite_is_link_up(rpmsg))
	{
		if (user_vdev->status & VIRTIO_CONFIG_S_DRIVER_OK)
		{
			rpmsg->link_state = 1U;
		}
	}

	(void)PRINTF("link is up...\r\n");

	/* mgmt ept */

	queue_mgmt = rpmsg_queue_create(rpmsg);
	if (queue_mgmt == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_mgmt...\r\n");
		while (1)
		{
		}
	}

	ept_mgmt = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, queue_mgmt);
	if (ept_mgmt == RL_NULL)
	{
		(void)PRINTF("failed to allocate ept_mgmt...\r\n");
		while (1)
		{
		}
	}

	/* can0 ept */

	queue_can0  = rpmsg_queue_create(rpmsg);
	if (queue_can0 == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_can0...\r\n");
		while (1)
		{
		}
	}

	ept_can0 = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR + 1, rpmsg_queue_rx_cb, queue_can0);
	if (ept_can0 == RL_NULL)
	{
		(void)PRINTF("failed to allocate ept_can0...\r\n");
		while (1)
		{
		}
	}

	/* can1 ept */

	queue_can1  = rpmsg_queue_create(rpmsg);
	if (queue_can1 == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_can1...\r\n");
		while (1)
		{
		}
	}

	ept_can1 = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR + 2, rpmsg_queue_rx_cb, queue_can1);
	if (ept_can1 == RL_NULL)
	{
		(void)PRINTF("failed to allocate ept_can1...\r\n");
		while (1)
		{
		}
	}

	/* resume data path tasks */

	vTaskResume(can0_task_handle);
	vTaskResume(can1_task_handle);

	/* announce remote to master */

	(void)rpmsg_ns_bind(rpmsg, app_nameservice_isr_cb, ((void *)0));
	SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	(void)rpmsg_ns_announce(rpmsg, ept_mgmt, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
	(void)PRINTF("nameservice announce sent...\r\n");

	/* wait for handshake message from remote core */

	(void)rpmsg_queue_recv(rpmsg, queue_mgmt, (uint32_t *)&remote_addr, handshake, sizeof(handshake), ((void *)0), RL_BLOCK);

	/* process mgmt tasks */

	while (1)
	{
		taskYIELD();
	}

	(void)PRINTF("mgmt done...\r\n");

	while (1)
	{
	}
}

static void can0_task(void *param)
{
	volatile uint32_t remote_addr;
	uint32_t txlen;
	uint32_t rxlen;
	void *txbuf;
	char *rxbuf;
	int32_t ret;

	(void)PRINTF("%s: start...\r\n", __func__);

	while (1)
	{
		rxbuf  = RL_NULL;
		rxlen = 0;
		txlen = 0;

		ret = rpmsg_queue_recv_nocopy(rpmsg, queue_can0, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
		if (ret == RL_SUCCESS && rxbuf != RL_NULL)
		{
			txbuf = rpmsg_lite_alloc_tx_buffer(rpmsg, &txlen, RL_TRUE);
			if (txbuf != RL_NULL)
			{
				if (rxlen <= txlen)
				{
					memcpy(txbuf, rxbuf, rxlen);
					rpmsg_lite_send_nocopy(rpmsg, ept_can1, remote_addr, txbuf, rxlen);
				}
				else
				{
					(void)PRINTF("%s: tx buffer is too small: %u < %u\r\n", __func__, txlen, rxlen);
				}
			}
			else
			{
				(void)PRINTF("%s: failed to alloc Tx buffer...\r\n", __func__);
			}

			(void)rpmsg_lite_release_rx_buffer(rpmsg, rxbuf);
		}

		taskYIELD();
	}

	(void)PRINTF("%s: done...\r\n", __func__);

	while (1)
	{
	}
}

static void can1_task(void *param)
{
	volatile uint32_t remote_addr;
	uint32_t txlen;
	uint32_t rxlen;
	void *txbuf;
	char *rxbuf;
	int32_t ret;

	(void)PRINTF("%s: start...\r\n", __func__);

	while (1)
	{
		rxbuf  = RL_NULL;
		rxlen = 0;
		txlen = 0;

		ret = rpmsg_queue_recv_nocopy(rpmsg, queue_can1, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
		if (ret == RL_SUCCESS && rxbuf != RL_NULL)
		{
			txbuf = rpmsg_lite_alloc_tx_buffer(rpmsg, &txlen, RL_TRUE);
			if (txbuf != RL_NULL)
			{
				if (rxlen <= txlen)
				{
					memcpy(txbuf, rxbuf, rxlen);
					rpmsg_lite_send_nocopy(rpmsg, ept_can0, remote_addr, txbuf, rxlen);
				}
				else
				{
					(void)PRINTF("%s: tx buffer is too small: %u < %u\r\n", __func__, txlen, rxlen);
				}
			}
			else
			{
				(void)PRINTF("%s: failed to alloc Tx buffer...\r\n", __func__);
			}

			(void)rpmsg_lite_release_rx_buffer(rpmsg, rxbuf);
		}

		taskYIELD();
	}

	(void)PRINTF("%s: done...\r\n", __func__);

	while (1)
	{
	}
}

int main(void)
{
	sc_ipc_t ipc = BOARD_InitRpc();
	BOARD_InitPins(ipc);
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitMemory();

	/*
	 *
	 */
	if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_5B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on MU_5B!\r\n");
	}

	if (sc_pm_set_resource_power_mode(ipc, SC_R_IRQSTR_M4_0, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on IRQSTEER!\r\n");
	}

	IRQSTEER_Init(IRQSTEER);
	IRQSTEER_EnableInterrupt(IRQSTEER, LSIO_MU8_INT_B_IRQn);

	if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_8B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on MU_8B!\r\n");
	}

	/* */

	(void)PRINTF("RPMSG shared base addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
	rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	if (rpmsg != RL_NULL)
	{
		(void)PRINTF("Tx: name(%s) id(%d)\r\n", rpmsg->tvq->vq_name, rpmsg->tvq->vq_queue_index);
		(void)PRINTF("Rx: name(%s) id(%d)\r\n", rpmsg->rvq->vq_name, rpmsg->rvq->vq_queue_index);
	}

	/* */

	if (xTaskCreate(mgmt_task, "mgmt_task", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &mgmt_task_handle) != pdPASS)
	{
		(void)PRINTF("failed to create mgmt task\r\n");
		while (1)
		{
		}
	}

	if (xTaskCreate(can0_task, "can0_task", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &can0_task_handle) != pdPASS)
	{
		(void)PRINTF("failed to create can0 task\r\n");
		while (1)
		{
		}
	}

	vTaskSuspend(can0_task_handle);

	if (xTaskCreate(can1_task, "can1_task", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &can1_task_handle) != pdPASS)
	{
		(void)PRINTF("failed to create can1 task\r\n");
		while (1)
		{
		}
	}

	vTaskSuspend(can1_task_handle);

	/* */

	vTaskStartScheduler();

	(void)PRINTF("Failed to start FreeRTOS\r\n");
	while (1)
	{
	}
}
