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

static char handshake[13];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static TaskHandle_t app_task_handle = NULL;

static void app_nameservice_isr_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
}

static void app_task(void *param)
{
	const struct remote_resource_table *volatile rsc_table = get_rsc_table();
	const struct fw_rsc_vdev *volatile user_vdev = &rsc_table->user_vdev;
	volatile uint32_t remote_addr;
	struct rpmsg_lite_endpoint *volatile my_ept_mgmt;
	struct rpmsg_lite_endpoint *volatile my_ept_can0;
	struct rpmsg_lite_endpoint *volatile my_ept_can1;
	volatile rpmsg_queue_handle queue_mgmt;
	volatile rpmsg_queue_handle queue_can0;
	volatile rpmsg_queue_handle queue_can1;
	struct rpmsg_lite_instance *volatile my_rpmsg;
	volatile rpmsg_ns_handle ns_handle;

	/* Print the initial banner */
	(void)PRINTF("\r\nRPMSG Ping-Pong FreeRTOS RTOS API Demo...\r\n");

	(void)PRINTF("RPMSG Share Base Addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
	my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	if (my_rpmsg != RL_NULL)
	{
		(void)PRINTF("Tx: name(%s) id(%d)\r\n", my_rpmsg->tvq->vq_name, my_rpmsg->tvq->vq_queue_index);
		(void)PRINTF("Rx: name(%s) id(%d)\r\n", my_rpmsg->rvq->vq_name, my_rpmsg->rvq->vq_queue_index);
	}

	while (0 == rpmsg_lite_is_link_up(my_rpmsg))
	{
		if (user_vdev->status & VIRTIO_CONFIG_S_DRIVER_OK)
		{
			my_rpmsg->link_state = 1U;
		}
	}

	(void)PRINTF("Link is up!\r\n");

	/* mgmt ept */

	queue_mgmt = rpmsg_queue_create(my_rpmsg);
	if (queue_can0 == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_mgmt...\r\n");
		for (;;)
		{
		}
	}

	my_ept_mgmt = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, queue_mgmt);
	if (my_ept_mgmt == RL_NULL)
	{
		(void)PRINTF("failed to allocate my_ept_mgmt...\r\n");
		for (;;)
		{
		}
	}

	ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, ((void *)0));
	SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	(void)rpmsg_ns_announce(my_rpmsg, my_ept_mgmt, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
	(void)PRINTF("Nameservice announce sent.\r\n");

	/* can0 ept */

	queue_can0  = rpmsg_queue_create(my_rpmsg);
	if (queue_can0 == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_can0...\r\n");
		for (;;)
		{
		}
	}

	my_ept_can0 = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR + 1, rpmsg_queue_rx_cb, queue_can0);
	if (my_ept_can0 == RL_NULL)
	{
		(void)PRINTF("failed to allocate my_ept_can0...\r\n");
		for (;;)
		{
		}
	}

	/* can1 ept */

	queue_can1  = rpmsg_queue_create(my_rpmsg);
	if (queue_can1 == RL_NULL)
	{
		(void)PRINTF("failed to allocate queue_can1...\r\n");
		for (;;)
		{
		}
	}

	my_ept_can1 = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR + 2, rpmsg_queue_rx_cb, queue_can1);
	if (my_ept_can1 == RL_NULL)
	{
		(void)PRINTF("failed to allocate my_ept_can1...\r\n");
		for (;;)
		{
		}
	}

	/* wait for handshake message from remote core */

	(void)rpmsg_queue_recv(my_rpmsg, queue_mgmt, (uint32_t *)&remote_addr, handshake, sizeof(handshake), ((void *)0), RL_BLOCK);

	while (1)
	{
		uint32_t txlen;
		uint32_t rxlen;
		void *txbuf;
		char *rxbuf;
		int32_t ret;

		ret = rpmsg_queue_get_current_size(queue_can0);
		if (ret > 0)
		{
			rxbuf  = RL_NULL;
			rxlen = 0;
			txlen = 0;

			ret = rpmsg_queue_recv_nocopy(my_rpmsg, queue_can0, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
			if (ret == RL_SUCCESS && rxbuf != RL_NULL)
			{
				txbuf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &txlen, RL_TRUE);
				if (txbuf != RL_NULL)
				{
					if (rxlen <= txlen)
					{
						memcpy(txbuf, rxbuf, rxlen);
						rpmsg_lite_send_nocopy(my_rpmsg, my_ept_can1, remote_addr, txbuf, rxlen);
					}
					else
					{
						(void)PRINTF("tx buffer is too small: %u < %u\r\n", txlen, rxlen);
					}
				}
				else
				{
					(void)PRINTF("Failed to alloc Tx buffer for can1...\r\n");
				}

				(void)rpmsg_lite_release_rx_buffer(my_rpmsg, rxbuf);
			}
			else
			{
				(void)PRINTF("No Rx data in can0...\r\n");
			}
		}

		if (rpmsg_queue_get_current_size(queue_can1))
		{
			rxbuf  = RL_NULL;
			rxlen = 0;
			txlen = 0;

			ret = rpmsg_queue_recv_nocopy(my_rpmsg, queue_can1, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
			if (ret == RL_SUCCESS && rxbuf != RL_NULL)
			{
				txbuf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &txlen, RL_TRUE);
				if (txbuf != RL_NULL)
				{
					if (rxlen <= txlen)
					{
						memcpy(txbuf, rxbuf, rxlen);
						rpmsg_lite_send_nocopy(my_rpmsg, my_ept_can0, remote_addr, txbuf, rxlen);
					}
					else
					{
						(void)PRINTF("tx buffer is too small: %u < %u\r\n", txlen, rxlen);
					}
				}
				else
				{
					(void)PRINTF("Failed to alloc Tx buffer for can0...\r\n");
				}

				(void)rpmsg_lite_release_rx_buffer(my_rpmsg, rxbuf);
			}
			else
			{
				(void)PRINTF("No Rx data in can1...\r\n");
			}
		}
	}

	(void)PRINTF("Ping pong done, deinitializing...\r\n");

	(void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept_mgmt);
	(void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept_can0);
	(void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept_can1);

	my_ept_mgmt = ((void *)0);
	my_ept_can0 = ((void *)0);
	my_ept_can1 = ((void *)0);

	(void)rpmsg_queue_destroy(my_rpmsg, queue_mgmt);
	(void)rpmsg_queue_destroy(my_rpmsg, queue_can0);
	(void)rpmsg_queue_destroy(my_rpmsg, queue_can1);

	queue_mgmt = ((void *)0);
	queue_can0 = ((void *)0);
	queue_can1 = ((void *)0);

	(void)rpmsg_ns_unbind(my_rpmsg, ns_handle);
	(void)rpmsg_lite_deinit(my_rpmsg);

	(void)PRINTF("Looping forever...\r\n");

	/* End of the example */
	for (;;)
	{
	}
}

/*!
 * @brief Main function
 */
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

	if (xTaskCreate(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, &app_task_handle) != pdPASS)
	{
		(void)PRINTF("\r\nFailed to create application task\r\n");
		for (;;)
		{
		}
	}

	vTaskStartScheduler();

	(void)PRINTF("Failed to start FreeRTOS on core0.\r\n");
	for (;;)
	{
	}
}
