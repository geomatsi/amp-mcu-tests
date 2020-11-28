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
#include "task.h"

#include "rsc_table.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "compat_linux.h"

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
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
} mgmt_data_t;

/* LED */

typedef struct gpio_out_pin {
	GPIO_Type *base;
	uint32_t pin;
} gpio_out_pin_t;

/* flexcan task data */

typedef struct flexcan_cb_t {
	bool txdone;
	bool rxdone;
	bool wakeup;
	bool failed;
	uint32_t errors;
} flexcan_cb_t;

typedef struct flexcan_data {
	CAN_Type *base;
	char name[32];
	struct rpmsg_lite_instance *volatile rpmsg;
	struct rpmsg_lite_endpoint *volatile ept;
	volatile rpmsg_queue_handle queue;
	flexcan_mb_transfer_t tx;
	flexcan_mb_transfer_t rx;
	flexcan_frame_t txframe; 
	flexcan_frame_t rxframe; 
	flexcan_handle_t handle;
	flexcan_cb_t cb;
	gpio_out_pin_t phy;
	gpio_out_pin_t led;
	/*
	 * Note that rx/tx are swapped for FlexCAN and rpmsg:
	 *   - rpmsg Rx becomes FlexCAN Tx
	 *   - FlexCAN Rx becomes rpmsg Tx
	 */
	void *rxbuf;
	void *txbuf;
} flexcan_data_t;

/* globals */ 

static TaskHandle_t mgmt_task_handle = NULL;
static TaskHandle_t can0_task_handle = NULL;
static TaskHandle_t can1_task_handle = NULL;

struct flexcan_data can0_data;
struct flexcan_data can1_data;
struct mgmt_data mgmt_data;

uint32_t remote_addr;

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

		default:
			(void)PRINTF("%s: FIXME: unknown status: %d\r\n", __func__, status);
			break;
	}
}

static void flexcan_setup(CAN_Type *canbase, flexcan_handle_t *handle, flexcan_cb_t *cb)
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
	struct mgmt_data *priv = (struct mgmt_data *)param;
	uint32_t addr;
	char msg[32];

	(void)PRINTF("%s: start...\r\n", priv->name);

	while (0 == rpmsg_lite_is_link_up(priv->rpmsg))
	{
		if (user_vdev->status & VIRTIO_CONFIG_S_DRIVER_OK)
		{
			priv->rpmsg->link_state = 1U;
		}
	}

	(void)PRINTF("%s: link is up...\r\n", priv->name);

	/* resume data path tasks */

	vTaskResume(can0_task_handle);
	vTaskResume(can1_task_handle);

	/* announce remote to master */

	(void)rpmsg_ns_bind(priv->rpmsg, app_nameservice_isr_cb, ((void *)0));
	SDK_DelayAtLeastUs(1000000U, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	(void)rpmsg_ns_announce(priv->rpmsg, priv->ept, RPMSG_LITE_NS_ANNOUNCE_STRING, (uint32_t)RL_NS_CREATE);
	(void)PRINTF("%s: nameservice announce sent...\r\n", priv->name);

	/* wait for handshake message from remote core */

	(void)rpmsg_queue_recv(priv->rpmsg, priv->queue, &remote_addr, msg, sizeof(msg), ((void *)0), RL_BLOCK);

	/* process mgmt tasks */

	while (1)
	{
		(void)rpmsg_queue_recv(priv->rpmsg, priv->queue, &addr, msg, sizeof(msg), ((void *)0), RL_BLOCK);
		if (addr != remote_addr)
		{
			(void)PRINTF("%s: unexpected remote addr: %u != %u\r\n", priv->name, addr, remote_addr);
		}

		(void)PRINTF("%s: mgmt recv: %s...\r\n", priv->name, msg);
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
	GPIO_PinWrite(priv->phy.base, priv->phy.pin, 0U);

	while (1)
	{
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
				(void)rpmsg_lite_release_rx_buffer(priv->rpmsg, priv->rxbuf);
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
			if (priv->rx.frame)
			{
				r2m((struct can_frame *)priv->txbuf, priv->rx.frame);
				rpmsg_lite_send_nocopy(priv->rpmsg, priv->ept, remote_addr, priv->txbuf, sizeof(struct can_frame));
			}

			priv->cb.rxdone = false;
			priv->rx.frame = NULL;
			priv->txbuf = NULL;
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
		(void)PRINTF("failed to allocate queue_mgmt...\r\n");
		while (1)
		{
		}
	}

	/* mgmt */

	queue = rpmsg_queue_create(rpmsg);
	if (queue == RL_NULL)
	{
		(void)PRINTF("failed to allocate mgmt queue...\r\n");
		while (1)
		{
		}
	}

	ept = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, queue);
	if (ept == RL_NULL)
	{
		(void)PRINTF("failed to allocate mgmt ept...\r\n");
		while (1)
		{
		}
	}

	memset(&mgmt_data, 0x0, sizeof(mgmt_data));

	strncpy(mgmt_data.name, "mgmt_task", sizeof(mgmt_data.name));
	mgmt_data.rpmsg = rpmsg;
	mgmt_data.queue = queue;
	mgmt_data.ept = ept;

	/* can0 */

	queue = rpmsg_queue_create(rpmsg);
	if (queue == RL_NULL)
	{
		(void)PRINTF("failed to allocate can0 queue...\r\n");
		while (1)
		{
		}
	}

	ept = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR + 1, rpmsg_queue_rx_cb, queue);
	if (ept == RL_NULL)
	{
		(void)PRINTF("failed to allocate can0 ept...\r\n");
		while (1)
		{
		}
	}

	memset(&can0_data, 0x0, sizeof(can0_data));

	strncpy(can0_data.name, "can0_task", sizeof(can0_data.name));
	can0_data.base = ADMA__CAN0;
	can0_data.rpmsg = rpmsg;
	can0_data.queue = queue;
	can0_data.ept = ept;

	can0_data.phy.base = LSIO__GPIO4;
	can0_data.phy.pin = 20U;
	GPIO_PinInit(can0_data.phy.base, can0_data.phy.pin, &phy);

	can0_data.led.base = LSIO__GPIO0;
	can0_data.led.pin = 15U;
	GPIO_PinInit(can0_data.led.base, can0_data.led.pin, &led);

	flexcan_setup(can0_data.base, (flexcan_handle_t *)&can0_data.handle, (flexcan_cb_t *)&can0_data.cb);

	/* can1 ept */

	queue = rpmsg_queue_create(rpmsg);
	if (queue == RL_NULL)
	{
		(void)PRINTF("failed to allocate can1 queue...\r\n");
		while (1)
		{
		}
	}

	ept = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR + 2, rpmsg_queue_rx_cb, queue);
	if (ept == RL_NULL)
	{
		(void)PRINTF("failed to allocate can1 ept...\r\n");
		while (1)
		{
		}
	}

	memset(&can1_data, 0x0, sizeof(can1_data));

	strncpy(can1_data.name, "can1_task", sizeof(can1_data.name));
	can1_data.base = ADMA__CAN1;
	can1_data.rpmsg = rpmsg;
	can1_data.queue = queue;
	can1_data.ept = ept;

	can1_data.phy.base = LSIO__GPIO1;
	can1_data.phy.pin = 25U;
	GPIO_PinInit(can1_data.phy.base, can1_data.phy.pin, &phy);

	can1_data.led.base = LSIO__GPIO3;
	can1_data.led.pin = 1U;
	GPIO_PinInit(can1_data.led.base, can1_data.led.pin, &led);

	flexcan_setup(can1_data.base, (flexcan_handle_t *)&can1_data.handle, (flexcan_cb_t *)&can1_data.cb);

	/* freertos: task init */

	if (xTaskCreate(mgmt_task, "mgmt_task", APP_TASK_STACK_SIZE, &mgmt_data, tskIDLE_PRIORITY + 2U, &mgmt_task_handle) != pdPASS)
	{
		(void)PRINTF("failed to create mgmt task\r\n");
		while (1)
		{
		}
	}

	if (xTaskCreate(can_task, "can0_task", APP_TASK_STACK_SIZE, &can0_data, tskIDLE_PRIORITY + 1U, &can0_task_handle) != pdPASS)
	{
		(void)PRINTF("failed to create can0 task\r\n");
		while (1)
		{
		}
	}

	vTaskSuspend(can0_task_handle);

	if (xTaskCreate(can_task, "can1_task", APP_TASK_STACK_SIZE, &can1_data, tskIDLE_PRIORITY + 1U, &can1_task_handle) != pdPASS)
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
