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

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"

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

#define DEV_CAN0	ADMA__CAN0
#define DEV_CAN1	ADMA__CAN1

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

/* Linux format */

#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */

#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */

#define be32_to_le32(x) ((uint32_t)(                         \
	(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |            \
	(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |            \
	(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |            \
	(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))

struct canfd_frame {
	uint32_t can_id;
	uint8_t len;
	uint8_t flags;
	uint8_t __res0;
	uint8_t __res1;
	uint32_t data[2] __attribute__((aligned(8)));
};

// can format conversion: master(Linux) to remote (NXP FreeRTOS)
void mtor(flexcan_frame_t *frame, struct canfd_frame *cfd)
{
	memset(frame, 0x0, sizeof(*frame));

	frame->length = cfd->len;
	frame->type = (cfd->can_id & CAN_RTR_FLAG) ? 0x1 : 0x0;
	if (cfd->can_id & CAN_EFF_FLAG) {
		frame->format = 0x1;
		frame->id = cfd->can_id & CAN_EFF_MASK;
	} else {
		frame->id = (cfd->can_id & CAN_SFF_MASK) << 18;
	}

	frame->dataWord0 = be32_to_le32(cfd->data[0]);
	frame->dataWord1 = be32_to_le32(cfd->data[1]);
}

// can format conversion: remote (NXP FreeRTOS) to master(Linux)
void rtom(struct canfd_frame *cfd, flexcan_frame_t *frame)
{
	memset(cfd, 0x0, sizeof(*cfd));

	if (frame->format == kFLEXCAN_FrameFormatExtend)
	{
		cfd->can_id = ((frame->id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	}
	else
	{
		cfd->can_id = (frame->id >> 18) & CAN_SFF_MASK;
	}

	if (frame->type == kFLEXCAN_FrameTypeRemote)
	{
		cfd->can_id |= CAN_RTR_FLAG;
	}

	cfd->len = frame->length;

	cfd->data[0] = be32_to_le32(frame->dataWord0);
	cfd->data[1] = be32_to_le32(frame->dataWord1);
}

/* rpmsg globals */ 

static struct rpmsg_lite_instance *volatile rpmsg;

struct rpmsg_lite_endpoint *volatile ept_can0;
struct rpmsg_lite_endpoint *volatile ept_can1;

volatile rpmsg_queue_handle queue_can0;
volatile rpmsg_queue_handle queue_can1;

static TaskHandle_t mgmt_task_handle = NULL;
static TaskHandle_t can0_task_handle = NULL;
static TaskHandle_t can1_task_handle = NULL;

/* flexcan globals */

typedef struct flexcan_cb_t {
	flexcan_mb_transfer_t txXfer;
	flexcan_mb_transfer_t rxXfer;
	bool txComplete;
	bool rxComplete;
	bool wakenUp;
	char *rxbuf;
	void *txbuf;
} flexcan_cb_t;

volatile flexcan_handle_t flexcanHandle0;
volatile flexcan_cb_t flexCbData0;
flexcan_frame_t txFrame0; 
flexcan_frame_t rxFrame0; 

volatile flexcan_handle_t flexcanHandle1;
volatile flexcan_cb_t flexCbData1;
flexcan_frame_t txFrame1; 
flexcan_frame_t rxFrame1; 

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
				data->rxComplete = true;
			}
			break;

		case kStatus_FLEXCAN_TxIdle:
			if (TX_MESSAGE_BUFFER_NUM == result)
			{
				data->txComplete = true;
			}
			break;

		case kStatus_FLEXCAN_WakeUp:
			data->wakenUp = true;
			break;

		default:
			break;
	}
}

static void flexcan_setup(CAN_Type *canbase, flexcan_handle_t *handle, flexcan_cb_t *cbdata)
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

	/* If special quantum setting is needed, set the timing parameters. */
	flexcanConfig.timingConfig.propSeg   = PROPSEG;
	flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
	flexcanConfig.timingConfig.phaseSeg2 = PSEG2;

	FLEXCAN_Init(canbase, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(canbase, handle, flexcan_callback, (void *)cbdata);

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
		(void)rpmsg_queue_recv(rpmsg, queue_mgmt, (uint32_t *)&remote_addr, handshake, sizeof(handshake), ((void *)0), RL_BLOCK);
		(void)PRINTF("%s: mgmt recv: %s...\r\n", __func__, handshake);
	}

	(void)PRINTF("%s: done...\r\n", __func__);

	while (1)
	{
	}
}

static void can0_task(void *param)
{
	volatile uint32_t remote_addr;
	status_t  status;
	uint32_t txlen;
	uint32_t rxlen;
	void *txbuf = RL_NULL;
	char *rxbuf = RL_NULL;

	(void)PRINTF("%s: start...\r\n", __func__);

	flexCbData0.txXfer.frame = NULL;
	flexCbData0.txComplete = false;

	flexCbData0.rxXfer.frame = NULL;
	flexCbData0.rxComplete = false;

	while (1)
	{
		if (flexCbData0.txComplete)
		{
			if (flexCbData0.txXfer.frame)
			{
				(void)rpmsg_lite_release_rx_buffer(rpmsg, flexCbData0.rxbuf);
				(void)PRINTF("%s: tx: ok\r\n", __func__);
			}

			flexCbData0.txXfer.frame = NULL;
			flexCbData0.txComplete = false;
			flexCbData0.rxbuf = NULL;
		}
		else
		{
			if (!flexCbData0.txXfer.frame)
			{
				if (!rxbuf)
				{
					(void)rpmsg_queue_recv_nocopy(rpmsg, queue_can0, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
					if (rxbuf)
					{
						mtor(&txFrame0, (struct canfd_frame *)rxbuf);
					}
				}

				if (rxbuf)
				{
					flexCbData0.txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
					flexCbData0.txXfer.frame = &txFrame0;
					flexCbData0.txComplete = false;
					flexCbData0.rxbuf = rxbuf;

					status = FLEXCAN_TransferSendNonBlocking(DEV_CAN0, (flexcan_handle_t *)&flexcanHandle0, (flexcan_mb_transfer_t *)&flexCbData0.txXfer);
					if (status != kStatus_Success)
					{
						(void)PRINTF("%s: failed to prepare xmit len %d: %d\r\n", __func__, rxlen, status);
						flexCbData0.txXfer.frame = NULL;
					} else {
						(void)PRINTF("%s: prepared tx: ok\r\n", __func__);
						rxbuf = NULL;
					}
				}
			}
		}

		if (flexCbData0.rxComplete)
		{
			if (flexCbData0.rxXfer.frame)
			{
				rtom((struct canfd_frame *)flexCbData0.txbuf, flexCbData0.rxXfer.frame);
				rpmsg_lite_send_nocopy(rpmsg, ept_can0, 0x400 /* FIXME remote_addr */,
						flexCbData0.txbuf, sizeof(struct canfd_frame));
				(void)PRINTF("%s: rx: ok\r\n", __func__);
			}

			flexCbData0.rxXfer.frame = NULL;
			flexCbData0.rxComplete = false;
			flexCbData0.txbuf = NULL;
		}
		else
		{
			if (!flexCbData0.rxXfer.frame)
			{
				if (!txbuf)
				{
					txbuf = rpmsg_lite_alloc_tx_buffer(rpmsg, &txlen, 0);
					if (!txbuf)
					{
						(void)PRINTF("%s: failed to alloc rx buffer...\r\n", __func__);
					}
				}

				if (txbuf)
				{
					flexCbData0.rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
					flexCbData0.rxXfer.frame = &rxFrame0;
					flexCbData0.rxComplete = false;
					flexCbData0.txbuf = txbuf;

					status = FLEXCAN_TransferReceiveNonBlocking(DEV_CAN0, (flexcan_handle_t *)&flexcanHandle0, (flexcan_mb_transfer_t *)&flexCbData0.rxXfer);
					if (status != kStatus_Success) {
						(void)PRINTF("%s: failed to prepare rx len %d: %d\r\n", __func__, txlen, status);
						flexCbData0.rxXfer.frame = NULL;
					} else {
						(void)PRINTF("%s: prepared rx: ok\r\n", __func__);
						txbuf = NULL;
					}
				}
			}
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
	status_t  status;
	uint32_t txlen;
	uint32_t rxlen;
	void *txbuf = RL_NULL;
	char *rxbuf = RL_NULL;

	(void)PRINTF("%s: start...\r\n", __func__);

	flexCbData1.txXfer.frame = NULL;
	flexCbData1.txComplete = false;

	flexCbData1.rxXfer.frame = NULL;
	flexCbData1.rxComplete = false;

	while (1)
	{
		if (flexCbData1.txComplete)
		{
			if (flexCbData1.txXfer.frame)
			{
				(void)rpmsg_lite_release_rx_buffer(rpmsg, flexCbData1.rxbuf);
				(void)PRINTF("%s: tx: ok\r\n", __func__);
			}

			flexCbData1.txXfer.frame = NULL;
			flexCbData1.txComplete = false;
			flexCbData1.rxbuf = NULL;
		}
		else
		{
			if (!flexCbData1.txXfer.frame)
			{
				if (!rxbuf)
				{
					(void)rpmsg_queue_recv_nocopy(rpmsg, queue_can1, (uint32_t *)&remote_addr, &rxbuf, &rxlen, 0);
					if (rxbuf)
					{
						mtor(&txFrame1, (struct canfd_frame *)rxbuf);
					}
				}

				if (rxbuf)
				{
					flexCbData1.txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
					flexCbData1.txXfer.frame = &txFrame1;
					flexCbData1.txComplete = false;
					flexCbData1.rxbuf = rxbuf;

					status = FLEXCAN_TransferSendNonBlocking(DEV_CAN1, (flexcan_handle_t *)&flexcanHandle1, (flexcan_mb_transfer_t *)&flexCbData1.txXfer);
					if (status != kStatus_Success)
					{
						(void)PRINTF("%s: failed to prepare xmit len %d: %d\r\n", __func__, rxlen, status);
						flexCbData1.txXfer.frame = NULL;
					} else {
						(void)PRINTF("%s: prepared tx: ok\r\n", __func__);
						rxbuf = NULL;
					}
				}
			}
		}


		if (flexCbData1.rxComplete)
		{
			if (flexCbData1.rxXfer.frame)
			{
				rtom((struct canfd_frame *)flexCbData1.txbuf, flexCbData1.rxXfer.frame);
				rpmsg_lite_send_nocopy(rpmsg, ept_can1, 0x400 /* FIXME remote_addr */,
							flexCbData1.txbuf, sizeof(struct canfd_frame));
				(void)PRINTF("%s: rx: ok\r\n", __func__);
			}

			flexCbData1.rxXfer.frame = NULL;
			flexCbData1.rxComplete = false;
			flexCbData1.txbuf = NULL;
		}
		else
		{
			if (!flexCbData1.rxXfer.frame)
			{
				if (!txbuf)
				{
					txbuf = rpmsg_lite_alloc_tx_buffer(rpmsg, &txlen, 0);
					if (!txbuf)
					{
						(void)PRINTF("%s: failed to alloc rx buffer...\r\n", __func__);
					}
				}

				if (txbuf)
				{

					flexCbData1.rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
					flexCbData1.rxXfer.frame = &rxFrame1;
					flexCbData1.rxComplete = false;
					flexCbData1.txbuf = txbuf;

					status = FLEXCAN_TransferReceiveNonBlocking(DEV_CAN1, (flexcan_handle_t *)&flexcanHandle1, (flexcan_mb_transfer_t *)&flexCbData1.rxXfer);
					if (status != kStatus_Success) {
						(void)PRINTF("%s: failed to prepare rx len %d: %d\r\n", __func__, txlen, status);
						flexCbData1.rxXfer.frame = NULL;
					} else {
						(void)PRINTF("%s: prepared rx: ok\r\n", __func__);
						txbuf = NULL;
					}
				}
			}
		}

		taskYIELD();
	}

	(void)PRINTF("%s: done...\r\n", __func__);

	while (1)
	{
	}
}
/* main */

int main(void)
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

	IRQSTEER_Init(IRQSTEER);
	NVIC_EnableIRQ(IRQSTEER_4_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, LSIO_MU8_INT_B_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN0_INT_IRQn);
	IRQSTEER_EnableInterrupt(IRQSTEER, ADMA_FLEXCAN1_INT_IRQn);

	if (sc_pm_set_resource_power_mode(ipc, SC_R_MU_8B, SC_PM_PW_MODE_ON) != SC_ERR_NONE)
	{
		PRINTF("Error: Failed to power on MU_8B!\r\n");
	}

	/* flexcan init */

	flexcan_setup(DEV_CAN0, (flexcan_handle_t *)&flexcanHandle0, (flexcan_cb_t *)&flexCbData0);
	flexcan_setup(DEV_CAN1, (flexcan_handle_t *)&flexcanHandle1, (flexcan_cb_t *)&flexCbData1);

	/* rpmsg init */

	(void)PRINTF("RPMSG shared base addr is 0x%x\r\n", RPMSG_LITE_SHMEM_BASE);
	rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
	if (rpmsg != RL_NULL)
	{
		(void)PRINTF("Tx: name(%s) id(%d)\r\n",
				rpmsg->tvq->vq_name, rpmsg->tvq->vq_queue_index);
		(void)PRINTF("Rx: name(%s) id(%d)\r\n",
				rpmsg->rvq->vq_name, rpmsg->rvq->vq_queue_index);
	}

	/* freertos: task init */

	if (xTaskCreate(mgmt_task, "mgmt_task", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2U, &mgmt_task_handle) != pdPASS)
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
