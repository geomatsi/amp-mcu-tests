/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_lpuart.h"
#include "fsl_irqsteer.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"
#include "fsl_mu.h"

/* rpmsg definitions */

#define VIRTIO_CONFIG_S_DRIVER_OK	(4)
#define RPMSG_LITE_LINK_ID		(0)
#define RPMSG_LITE_SHMEM_BASE		(VDEV0_VRING_BASE)
#define RPMSG_LITE_NS_ANNOUNCE_STRING	"can-rpmsg-imx"
#define APP_TASK_STACK_SIZE		(256U)

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30U)
#endif

/* control path definitions */

#define CTRL_MU_CHAN	1

/* spican definitions */

#define EXAMPLE_SPI_CLK_SOURCE	(SC_40MHZ)
#define EXAMPLE_TX_FIFO		CAN_FIFO_CH2
#define EXAMPLE_RX_FIFO		CAN_FIFO_CH1

/* flexcan definitions */

/*
 * Disable CAN clocks control by SDK
 */
#undef FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL
#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL	1

/*
 * When CLK_SRC=1, the protocol engine works at fixed frequency of 160M.
 * If other frequency wanted, please use CLK_SRC=0 and set the working frequency for SC_R_CAN_0.
 */
#define EXAMPLE_CAN_CLK_SOURCE (kFLEXCAN_ClkSrc1)
#define EXAMPLE_CAN_CLK_FREQ   (SC_160MHZ)
#define EXAMPLE_CAN_BITRATE	1000000U
#define EXAMPLE_CAN_DBITRATE	4000000U

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

struct can_handler_data;

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
	bool present;
	bool active_low;
	GPIO_Type *base;
	uint32_t pin;
} gpio_out_pin_t;

typedef struct flexcan_cb_t {
	struct can_handler_data *priv;
	bool txdone;
	bool rxdone;
	bool wakeup;
	bool failed;
	uint32_t errors;
	uint32_t rxflag;
	uint32_t txflag;
} flexcan_cb_t;

typedef enum can_type {
	TYPE_UNDEFINED	= 0,
	TYPE_FLEXCAN	= 1,
	TYPE_MCP2517FD	= 2,
} can_type_t;

typedef struct can_handler_ops {
	int32_t (*ifup)(struct can_handler_data *);
	int32_t (*ifdown)(struct can_handler_data *);
} can_handler_ops_t;

typedef struct can_handler_data {
	can_type_t type;
	char name[32];
	bool active;
	uint8_t id;

	/* can settings */
	bool is_canfd;
	uint32_t bitrate;
	uint32_t dbitrate;

	/* hardware-specific settings */
	union {
		/* Freescale FlexCAN */
		struct {
			CAN_Type *base;
			flexcan_mb_transfer_t tx;
			flexcan_mb_transfer_t rx;
			flexcan_fd_frame_t txframe;
			flexcan_fd_frame_t rxframe;
			flexcan_handle_t handle;
			flexcan_cb_t cb;
		} flexcan;
		/* Microchip MCP2517FD */
		struct {
			LPSPI_Type *base;
			gpio_out_pin_t ncs;
		} mcp;
	};

	can_handler_ops_t ops;

	/* xceiver stand-by */
	gpio_out_pin_t stb;
	/* rx/tx led */
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
} can_handler_data_t;

/* */

int can_count(void);
void board_hw_init(void);

#endif /* COMMON_DATA_H_ */
