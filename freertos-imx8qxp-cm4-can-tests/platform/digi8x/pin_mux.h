/*
 * Copyright 2017-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

#define BOARD_INITPINS_M40_UART0_RX_PIN_FUNCTION_ID	SC_P_ADC_IN2
#define BOARD_INITPINS_M40_UART0_TX_PIN_FUNCTION_ID	SC_P_ADC_IN3

#define BOARD_INITPINS_BB_CAN0_RX_PIN_FUNCTION_ID	SC_P_FLEXCAN0_RX
#define BOARD_INITPINS_BB_CAN0_TX_PIN_FUNCTION_ID	SC_P_FLEXCAN0_TX
#define BOARD_INITPINS_BB_CAN1_RX_PIN_FUNCTION_ID	SC_P_FLEXCAN1_RX
#define BOARD_INITPINS_BB_CAN1_TX_PIN_FUNCTION_ID	SC_P_FLEXCAN1_TX
#define BOARD_INITPINS_BB_CAN2_RX_PIN_FUNCTION_ID	SC_P_FLEXCAN2_RX
#define BOARD_INITPINS_BB_CAN2_TX_PIN_FUNCTION_ID	SC_P_FLEXCAN2_TX

/* CAN1 STBY */
#define BOARD_INITPINS_BB_GPIO1_IO00_PIN_FUNCTION_ID	SC_P_SPI2_CS0
/* CAN2 STBY */
#define BOARD_INITPINS_BB_GPIO1_IO02_PIN_FUNCTION_ID	SC_P_SPI2_SDI
/* CAN2/RS485 SWITCH */
#define BOARD_INITPINS_BB_GPIO3_IO23_PIN_FUNCTION_ID    SC_P_QSPI0B_SS0_B

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 * @param ipc scfw ipchandle.
 *
 */
void BOARD_InitPins(sc_ipc_t ipc);                         /*!< Function assigned for the core: Cortex-M4F[m4] */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
