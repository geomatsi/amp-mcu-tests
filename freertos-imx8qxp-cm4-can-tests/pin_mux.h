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

/* ADC_IN2 (coord V32), M40_UART0_RX */
#define BOARD_INITPINS_M40_UART0_RX_PIN_FUNCTION_ID	SC_P_ADC_IN2

/* ADC_IN3 (coord V30), M40_UART0_TX */
#define BOARD_INITPINS_M40_UART0_TX_PIN_FUNCTION_ID	SC_P_ADC_IN3

/* FLEXCAN0_RX (coord Y34), BB_CAN0_RX/J13C[25] */
#define BOARD_INITPINS_BB_CAN0_RX_PIN_FUNCTION_ID	SC_P_FLEXCAN0_RX

/* FLEXCAN0_TX (coord Y32), BB_CAN0_TX/J13C[26] */
#define BOARD_INITPINS_BB_CAN0_TX_PIN_FUNCTION_ID	SC_P_FLEXCAN0_TX

/* FLEXCAN1_RX TBD */
#define BOARD_INITPINS_BB_CAN1_RX_PIN_FUNCTION_ID	SC_P_FLEXCAN1_RX

/* FLEXCAN1_TX TBD */
#define BOARD_INITPINS_BB_CAN1_TX_PIN_FUNCTION_ID	SC_P_FLEXCAN1_TX


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
