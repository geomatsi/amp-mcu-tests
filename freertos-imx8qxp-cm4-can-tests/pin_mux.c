/*
 * Copyright 2017-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v7.0
processor: MIMX8QX6xxxFZ
package_id: MIMX8QX6AVLFZ
mcu_data: ksdk2_0
processor_version: 0.7.11
board: MIMX8QX-MEK-REV-B
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "pin_mux.h"
#include "fsl_common.h"
#include "main/imx8qx_pads.h"
#include "svc/pad/pad_api.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'false', coreID: m4}
- pin_list:
  - {pin_num: V32, peripheral: M40__UART0, signal: uart_rx, pin_signal: ADC_IN2, PULL: PULL_0, sw_config: sw_config_0}
  - {pin_num: V30, peripheral: M40__UART0, signal: uart_tx, pin_signal: ADC_IN3, PULL: PULL_0, sw_config: sw_config_0}
  - {pin_num: Y34, peripheral: ADMA__FLEXCAN0, signal: flexcan_rx, pin_signal: FLEXCAN0_RX, PULL: PULL_0, sw_config: sw_config_0}
  - {pin_num: Y32, peripheral: ADMA__FLEXCAN0, signal: flexcan_tx, pin_signal: FLEXCAN0_TX, PULL: PULL_0, sw_config: sw_config_0}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
#if defined(BOARD_ICORE8X)
void BOARD_InitPins(sc_ipc_t ipc)
{
	sc_err_t err = SC_ERR_NONE;

	err = sc_pad_set_all(ipc, BOARD_INITPINS_M40_UART0_RX_PIN_FUNCTION_ID, 1U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_M40_UART0_TX_PIN_FUNCTION_ID, 1U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN0_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN0_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN1_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN1_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO0_IO15_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO3_IO01_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO4_IO20_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO3_IO01_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}
}

#elif defined(BOARD_DIGI8X)

void BOARD_InitPins(sc_ipc_t ipc)
{
	sc_err_t err = SC_ERR_NONE;

	err = sc_pad_set_all(ipc, BOARD_INITPINS_M40_UART0_RX_PIN_FUNCTION_ID, 1U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_M40_UART0_TX_PIN_FUNCTION_ID, 1U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN0_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN0_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN1_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN1_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN2_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_CAN2_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO1_IO00_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO1_IO02_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}

	err = sc_pad_set_all(ipc, BOARD_INITPINS_BB_GPIO3_IO23_PIN_FUNCTION_ID, 4U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);
	if (SC_ERR_NONE != err)
	{
		assert(false);
	}
}

#endif

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
