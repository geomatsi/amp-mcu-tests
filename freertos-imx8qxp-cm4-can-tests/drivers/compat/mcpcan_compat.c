#include "compat_linux.h"
#include "mcpcan_compat.h"
#include "drv_canfdspi_api.h"

extern uint32_t remote_addr;
extern gpio_pin_config_t H;
extern gpio_pin_config_t L;

static CAN_BITTIME_SETUP mcp_bitrate_convert(uint32_t bitrate, uint32_t dbitrate)
{
	if (bitrate == 250000 && dbitrate == 1000000)
		return CAN_250K_1M;

	if (bitrate == 250000 && dbitrate == 2000000)
		return CAN_250K_2M;

	if (bitrate == 250000 && dbitrate == 3000000)
		return CAN_250K_3M;

	if (bitrate == 250000 && dbitrate == 4000000)
		return CAN_250K_4M;

	if (bitrate == 500000 && dbitrate == 1000000)
		return CAN_500K_1M;

	if (bitrate == 500000 && dbitrate == 2000000)
		return CAN_500K_2M;

	if (bitrate == 500000 && dbitrate == 3000000)
		return CAN_500K_3M;

	if (bitrate == 500000 && dbitrate == 4000000)
		return CAN_500K_4M;

	// For now should not be here due to the list of supported bitrates
	// that is passed to Linux driver on i.MX8 host.

	return CAN_500K_1M;
}

int32_t mcp_init(can_handler_data_t *handler)
{
	gpio_out_pin_t *ncs = &handler->mcp.ncs;

	if (ncs->present)
	{
		/* init and _disable_ chip select */
		GPIO_PinInit(ncs->base, ncs->pin, ncs->active_low ? &H : &L);
	}

	return 0;
}

int32_t mcp_up(can_handler_data_t *handler)
{
	CAN_RX_FIFO_CONFIG rxConfig;
	CAN_TX_FIFO_CONFIG txConfig;
	CAN_OSC_STATUS osc_status;
	CAN_BITTIME_SETUP rate;
	CAN_OSC_CTRL oscCtrl;
	CAN_CONFIG config;
	REG_CiFLTOBJ fObj;
	REG_CiMASK mObj;
	uint32_t volatile count = 0;
	uint8_t cs = handler->id;
	int ret = 0;

	// Reset device
	ret = DRV_CANFDSPI_Reset(handler->id);
	if (ret)
	{
		(void)PRINTF("%s: failed to reset MCP: %d\r\n", __func__, ret);
		goto out;
	}

	(void)PRINTF("%s: setting clock...\r\n", __func__);

	DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
	oscCtrl.PllEnable = 0;
	oscCtrl.OscDisable = 0;
	oscCtrl.SclkDivide = 0;

	ret = DRV_CANFDSPI_OscillatorControlSet(cs, oscCtrl);
	if (ret)
	{
		(void)PRINTF("%s: failed to config oscillator: %d\r\n", __func__, ret);
		goto out;
	}

	while (count++ < 0xFFFF)
	{
		ret = DRV_CANFDSPI_OscillatorStatusGet(cs, &osc_status);
		if (ret)
		{
			(void)PRINTF("%s: failed to get oscillator status: %d\r\n", __func__, ret);
			goto out;
		}

		if (osc_status.OscReady)
		{
			break;
		}
	}

	(void)PRINTF("pll-%d, osc-%d, sclk-%d\r\n",
			osc_status.PllReady, osc_status.OscReady, osc_status.SclkReady);

	// Enable ECC and initialize RAM
	ret = DRV_CANFDSPI_EccEnable(cs);
	if (ret)
	{
		(void)PRINTF("%s: failed to enable ECC: %d\r\n", __func__, ret);
		goto out;
	}

	ret = DRV_CANFDSPI_RamInit(cs, 0xff);
	if (ret)
	{
		(void)PRINTF("%s: failed to init RAM: %d\r\n", __func__, ret);
		goto out;
	}

	// Configure device
	DRV_CANFDSPI_ConfigureObjectReset(&config);
	config.IsoCrcEnable = 1;
	config.StoreInTEF = 0;

	ret = DRV_CANFDSPI_Configure(cs, &config);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure MCP: %d\r\n", __func__, ret);
		goto out;
	}

	// Setup TX FIFO
	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
	txConfig.PayLoadSize = CAN_PLSIZE_64;
	txConfig.TxPriority = 1;
	txConfig.FifoSize = 7;

	ret = DRV_CANFDSPI_TransmitChannelConfigure(cs, EXAMPLE_TX_FIFO, &txConfig);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure xmit channel: %d\r\n", __func__, ret);
		goto out;
	}

	// Setup RX FIFO
	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
	rxConfig.PayLoadSize = CAN_PLSIZE_64;
	rxConfig.FifoSize = 15;

	ret = DRV_CANFDSPI_ReceiveChannelConfigure(cs, EXAMPLE_RX_FIFO, &rxConfig);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure recv channel: %d\r\n", __func__, ret);
		goto out;
	}

	// Setup bit time
	rate = mcp_bitrate_convert(handler->bitrate, handler->dbitrate);
	ret = DRV_CANFDSPI_BitTimeConfigure(cs, rate, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure bittime: %d\r\n", __func__, ret);
		goto out;
	}

	// Setup RX Filter
	fObj.word = 0x0;
	fObj.bF.SID = 0x0;
	fObj.bF.EXIDE = 0x0;
	fObj.bF.EID = 0x00;

	DRV_CANFDSPI_FilterObjectConfigure(cs, CAN_FILTER0, &fObj.bF);

	// Setup RX Mask
	mObj.word = 0x0;
	mObj.bF.MSID = 0x0;
	mObj.bF.MIDE = 0x0;
	mObj.bF.MEID = 0x0;
	DRV_CANFDSPI_FilterMaskConfigure(cs, CAN_FILTER0, &mObj.bF);

	// Link FIFO and Filter
	DRV_CANFDSPI_FilterToFifoLink(cs, CAN_FILTER0, EXAMPLE_RX_FIFO, true);

	// Setup Transmit and Receive events and interrupts
	ret = DRV_CANFDSPI_GpioModeConfigure(cs, GPIO_MODE_INT, GPIO_MODE_INT);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure INT gpio: %d\r\n", __func__, ret);
		goto out;
	}

	ret = DRV_CANFDSPI_TransmitChannelEventEnable(cs, EXAMPLE_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure xmit events: %d\r\n", __func__, ret);
		goto out;
	}

	ret = DRV_CANFDSPI_ReceiveChannelEventEnable(cs, EXAMPLE_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure recv events: %d\r\n", __func__, ret);
		goto out;
	}

	ret = DRV_CANFDSPI_ModuleEventEnable(cs,
			CAN_RX_INVALID_MESSAGE_EVENT |
			CAN_BUS_WAKEUP_EVENT |
			CAN_BUS_ERROR_EVENT |
			CAN_SYSTEM_ERROR_EVENT |
			CAN_OPERATION_MODE_CHANGE_EVENT |
			CAN_TX_EVENT |
			CAN_RX_EVENT);
	if (ret)
	{
		(void)PRINTF("%s: failed to enable events: %d\r\n", __func__, ret);
		goto out;
	}

	// Select Normal Mode
	ret = DRV_CANFDSPI_OperationModeSelect(cs, CAN_NORMAL_MODE);
	if (ret)
	{
		(void)PRINTF("%s: failed to configure mode: %d\r\n", __func__, ret);
		goto out;
	}

out:
	return ret;
}

int32_t mcp_down(can_handler_data_t *handler)
{
	/* TODO */
	return 0;
}

void mcpcan_task(void *param)
{
	can_handler_data_t *priv = (struct can_handler_data *)param;
	uint8_t txd[MAX_DATA_BYTES];
	uint8_t rxd[MAX_DATA_BYTES];
	CAN_TX_FIFO_EVENT txFlags;
	CAN_RX_FIFO_EVENT rxFlags;
        CAN_ERROR_STATE	stateFlags;
	CAN_MODULE_EVENT flags;
	uint16_t clearable_mask =
		CAN_RX_INVALID_MESSAGE_EVENT |
		CAN_BUS_WAKEUP_EVENT |
		CAN_BUS_ERROR_EVENT |
		CAN_SYSTEM_ERROR_EVENT |
		CAN_OPERATION_MODE_CHANGE_EVENT;
	CAN_TX_MSGOBJ txObj;
	CAN_RX_MSGOBJ rxObj;
	uint32_t txlen;
	uint32_t rxlen;
	uint32_t addr;
	int ret;

	if (priv->type != TYPE_MCP2517FD)
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

			// Block until CAN device is re-opened

			if (xSemaphoreTake(priv->runsem, portMAX_DELAY) != pdTRUE )
			{
				(void)PRINTF("%s: failed to take semaphore\r\n", priv->name);
			}

			(void)PRINTF("%s: start...\r\n", priv->name);
		}

		taskENTER_CRITICAL();

		ret = DRV_CANFDSPI_ModuleEventGet(priv->id, &flags);
		if (ret < 0)
		{
			(void)PRINTF("%s: failed to get events: %d\r\n", priv->name, ret);
		}
		else if (flags & clearable_mask)
		{
			ret  = DRV_CANFDSPI_ModuleEventClear(priv->id, flags & clearable_mask);
			if (ret < 0)
			{
				(void)PRINTF("%s: failed to clear events: %d\r\n", priv->name, ret);
			}

			if (flags & CAN_BUS_ERROR_EVENT)
			{
				ret = DRV_CANFDSPI_ErrorStateGet(priv->id, &stateFlags);
				if (ret < 0)
				{
					(void)PRINTF("%s: failed to get error state: %d\r\n", priv->name, ret);
				} else {
					(void)PRINTF("%s: state errors: 0x%x\r\n", priv->name, stateFlags);
				}
			}

			if (flags & CAN_RX_INVALID_MESSAGE_EVENT)
			{
				ret = DRV_CANFDSPI_BusDiagnosticsClear(priv->id);
				if (ret < 0)
				{
					(void)PRINTF("%s: failed to clear diag: %d\r\n", priv->name, ret);
				}
			}
		}
		else
		{
			/* pass */
		}

		taskEXIT_CRITICAL();
		taskYIELD();
		taskENTER_CRITICAL();

		ret = rpmsg_queue_recv_nocopy(priv->rpmsg, priv->queue, (uint32_t *)&addr, (char **)&priv->rxbuf, &rxlen, 0);
		if (ret == RL_SUCCESS)
		{
			ret = to_mcpcan(&txObj, txd, priv->rxbuf, rxlen);
			(void)rpmsg_lite_release_rx_buffer(priv->rpmsg, priv->rxbuf);

			if (ret < 0)
			{
				(void)PRINTF("%s: invalid host frame: %d\r\n", priv->name, ret);
			}
			else
			{
				ret = DRV_CANFDSPI_TransmitChannelEventGet(priv->id, EXAMPLE_TX_FIFO, &txFlags);
				if (ret < 0)
				{
					(void)PRINTF("%s: failed to get TX FIFO events: %d\r\n", priv->name, ret);
				}
				else
				{
					if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT)
					{
						ret = DRV_CANFDSPI_TransmitChannelLoad(priv->id, EXAMPLE_TX_FIFO,
								&txObj, txd, can_dlc2len(txObj.bF.ctrl.DLC), true);
						if (ret < 0)
						{
							(void)PRINTF("%s: failed to send frame: %d\r\n", priv->name, ret);
						}
					}
				}
			}
		}

		taskEXIT_CRITICAL();
		taskYIELD();
		taskENTER_CRITICAL();

		ret = DRV_CANFDSPI_ReceiveChannelEventGet(priv->id, EXAMPLE_RX_FIFO, &rxFlags);
		if (ret < 0)
		{
			(void)PRINTF("%s: failed to get TX FIFO events: %d\r\n", priv->name, ret);
		}

		if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT)
		{
			if (!priv->txbuf)
			{
				priv->txbuf = rpmsg_lite_alloc_tx_buffer(priv->rpmsg, &txlen, 0);
				if (priv->txbuf == RL_NULL)
				{
					(void)PRINTF("%s: failed to alloc rpmsg tx buffer\r\n", priv->name);
				}
			}

			if (priv->txbuf)
			{
				ret = DRV_CANFDSPI_ReceiveMessageGet(priv->id, EXAMPLE_RX_FIFO, &rxObj,
						rxd, MAX_DATA_BYTES);
				if (ret < 0)
				{
					(void)PRINTF("%s: failed to recv frame: %d\r\n", priv->name, ret);
				}
				else
				{
					ret = from_mcpcan(priv->txbuf, &rxObj, rxd);
					if (ret < 0)
					{
						(void)PRINTF("%s: invalid device frame: %d\r\n", priv->name, ret);
					}
					else
					{
						rpmsg_lite_send_nocopy(priv->rpmsg, priv->ept, remote_addr,
								priv->txbuf, ret);
						priv->txbuf = NULL;
					}
				}

				ret = DRV_CANFDSPI_ModuleEventClear(priv->id, CAN_RX_EVENT);
				if (ret < 0)
				{
					(void)PRINTF("%s: failed to clear Rx events: %d\r\n", priv->name, ret);
				}
			}
		}

		taskEXIT_CRITICAL();

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
