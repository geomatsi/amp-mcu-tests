#include "compat_linux.h"
#include "flexcan_compat.h"

extern uint32_t remote_addr;
extern gpio_pin_config_t H;
extern gpio_pin_config_t L;

static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	struct flexcan_cb_t *data = (struct flexcan_cb_t *)userData;
	can_handler_data_t *priv = data->priv;
	bool error = false;
	uint32_t reg_esr;

	switch (status)
	{
		case kStatus_FLEXCAN_RxIdle:
			if (RX_MESSAGE_BUFFER_NUM == result)
			{
				data->rxdone = true;
			}
			else
			{
				(void)PRINTF("%s: RX: unexpected mbox %u\r\n", __func__, result);
			}
			break;

		case kStatus_FLEXCAN_RxOverflow:
			switch (result)
			{
				case RX_MESSAGE_BUFFER_NUM:
					data->rxflag = kStatus_FLEXCAN_RxOverflow;
					data->rxdone = true;
					break;
				default:
					(void)PRINTF("%s: overflow: status: %d result %u\r\n",
							__func__, status, result);
					break;
			}
			break;

		case kStatus_FLEXCAN_TxIdle:
			if (TX_MESSAGE_BUFFER_NUM == result)
			{
				data->txdone = true;
			}
			else
			{
				(void)PRINTF("%s: TX: unexpected mbox %u\r\n", __func__, result);
			}
			break;

		case kStatus_FLEXCAN_WakeUp:
			data->wakeup = true;
			break;

		case kStatus_FLEXCAN_ErrorStatus:
			reg_esr = FLEXCAN_GetStatusFlags(priv->flexcan.base);

			(void)PRINTF("%s:%s: failed: status(%d) result(%u) esr1(%s %s %s %s %s %s %s FLTCONF(%d) %s %s %s %s %s %s)\r\n",
					__func__, priv->name, status, result,
					reg_esr & kFLEXCAN_FDErrorIntFlag ? "ERRINT_FAST" : "",
					reg_esr & kFLEXCAN_BusoffDoneIntFlag ? "BOFFDONEINT" : "",
					reg_esr & kFLEXCAN_SynchFlag ? "SYNCH" : "",
					reg_esr & kFLEXCAN_TxWarningIntFlag ? "TWRNINT" : "",
					reg_esr & kFLEXCAN_RxWarningIntFlag ? "RWRNINT" : "",
					reg_esr & kFLEXCAN_TxErrorWarningFlag ? "TXWRN" : "",
					reg_esr & kFLEXCAN_RxErrorWarningFlag ? "RXWRN" : "",
					(reg_esr & kFLEXCAN_FaultConfinementFlag) >> 4,
					reg_esr & kFLEXCAN_IdleFlag ? "IDLE" : "",
					reg_esr & kFLEXCAN_TransmittingFlag ? "TX" : "",
					reg_esr & kFLEXCAN_ReceivingFlag ? "RX" : "",
					reg_esr & kFLEXCAN_BusOffIntFlag ? "BOFFINT" : "",
					reg_esr & kFLEXCAN_ErrorIntFlag ? "ERRINT" : "",
					reg_esr & kFLEXCAN_WakeUpIntFlag ? "WAKINT" : "");

			FLEXCAN_ClearStatusFlags(base,
					kFLEXCAN_RxWarningIntFlag |
					kFLEXCAN_TxWarningIntFlag |
					kFLEXCAN_ErrorIntFlag |
					kFLEXCAN_BusOffIntFlag);

			if (reg_esr & kFLEXCAN_ErrorIntFlag)
			{
				FLEXCAN_DisableInterrupts(priv->flexcan.base, (uint32_t)kFLEXCAN_ErrorInterruptEnable);
				error = true;
			}

			if (reg_esr & (kFLEXCAN_ErrorIntFlag | kFLEXCAN_ReceivingFlag | kFLEXCAN_RxWarningIntFlag))
			{
				FLEXCAN_TransferFDAbortReceive(priv->flexcan.base, &priv->flexcan.handle, RX_MESSAGE_BUFFER_NUM);
				priv->flexcan.cb.rxdone = false;
				if (priv->flexcan.rx.framefd)
				{
					priv->flexcan.rx.framefd = NULL;
				}
			}

			if (reg_esr & (kFLEXCAN_ErrorIntFlag | kFLEXCAN_TransmittingFlag | kFLEXCAN_TxWarningIntFlag))
			{
				FLEXCAN_TransferFDAbortSend(priv->flexcan.base, &priv->flexcan.handle, TX_MESSAGE_BUFFER_NUM);
				priv->flexcan.cb.txdone = false;
				if (priv->flexcan.tx.framefd)
				{
					priv->flexcan.tx.framefd = NULL;
					priv->rxbuf = NULL;
				}
			}

			data->errors = result;
			data->failed = true;
			break;

		case kStatus_Fail:
			switch (result)
			{
				case RX_MESSAGE_BUFFER_NUM:
					data->rxflag = kStatus_FLEXCAN_RxBusy;
					data->rxdone = true;
					break;
				default:
					(void)PRINTF("%s: failed: status: %d result %u\r\n",
							__func__, status, result);
					break;
			}
			break;

		default:
			(void)PRINTF("%s: FIXME: unknown status: %d\r\n", __func__, status);
			break;
	}

	if (!error)
	{
		FLEXCAN_EnableInterrupts(priv->flexcan.base, (uint32_t)kFLEXCAN_ErrorInterruptEnable);
	}
}

int32_t flexcan_init(can_handler_data_t *handler)
{
	CAN_Type *canbase = handler->flexcan.base;
	int i;

	/* Cleanup MBoxes */
	for (i = 0; i < 64; i++)
	{
		FLEXCAN_SetRxMbConfig(canbase, i, NULL, false);
	}

	return 0;
}

int32_t flexcan_up(can_handler_data_t *handler)
{
	flexcan_handle_t *handle = &handler->flexcan.handle;
	CAN_Type *canbase = handler->flexcan.base;
	flexcan_cb_t *cb = &handler->flexcan.cb;
	flexcan_timing_config_t timing_config;
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
	flexcanConfig.baudRateFD = handler->dbitrate;
	flexcanConfig.baudRate = handler->bitrate;
	flexcanConfig.maxMbNum = 14;

	/* If special quantum setting is needed, set the timing parameters. */
	flexcanConfig.timingConfig.propSeg   = PROPSEG;
	flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
	flexcanConfig.timingConfig.phaseSeg2 = PSEG2;

	flexcanConfig.timingConfig.fpropSeg   = FPROPSEG;
	flexcanConfig.timingConfig.fphaseSeg1 = FPSEG1;
	flexcanConfig.timingConfig.fphaseSeg2 = FPSEG2;

	memset(&timing_config, 0, sizeof(flexcan_timing_config_t));

	if (FLEXCAN_FDCalculateImprovedTimingValues(flexcanConfig.baudRate, flexcanConfig.baudRateFD,
				EXAMPLE_CAN_CLK_FREQ, &timing_config))
	{
		memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
	}
	else
	{
		(void)PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
	}

	FLEXCAN_FDInit(canbase, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, kFLEXCAN_64BperMB, true);

	/* Enable ISO CAN FD protocol ISO-11898-1 */
	FLEXCAN_EnterFreezeMode(canbase);
        canbase->CTRL2 |= (0x1U << 12);
	FLEXCAN_ExitFreezeMode(canbase);

	/* Setup Rx global mask: accept all packets */
	FLEXCAN_SetRxMbGlobalMask(canbase, 0x0);

	/* Setup Rx Message Buffer. */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type   = kFLEXCAN_FrameTypeData;
	mbConfig.id     = FLEXCAN_ID_STD(0x0);

	FLEXCAN_SetFDRxMbConfig(canbase, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

	/* Setup Tx Message Buffer. */
	FLEXCAN_SetFDTxMbConfig(canbase, TX_MESSAGE_BUFFER_NUM, true);

	/* Create FlexCAN handle structure and set call back function. */
	FLEXCAN_TransferCreateHandle(canbase, handle, flexcan_callback, (void *)cb);

	return 0;
}

int32_t flexcan_down(can_handler_data_t *handler)
{
	flexcan_handle_t *handle = &handler->flexcan.handle;
	CAN_Type *canbase = handler->flexcan.base;

	FLEXCAN_TransferCreateHandle(canbase, handle, NULL, (void *)NULL);
	FLEXCAN_EnterFreezeMode(canbase);
	FLEXCAN_Deinit(canbase);

	return 0;
}

void flexcan_task(void *param)
{
	can_handler_data_t *priv = (struct can_handler_data *)param;
	status_t  status;
	uint32_t txlen;
	uint32_t rxlen;
	uint32_t addr;
	int ret;

	if (priv->type != TYPE_FLEXCAN)
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

			// Tx cleanup:

			priv->flexcan.cb.txdone = false;
			priv->flexcan.tx.framefd = NULL;
			priv->rxbuf = NULL;

			// Rx cleanup:

			priv->flexcan.cb.rxdone = false;
			priv->flexcan.rx.framefd = NULL;
			priv->flexcan.cb.rxflag = 0;

			// Block until CAN device is re-opened

			if (xSemaphoreTake(priv->runsem, portMAX_DELAY) != pdTRUE )
			{
				(void)PRINTF("%s: failed to take semaphore\r\n", priv->name);
			}

			(void)PRINTF("%s: start...\r\n", priv->name);
		}

		if (priv->flexcan.cb.failed)
		{
			(void)PRINTF("%s: errors (%u)\r\n", priv->name, priv->flexcan.cb.errors);
			priv->flexcan.cb.failed = false;
			priv->flexcan.cb.errors = 0x0;
		}

		if (priv->flexcan.cb.txdone)
		{
			if (priv->rxbuf)
			{
				if (priv->led.present)
				{
					GPIO_PinWrite(priv->led.base, priv->led.pin,
							priv->led.active_low ? 1U : 0U);
				}
			}

			priv->flexcan.cb.txdone = false;
			priv->flexcan.tx.framefd = NULL;
			priv->rxbuf = NULL;
		}
		else
		{
			if (!priv->flexcan.tx.framefd)
			{
				if (!priv->rxbuf)
				{
					(void)rpmsg_queue_recv_nocopy(priv->rpmsg, priv->queue, (uint32_t *)&addr, (char **)&priv->rxbuf, &rxlen, 0);
					if (priv->rxbuf)
					{
						ret = to_flexcan(&priv->flexcan.txframe, priv->rxbuf, rxlen);
						(void)rpmsg_lite_release_rx_buffer(priv->rpmsg, priv->rxbuf);

						if (ret < 0)
						{
							(void)PRINTF("%s: invalid host frame: %d\r\n", priv->name, ret);
							priv->rxbuf = NULL;
						}

					}
				}

				if (priv->rxbuf)
				{
					priv->flexcan.tx.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
					priv->flexcan.tx.framefd = &priv->flexcan.txframe;
					priv->flexcan.cb.txdone = false;

					status = FLEXCAN_TransferFDSendNonBlocking(priv->flexcan.base, &priv->flexcan.handle, &priv->flexcan.tx);
					if (status != kStatus_Success)
					{
						(void)PRINTF("%s: failed to prepare xmit len %d: %d\r\n", priv->name, rxlen, status);
						priv->flexcan.tx.framefd = NULL;
					} else {
						if (priv->led.present)
						{
							GPIO_PinWrite(priv->led.base, priv->led.pin,
									priv->led.active_low ? 0U : 1U);
						}
					}
				}
			}
		}

		if (priv->flexcan.cb.rxdone)
		{
			switch (priv->flexcan.cb.rxflag)
			{
				case kStatus_FLEXCAN_RxOverflow:
					FLEXCAN_ClearMbStatusFlags(priv->flexcan.base,
								kFLEXCAN_RxFifoOverflowFlag);
					/* drop fame */
					(void)PRINTF("%s: Rx Overflow\r\n", priv->name);
					priv->flexcan.rx.framefd = NULL;
					break;
				case kStatus_FLEXCAN_RxBusy:
					/* drop fame */
					(void)PRINTF("%s: Rx Busy\r\n", priv->name);
					priv->flexcan.rx.framefd = NULL;
					break;
				default:
					/* normal reception */
					ret = from_flexcan(priv->txbuf, priv->flexcan.rx.framefd);
					if (ret > 0)
					{
						rpmsg_lite_send_nocopy(priv->rpmsg, priv->ept, remote_addr,
								priv->txbuf, ret);
						priv->txbuf = NULL;
					}
					else
					{
						(void)PRINTF("%s: invalid device frame: %d\r\n", priv->name, ret);
					}

					priv->flexcan.rx.framefd = NULL;
					break;
			}

			priv->flexcan.cb.rxdone = false;
			priv->flexcan.cb.rxflag = 0;
		}
		else
		{
			if (!priv->flexcan.rx.framefd)
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
					priv->flexcan.rx.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
					priv->flexcan.rx.framefd = &priv->flexcan.rxframe;
					priv->flexcan.cb.rxdone = false;

					status = FLEXCAN_TransferFDReceiveNonBlocking(priv->flexcan.base, &priv->flexcan.handle, &priv->flexcan.rx);
					if (status != kStatus_Success) {
						(void)PRINTF("%s: failed to prepare rx len %d: %d\r\n", priv->name, txlen, status);
						priv->flexcan.rx.framefd = NULL;
					}
				}
			}
		}

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

int32_t flexcan_bitrate(can_handler_data_t *handler, u32 *bitrate, u32 *dbitrate)
{
	handler->bitrate = *bitrate;
	handler->dbitrate = *dbitrate;

	return 0;
}
