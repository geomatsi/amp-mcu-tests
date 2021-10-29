/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Linux compatibility header
 *
 */

#include "compat_linux.h"

/* CAN DLC conversion helpers */

static const uint8_t dlc2len[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

uint8_t can_dlc2len(uint8_t can_dlc)
{
	        return dlc2len[can_dlc & 0x0F];
}

static const uint8_t len2dlc[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8,	/* 0 - 8 */
	9, 9, 9, 9,			/* 9 - 12 */
	10, 10, 10, 10,			/* 13 - 16 */
	11, 11, 11, 11,			/* 17 - 20 */
	12, 12, 12, 12,			/* 21 - 24 */
	13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
	14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
	14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
	15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
	15, 15, 15, 15, 15, 15, 15, 15	/* 57 - 64 */
};

uint8_t can_len2dlc(uint8_t len)
{
	if (len > 64)
		return 0xF;

	return len2dlc[len];
}

int to_flexcan(flexcan_fd_frame_t *frame, void *buffer, uint32_t len)
{
	struct canfd_frame *cfd = (struct canfd_frame *)buffer;
	uint32_t *data = (uint32_t *)cfd->data;
	int i;

	memset(frame, 0x0, sizeof(*frame));

	if (cfd->can_id & CAN_EFF_FLAG)
	{
		frame->id = FLEXCAN_ID_EXT(cfd->can_id);
		frame->format = kFLEXCAN_FrameFormatExtend;
	}
	else
	{
		frame->id = FLEXCAN_ID_STD(cfd->can_id);
		frame->format = kFLEXCAN_FrameFormatStandard;
	}


	if (cfd->can_id & CAN_RTR_FLAG)
	{
		frame->type = kFLEXCAN_FrameTypeRemote;
	}
	else
	{
		frame->type = kFLEXCAN_FrameTypeData;
	}

	frame->length = can_len2dlc(cfd->len);

	switch (len)
	{
		case CANFD_MTU:
			if (cfd->flags & CANFD_BRS)
			{
				frame->brs = 0x1;
			}
			break;
		case CAN_MTU:
			break;
		default:
			return -1;
	}

	for (i = 0; i < cfd->len; i += 4)
	{
		frame->dataWord[i / 4] = swap(*(data + i/4));
	}

	return 0;
}

int from_flexcan(void *buffer, flexcan_fd_frame_t *frame)
{
	struct canfd_frame *cfd = (struct canfd_frame *)buffer;
	uint32_t *data = (uint32_t *)cfd->data;
	int mtu = CANFD_MTU;
	int i;

	memset(cfd, 0x0, sizeof(*cfd));

	cfd->len = can_dlc2len(frame->length);

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

	for (i = 0; i < cfd->len; i += 4)
	{
		*(data + i / 4) = swap(frame->dataWord[i / 4]);
	}

	return mtu;
}

int from_mcpcan(void *buffer, CAN_RX_MSGOBJ *obj, uint8_t *data)
{
	struct canfd_frame *cfd = (struct canfd_frame *)buffer;
	int mtu = CAN_MTU;

	memset(cfd, 0x0, sizeof(*cfd));

	if (obj->bF.ctrl.IDE)
	{
		cfd->can_id = obj->bF.id.SID << 18 | obj->bF.id.EID | CAN_EFF_FLAG;
	}
	else
	{
		cfd->can_id = obj->bF.id.SID & CAN_SFF_MASK;
	}

	if (obj->bF.ctrl.RTR)
	{
		cfd->can_id |= CAN_RTR_FLAG;
	}

	if (obj->bF.ctrl.FDF)
	{
		if (obj->bF.ctrl.BRS)
		{
			cfd->flags |= CANFD_BRS;
		}

		if (obj->bF.ctrl.ESI)
		{
			cfd->flags |= CANFD_ESI;
		}

		mtu = CANFD_MTU;
	}

	cfd->len = can_dlc2len(obj->bF.ctrl.DLC);
	memcpy(cfd->data, data, cfd->len);

	return mtu;
}

int to_mcpcan(CAN_TX_MSGOBJ *obj, uint8_t *data, void *buffer, uint32_t len)
{
	struct canfd_frame *cfd = (struct canfd_frame *)buffer;

	memset(data, 0x0, MAX_DATA_BYTES);
	memset(obj, 0x0, sizeof(*obj));

	switch (len)
	{
		case CANFD_MTU:
			if (cfd->flags & CANFD_BRS)
			{
				obj->bF.ctrl.BRS = 0x1;
			}

			if (cfd->flags & CANFD_ESI)
			{
				obj->bF.ctrl.ESI = 0x1;
			}

			obj->bF.ctrl.FDF = 0x1;
			break;
		case CAN_MTU:
			break;
		default:
			return -1;
			break;
	}

	if (cfd->can_id & CAN_EFF_FLAG)
	{
		obj->bF.id.SID = (cfd->can_id & CAN_EFF_MASK) >> 18;
		obj->bF.id.EID = (cfd->can_id & CAN_EFF_MASK) & ~(CAN_SFF_MASK << 18);
		obj->bF.ctrl.IDE = 0x1;
	}
	else
	{
		obj->bF.id.SID = cfd->can_id & CAN_SFF_MASK;
	}

	if (cfd->can_id & CAN_RTR_FLAG)
	{
		obj->bF.ctrl.RTR = 0x1;
	}

	obj->bF.ctrl.DLC = can_len2dlc(cfd->len);
	memcpy(data, cfd->data, cfd->len);

	return 0;
}
