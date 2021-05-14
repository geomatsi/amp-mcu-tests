/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Linux compatibility header
 *
 */

#include <stdlib.h>

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

/*
 *  CAN frame format conversion
 *  - master(Linux) to remote (NXP FreeRTOS)
 *
 */
void m2r(flexcan_frame_t *frame, struct can_frame *cfd)
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

	frame->dataWord0 = swap(cfd->data[0]);
	frame->dataWord1 = swap(cfd->data[1]);
}

/*
 *  CAN frame format conversion
 *  - remote (NXP FreeRTOS) to master(Linux)
 *
 */

void r2m(struct can_frame *cfd, flexcan_frame_t *frame)
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

	cfd->data[0] = swap(frame->dataWord0);
	cfd->data[1] = swap(frame->dataWord1);
}


