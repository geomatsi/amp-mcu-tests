#include <stdlib.h>

#include "compat_linux.h"

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


