/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CAN_LINUX_H_
#define CAN_LINUX_H_

#include "fsl_flexcan.h"

#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */

#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */

#define swap(x) ((uint32_t)(                         		\
	(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |	\
	(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |	\
	(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |	\
	(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))

struct can_frame {
	uint32_t can_id;
	uint8_t len;
	uint8_t flags;
	uint8_t __res0;
	uint8_t __res1;
	uint32_t data[2] __attribute__((aligned(8)));
};

void r2m(struct can_frame *, flexcan_frame_t *);
void m2r(flexcan_frame_t *, struct can_frame *);

#endif /* CAN_LINUX_H_ */
