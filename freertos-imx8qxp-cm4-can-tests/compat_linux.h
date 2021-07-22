/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Linux compatibility header
 *
 */

#ifndef COMPAT_LINUX_H_
#define COMPAT_LINUX_H_

#include <stdint.h>

#include "fsl_flexcan.h"

#include "drv_canfdspi_api.h"

#define __packed	__attribute__((__packed__))

typedef uint16_t	__le16;
typedef uint32_t	__le32;
typedef uint8_t		u8;

#define  EIO              5  /* I/O error */
#define  EFAULT          14  /* Bad address */
#define  EINVAL          22  /* Invalid argument */
#define  ENODEV          19  /* No such device */
#define  ENOSYS          38  /* Function not implemented */
#define  EALREADY       114  /* Operation already in progress */

#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */

#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */

#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */

#define swap(x) ((uint32_t)(                         		\
	(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |	\
	(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |	\
	(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |	\
	(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))

struct can_frame {
	uint32_t can_id;	/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t len;		/* frame payload length in byte (0 .. 8) */
	uint8_t __pad;		/* padding */
	uint8_t __res0;		/* reserved / padding */
	uint8_t __res1;		/* reserved / padding */
	uint32_t data[2] __attribute__((aligned(8)));
};

struct canfd_frame {
	uint32_t can_id;	/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t len;		/* frame payload length in byte */
	uint8_t flags;		/* additional flags for CAN FD */
	uint8_t __res0;		/* reserved / padding */
	uint8_t __res1;		/* reserved / padding */
	uint8_t data[64] __attribute__((aligned(8)));
};

#define CAN_MTU		(sizeof(struct can_frame))
#define CANFD_MTU	(sizeof(struct canfd_frame))

uint8_t can_dlc2len(uint8_t);
uint8_t can_len2dlc(uint8_t);

int from_flexcan(void *, flexcan_fd_frame_t *);
int to_flexcan(flexcan_fd_frame_t *, void *, uint32_t);

int from_mcpcan(void *, CAN_RX_MSGOBJ *, uint8_t *);
int to_mcpcan(CAN_TX_MSGOBJ *, uint8_t *, void *, uint32_t);

#endif /* COMPAT_LINUX_H_ */
