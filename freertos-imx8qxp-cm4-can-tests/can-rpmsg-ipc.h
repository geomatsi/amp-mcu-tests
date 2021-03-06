/* SPDX-License-Identifier: GPL-2.0-only
 *
 * FlexCAN rpmsg driver driver
 *
 */

#include "compat_linux.h"

#define CM4_MAJOR_VER	0
#define CM4_MINOR_VER	1

struct can_rpmsg_ctrl_hdr {
	__le16 type;
	__le16 len;
} __packed;

/* events */

struct can_rpmsg_evt {
	struct can_rpmsg_ctrl_hdr hdr;
	__le16 id;
	u8 rsvd[2];
} __packed;

/* commands */

enum can_rpmsg_ctrl_type {
	CAN_RPMSG_CTRL_CMD = 1,
	CAN_RPMSG_CTRL_RSP = 2,
	CAN_RPMSG_CTRL_EVT = 3,
};

enum can_rpmsg_cmd_type {
	CAN_RPMSG_CMD_INIT	= 0x0001,
	CAN_RPMSG_CMD_UP	= 0x0002,
	CAN_RPMSG_CMD_DOWN	= 0x0003,
};

struct can_rpmsg_cmd {
	struct can_rpmsg_ctrl_hdr hdr;
	__le16 id;
	__le16 seq;
} __packed;

struct can_rpmsg_rsp {
	struct can_rpmsg_ctrl_hdr hdr;
	__le16 id;
	__le16 seq;
	__le16 result;
	u8 rsvd[2];
} __packed;

struct can_rpmsg_cmd_init {
	struct can_rpmsg_cmd hdr;
	__le16 major;
	__le16 minor;
} __packed;

struct can_rpmsg_cmd_init_rsp {
	struct can_rpmsg_rsp hdr;
	__le16 major;
	__le16 minor;
	__le32 bitrate;
	__le16 devnum;
} __packed;

struct can_rpmsg_cmd_up {
	struct can_rpmsg_cmd hdr;
	__le32 index;
} __packed;

struct can_rpmsg_cmd_down {
	struct can_rpmsg_cmd hdr;
	__le32 index;
} __packed;
