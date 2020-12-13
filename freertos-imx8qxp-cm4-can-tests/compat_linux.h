/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Linux compatibility header
 *
 */

#ifndef COMPAT_LINUX_H_
#define COMPAT_LINUX_H_

#include <stdint.h>

#define __packed	__attribute__((__packed__))

typedef uint16_t	__le16;
typedef uint32_t	__le32;
typedef uint8_t		u8;

#define  EFAULT          14  /* Bad address */
#define  ENODEV          19  /* No such device */
#define  ENOSYS          38  /* Function not implemented */
#define  EALREADY       114  /* Operation already in progress */

#endif /* COMPAT_LINUX_H_ */
