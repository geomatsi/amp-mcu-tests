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

#endif /* COMPAT_LINUX_H_ */
