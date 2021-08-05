#ifndef MCPCAN_COMPAT_H_
#define MCPCAN_COMPAT_H_

#include "common.h"

int32_t mcp_init(can_handler_data_t *handler);
int32_t mcp_up(can_handler_data_t *handler);
int32_t mcp_down(can_handler_data_t *handler);
int32_t mcp_bitrate(can_handler_data_t *handler, u32 *bitrate, u32 *dbitrate);

void mcpcan_task(void *param);

#endif /* MCPCAN_COMPAT_H_ */
