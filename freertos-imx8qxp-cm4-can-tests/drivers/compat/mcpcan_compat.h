#ifndef MCPCAN_COMPAT_H_
#define MCPCAN_COMPAT_H_

#include "common.h"

int32_t mcp_init(can_handler_data_t *handler);
int32_t mcp_up(can_handler_data_t *handler);
int32_t mcp_down(can_handler_data_t *handler);
void mcpcan_task(void *param);

#endif /* MCPCAN_COMPAT_H_ */
