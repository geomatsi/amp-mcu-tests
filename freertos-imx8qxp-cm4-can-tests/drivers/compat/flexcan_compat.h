#ifndef FLEXCAN_COMPAT_H_
#define FLEXCAN_COMPAT_H_

#include "common.h"

int32_t flexcan_init(can_handler_data_t *handler);
int32_t flexcan_up(can_handler_data_t *handler);
int32_t flexcan_down(can_handler_data_t *handler);
void flexcan_task(void *param);

#endif /* FLEXCAN_COMPAT_H_ */
