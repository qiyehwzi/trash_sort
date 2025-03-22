#ifndef LOOP_TASK_H
#define LOOP_TASK_H

#include "struct_typedef.h"
#include "chassis_task.h"

extern fp32 x_position;
extern fp32 y_position;
extern int yaw_pitch_flag;

extern usart_rx_mes usart_rx_mes_real_converted;

extern void loop_task(void const * argument);

#endif
