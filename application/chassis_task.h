#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stdint.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "TD.h"
#include "controller.h"

#define COMPETITION 1

#define HEAD1 0xFE
#define HEAD2 0xEE

#define TAIL1 0xFF
#define TAIL2 0xFF

#define X_POSITION_PID_MAX_OUT 100.0f
#define X_POSITION_PID_MAX_IOUT 0.0f
#define X_POSITION_PID_KP 2.0f
#define X_POSITION_PID_KI 0.0f
#define X_POSITION_PID_KD 0.0f

#define X_SPEED_PID_MAX_OUT 10000.0f
#define X_SPEED_PID_MAX_IOUT 0.0f
#define X_SPEED_PID_KP 2000.0f
#define X_SPEED_PID_KI 0.0f
#define X_SPEED_PID_KD 0.0f


#define Y_POSITION_PID_MAX_OUT 100.0f
#define Y_POSITION_PID_MAX_IOUT 0.0f
#define Y_POSITION_PID_KP 3.0f
#define Y_POSITION_PID_KI 0.0f
#define Y_POSITION_PID_KD 0.0f

#define Y_SPEED_PID_MAX_OUT 10000.0f
#define Y_SPEED_PID_MAX_IOUT 0.0f
#define Y_SPEED_PID_KP 3000.0f
#define Y_SPEED_PID_KI 0.0f
#define Y_SPEED_PID_KD 0.0f

//编码器参考值
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8192
//减速比
#define REDUCTION_RATIO_3508				187.0f/3591.0f
#define REDUCTION_RATIO_2006				1.0f/36.0f
//电机单位转化参数
#define MOTOR_ECD_TO_ANGLE_2006     0.000021305288720633f
#define MOTOR_ECD_TO_ANGLE_3508     0.00003994074176199f
#define RPM_TO_RAD_S							  0.104719755119659774f

#pragma pack(1)
typedef struct
{
  uint8_t	head1;
  uint8_t	head2;
} header;

typedef struct
{
  uint8_t	tail1;
  uint8_t	tail2;
} tailer;

typedef struct
{
  header head;
  fp32 	x_position;
  fp32 	y_position;
  int		variety;//0,1,2,3
	fp32	yaw_flag;
	tailer tail;
} usart_rx_mes;

typedef struct
{
  header head;
  int 	is_ready;
  int		state;
	int		load;
	float	temp;
	tailer tail;
} usart_tx_mes;
#pragma pack()

typedef struct
{
  const motor_measure_t *motor_measure;
  fp32 		speed;
  fp32 		speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	const motor_measure_t *motor_measure;
	fp32 		speed;
	fp32 		speed_set;	
	fp32 		position;
	fp32 		position_set;
	fp32 		offset_ecd;
	int32_t round_cnt;
	int16_t give_current;
}position_motor_t;

typedef struct
{
	position_motor_t motor_picture;								
	position_motor_t motor_clamp;			  			
		                  
	PID_t picture_motor_position_pid;
	PID_t picture_motor_speed_pid;
	PID_t clamp_motor_position_pid;
	PID_t clamp_motor_speed_pid;
	
	uint8_t  chassis_mode;//模式
	uint8_t  picture_ready_mode;
	uint8_t  clamp_ready_mode;
	uint8_t  motor_mode_x;
	uint8_t  motor_mode_y;
	uint8_t  crash_motor_direction;
	
	float dt;
	uint32_t  DWT_Count;	
} chassis_t;

extern uint32_t IDLEIRQ;
extern uint32_t DMA_Residual_length;                      
extern uint8_t rx_buffer[40];  //接收数据缓存数组
extern uint8_t tx_buffer[20];
extern uint8_t rx_buffer_win[16];
extern usart_rx_mes usart_rx_mes_real;
extern usart_tx_mes uasrt_tx_mes_real;
extern int serial_rx_flag;
extern int serial_rx_mes_flag;
extern int motor_flag;

extern int16_t picture_reset_count;
extern int16_t clamp_reset_count;
extern chassis_t chassis;
extern void StartchassisTask(void const * argument);

#endif
