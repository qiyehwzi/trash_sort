#include "chassis_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "usart.h"
#include "tim.h"
#include "pid.h"
#include "loop_task.h"

//x -> picture, y -> clamp

uint32_t IDLEIRQ = 0;
uint32_t DMA_Residual_length = 0;                      
uint8_t rx_buffer[40]={0};  //接收数据缓存数组
uint8_t tx_buffer[20]={0};
uint8_t rx_buffer_win[16]={0};
usart_rx_mes usart_rx_mes_real;
usart_tx_mes uasrt_tx_mes_real;
int serial_rx_flag;
int serial_rx_mes_flag;
int motor_flag;

int16_t picture_reset_count;
int16_t clamp_reset_count;

chassis_t chassis;

void chassis_init(chassis_t *chassis_init);
static void chassis_set_mode(chassis_t *chassis_set_mode);
static void chassis_feedback_update(chassis_t *chassis_update);
static void chassis_control_loop(chassis_t *chassis_control_loop);
static void chassis_set_contorl(chassis_t *chassis_control);

void DMA_UART8_Send(uint8_t *buf,uint8_t len)//串口发送封装
{
//	if(HAL_UART_Transmit_DMA(&huart8, buf,len)!= HAL_OK) //判断是否发送正常，如果出现异常则进入异常中断函数
//	{
//	 Error_Handler();
//	}
	HAL_UART_Transmit_DMA(&huart8, buf,len);
}

void DMA_UART8_Read(uint8_t *Data,uint8_t len)//串口接收封装
{
	HAL_UART_Receive_DMA(&huart8,Data,len);//重新打开DMA接收
}

void StartchassisTask(void const * argument)
{
	osDelay(500);
	chassis_init(&chassis);
	
	uasrt_tx_mes_real.head.head1 = HEAD1;
	uasrt_tx_mes_real.head.head2 = HEAD2;
	uasrt_tx_mes_real.tail.tail1 = TAIL1;
	uasrt_tx_mes_real.tail.tail2 = TAIL2;
	
//	uasrt_tx_mes_real.load = 1;
//	uasrt_tx_mes_real.state = 0;
//	uasrt_tx_mes_real.temp = 2.718;
	
	while(1)
  {
    osDelay(1);
		
		chassis.dt = DWT_GetDeltaT(&chassis.DWT_Count);
		
		chassis_set_mode(&chassis);
		chassis_feedback_update(&chassis);
		chassis_set_contorl(&chassis);
		chassis_control_loop(&chassis);
		
		if(chassis.chassis_mode == 0)
		{
			CAN_chassis_can1(0,0,0,0);
		}
		else
		{
			CAN_chassis_can1(0,chassis.motor_picture.give_current,chassis.motor_clamp.give_current,0);
		}
		
		if(chassis.chassis_mode == 1 && motor_flag == 0)
		{
			chassis.motor_mode_x = 1;
			chassis.motor_mode_y = 1;
			motor_flag++;
		}
		
		if(chassis.chassis_mode == 0)
		{
			chassis.motor_mode_x = 0;
			chassis.motor_mode_y = 0;
			motor_flag = 0;
		}
		
		osDelay(1);
		
		memcpy(tx_buffer,&uasrt_tx_mes_real,20);
		DMA_UART8_Send(tx_buffer, sizeof(tx_buffer));
		
		if(recv_end_flag == 1)  //接收完成标志
		{
				rx_len = 0;//清除计数
				recv_end_flag = 0;//清除接收结束标志位
//			for(uint8_t i=0;i<rx_len;i++)
//				{
//					rx_buffer[i]=0;//清接收缓存
//				}
				memset(rx_buffer,0,rx_len);
		}
		
//		int i=1000; //循环时间间隔1s
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,50); //相当于一个周期内（20ms）有0.5ms高脉冲
//		osDelay(i);
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,250); //相当于一个周期内（20ms）有2.5ms高脉冲
//		osDelay(i);
		
		HAL_UART_Receive_DMA(&huart8,rx_buffer,BUFFER_SIZE);//重新打开DMA接收
  }
}

void chassis_init(chassis_t *chassis_init)
{
	chassis_init->chassis_mode = 0;
	chassis_init->picture_ready_mode = 0;
	chassis_init->clamp_ready_mode = 0;
	chassis_init->motor_clamp.motor_measure = get_clamp_motor_point();
	chassis_init->motor_picture.motor_measure = get_picture_motor_point();
	
	//初始化电机pid
	//夹矿
	PID_Init(&chassis_init->picture_motor_position_pid, X_POSITION_PID_MAX_OUT, X_POSITION_PID_MAX_IOUT,0.0f,
					X_POSITION_PID_KP, X_POSITION_PID_KI, X_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
	PID_Init(&chassis_init->picture_motor_speed_pid, X_SPEED_PID_MAX_OUT, X_SPEED_PID_MAX_IOUT,0.0f,
					X_SPEED_PID_KP, X_SPEED_PID_KI, X_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);		
	//图传抬升
	PID_Init(&chassis_init->clamp_motor_position_pid, Y_POSITION_PID_MAX_OUT, Y_POSITION_PID_MAX_IOUT,0.0f,
					Y_POSITION_PID_KP, Y_POSITION_PID_KI, Y_POSITION_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,4,0x11);
	PID_Init(&chassis_init->clamp_motor_speed_pid, Y_SPEED_PID_MAX_OUT, Y_SPEED_PID_MAX_IOUT,0.0f,
					Y_SPEED_PID_KP, Y_SPEED_PID_KI, Y_SPEED_PID_KD,0.0f,0.0f,0.00106103295394596890512589175582f,0.0f,5,0x11);
	//更新一次数据
	chassis_feedback_update(chassis_init);	
	chassis_init->motor_picture.offset_ecd = chassis_init->motor_picture.motor_measure->ecd;
	chassis_init->motor_clamp.offset_ecd = chassis_init->motor_clamp.motor_measure->ecd;
}

static void chassis_set_mode(chassis_t *chassis_set_mode)
{
		if (chassis_set_mode == NULL)
		{
				return;
		}
		
		if(chassis_set_mode->motor_mode_x == 1)
		{
				if(fabs(chassis_set_mode->motor_picture.speed) < 1.0f)
						picture_reset_count++;
				else
						picture_reset_count = 0;
				
				if(picture_reset_count >= 150)
				{
					chassis_set_mode->picture_ready_mode = 1;
				}
		}
		
		if(chassis_set_mode->motor_mode_y == 1)
		{
				if(fabs(chassis_set_mode->motor_clamp.speed) < 1.0f)
						clamp_reset_count++;
				else
						clamp_reset_count = 0;
				
				if(clamp_reset_count >= 150)
				{
					chassis_set_mode->clamp_ready_mode = 1;
				}
		}
		
		if(chassis_set_mode->picture_ready_mode && chassis_set_mode->clamp_ready_mode)
		{
				chassis_set_mode->motor_picture.round_cnt = 0;
				chassis_set_mode->motor_picture.offset_ecd = chassis_set_mode->motor_picture.motor_measure->ecd;
				chassis_set_mode->motor_mode_x = 2;
				picture_reset_count = 0;

				chassis_set_mode->motor_clamp.round_cnt = 0;
				chassis_set_mode->motor_clamp.offset_ecd = chassis_set_mode->motor_clamp.motor_measure->ecd;
				chassis_set_mode->motor_mode_y = 2;
				clamp_reset_count = 0;
			
				chassis_set_mode->picture_ready_mode = 0;
				chassis_set_mode->clamp_ready_mode = 0;
		}
}

static void chassis_feedback_update(chassis_t *chassis_update)
{
    if (chassis_update == NULL)
    {
        return;
    }
		//夹矿
		chassis_update->motor_picture.position = (chassis_update->motor_picture.round_cnt * ECD_RANGE + chassis_update->motor_picture.motor_measure->ecd - chassis_update->motor_picture.offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;
    chassis_update->motor_picture.speed = chassis_update->motor_picture.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;		
		//图传抬升
		chassis_update->motor_clamp.position = (chassis_update->motor_clamp.round_cnt * ECD_RANGE + chassis_update->motor_clamp.motor_measure->ecd - chassis_update->motor_clamp.offset_ecd) * MOTOR_ECD_TO_ANGLE_2006;
    chassis_update->motor_clamp.speed = chassis_update->motor_clamp.motor_measure->speed_rpm * RPM_TO_RAD_S * REDUCTION_RATIO_2006;
}

static void chassis_set_contorl(chassis_t *chassis_control)
{
		if (chassis_control == NULL)
		{
				return;
		}
		
		if(chassis_control->motor_mode_x == 1)
		{
				chassis_control->motor_picture.speed_set = 7.0f;
		}
		else if(chassis_control->motor_mode_x ==2)
		{
				if(usart_rx_mes_real.variety == 0)
				{
					chassis_control->motor_picture.position_set = -1.00f;
				}
				else
				{
					chassis_control->motor_picture.position_set = x_position;
				}
		}
		
		if(chassis_control->motor_mode_y == 1)
		{
				chassis_control->motor_clamp.speed_set = 7.0f;
		}
		else if(chassis_control->motor_mode_y ==2)
		{
				if(usart_rx_mes_real.variety == 0)
				{
					chassis_control->motor_picture.position_set = -1.00f;
				}
				else
				{
					chassis_control->motor_picture.position_set = y_position;
				}
		}
}

static void chassis_control_loop(chassis_t *chassis_control_loop)
{

		if(chassis_control_loop->motor_mode_x == 0)
		{
				chassis_control_loop->motor_clamp.give_current = 0;
		}
		else if(chassis_control_loop->motor_mode_x == 1)
		{
//				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);
//				chassis_control_loop->motor_clamp.give_current = int16_constrain((int16_t)(chassis_control_loop->clamp_motor_speed_pid.Output),-7000,7000);
				chassis_control_loop->motor_clamp.give_current = 1000;
		}
		else if(chassis_control_loop->motor_mode_x == 2)
		{
				PID_Calculate(&chassis_control_loop->clamp_motor_position_pid, chassis_control_loop->motor_clamp.position, chassis_control_loop->motor_clamp.position_set);
				chassis_control_loop->motor_clamp.speed_set = chassis_control_loop->clamp_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->clamp_motor_speed_pid, chassis_control_loop->motor_clamp.speed, chassis_control_loop->motor_clamp.speed_set);
				chassis_control_loop->motor_clamp.give_current = (int16_t)(chassis_control_loop->clamp_motor_speed_pid.Output);
		}
		
		if(chassis_control_loop->motor_mode_y == 0)
		{
				chassis_control_loop->motor_picture.give_current = 0;
		}
		else if(chassis_control_loop->motor_mode_y == 1)
		{
//				PID_Calculate(&chassis_control_loop->picture_motor_speed_pid, chassis_control_loop->motor_picture.speed, chassis_control_loop->motor_picture.speed_set);
//				chassis_control_loop->motor_picture.give_current = int16_constrain((int16_t)(chassis_control_loop->picture_motor_speed_pid.Output),-7000,7000);
				chassis_control_loop->motor_picture.give_current = 1000;
		}
		else if(chassis_control_loop->motor_mode_y == 2)
		{
				PID_Calculate(&chassis_control_loop->picture_motor_position_pid, chassis_control_loop->motor_picture.position, chassis_control_loop->motor_picture.position_set);
				chassis_control_loop->motor_picture.speed_set = chassis_control_loop->picture_motor_position_pid.Output;
				PID_Calculate(&chassis_control_loop->picture_motor_speed_pid, chassis_control_loop->motor_picture.speed, chassis_control_loop->motor_picture.speed_set);
				chassis_control_loop->motor_picture.give_current = (int16_t)(chassis_control_loop->picture_motor_speed_pid.Output);
		}
}

void UART8_IRQHandler(void)
{
	uint32_t tmp_flag = 0;
	uint32_t temp;
	
	tmp_flag =__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE); //获取IDLE标志位
	if((tmp_flag != RESET))//idle标志被置位
	{
			IDLEIRQ++;
			__HAL_DMA_DISABLE(&hdma_uart8_rx);//disable
			
			__HAL_UART_CLEAR_IDLEFLAG(&huart8);//清除标志位
			
			__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx,DMA_FLAG_TCIF2_6);
			
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);// 获取DMA中未传输的数据个数   
			
			rx_len =  BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
			
//			if(serial_rx_flag == 0 && rx_buffer[0] == 0xFE)
//			{
//				serial_rx_flag = 1;
//			}
//			
//			if(serial_rx_flag == 1 && rx_buffer[0] == 0xEE)
//			{
//				serial_rx_flag = 2;
//			}
//			
//			if(serial_rx_flag == 2 && serial_rx_mes_flag < 16)
//			{
//				rx_buffer_win[serial_rx_mes_flag] = rx_buffer[0];
//				serial_rx_mes_flag++;
//			}
//			else if(serial_rx_flag == 2 && serial_rx_mes_flag >= 16)
//			{
//				serial_rx_mes_flag = 0;
//				serial_rx_flag = 3;
//			}
//			
//			if(serial_rx_flag == 3 && rx_buffer[0] == 0xFF)
//			{
//				serial_rx_flag = 4;
//			}
//			if(serial_rx_flag == 3 && rx_buffer[0] != 0xFF)
//			{
//				serial_rx_flag = 0;
//			}
//			
//			if(serial_rx_flag == 4 && rx_buffer[0] == 0xFF)
//			{
//				serial_rx_flag = 0;
//				memcpy(&usart_rx_mes_real,rx_buffer_win,16);
//			}
//			if(serial_rx_flag == 4 && rx_buffer[0] != 0xFF)
//			{
//				serial_rx_flag = 0;
//			}
				
			memcpy(&usart_rx_mes_real,rx_buffer,20);
				
//			if(rx_len == 2)
//			{
					recv_end_flag = 1;	// 接受完成标志位置1
//			}
				
			__HAL_DMA_ENABLE(&hdma_uart8_rx);
	 }
		HAL_UART_IRQHandler(&huart8);//使用HAL库启动DMA必须使用
}

//A板主按键启动
//void EXTI2_IRQHandler(void)
//{
//	if(chassis.chassis_mode == 0)
//	{
//		chassis.chassis_mode = 1;
//	}
//	else
//	{
//		chassis.chassis_mode = 0;
//	}
//	
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
//}
