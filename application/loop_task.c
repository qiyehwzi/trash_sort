#include "loop_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gpio.h"
#include "tim.h"

//variety:0没垃圾
//1：可回收
//2：有害
//3：厨余
//4：其它

fp32 x_position;
fp32 y_position;
int yaw_pitch_flag;

usart_rx_mes usart_rx_mes_real_converted;

void action_motors_function(fp32 side_baffle, fp32 up_holding_jaw, fp32 middele_holding_jaw, fp32 down_holding_jaw, fp32 rotate, fp32 pour, fp32 x, fp32 y);
void trash_zero_yaw_pitch(void);
void trash_one_yaw_pitch(void);
void trash_two_yaw_pitch(void);
void trash_three_yaw_pitch(void);
void trash_four_yaw_pitch(void);

void loop_task(void const * argument)
{	
	osDelay(2000);
	//四个挡板
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,205);//4 相当于一个周期内（20ms）有0.5ms高脉冲 下
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,250);//2
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,220);//1
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);//3
	//夹爪
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,195); //向上
	osDelay(2000);
	trash_zero_yaw_pitch();
	osDelay(2000);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,183);//4 相当于一个周期内（20ms）有0.5ms高脉冲 上
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,220);//2
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,170);//1
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);//3
	#if COMPETITION
	chassis.chassis_mode = 0;
	#endif
	#if 1-COMPETITION
	chassis.chassis_mode = 1;
	#endif
	osDelay(20000);
	uasrt_tx_mes_real.is_ready = 1;
  for(;;)
  {
//			#if 1-COMPETITION
//		//四个挡板
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,90); //相当于一个周期内（20ms）有0.5ms高脉冲 上
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,90);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,90);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);
		
		//夹爪（从上到下tim4的ch1,2,3）
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,170); //向下
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,145);//x
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,170);//向上
		
		//倾倒
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,120);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,80);
		
//		osDelay(1000);
		
//		//四个挡板
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,150); //相当于一个周期内（20ms）有2.5ms高脉冲 下
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,150);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,150);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);
		
//		//夹爪
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,195); //向上
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,220);//y
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,245);//向下
		
		//倾倒
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,230);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,220);
//		osDelay(1000);
//		#endif
			
			
//		if(usart_rx_mes_real_converted.variety == 0)
//		{
//			action_motors_function(0.0,1.0,0.0,0.0,0.0,0.0,-1.0,-1.0);
//		}
//		else if(usart_rx_mes_real_converted.variety == 1)
//		{
//			action_motors_function(0.0,1.0,0.0,0.0,0.0,0.0,-1.0,-1.0);
//		}
//		else if(usart_rx_mes_real_converted.variety == 2)
//		{
//			action_motors_function(0.0,1.0,0.0,0.0,0.0,0.0,-1.0,-1.0);
//		}
//		else if(usart_rx_mes_real_converted.variety == 3)
//		{
//			action_motors_function(0.0,1.0,0.0,0.0,0.0,0.0,-1.0,-1.0);
//		}
//		else if(usart_rx_mes_real_converted.variety == 4)
//		{
//			action_motors_function(0.0,1.0,0.0,0.0,0.0,0.0,-1.0,-1.0);
//		}
//		osDelay(1);
//			if(distant >= (float)2.0)
//			{
				uasrt_tx_mes_real.load = 0; 
			if(yaw_pitch_flag == 0)
			{
			if(usart_rx_mes_real_converted.variety == 1)
			{
				trash_one_yaw_pitch();
			}
			else if(usart_rx_mes_real_converted.variety == 2)
			{
				trash_two_yaw_pitch();
			}
			else if(usart_rx_mes_real_converted.variety == 3)
			{
				trash_three_yaw_pitch();
			}
			else if(usart_rx_mes_real_converted.variety == 4)
			{
				trash_four_yaw_pitch();
			}
			}
//			}
			else
			{
				uasrt_tx_mes_real.load = 1; 
			}
			
			osDelay(1);
  }
}

//（归一化）四个侧挡板角度，上舵机角度，中舵机角度，下舵机角度，yaw角度，倾倒角度，x2006位置，y2006位置
// 0                        1            2           3          4        5          6         7
void action_motors_function(fp32 side_baffle, fp32 up_holding_jaw, fp32 middele_holding_jaw, fp32 down_holding_jaw, fp32 rotate, fp32 pour, fp32 x, fp32 y)
{
		//四个挡板
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,(220-170)*side_baffle+170); //相当于一个周期内（20ms）有0.5ms高脉冲
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,(220-170)*side_baffle+170);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,(220-170)*side_baffle+170);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,(220-170)*side_baffle+170);
		
		//夹爪（从上到下tim4的ch1,2,3）
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(195-170)*up_holding_jaw+170); //170向下
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,(220-145)*middele_holding_jaw+145);//145-x,220-y
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,(245-170)*down_holding_jaw+170);//170向上
		
		//倾倒
		if(usart_rx_mes_real_converted.yaw_flag >= (fp32)0.0 && usart_rx_mes_real_converted.yaw_flag <= (fp32)1.0)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(220-170)*usart_rx_mes_real_converted.yaw_flag+170);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,(220-170)*rotate+170);
		}
		
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,(220-170)*pour+170);
		
		//xy电机位置
		x_position = x;
		y_position = y;
}

void trash_zero_yaw_pitch(void)
{
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,85);
		yaw_pitch_flag = 0;
}

void trash_one_yaw_pitch(void)
{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,205);//相当于一个周期内（20ms）有0.5ms高脉冲 下
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,250);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,220);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);//3
		osDelay(2000);
		if(yaw_pitch_flag == 0)
		{
			//pitch
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
			//yaw
		}
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,120);
		yaw_pitch_flag = 1;
		osDelay(1000);
		
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,110);
		
		uasrt_tx_mes_real.state = 1;
		osDelay(500);
		uasrt_tx_mes_real.state = 0;
		osDelay(2000);
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,85);
		
		osDelay(2000);
		
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,183);//相当于一个周期内（20ms）有0.5ms高脉冲 上
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,220);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,170);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);//3
		yaw_pitch_flag = 0;
}

void trash_two_yaw_pitch(void)
{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,205);//相当于一个周期内（20ms）有0.5ms高脉冲 下
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,250);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,220);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);//3
		osDelay(2000);
		if(yaw_pitch_flag == 0)
		{
			//pitch
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		}
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,250);
		yaw_pitch_flag = 1;
		osDelay(1000);
	
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,110);
		
		uasrt_tx_mes_real.state = 1;
		osDelay(500);
		uasrt_tx_mes_real.state = 0;
		osDelay(2000);
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,85);
		
		osDelay(2000);
		
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,183);//相当于一个周期内（20ms）有0.5ms高脉冲 上
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,220);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,170);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);//3
		
		yaw_pitch_flag = 0;
}

void trash_three_yaw_pitch(void)
{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,205);//相当于一个周期内（20ms）有0.5ms高脉冲 下
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,250);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,220);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);//3
		osDelay(2000);
		if(yaw_pitch_flag == 0)
		{
			//pitch
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		}
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,120);
		yaw_pitch_flag = 1;
		osDelay(1000);
		
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,160);
		
		uasrt_tx_mes_real.state = 1;
		osDelay(500);
		uasrt_tx_mes_real.state = 0;
		osDelay(2000);
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,85);
		
		osDelay(2000);
		
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,183);//相当于一个周期内（20ms）有0.5ms高脉冲 上
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,220);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,170);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);//3
		
		yaw_pitch_flag = 0;
}

void trash_four_yaw_pitch(void)
{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,205);//相当于一个周期内（20ms）有0.5ms高脉冲 下
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,250);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,220);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,150);//3
		osDelay(2000);
		if(yaw_pitch_flag == 0)
		{
			//pitch
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		}
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,50);
		yaw_pitch_flag = 1;
		osDelay(1000);
		
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,110);
		
		uasrt_tx_mes_real.state = 1;
		osDelay(500);
		uasrt_tx_mes_real.state = 0;
		osDelay(2000);
		//pitch
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,140);
		//yaw
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,85);
		
		osDelay(2000);
		
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,183);//相当于一个周期内（20ms）有0.5ms高脉冲 上
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,220);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,170);//1
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,90);//3
		
		yaw_pitch_flag = 0;
}
