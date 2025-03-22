#include "two_delay_task.h"
#include "loop_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "loop_task.h"

fp32 last_variety;
fp32 last_yaw_flag;

void two_delay_Task(void const * argument)
{
	osDelay(2000);
  for(;;)
  {
			osDelay(2000);
			last_variety = usart_rx_mes_real.variety;
			last_yaw_flag = usart_rx_mes_real.yaw_flag;
//			if(last_variety == usart_rx_mes_real.variety)
//			{
				usart_rx_mes_real_converted.variety = usart_rx_mes_real.variety;
//			}
//			else
//			{
//				yaw_pitch_flag = 0;
//			}
//			if(last_yaw_flag == usart_rx_mes_real.yaw_flag)
//			{
//				usart_rx_mes_real_converted.yaw_flag = usart_rx_mes_real.yaw_flag;
//			}		
  }
}

