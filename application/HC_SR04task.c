#include "HC_SR04task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "string.h"

//使用TIM5-CH4对应PI0接Echo，PF0接Trig

float distant;      //测量距离
uint32_t measure_Buf[3] = {0};   //存放定时器计数值的数组
uint8_t  measure_Cnt = 0;    //状态标志位
uint32_t high_time;   //超声波模块返回的高电平时间
uint8_t txData1[] = "---高电平时间---\n";
uint8_t txData2[4]; 
uint8_t txData3[] = "---检测距离为---\n";
uint8_t txData4;

void SR04_GetData(void);

void HC_SR04Task(void const * argument)
{
  while(1)
  {
		SR04_GetData();
    osDelay(1500);//测距周期1500ms
  }
}

void SR04_GetData(void)
{
switch (measure_Cnt){
	case 0:
         TRIG_H;
         osDelay(30);
         TRIG_L;
    
		measure_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);	//启动输入捕获                                                                                    		
        break;
	case 3:
		high_time = measure_Buf[1]- measure_Buf[0];    //高电平时间
//        printf("\r\n----高电平时间-%d-us----\r\n",high_time);

				memcpy(txData2, &high_time, sizeof(high_time));

		distant=(high_time*0.034)/2;  //单位cm
//        printf("\r\n-检测距离为-%.2f-cm-\r\n",distant);

				txData4 = (uint8_t)(distant * 1000);

		measure_Cnt = 0;  //清空标志位
        TIM5->CNT=0;     //清空计时器计数
		break;
				
	}
}

void TIM5_IRQHandler(void)
{
	
	switch(measure_Cnt){
		case 1:
			measure_Buf[0] = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);//获取当前的捕获值.
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
			measure_Cnt++;                                            
			break;              
		case 2:
			measure_Buf[1] = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);//获取当前的捕获值.
			HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_4); //停止捕获   或者: __HAL_TIM_DISABLE(&htim5);
			measure_Cnt++;  
	}
		
  HAL_TIM_IRQHandler(&htim5);

}

