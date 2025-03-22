#include "HC_SR04task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "string.h"

//ʹ��TIM5-CH4��ӦPI0��Echo��PF0��Trig

float distant;      //��������
uint32_t measure_Buf[3] = {0};   //��Ŷ�ʱ������ֵ������
uint8_t  measure_Cnt = 0;    //״̬��־λ
uint32_t high_time;   //������ģ�鷵�صĸߵ�ƽʱ��
uint8_t txData1[] = "---�ߵ�ƽʱ��---\n";
uint8_t txData2[4]; 
uint8_t txData3[] = "---������Ϊ---\n";
uint8_t txData4;

void SR04_GetData(void);

void HC_SR04Task(void const * argument)
{
  while(1)
  {
		SR04_GetData();
    osDelay(1500);//�������1500ms
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
		HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);	//�������벶��                                                                                    		
        break;
	case 3:
		high_time = measure_Buf[1]- measure_Buf[0];    //�ߵ�ƽʱ��
//        printf("\r\n----�ߵ�ƽʱ��-%d-us----\r\n",high_time);

				memcpy(txData2, &high_time, sizeof(high_time));

		distant=(high_time*0.034)/2;  //��λcm
//        printf("\r\n-������Ϊ-%.2f-cm-\r\n",distant);

				txData4 = (uint8_t)(distant * 1000);

		measure_Cnt = 0;  //��ձ�־λ
        TIM5->CNT=0;     //��ռ�ʱ������
		break;
				
	}
}

void TIM5_IRQHandler(void)
{
	
	switch(measure_Cnt){
		case 1:
			measure_Buf[0] = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);//��ȡ��ǰ�Ĳ���ֵ.
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);  //����Ϊ�½��ز���
			measure_Cnt++;                                            
			break;              
		case 2:
			measure_Buf[1] = HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_4);//��ȡ��ǰ�Ĳ���ֵ.
			HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_4); //ֹͣ����   ����: __HAL_TIM_DISABLE(&htim5);
			measure_Cnt++;  
	}
		
  HAL_TIM_IRQHandler(&htim5);

}

