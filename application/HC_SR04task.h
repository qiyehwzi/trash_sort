//#ifndef HC_SR04TASK_H
//#define HC_SR04TASK_H

//#include "main.h"

//#define TRIG_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
//#define TRIG_L  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)

//#define TRIG2_H  HAL_GPIO_WritePin(Trig2_GPIO_Port,Trig2_Pin,GPIO_PIN_SET)
//#define TRIG2_L  HAL_GPIO_WritePin(Trig2_GPIO_Port,Trig2_Pin,GPIO_PIN_RESET)

//extern void HC_SR04Task(void const * argument);

//#endif


#ifndef HC_SR04TASK_H
#define HC_SR04TASK_H

#include "main.h"

#define TRIG_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
#define TRIG_L  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)

extern void HC_SR04Task(void const * argument);

extern float distant;      //��������
extern uint32_t measure_Buf[3];   //��Ŷ�ʱ������ֵ������
extern uint8_t  measure_Cnt;    //״̬��־λ
extern uint32_t high_time;   //������ģ�鷵�صĸߵ�ƽʱ��
extern uint8_t txData1[];
extern uint8_t txData2[4]; 
extern uint8_t txData3[];
extern uint8_t txData4;

#endif

