#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis_task.h"

extern CAN_HandleTypeDef hcan1;

#define get_motor_measure(ptr, data)                                    \
{                                                                   		\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
}

motor_measure_t CAN1_rx_buffer[4];

static CAN_TxHeaderTypeDef  chassis_can1_tx_message;
static uint8_t              chassis_can1_send_data[8];	

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		if(hcan == &CHASSIS_CAN1)
		{
				switch (rx_header.StdId)
				{
						case CAN_2006_2_ID:
						case CAN_2006_3_ID:
						case CAN_2006_4_ID:
						{
								static uint8_t i = 0;
								i = rx_header.StdId - CAN_2006_1_ID;
								get_motor_measure(&CAN1_rx_buffer[i], rx_data)
								if(i == 1)
								{
									if(CAN1_rx_buffer[1].ecd - CAN1_rx_buffer[1].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_picture.round_cnt--;
									}
									else if(CAN1_rx_buffer[1].ecd - CAN1_rx_buffer[1].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_picture.round_cnt++;
									}
								}
								if(i == 2)
								{
									if(CAN1_rx_buffer[2].ecd - CAN1_rx_buffer[2].last_ecd > HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt--;
									}
									else if(CAN1_rx_buffer[2].ecd - CAN1_rx_buffer[2].last_ecd < -HALF_ECD_RANGE)
									{
										chassis.motor_clamp.round_cnt++;
									}
								}
						}
						default:
						{
								break;
						}
				}
		}
}


void CAN_chassis_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
		uint32_t send_mail_box;
    chassis_can1_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_can1_tx_message.IDE = CAN_ID_STD;
    chassis_can1_tx_message.RTR = CAN_RTR_DATA;
    chassis_can1_tx_message.DLC = 0x08;
    chassis_can1_send_data[0] = motor1 >> 8;
    chassis_can1_send_data[1] = motor1;
    chassis_can1_send_data[2] = motor2 >> 8;
    chassis_can1_send_data[3] = motor2;
    chassis_can1_send_data[4] = motor3 >> 8;
    chassis_can1_send_data[5] = motor3;
    chassis_can1_send_data[6] = motor4 >> 8;
    chassis_can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN1, &chassis_can1_tx_message, chassis_can1_send_data, &send_mail_box);
}

const motor_measure_t *get_picture_motor_point()
{
	return &CAN1_rx_buffer[1];
}

const motor_measure_t *get_clamp_motor_point()
{
	return &CAN1_rx_buffer[2];
}

const motor_measure_t *get_lift_motor_point()
{
	return &CAN1_rx_buffer[3];
}
