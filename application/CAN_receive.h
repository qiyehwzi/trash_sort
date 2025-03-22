#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN1 hcan1

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_2006_1_ID = 0x201,
	CAN_2006_2_ID = 0x202,
	CAN_2006_3_ID = 0x203, 
	CAN_2006_4_ID = 0x204
}can_msg_id_2006_e;

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint16_t count;
} motor_measure_t;

extern void CAN_chassis_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_chassis_motor_point(uint8_t i);
extern const motor_measure_t *get_suspension_motor_point(uint8_t i);
extern const motor_measure_t *get_clamp_motor_point(void);
extern const motor_measure_t *get_picture_motor_point(void);

#endif
