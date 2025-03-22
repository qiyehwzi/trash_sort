/**
  ****************************(C) COPYRIGHT 2023 TJU****************************
  * @file       6dof_kinematic.c/h
  * @brief      Kinematics solution of robotic arm task,
  *             机械臂运动学解算任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-20-2023     王艺文              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TJU****************************
  */
#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H


#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"


#ifndef user_malloc
	#ifdef _CMSIS_OS_H
		#define user_malloc pvPortMalloc
	#else
		#define user_malloc malloc
	#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init     arm_mat_init_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Inverse arm_mat_inverse_f32

#define RAD_TO_DEG  57.29577951308232088f

#define LINK1_LENGTH 260.0f
#define LINK2_LENGTH 235.0f

#define motor_to_real 23.873241463f
#define sc_to_arm  2.6f
#define arm_length 600.89f

typedef struct 
{
		float theta[5];
		float high_position;
}Joint6D_t;

typedef struct 
{
		float theta[4][5];//逆解算的四组解
		float high_position;
}Solver6D_t;

typedef struct 
{
    float X, Y, Z;
    float Yaw_deg, Pitch_deg, Roll_deg;//A-yaw,B-pitch,C-roll'
		float Yaw_rad, Pitch_rad, Roll_rad;//A-yaw,B-pitch,C-roll
		float Q[4];
}Pose6D_t;

typedef struct
{
		float _alpha[5];
		float _a[5];
	  float _d[5];
	  int8_t MatStatus;
	
		mat _T[5];
		float *_T_data[5];
		mat _R0_50;
		float *_R0_50_data;
		mat _R50_5;
		float *_R50_5_data;
		mat _R05;
		float *_R05_data;
		mat _R50_51_inv;
		float *_R50_51_inv_data;
		mat _R51_5;
		float *_R51_5_data;
		mat _P45;
		float *_P45_data;
		mat _P45_temp;
		float *_P45_temp_data;
	
	  mat _temp_trans;
		float *_temp_trans_data;
		mat _temp_caculate;
		float *_temp_caculate_data;
	
		mat fk_temp_matrix[2];
		float *fk_temp_matrix_data[2];		
	
		mat _ik_temp_matrix;
		float *_ik_temp_matrix_data;
	
	
} Robotic_6DOF;


void Joint6D_Init(Robotic_6DOF * Robotic_6D);

void SolveFK(Robotic_6DOF * Robotic_6D, const Joint6D_t *_Joint6D, Pose6D_t *_Pose6D);

bool SolveIK(Robotic_6DOF * Robotic_6D, Pose6D_t *_inputPose6D, Solver6D_t *_Out_Solver6D, const Joint6D_t *_lastJoint6D, uint8_t _Quaterniont_mode);


#endif
