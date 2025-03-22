/**
  ****************************(C) COPYRIGHT 2023 TJU****************************
  * @file       6dof_kinematic.c/h
  * @brief      Kinematics solution of robotic arm,
  *             机械臂运动学解算
  * @note       
  * @history
  *  Version    Date            Author          Modificxtion
  *  V1.0.0     Nov-20-2023     Wang Yiwen       1. done
  *  V1.0.1     Dec-1-2023      Wang Yiwen       1. add quaterniont transform
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TJU****************************
	*/

#include "6dof_kinematic.h"
#include "cmsis_os.h"

float cosf(float x);
float sinf(float x);
static void EulerAngleToRotMat(const float* _eulerAngles, float* row_R);
static void RotMatToEulerAngle(const float* row_R, float* _eulerAngles);
static void QuaToRotMat(const float *_Q, float* row_R);
static void RotMatToQua(const float* row_R, float *_Q);
static void QuaToEulerAngle(const float *_Q, float* _eulerAngles);
static void EulerAngleToQua(const float* _eulerAngles, float *_Q);

//机械臂求解初始化
void Joint6D_Init(Robotic_6DOF * Robotic_6D)
{
		//建立DH参数表(alpha,a,d,theta)
    float DH_Matrix[5][4] = {
        {0.0f,          0.0f,        0.0f,       0.0f},
        {0.0f,         	210.0f,      0.0f,       -PI/2.0f},
        {-PI/2,        	0.0f,        260.0f,     0.0f},
        {PI/2,          0.0f,     	 0.0f,       0.0f},
        {-PI/2,         0.0f,    		 0.0f,       0.0f}};
		
		//更新DH参数->Robotic_6D结构体
		for(int i = 0; i < 5; i++)
		{
				Robotic_6D->_alpha[i]=	DH_Matrix[i][0];
				Robotic_6D->_a[i]=	DH_Matrix[i][1];
				Robotic_6D->_d[i]=	DH_Matrix[i][2];
		}

    //初始化位姿变换矩阵T
		for(int i = 0; i < 5; i++)
		{
		    Robotic_6D->_T_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->_T_data[i], 0, sizeof(float) * 4 * 4);
			  Matrix_Init(&Robotic_6D->_T[i], 4, 4, (float *)Robotic_6D->_T_data[i]);
		}
		
		//初始化正解算temp矩阵
		for(int i = 0; i < 2; i++)
		{
				Robotic_6D->fk_temp_matrix_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->fk_temp_matrix_data[i], 0, sizeof(float) * 4 * 4);
				Matrix_Init(&Robotic_6D->fk_temp_matrix[i], 4, 4, (float *)Robotic_6D->fk_temp_matrix_data[i]);		
		}
		
		//初始化R05'旋转矩阵
		Robotic_6D->_R50_51_inv_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R50_51_inv_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R50_51_inv, 3, 3, (float *)Robotic_6D->_R50_51_inv_data);
		
		Robotic_6D->_temp_caculate_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_temp_caculate_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_temp_caculate, 3, 3, (float *)Robotic_6D->_temp_caculate_data);

		//初始化R0_50旋转矩阵，即关节0坐标系到关节5坐标系的初始旋转矩阵
		Robotic_6D->_R0_50_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R0_50_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R0_50, 3, 3, (float *)Robotic_6D->_R0_50_data);		
		//初始化R25旋转矩阵
		Robotic_6D->_R51_5_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R51_5_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R51_5, 3, 3, (float *)Robotic_6D->_R51_5_data);
		//初始化R50_5矩阵，即自定义控制器传来的矿石初始位姿到需要的位姿的旋转矩阵
		Robotic_6D->_R50_5_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R50_5_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R50_5, 3, 3, (float *)Robotic_6D->_R50_5_data);
		//初始化R05旋转矩阵
		Robotic_6D->_R05_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R05_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R05, 3, 3, (float *)Robotic_6D->_R05_data);
		
		Robotic_6D->_temp_trans_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		Robotic_6D->_temp_trans_data[0] = 0.0f;
		Robotic_6D->_temp_trans_data[1] = 0.0f;
		Robotic_6D->_temp_trans_data[2] = 1.0f;
		Robotic_6D->_temp_trans_data[3] = 0.0f;
		Robotic_6D->_temp_trans_data[4] = 1.0f;
		Robotic_6D->_temp_trans_data[5] = 0.0f;
		Robotic_6D->_temp_trans_data[6] = -1.0f;
		Robotic_6D->_temp_trans_data[7] = 0.0f;
		Robotic_6D->_temp_trans_data[8] = 0.0f;
		Matrix_Init(&Robotic_6D->_temp_trans, 3, 3, (float *)Robotic_6D->_temp_trans_data);
		
		//初始化逆解算temp矩阵		
		Robotic_6D->_ik_temp_matrix_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_ik_temp_matrix_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_ik_temp_matrix, 3, 3, (float *)Robotic_6D->_ik_temp_matrix_data);	
		
		//初始化P45和temp矩阵
		Robotic_6D->_P45_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P45_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P45, 3, 1, (float *)Robotic_6D->_P45_data);	
		Robotic_6D->_P45_temp_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P45_temp_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P45_temp, 3, 1, (float *)Robotic_6D->_P45_temp_data);		
}

//机械臂正向运动学求解
void SolveFK(Robotic_6DOF * Robotic_6D, const Joint6D_t *_Joint6D, Pose6D_t *_Pose6D)
{
    float cosq, sinq;
    float cosx, sina;	
		float R05[9];
		float EulerAngles[3];
		float Quaterniont[4];
		
		//i-1 ---
		//i    |	单个连杆T矩阵赋值
    for (int i = 0; i < 5; i++)
    {
        cosx = cosf(Robotic_6D->_alpha[i]);//alpha i-1
        sina = sinf(Robotic_6D->_alpha[i]);
        cosq = cosf(_Joint6D->theta[i]);//theta i
        sinq = sinf(_Joint6D->theta[i]);
				
				Robotic_6D->_T_data[i][0] = cosq;
        Robotic_6D->_T_data[i][1] = -sinq;
        Robotic_6D->_T_data[i][2] = 0.0f;
        Robotic_6D->_T_data[i][3] = Robotic_6D->_a[i];
        Robotic_6D->_T_data[i][4] =  cosx * sinq;
        Robotic_6D->_T_data[i][5] =  cosx * cosq;
        Robotic_6D->_T_data[i][6] = -sina;
        Robotic_6D->_T_data[i][7] = -sina * Robotic_6D->_d[i];
        Robotic_6D->_T_data[i][8] =  sina * sinq;
			  Robotic_6D->_T_data[i][9] =  sina * cosq;
			  Robotic_6D->_T_data[i][10] = cosx;
			  Robotic_6D->_T_data[i][11] = cosx * Robotic_6D->_d[i];
			  Robotic_6D->_T_data[i][12] = 0.0f;
			  Robotic_6D->_T_data[i][13] = 0.0f;
			  Robotic_6D->_T_data[i][14] = 0.0f;
			  Robotic_6D->_T_data[i][15] = 1.0f;
    }	
		
		//连杆矩阵相乘（T[i-1]）
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_T[0], &Robotic_6D->_T[1], &Robotic_6D->fk_temp_matrix[0]);		
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[2], &Robotic_6D->fk_temp_matrix[1]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[1], &Robotic_6D->_T[3], &Robotic_6D->fk_temp_matrix[0]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[4], &Robotic_6D->fk_temp_matrix[1]);
		
		//取最终坐标变换矩阵中的旋转部分
		for(int i = 0; i < 3; i++)
		{
				for(int j = 0; j < 3; j++)
				{
						R05[i * 3 + j] = Robotic_6D->fk_temp_matrix_data[1][i * 4 + j];
				}
		}
		
		//旋转矩阵->末端欧拉角Z-Y-X
		RotMatToEulerAngle(R05, &(EulerAngles[0]));
		//旋转矩阵->四元数
		RotMatToQua(R05, &(Quaterniont[0]));
		
		_Pose6D->X = Robotic_6D->fk_temp_matrix_data[1][3];
		_Pose6D->Y = Robotic_6D->fk_temp_matrix_data[1][7];
		_Pose6D->Z = Robotic_6D->fk_temp_matrix_data[1][11];
		_Pose6D->Yaw_rad = EulerAngles[0] * RAD_TO_DEG;
		_Pose6D->Pitch_rad = EulerAngles[1] * RAD_TO_DEG;
		_Pose6D->Roll_rad = EulerAngles[2] * RAD_TO_DEG;
		_Pose6D->Yaw_deg = EulerAngles[0];
		_Pose6D->Pitch_deg = EulerAngles[1];
		_Pose6D->Roll_deg = EulerAngles[2];
		_Pose6D->Q[0] = Quaterniont[0];
		_Pose6D->Q[1] = Quaterniont[1];
		_Pose6D->Q[2] = Quaterniont[2];
		_Pose6D->Q[3] = Quaterniont[3];
}

float R05[9];
float R50_5[9];
float xyz[3];
float Euler[3];
float Euler2[3];
float P45[3];
float q345[2][3];//theta3,theta4,theta5
float q12[2][2];//theta1,theta2
float q0, a0,temp1,temp2,q_sum;
float x,y,z;
//机械臂逆向运动学求解
bool SolveIK(Robotic_6DOF * Robotic_6D, Pose6D_t *_inputPose6D, Solver6D_t *_Out_Solver6D, const Joint6D_t *_lastJoint6D, uint8_t _Quaterniont_mode)
{

		float tmp;//中间变量

		float Quaterniont[4];
		int ind_theta4=2;//多解性序号
		float R0_51_inv[9];
		float R51_5[9];
		float temp_caculate[9];
		
		//给R050赋值
		float _R0_50_data_tem[9]={0,0,1,
															-1,0,0,
															0,-1,0};
		memcpy(Robotic_6D->_R0_50_data,_R0_50_data_tem,sizeof(_R0_50_data_tem));
		//根据末端位姿求解旋转矩阵
		if(_Quaterniont_mode)//四元数模式（视觉模式）
		{
				//四元数->旋转矩阵
				QuaToRotMat(&(_inputPose6D->Q[0]), Robotic_6D->_R50_5_data);
				for(int i = 0; i < 3; i++)
				//取旋转矩阵,用来看数据
				{
						for(int j = 0; j < 3; j++)
						{
								R50_5[i * 3 + j] = Robotic_6D->_R50_5_data[i * 3 + j];
						}
				}
				//计算R05
				Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_R0_50,&Robotic_6D->_R50_5,&Robotic_6D->_R05);
				for(int i = 0; i < 3; i++)
				//取旋转矩阵,用来看数据
				{
						for(int j = 0; j < 3; j++)
						{
								R05[i * 3 + j] = Robotic_6D->_R05_data[i * 3 + j];
						}
				}				
				//旋转矩阵->末端欧拉角Z-Y-X for watch
				RotMatToEulerAngle(R50_5, &(Euler2[0]));
				RotMatToEulerAngle(R05, &(Euler[0]));
				//目标末端姿态的欧拉角更新
				_inputPose6D->Yaw_rad = Euler2[0] * RAD_TO_DEG;//yaw
				_inputPose6D->Pitch_rad = Euler2[1] * RAD_TO_DEG;//pitch
				_inputPose6D->Roll_rad = Euler2[2] * RAD_TO_DEG;//roll					
		}
		else//欧拉角模式
		{
				Euler[0]=_inputPose6D->Yaw_rad/RAD_TO_DEG;
				Euler[1]=_inputPose6D->Pitch_rad/RAD_TO_DEG;
				Euler[2]=_inputPose6D->Roll_rad/RAD_TO_DEG;
				//欧拉角->旋转矩阵
				EulerAngleToRotMat(&(Euler[0]), Robotic_6D->_R05_data);

				//取旋转矩阵
				for(int i = 0; i < 3; i++)
				{
						for(int j = 0; j < 3; j++)
						{
								R05[i * 3 + j] = Robotic_6D->_R05_data[i * 3 + j];
						}
				}
				//旋转矩阵->四元数
				RotMatToQua(R05, &(Quaterniont[0]));
				//目标末端姿态的四元数更新
				_inputPose6D->Q[0] = Quaterniont[0];
				_inputPose6D->Q[1] = Quaterniont[1];
				_inputPose6D->Q[2] = Quaterniont[2];
				_inputPose6D->Q[3] = Quaterniont[3];			
		}
		
		//上次解算的已知角度赋值（但其实没卵用）
		for (int i = 0; i < 2; i++)
    {
        q12[i][0]  = _lastJoint6D->theta[0];
        q12[i][1]  = _lastJoint6D->theta[1];
        q345[i][0] = _lastJoint6D->theta[2];
        q345[i][1] = _lastJoint6D->theta[3];
        q345[i][2] = _lastJoint6D->theta[4];
    }
		
		//步骤0，求解关节3、4、5的原点坐标
		Robotic_6D->_P45_data[0] = 0.0f;
		Robotic_6D->_P45_data[1] = 0.0f;		
		Robotic_6D->_P45_data[2] = 30.89f + 100.0f;
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_R05, &Robotic_6D->_P45, &Robotic_6D->_P45_temp);
		//for watch
		for(int i = 0; i < 3; i++)
			{
				P45[i] = Robotic_6D->_P45_temp_data[i];
			}
		//赋值x,y,z
	  float k = (600.0f/17.36f);//抬升长度转化为3508总角度
		x = arm_length - _inputPose6D->X * motor_to_real * sc_to_arm ;//先换算到自定义控制器真实长度，再换算到机械臂真实长度
		y = -_inputPose6D->Y * motor_to_real * sc_to_arm ;
		z = -(_inputPose6D->Z * motor_to_real * sc_to_arm)/k;
		xyz [0] = x ;//- Robotic_6D->_P45_temp_data[0];
		xyz [1] = y ;//- Robotic_6D->_P45_temp_data[1];
		xyz [2] = z ;//- Robotic_6D->_P45_temp_data[2]/(600.0f/17.36f);
		_Out_Solver6D->high_position = xyz[2];
		//步骤1：求解theta1和theta2(以x轴为基准)
		q0 = atan2f(y,x); 
		arm_sqrt_f32(x*x + y*y , &a0);
		temp1 = (x*x + y*y + Robotic_6D->_a[1]*Robotic_6D->_a[1] - Robotic_6D->_d[2]*Robotic_6D->_d[2])/(2*a0*Robotic_6D->_a[1]);
		if(fabs(temp1)<1)
		{
		q12[0][0] = acos(temp1) + q0;
		q12[1][0] =  -acos(temp1) + q0;
		}
		temp2 = (x*x + y*y - Robotic_6D->_a[1]*Robotic_6D->_a[1] + Robotic_6D->_d[2]*Robotic_6D->_d[2])/(2*a0*Robotic_6D->_d[2]);
		if(fabs(temp2)< 1)
		{
		q12[0][1] = -(acos(temp2) - q0 + q12[0][0]);
		q12[1][1] = acos(temp2) + q0 - q12[1][0];
		}
		
		//步骤2：求解theta3,4,5
		for(int i = 0; i < 2; i++)
		{
			q_sum = -q12[i][0] - q12[i][1];
			Robotic_6D->_R50_51_inv_data[0] =  cosf(q_sum);
			Robotic_6D->_R50_51_inv_data[1] =  0;
			Robotic_6D->_R50_51_inv_data[2] =  -sinf(q_sum);
			Robotic_6D->_R50_51_inv_data[3] =  0;
			Robotic_6D->_R50_51_inv_data[4] =  1;
			Robotic_6D->_R50_51_inv_data[5] =  0;
			Robotic_6D->_R50_51_inv_data[6] =  sinf(q_sum);
			Robotic_6D->_R50_51_inv_data[7] =  0;
			Robotic_6D->_R50_51_inv_data[8] =  cosf(q_sum);
			Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_R50_51_inv, &Robotic_6D->_R50_5, &Robotic_6D->_R51_5);
			for(int i = 0; i <9; i++)
			{
				R0_51_inv[i] =  Robotic_6D->_R50_51_inv_data[i];
				R51_5[i] = Robotic_6D->_R51_5_data[i];
			}
			arm_sqrt_f32(Robotic_6D->_R51_5_data[2] * Robotic_6D->_R51_5_data[2] + Robotic_6D->_R51_5_data[5] * Robotic_6D->_R51_5_data[5], &tmp);									
			
			//theta4=0
			if(tmp < 0.00001f)
			{
					//特殊解：theta4=0/theta4=180，其中一组不满足约束范围，故在本代码中认为是单解
					for(int k = 0; k < ind_theta4; k++)
					{
							float s3,s5;
							float c3,c5;									
							//theta4
							q345[k][1] = k * PI;									
							//theta3,在theta4接近0°时只能求出theta3和theta5的和或差。所以theta3用上一次的角度，而不用0°
							q345[k][0] = _lastJoint6D->theta[2];
							//theta5
							c3 = cosf(_lastJoint6D->theta[2]);
							s3 = sinf(_lastJoint6D->theta[2]);												
							s5 =  -s3 * Robotic_6D->_R51_5_data[0] - c3 * Robotic_6D->_R51_5_data[1];
							c5 =  c3 * Robotic_6D->_R51_5_data[0] - s3 * Robotic_6D->_R51_5_data[1] ;
							q345[k][2] = atan2f(s5,c5);
							//单解
					}
			}
			else
			{
							//双解
					for(int k = 0; k < ind_theta4; k++)
					{
							//theta4
							q345[k][1] = atan2f((2 * k - 1) * tmp, Robotic_6D->_R51_5_data[8]);//原来是8
							//theta3,theta5
							float sinq4 = sinf(q345[k][1]);
							q345[k][0] = atan2f(Robotic_6D->_R51_5_data[5] / sinq4, Robotic_6D->_R51_5_data[2] / sinq4);
							q345[k][2] = atan2f( -Robotic_6D->_R51_5_data[7] / sinq4,  -Robotic_6D->_R51_5_data[6] / sinq4);
							//双解
					}
			}								
//		
//		
//			//赋值
			for(int k = 0; k < ind_theta4; k++)
			{
					//theta1赋值 
					if(q12[i][0] > PI)
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0] - 2 * PI;
					}	
					else if(q12[i][0] < -PI)
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0] + 2 * PI;
					}
					else
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0];
					}
					//theta2赋值					
					if(q12[i][1] > PI)
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1] - 2 * PI;
					}	
					else if(q12[i][0] < -PI)
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1] + 2 * PI;
					}
					else
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1];
					}
					//theta3,theta4,theta5赋值
					for(int n = 0; n < 3; n++)
					{
							if(q345[k][n] > PI)
							{
									_Out_Solver6D->theta[2 * i + k][2 + n] = q345[k][n] - 2 * PI;
							}	
							else if(q345[k][n] < -PI)
							{
									_Out_Solver6D->theta[2 * i + k][2 + n] = q345[k][n] + 2 * PI;
							}
							else
							{
									_Out_Solver6D->theta[2 * i+ k][2 + n] = q345[k][n];
							}		
					}				
			}
		}
//		osDelay(1);
		
		return true;
}

float cosf(float x)
{
    return arm_cos_f32(x);
}

float sinf(float x)
{
    return arm_sin_f32(x);
}

//Z-Y-X欧拉角(Rzyx = Rz*Ry*Rx)->旋转矩阵(ROW)
static void EulerAngleToRotMat(const float* _eulerAngles, float* row_R)//Z-Y-X
{
    float cx, cy, cz, sx, sy, sz;
    cz = cosf(_eulerAngles[0]);
    cy = cosf(_eulerAngles[1]);
    cx = cosf(_eulerAngles[2]);
    sz = sinf(_eulerAngles[0]);
    sy = sinf(_eulerAngles[1]);
    sx = sinf(_eulerAngles[2]);

    row_R[0] = cy * cz;
    row_R[1] = sx * sy * cz - cx * sz;
    row_R[2] = cx * sy * cz + sx * sz;
    row_R[3] = cy * sz;
    row_R[4] = sx * sy * sz + cx * cz;
    row_R[5] = cx * sy * sz - sx * cz;
    row_R[6] = -sy;
    row_R[7] = cy * sx;
    row_R[8] = cx * cy;
}

//旋转矩阵->Z-Y-X欧拉角
static void RotMatToEulerAngle(const float* row_R, float* _eulerAngles)
{
    float Y, P, R;
		//example:atan2f(sin?,cos?);
		Y = atan2f(row_R[3], row_R[0]);
		P = atan2f(-row_R[6], sqrtf(row_R[7] * row_R[7] + row_R[8] * row_R[8]));
		R = atan2f(row_R[7], row_R[8]);
    
		_eulerAngles[0] = Y;
    _eulerAngles[1] = P;
    _eulerAngles[2] = R;
}

//四元数(q=w+xi+yj+zk)->旋转矩阵
static void QuaToRotMat(const float *_Q, float* row_R)
{
  row_R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  row_R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  row_R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  row_R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  row_R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  row_R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  row_R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  row_R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  row_R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
}

//旋转矩阵->四元数
static void RotMatToQua(const float* row_R, float *_Q)
{
		float trace = row_R[0] + row_R[4] + row_R[8];
	
    if (trace > 0.0f) 
    {
        float s = sqrt(trace + 1.0f);
        _Q[0] = (s * 0.5f);
        s = 0.5f / s;
        _Q[1] = ((row_R[7] - row_R[5]) * s);
        _Q[2] = ((row_R[2] - row_R[6]) * s);
        _Q[3] = ((row_R[3] - row_R[1]) * s);
    } 
    
    else 
    {
        int i = row_R[0] < row_R[4] ? (row_R[4] < row_R[8] ? 2 : 1) : (row_R[0] < row_R[8] ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(row_R[i * 3 + i] - row_R[j * 3 + j]  - row_R[k * 3 + k] + 1.0f);
        _Q[i + 1] = s * 0.5f;
        s = 0.5f / s;

        _Q[0] 		= (row_R[k * 3 + j] - row_R[j * 3 + k]) * s;
        _Q[j + 1] = (row_R[j * 3 + i] + row_R[i * 3 + j]) * s;
        _Q[k + 1] = (row_R[k * 3 + i] + row_R[i * 3 + k]) * s;
    }
}

//四元数->Z-Y-X欧拉角
static void QuaToEulerAngle(const float *_Q, float* _eulerAngles)
{
	float row_R[9];
  row_R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  row_R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  row_R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  row_R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  row_R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  row_R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  row_R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  row_R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  row_R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
	
//	column_R[0] = row_R[0];
//	column_R[3] = row_R[1];
//	column_R[6] = row_R[2];
//	column_R[1] = row_R[3];
//	column_R[4] = row_R[4];
//	column_R[7] = row_R[5];
//	column_R[2] = row_R[6];
//	column_R[5] = row_R[7];
//	column_R[8] = row_R[8];
	
	float Y, P, R;
	Y = atan2f(row_R[3], row_R[0]);
	P = atan2f(-row_R[6], sqrtf(row_R[7] * row_R[7] + row_R[8] * row_R[8]));
	R = atan2f(row_R[7], row_R[8]);
	
	_eulerAngles[0] = Y;
	_eulerAngles[1] = P;
	_eulerAngles[2] = R;
}

//Z-Y-X欧拉角->四元数
static void EulerAngleToQua(const float* _eulerAngles, float *_Q)
{
	float cx, cy, cz, sx, sy, sz;
	float row_R[9];
	cz = cosf(_eulerAngles[0]);
	cy = cosf(_eulerAngles[1]);
	cx = cosf(_eulerAngles[2]);
	sz = sinf(_eulerAngles[0]);
	sy = sinf(_eulerAngles[1]);
	sx = sinf(_eulerAngles[2]);

	row_R[0] = cy * cz;
	row_R[1] = -cx * sz + sx * sy * cz;
	row_R[2] =  sx * sz + cx * sy * cz;
	row_R[3] = cy * sz;
	row_R[4] = sx * sy * sz + cx * cz;
	row_R[5] = cx * sy * sz - sx * cz;
	row_R[6] = -sy;
	row_R[7] = cy * sx;
	row_R[8] = cx * cy;

//	column_R[0] = row_R[0];
//	column_R[3] = row_R[1];
//	column_R[6] = row_R[2];
//	column_R[1] = row_R[3];
//	column_R[4] = row_R[4];
//	column_R[7] = row_R[5];
//	column_R[2] = row_R[6];
//	column_R[5] = row_R[7];
//	column_R[8] = row_R[8];
	
	float trace = row_R[0] + row_R[4] + row_R[8];
	if (trace > 0.0f) 
	{
			float s = sqrt(trace + 1.0f);
			_Q[0] = (s * 0.5f);
			s = 0.5f / s;
			_Q[1] = ((row_R[7] - row_R[5]) * s);
			_Q[2] = ((row_R[2] - row_R[6]) * s);
			_Q[3] = ((row_R[3] - row_R[1]) * s);
	} 
	
	else 
	{
			int i = row_R[0] < row_R[4] ? (row_R[4] < row_R[8] ? 2 : 1) : (row_R[0] < row_R[8] ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			float s = sqrt(row_R[i * 3 + i] - row_R[j * 3 + j] - row_R[k * 3 + k] + 1.0f);
			_Q[i + 1] = s * 0.5f;
			s = 0.5f / s;

			_Q[0] 		= (row_R[k * 3 + j] - row_R[j * 3 + k]) * s;
			_Q[j + 1] = (row_R[j * 3 + i] + row_R[i * 3 + j]) * s;
			_Q[k + 1] = (row_R[k * 3 + i] + row_R[i * 3 + k]) * s;
	}
}
