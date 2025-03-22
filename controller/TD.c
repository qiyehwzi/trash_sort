#include "arm_math.h"
#include "user_lib.h"
#include "TD.h"

float Sign(float x)
{
		if(x > 0.0f)
		{
				return 1.0f;
		}
		else if(x < 0.0f)
		{
				return -1.0f;
		}		
		else
		{
				return 0.0f;
		}
}

float fsg(float x,float d)
{
		return (Sign(x+d)-Sign(x-d))/2.0f;
}
//�����ٶȣ��˲�������Ƶ�ʣ���ʼֵ
void TD_init(TD_t *td,float r,float N0,float h,float x)
{
		td->r  = r;
		td->h  = h;
		td->N0 = N0;
	 
	  td->ddx = 0.0f;
	  td->dx = 0.0f;	  
	  td->x = x;	
 	
}

//��ʼֵ
void TD_set_x(TD_t *td,float x)
{
	  td->x = x;	 	
}


//ADRC���ٸ���΢����TD���Ľ����㷨fhan
float TD_calc(TD_t *td,float v)//����ADRC���ȹ���
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;                            						//ADRC״̬���������
  x1_delta=td->x-v;                     									//��x1-v(k)���x1�õ���ɢ���¹�ʽ
  td->h0=td->N0*td->h;       															//��h0���h��������ٸ���΢�����ٶȳ�������
  d=td->r*td->h0*td->h0;     															//d=rh^2;
  a0=td->h0*td->dx;                												//a0=h*x2
  y=x1_delta+a0;                               						//y=x1+a0
  a1=sqrt(d*(d+8*fabs(y)));                       				//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign(y)*(a1-d)/2;                                 //a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*fsg(y,d)+a2*(1-fsg(y,d));
  td->fhan = -td->r*(a/d)*fsg(a,d)-td->r*Sign(a)*(1-fsg(a,d)); 		//�õ�����΢�ּ��ٶȸ�����

	td->ddx = td->fhan;
  td->dx += td->h*td->ddx;									//�������ٸ���״̬��΢��dx 
	td->x  += td->h*td->dx;										//�������ٸ���״̬��x
	
	return td->dx;
}


//ADRC���ٸ���΢����TD���Ľ����㷨fhan
float TD_calc_angle(TD_t *td,float v)//����ADRC���ȹ���
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;                            						//ADRC״̬���������
  x1_delta = td->x-v;                     									//��x1-v(k)���x1�õ���ɢ���¹�ʽ
	
	if(fabs(x1_delta) > PI)
	{
			if(v < 0)
			{
					v += 2*PI;
			}
			else if(v > 0)
			{
					v -= 2*PI;
			}
			x1_delta = td->x-v;
	}
	
  td->h0=td->N0*td->h;       															//��h0���h��������ٸ���΢�����ٶȳ�������
  d=td->r*td->h0*td->h0;     															//d=rh^2;
  a0=td->h0*td->dx;                												//a0=h*x2
  y=x1_delta+a0;                               						//y=x1+a0
  a1=sqrt(d*(d+8*fabs(y)));                       				//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign(y)*(a1-d)/2;                                 //a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*fsg(y,d)+a2*(1-fsg(y,d));
  td->fhan = -td->r*(a/d)*fsg(a,d)-td->r*Sign(a)*(1-fsg(a,d)); 		//�õ�����΢�ּ��ٶȸ�����

	td->ddx = td->fhan;
  td->dx += td->h*td->ddx;									//�������ٸ���״̬��΢��dx 
	td->x  += td->h*td->dx;										//�������ٸ���״̬��x
	td->x 	= rad_format(td->x);
	
	return td->dx;
}

