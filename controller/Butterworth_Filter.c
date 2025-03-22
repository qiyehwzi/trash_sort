#include "Butterworth_Filter.h"


/**
 * @param[in]       Filter        	Butter_worth_Filter_t�ṹ��
 * @param[in]       Fc          	��ֹƵ��
 * @param[in]       Ts        		��������
 */

void Butter_worth_Filter_init(Butter_worth_Filter_t *Filter,fp32 Fc,fp32 Ts)
{
		Filter->Fc = Fc;
		Filter->Ts = Ts;
		Filter->Yout[0] = 0;
		Filter->Yout[1] = 0;
		Filter->Yout[2] = 0;
		Filter->Xin[0] =  0;
		Filter->Xin[1] =  0;
		Filter->Xin[2] =  0;
		
		//Wc = 2 * pi * Fc��FcΪ��ֹƵ��	
		Filter->Wc = 2.0f * 3.1415926f *Fc;
		//DpΪ����ϵ����damp����д����һ������Ϊ0.707
		Filter->Dp = 0.707f;
		//����b0��a0��a1��a2��Ϊ�˷�����ʾ���ں������ˣ�ʵ����Ӧ�ð����ĸ�������ԭʼ��Wc��Ts��Dp��������ɺ궨�壬���ټ�����
		Filter->b0 = Filter->Wc * Filter->Wc *Filter->Ts * Filter->Ts;
		Filter->a0 = 4.0f + 4.0f * Filter->Dp * Filter->Wc * Ts + Filter->Wc * Filter->Wc * Filter->Ts * Filter->Ts;
		Filter->a1 = -8.0f +2.0f * Filter->Wc * Filter->Wc * Ts * Filter->Ts;
		Filter->a2 = 4.0f - 4.0f * Filter->Dp * Filter->Wc * Ts + Filter->Wc * Filter->Wc * Filter->Ts * Filter->Ts;

}


/**
 * @param[in]       Filter        	Butter_worth_Filter_t�ṹ��
 * @param[in]       xin          	����
 */

fp32 Butter_worth_Filter_calc(Butter_worth_Filter_t *Filter,fp32 xin)
{
	
    Filter->Xin[2] = xin;
    //Ψһ�ı���Ҫ���ļ��㣬�������ü��������Ǻܴ󣬿��Կ���FPU�������㸺��
    Filter->Yout[2] = (Filter->b0 * Filter->Xin[2] + 2.0f * Filter->b0 * Filter->Xin[1] + Filter->b0 * Filter->Xin[0] - Filter->a1 * Filter->Yout[1] - Filter->a2 * Filter->Yout[0]) / Filter->a0;
    //Vo(n-2) = Vo(n-1)��������Ϊ������һ���������ں󣬵�n-1������źű���˵�n-2������źţ�ͬ���n������źű���˵�n-1������ź�
    Filter->Yout[0] = Filter->Yout[1];
    //Vo(n-1) = Vo(n)
    Filter->Yout[1] = Filter->Yout[2];
    //Vi(n-2) = Vi(n-1)
    Filter->Xin[0] = Filter->Xin[1];
    //Vi(n-1) = Vi(n)
    Filter->Xin[1] = Filter->Xin[2];
    //����������Y(n)
	
    return Filter->Yout[2];
}
