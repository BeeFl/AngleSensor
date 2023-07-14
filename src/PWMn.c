#include "PWMn.h"

unsigned int PWM_high;	//����PWMռ�ձȼĴ�������PWM����ߵ�ƽ��PCAʱ�����������ռ�ձ�д���������
unsigned int PWM_low;	//����PWM����͵�ƽ��PCAʱ�����������
unsigned int CCAP0_tmp; //����CCAP0��װ��Ӱ��Ĵ�����

// ����: д��ռ�ձ�����
void PWMn_SetHighReg(unsigned int high)
{
	if (high > PWM_HIGH_MAX)
		high = PWM_HIGH_MAX; //���д��������ռ�ձ����ݣ�ǿ��Ϊ���ռ�ձȡ�
	if (high < PWM_HIGH_MIN)
		high = PWM_HIGH_MIN;   //���д��С����Сռ�ձ����ݣ��򷵻ش������2��
	CR = 0;					   //ֹͣPCA��
	PWM_high = high;		   //��������ȷ��Χ����װ��ռ�ձȼĴ�����
	PWM_low = PWM_DUTY - high; //���㲢����PWM����͵�ƽ��PCAʱ�����������
	CR = 1;					   //����PCA��
}

// ����: ��ʼ������
void PWMn_init(unsigned int high)
{
	P1M1 &= ~0x08, P1M0 |= 0x08; //CCAP0ʹ��PUSH-PULL���ģʽ��STC12C5A60S2ϵ��, P1.3��this
	CCON = 0;					 //���CF��CR��CCF0��CCF1
	IPH |= 0x80;				 //PCA�ж�ʹ��������ȼ�
	PPCA = 1;
	CMOD = (PCA_IDLE_DISABLE << 7) | (PCA_SOURCE_SELECT << 1); //��ʼ��PCAģʽ�Ĵ�������������PWMn.h��ѡ��
	CCAPM0 = 0x4D;											   //�������ģʽ������Ƚ�ƥ���ж�(ECCF0=1)��
	CL = 0;													   //���PCA������������
	CH = 0;
	CCAP0_tmp = 0;		   //���CCAP0��װ��Ӱ��Ĵ�����
	PWMn_SetHighReg(high); //��ʼ��ռ�ձ����ݡ�
	CR = 1;				   //����PCA��
	EA = 1;				   //�������ж�
}

// ����: PCA�жϷ������
void PCA_interrupt(void) interrupt 7
{
	if (CCF0 == 1) //PCAģ��0�ж�
	{
		CCF0 = 0; //��PCAģ��0�жϱ�־

		if (CCP0 == 1)
			CCAP0_tmp += PWM_high; //���Ϊ�ߵ�ƽ�����Ӱ��Ĵ���װ�ظߵ�ƽʱ�䳤��
		else
			CCAP0_tmp += PWM_low;				  //���Ϊ�͵�ƽ�����Ӱ��Ĵ���װ�ص͵�ƽʱ�䳤��
		CCAP0L = (unsigned char)CCAP0_tmp;		  //��Ӱ��Ĵ���д�벶��Ĵ�������дCCAP0L
		CCAP0H = (unsigned char)(CCAP0_tmp >> 8); //��дCCAP0H
	}
}