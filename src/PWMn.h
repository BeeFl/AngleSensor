#ifndef PWMn_H
#define PWMn_H

#define MAIN_Fosc 11059200L //����ʱ��

#define PCA_IDLE_DISABLE 0	//1: MCU��IDLEģʽʱ��ֹPCA������	0:  MCU��IDLEģʽʱ����PCA������
#define PCA_SOURCE_SELECT 4 //ѡ��PCA�Ļ�׼ʱ��Դ��                             \
							//0��ϵͳʱ��Fosc/12��                                 \
							//1��ϵͳʱ��Fosc/2��                                  \
							//2����ʱ��0�������                                 \
							//3��ECI/P3.4�ŵ��ⲿʱ�����루���=Fosc/2���� \
							//4��ϵͳʱ��Fosc��                                    \
							//5��ϵͳʱ��Fosc/4��                                  \
							//6��ϵͳʱ��Fosc/6��                                  \
							//7��ϵͳʱ��Fosc/8��
#define PWM_DUTY 4096		//����PWM�����ڣ���ֵΪPCA��ѡ���ʱ�����������

/**********************************************************************************************************/

#include "STC12C5A60S2.h"

sbit CCP0 = P1 ^ 3; //STC12C5A60S2 this
sbit CCP1 = P1 ^ 4; //STC12C5A60S2

#define PWM_HIGH_MAX PWM_DUTY - 256 //����PWM��������ռ�ձȡ�
#define PWM_HIGH_MIN 256			//����PWM�������Сռ�ձȡ�

#endif
