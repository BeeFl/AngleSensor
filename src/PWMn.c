#include "PWMn.h"

unsigned int PWM_high;	//定义PWM占空比寄存器，即PWM输出高电平的PCA时钟脉冲个数（占空比写入变量）。
unsigned int PWM_low;	//定义PWM输出低电平的PCA时钟脉冲个数。
unsigned int CCAP0_tmp; //定义CCAP0重装载影射寄存器。

// 描述: 写入占空比数据
void PWMn_SetHighReg(unsigned int high)
{
	if (high > PWM_HIGH_MAX)
		high = PWM_HIGH_MAX; //如果写入大于最大占空比数据，强制为最大占空比。
	if (high < PWM_HIGH_MIN)
		high = PWM_HIGH_MIN;   //如果写入小于最小占空比数据，则返回错误代码2。
	CR = 0;					   //停止PCA。
	PWM_high = high;		   //数据在正确范围，则装入占空比寄存器。
	PWM_low = PWM_DUTY - high; //计算并保存PWM输出低电平的PCA时钟脉冲个数。
	CR = 1;					   //启动PCA。
}

// 描述: 初始化程序。
void PWMn_init(unsigned int high)
{
	P1M1 &= ~0x08, P1M0 |= 0x08; //CCAP0使用PUSH-PULL输出模式，STC12C5A60S2系列, P1.3。this
	CCON = 0;					 //清除CF、CR、CCF0、CCF1
	IPH |= 0x80;				 //PCA中断使用最高优先级
	PPCA = 1;
	CMOD = (PCA_IDLE_DISABLE << 7) | (PCA_SOURCE_SELECT << 1); //初始化PCA模式寄存器，这两项在PWMn.h中选择。
	CCAPM0 = 0x4D;											   //高速输出模式，允许比较匹配中断(ECCF0=1)。
	CL = 0;													   //清空PCA基本计数器。
	CH = 0;
	CCAP0_tmp = 0;		   //清空CCAP0重装载影射寄存器。
	PWMn_SetHighReg(high); //初始化占空比数据。
	CR = 1;				   //启动PCA。
	EA = 1;				   //允许总中断
}

// 描述: PCA中断服务程序。
void PCA_interrupt(void) interrupt 7
{
	if (CCF0 == 1) //PCA模块0中断
	{
		CCF0 = 0; //清PCA模块0中断标志

		if (CCP0 == 1)
			CCAP0_tmp += PWM_high; //输出为高电平，则给影射寄存器装载高电平时间长度
		else
			CCAP0_tmp += PWM_low;				  //输出为低电平，则给影射寄存器装载低电平时间长度
		CCAP0L = (unsigned char)CCAP0_tmp;		  //将影射寄存器写入捕获寄存器，先写CCAP0L
		CCAP0H = (unsigned char)(CCAP0_tmp >> 8); //后写CCAP0H
	}
}