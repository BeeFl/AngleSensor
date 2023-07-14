#ifndef PWMn_H
#define PWMn_H

#define MAIN_Fosc 11059200L //定义时钟

#define PCA_IDLE_DISABLE 0	//1: MCU在IDLE模式时禁止PCA工作。	0:  MCU在IDLE模式时允许PCA工作。
#define PCA_SOURCE_SELECT 4 //选择PCA的基准时钟源。                             \
							//0：系统时钟Fosc/12。                                 \
							//1：系统时钟Fosc/2。                                  \
							//2：定时器0的溢出。                                 \
							//3：ECI/P3.4脚的外部时钟输入（最大=Fosc/2）。 \
							//4：系统时钟Fosc。                                    \
							//5：系统时钟Fosc/4。                                  \
							//6：系统时钟Fosc/6。                                  \
							//7：系统时钟Fosc/8。
#define PWM_DUTY 4096		//定义PWM的周期，数值为PCA所选择的时钟脉冲个数。

/**********************************************************************************************************/

#include "STC12C5A60S2.h"

sbit CCP0 = P1 ^ 3; //STC12C5A60S2 this
sbit CCP1 = P1 ^ 4; //STC12C5A60S2

#define PWM_HIGH_MAX PWM_DUTY - 256 //限制PWM输出的最大占空比。
#define PWM_HIGH_MIN 256			//限制PWM输出的最小占空比。

#endif
