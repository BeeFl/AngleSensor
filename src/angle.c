#include "STC.h"
#include <intrins.h>
#include "PWMn.h"

#define BUF_LENTH 64 //定义串口1接收缓冲长度

//通过控制pwm占空比来输出4-20mA的电流
#define PWM_START 960 //对应4mA电流的初始量（必须准确）
#define PWM_END 3600  //对应20mA电流的初始量（不必准确）

#define DIGITAL_DELAY 15

#define SSIG 1		//1: 忽略SS脚，由MSTR位决定主机还是从机		0: SS脚用于决定主从机。
#define SPEN 1		//1: 允许SPI，								0：禁止SPI，所有SPI管脚均为普通IO
#define DORD 0		//1：LSB先发，								0：MSB先发
#define MSTR 1		//1：设为主机								0：设为从机
#define CPOL 1		//1: 空闲时SCLK为高电平，					0：空闲时SCLK为低电平
#define CPHA 1		//SPI时钟相位选择 设置为1时，数据在SPICLK的前时钟沿驱动，并在后时钟沿采样
#define SPR1 0		//SPR1,SPR0   00: fosc/4,     01: fosc/16
#define SPR0 0		//            10: fosc/64,    11: fosc/128
#define SPEED_4 0	// fosc/4
#define SPEED_16 1	// fosc/16
#define SPEED_64 2	// fosc/64
#define SPEED_128 3 // fosc/128

#define SPIF 0x80 //SPI传输完成标志。写入1清0。
#define WCOL 0x40 //SPI写冲突标志。写入1清0。

/*************** 用户定义参数 *****************************/

#define MAIN_Fosc 11059200L //定义主时钟，我用的是11.0592MHZ->11059200L
#define Baudrate1 9600L		//定义串口1波特率
#define Baudrate2 9600L		//定义串口2波特率

/****************** 编译器自动生成，用户请勿修改 ************************************/

#define T1_TimerReload (256 - MAIN_Fosc / 192 / Baudrate1) //计算12T模式下定时器1重载值

#define BRT_Reload (256 - MAIN_Fosc / 12 / 16 / Baudrate2) //计算12T模式下BRT重载值

#define TI2 (S2CON & 0x02) != 0	 //串口2发送中断
#define RI2 (S2CON & 0x01) != 0	 //串口2接收中断
#define CLR_TI2() S2CON &= ~0x02 //清除串口2发送中断
#define CLR_RI2() S2CON &= ~0x01 //清除串口2接收中断
/**********************************************************/

unsigned char idata receive_data; //SPI接收中断中接收从MT6816发送的角度数据

unsigned char uart0_wr;					   //串口1写指针
unsigned char uart0_rd;					   //串口1读指针
unsigned char idata RX0_Buffer[BUF_LENTH]; //串口接收缓存字符数组

unsigned char code AscLed[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90}; //数码管硬编码

bit B_TI;  //串口1中断标志位
bit B_TI2; //串口2中断标志位

sbit SPI_SCL = P1 ^ 7;	//SPI卡同步时钟
sbit SPI_MISO = P1 ^ 6; //SPI卡同步数据
sbit SPI_MOSI = P1 ^ 5; //SPI卡同步数据
sbit SPI_CS = P1 ^ 4;	//SPI卡片选

//sbit PWM = P1^3;PWM信号输出的端口
sbit DIR485 = P0 ^ 0; //485信号的收发控制引脚

//以下是数码管位码引脚
sbit DIG1 = P0 ^ 1;
sbit DIG2 = P0 ^ 2;
sbit DIG3 = P0 ^ 3;
sbit DIG4 = P0 ^ 4;

/*************	本地函数声明	**************/
void SPI_init(void);					//SPI初始化函数
void uart0_init(void);					//串口0初始化函数
void uart2_init(void);					//串口2初始化函数
void port_init(void);					//端口初始化
void SPI_test(void);					//SPI功能测试函数
void SPI_send(unsigned char c);			//通过SPI发送字符到MT6816
void uart0_send(unsigned char c);		//通过串口1发送字符到计算机，调试用
void uart2_send(unsigned char c);		//通过串口2发送字符给485芯片
void Delay_ms(unsigned int n);			//MS延时函数(12M晶振下测试)
void PWM_test();						//PWM功能测试函数
void Digital_tube_four(float fangle);	//数码管显示函数
void Digital_tube_four_for_error(void); //数码管错误显示函数
void angle_data_process(void);			//处理角度数据
void tx(unsigned int dat);
void tx2(unsigned int dat);
void PrintString(unsigned char code *puts);
void PrintString2(unsigned char code *puts);
void real_main(void);
/*************	本地变量声明	**************/
unsigned int pwm; //定义用户操作的PWM变量

unsigned int iangle; //以下是关于数码管显示的变量声明
float xiaoshu;
unsigned int bai;
unsigned int shi;
unsigned int ge;

/****************  外部函数声明和外部变量声明 *****************/
extern unsigned int PWM_high; //PWM占空比寄存器，即PWM输出高电平的PCA时钟脉冲个数（占空比写入变量）。
void PWMn_SetHighReg(unsigned int high);
void PWMn_init(unsigned int high);

bit send;

unsigned int DATA_1; //
unsigned int DATA_2; //
unsigned int DATA_send;
unsigned char state;
float angle;
float angle_tmp;
unsigned int angle_tmp_int;
float angle_tmp_decimal;
//****************************************************
//主函数
//****************************************************
void main()
{
	real_main();
}

void real_main(void)
{
	AUXR1 |= (1 << 4); //将UART2从P1口切换到 RXD2--P1.2切换到P4.2   TXD2---P1.3切换到P4.3
	uart2_init();	   //串口2初始化
	DIR485 = 1;		   //485默认是发状态
	uart0_init();	   //串口1初始化
	SPI_init();		   //SPI初始化
	pwm = PWM_HIGH_MIN;
	PWMn_init(pwm);
	Delay_ms(1000);
	uart0_rd = 0;
	uart0_wr = 0;
	send = 0;
	state = 0;
	PrintString2("test for uart2\r\n");

	while (1)
	{
		SPI_CS = 0;		//SPI片选必须一直置0，不然无数据
		SPI_send(0x83); //第一次 为无关数 忽略
		SPI_send(0x83); //第二次为高8位
		DATA_1 = receive_data;
		SPI_send(0x83); //第三次为低8位
		DATA_2 = receive_data;

		if ((receive_data & 0x02) == 0x02) //进行磁场强度测试，如果强度不够，打印cannot detect enough magnetic field
		{
			PrintString2("cannot detect enough magnetic field\r\n"); //收到这个且乱码了
			state = 2;
		}
		else
		{
			DATA_1 = DATA_1 << 8;
			DATA_send = DATA_1 | DATA_2;
			DATA_send = DATA_send >> 2;
			tx2(DATA_send);
			state = 1;
		}
		if (state == 1)
		{
			angle_data_process();
		}

		if (state == 0 || state == 2)
		{
			Digital_tube_four_for_error();
		}
		else
		{
			Digital_tube_four(angle);
			if (angle <= 330.0)
			{
				pwm = PWM_START + angle * 2 * 4;
				PWMn_SetHighReg(pwm);
			}
		}
	}
}

/**********************************************/
void SPI_init(void) //配置SPI接口
{
	SPCTL = (SSIG << 7) + (SPEN << 6) + (DORD << 5) + (MSTR << 4) + (CPOL << 3) + (CPHA << 2) + SPEED_64;
	IE2 |= 0x02; //允许SPI中断
}

void port_init(void)
{
	P0M1 = 0x00;
	P0M0 = 0x1E;
}

void angle_data_process(void)
{
	angle = (DATA_send * 1.0 / 16384.0) * 360;
	angle_tmp = angle * 10;
	angle_tmp_int = (unsigned int)angle_tmp;
	angle_tmp_decimal = angle_tmp - angle_tmp_int;
	if (angle_tmp_decimal > 0.5)
	{
		angle_tmp_int++; //进位
	}
	angle_tmp = angle_tmp_int * 1.0 / 10.0;
	angle_tmp_int = (unsigned int)angle_tmp;
	angle_tmp_decimal = angle_tmp - angle_tmp_int;
	if (angle_tmp_decimal < 0.75 && angle_tmp_decimal > 0.25)
	{
		angle_tmp_decimal = 0.5;
	}
	else if (angle_tmp_decimal < 0.25)
	{
		angle_tmp_decimal = 0.0;
	}
	else if (angle_tmp_decimal > 0.75)
	{
		angle_tmp_decimal = 1.0;
	}
	angle = angle_tmp_int * 1.0 + angle_tmp_decimal;
}

void SPI_send(unsigned char c)
{
	CR = 0;				  //关闭PCA
	SPSTAT = SPIF + WCOL; //清0 SPIF和WCOL标志
	SPDAT = c;

	//	LED_RX2 = 0;//点亮灯，看程序是否运行到这 ->这条代码非常重要，删除就没用了(可能是提供了必要的延时）
	_nop_();
	_nop_();
	_nop_();
	while ((SPSTAT & SPIF) == 0)
		;				  //等待发送完成
						  //	PrintString2("test for first-1\r\n");
	SPSTAT = SPIF + WCOL; //清0 SPIF和WCOL标志

	CR = 1; //启动PCA。
	while (send == 0)
		;
	//	PrintString2("test for first-2\r\n");
	send = 0;
}

void show_for_begin()
{
	port_init();

	P2 = 0xC0;
	DIG1 = 1;
	DIG2 = 0;
	DIG3 = 0;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	P2 = 0xC0;
	DIG1 = 0;
	DIG2 = 1;
	DIG3 = 0;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	P2 = 0xC0 & 0x7F; //点亮小数点
	DIG1 = 0;
	DIG2 = 0;
	DIG3 = 1;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	P2 = 0xC0;
	DIG1 = 0;
	DIG2 = 0;
	DIG3 = 0;
	DIG4 = 1;
	Delay_ms(DIGITAL_DELAY);
}

void Digital_tube_four_for_error(void)
{
	port_init();
	P2 = 0xBF;
	DIG1 = 1;
	DIG2 = 1;
	DIG3 = 1;
	DIG4 = 1;
}

void Digital_tube_four(float fangle) //如180.5，已做四舍五入
{
	port_init();
	//P2端口连接
	iangle = (unsigned int)fangle; //180
	xiaoshu = fangle - iangle;

	bai = iangle / 100;
	iangle = iangle % 100;
	P2 = AscLed[bai];
	DIG1 = 1;
	DIG2 = 0;
	DIG3 = 0;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	shi = iangle / 10;
	iangle = iangle % 10;
	P2 = AscLed[shi];
	DIG1 = 0;
	DIG2 = 1;
	DIG3 = 0;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	ge = iangle;
	P2 = AscLed[ge] & 0x7F; //点亮小数点
	DIG1 = 0;
	DIG2 = 0;
	DIG3 = 1;
	DIG4 = 0;
	Delay_ms(DIGITAL_DELAY);

	if (xiaoshu == 0.5)
	{
		P2 = AscLed[5];
	}
	else
	{
		P2 = AscLed[0];
	}
	DIG1 = 0;
	DIG2 = 0;
	DIG3 = 0;
	DIG4 = 1;
}

void PWM_test() //PWM功能测试函数
{
	//阶数：4
	//起点：949-》4mA
	//终点：3646-》20.1mA

	//	pwm = PWM_HIGH_MIN;		//pwm初值
	pwm = 949;
	PWMn_init(pwm); //初始化pwm

	while (1)
	{
		//		Delay_ms(6000);	//延时
		//		pwm += 10;
		//		if(pwm >= 3646)	pwm = 3606;
		////		PWMn_SetHighReg(pwm);		//更新PWM的占空比
	}
}

void uart0_send(unsigned char c)
{
	B_TI = 0;
	SBUF = c; //通过串口发送
	while (!B_TI)
		;
	B_TI = 0;
}

void uart2_send(unsigned char c)
{
	B_TI2 = 0;
	S2BUF = c;
	while (!B_TI2)
		;
	B_TI2 = 0;
}

void uart0_init(void)
{
	PCON |= 0x80;				  //UART0 Double Rate Enable
	SCON = 0x50;				  //UART0 set as 10bit , UART0 RX enable sl:8位吧，不是10位
	TMOD &= ~(1 << 6);			  //Timer1 Set as Timer, 12T
	TMOD = (TMOD & ~0x30) | 0x20; //Timer1 set as 8 bits auto relaod
	TH1 = T1_TimerReload;		  //Load the timer
	TR1 = 1;					  //start the timer
	ES = 1;
	EA = 1;
}

/**********************************************/
void uart2_init(void)
{
	AUXR |= (1 << 3);				   //串口2波特率加倍 S2SMOD
	S2CON = (S2CON & 0x3f) | (1 << 6); //串口2模式1，8位UART，(2^S2SMOD / 32) * BRT溢出率
	S2CON |= 1 << 4;				   //允许串2接收

	AUXR |= 1 << 4; //baudrate use BRT
	BRT = BRT_Reload;

	IE2 |= 1; //允许串口2中断
}

/**********************************************/
void UART0_RCV(void) interrupt 4
{
	if (RI)
	{
		RI = 0;
		RX0_Buffer[uart0_wr] = SBUF;
		if (++uart0_wr >= BUF_LENTH)
			uart0_wr = 0;
	}

	if (TI)
	{
		TI = 0;
		B_TI = 1;
	}
}

/**********************************************/
void UART2_RCV(void) interrupt 8
{
	if (TI2)
	{
		CLR_TI2();
		B_TI2 = 1;
	}
}
/**********************************************/
void SPI_Transivion(void) interrupt 9
{
	CR = 1; //打开PCA
	send = 1;
	//	SPI_CS = 0;
	receive_data = SPDAT;
	SPSTAT = SPIF + WCOL; //清0 SPIF和WCOL标志
	//	uart0_send(SPI_RxBuffer[SPI_RxWr]);中断不能调用函数
	//	SPI_CS = 1;
	//	if(++SPI_RxWr >= SPI_BUF_LENTH)		SPI_RxWr = 0;
}

void Delay_ms(unsigned int n)
{
	unsigned int i, j;
	for (i = 0; i < n; i++)
		for (j = 0; j < 123; j++)
			;
}

void tx(unsigned int dat)
{
	uart0_send('D');
	uart0_send('a');
	uart0_send('t');
	uart0_send('a');
	uart0_send('=');
	uart0_send(dat / 10000 + '0');
	uart0_send(dat % 10000 / 1000 + '0');
	uart0_send(dat % 1000 / 100 + '0');
	uart0_send(dat % 100 / 10 + '0');
	uart0_send(dat % 10 + '0');
	uart0_send(0x0d);
	uart0_send(0x0a);
}

void tx2(unsigned int dat)
{
	uart2_send('D');
	uart2_send('a');
	uart2_send('t');
	uart2_send('a');
	uart2_send('=');
	uart2_send(dat / 10000 + '0');
	uart2_send(dat % 10000 / 1000 + '0');
	uart2_send(dat % 1000 / 100 + '0');
	uart2_send(dat % 100 / 10 + '0');
	uart2_send(dat % 10 + '0');
	uart2_send(0x0d);
	uart2_send(0x0a);
}

void PrintString(unsigned char code *puts) //发送一串字符串
{
	for (; *puts != 0; puts++)
		uart0_send(*puts); //遇到停止符0结束
}
void PrintString2(unsigned char code *puts) //发送一串字符串
{
	for (; *puts != 0; puts++)
		uart2_send(*puts); //遇到停止符0结束
}
