#include "STC.h"
#include <intrins.h>
#include "PWMn.h"

#define BUF_LENTH 64 //���崮��1���ջ��峤��

//ͨ������pwmռ�ձ������4-20mA�ĵ���
#define PWM_START 960 //��Ӧ4mA�����ĳ�ʼ��������׼ȷ��
#define PWM_END 3600  //��Ӧ20mA�����ĳ�ʼ��������׼ȷ��

#define DIGITAL_DELAY 15

#define SSIG 1		//1: ����SS�ţ���MSTRλ�����������Ǵӻ�		0: SS�����ھ������ӻ���
#define SPEN 1		//1: ����SPI��								0����ֹSPI������SPI�ܽž�Ϊ��ͨIO
#define DORD 0		//1��LSB�ȷ���								0��MSB�ȷ�
#define MSTR 1		//1����Ϊ����								0����Ϊ�ӻ�
#define CPOL 1		//1: ����ʱSCLKΪ�ߵ�ƽ��					0������ʱSCLKΪ�͵�ƽ
#define CPHA 1		//SPIʱ����λѡ�� ����Ϊ1ʱ��������SPICLK��ǰʱ�������������ں�ʱ���ز���
#define SPR1 0		//SPR1,SPR0   00: fosc/4,     01: fosc/16
#define SPR0 0		//            10: fosc/64,    11: fosc/128
#define SPEED_4 0	// fosc/4
#define SPEED_16 1	// fosc/16
#define SPEED_64 2	// fosc/64
#define SPEED_128 3 // fosc/128

#define SPIF 0x80 //SPI������ɱ�־��д��1��0��
#define WCOL 0x40 //SPIд��ͻ��־��д��1��0��

/*************** �û�������� *****************************/

#define MAIN_Fosc 11059200L //������ʱ�ӣ����õ���11.0592MHZ->11059200L
#define Baudrate1 9600L		//���崮��1������
#define Baudrate2 9600L		//���崮��2������

/****************** �������Զ����ɣ��û������޸� ************************************/

#define T1_TimerReload (256 - MAIN_Fosc / 192 / Baudrate1) //����12Tģʽ�¶�ʱ��1����ֵ

#define BRT_Reload (256 - MAIN_Fosc / 12 / 16 / Baudrate2) //����12Tģʽ��BRT����ֵ

#define TI2 (S2CON & 0x02) != 0	 //����2�����ж�
#define RI2 (S2CON & 0x01) != 0	 //����2�����ж�
#define CLR_TI2() S2CON &= ~0x02 //�������2�����ж�
#define CLR_RI2() S2CON &= ~0x01 //�������2�����ж�
/**********************************************************/

unsigned char idata receive_data; //SPI�����ж��н��մ�MT6816���͵ĽǶ�����

unsigned char uart0_wr;					   //����1дָ��
unsigned char uart0_rd;					   //����1��ָ��
unsigned char idata RX0_Buffer[BUF_LENTH]; //���ڽ��ջ����ַ�����

unsigned char code AscLed[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90}; //�����Ӳ����

bit B_TI;  //����1�жϱ�־λ
bit B_TI2; //����2�жϱ�־λ

sbit SPI_SCL = P1 ^ 7;	//SPI��ͬ��ʱ��
sbit SPI_MISO = P1 ^ 6; //SPI��ͬ������
sbit SPI_MOSI = P1 ^ 5; //SPI��ͬ������
sbit SPI_CS = P1 ^ 4;	//SPI��Ƭѡ

//sbit PWM = P1^3;PWM�ź�����Ķ˿�
sbit DIR485 = P0 ^ 0; //485�źŵ��շ���������

//�����������λ������
sbit DIG1 = P0 ^ 1;
sbit DIG2 = P0 ^ 2;
sbit DIG3 = P0 ^ 3;
sbit DIG4 = P0 ^ 4;

/*************	���غ�������	**************/
void SPI_init(void);					//SPI��ʼ������
void uart0_init(void);					//����0��ʼ������
void uart2_init(void);					//����2��ʼ������
void port_init(void);					//�˿ڳ�ʼ��
void SPI_test(void);					//SPI���ܲ��Ժ���
void SPI_send(unsigned char c);			//ͨ��SPI�����ַ���MT6816
void uart0_send(unsigned char c);		//ͨ������1�����ַ����������������
void uart2_send(unsigned char c);		//ͨ������2�����ַ���485оƬ
void Delay_ms(unsigned int n);			//MS��ʱ����(12M�����²���)
void PWM_test();						//PWM���ܲ��Ժ���
void Digital_tube_four(float fangle);	//�������ʾ����
void Digital_tube_four_for_error(void); //����ܴ�����ʾ����
void angle_data_process(void);			//����Ƕ�����
void tx(unsigned int dat);
void tx2(unsigned int dat);
void PrintString(unsigned char code *puts);
void PrintString2(unsigned char code *puts);
void real_main(void);
/*************	���ر�������	**************/
unsigned int pwm; //�����û�������PWM����

unsigned int iangle; //�����ǹ����������ʾ�ı�������
float xiaoshu;
unsigned int bai;
unsigned int shi;
unsigned int ge;

/****************  �ⲿ�����������ⲿ�������� *****************/
extern unsigned int PWM_high; //PWMռ�ձȼĴ�������PWM����ߵ�ƽ��PCAʱ�����������ռ�ձ�д���������
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
//������
//****************************************************
void main()
{
	real_main();
}

void real_main(void)
{
	AUXR1 |= (1 << 4); //��UART2��P1���л��� RXD2--P1.2�л���P4.2   TXD2---P1.3�л���P4.3
	uart2_init();	   //����2��ʼ��
	DIR485 = 1;		   //485Ĭ���Ƿ�״̬
	uart0_init();	   //����1��ʼ��
	SPI_init();		   //SPI��ʼ��
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
		SPI_CS = 0;		//SPIƬѡ����һֱ��0����Ȼ������
		SPI_send(0x83); //��һ�� Ϊ�޹��� ����
		SPI_send(0x83); //�ڶ���Ϊ��8λ
		DATA_1 = receive_data;
		SPI_send(0x83); //������Ϊ��8λ
		DATA_2 = receive_data;

		if ((receive_data & 0x02) == 0x02) //���дų�ǿ�Ȳ��ԣ����ǿ�Ȳ�������ӡcannot detect enough magnetic field
		{
			PrintString2("cannot detect enough magnetic field\r\n"); //�յ������������
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
void SPI_init(void) //����SPI�ӿ�
{
	SPCTL = (SSIG << 7) + (SPEN << 6) + (DORD << 5) + (MSTR << 4) + (CPOL << 3) + (CPHA << 2) + SPEED_64;
	IE2 |= 0x02; //����SPI�ж�
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
		angle_tmp_int++; //��λ
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
	CR = 0;				  //�ر�PCA
	SPSTAT = SPIF + WCOL; //��0 SPIF��WCOL��־
	SPDAT = c;

	//	LED_RX2 = 0;//�����ƣ��������Ƿ����е��� ->��������ǳ���Ҫ��ɾ����û����(�������ṩ�˱�Ҫ����ʱ��
	_nop_();
	_nop_();
	_nop_();
	while ((SPSTAT & SPIF) == 0)
		;				  //�ȴ��������
						  //	PrintString2("test for first-1\r\n");
	SPSTAT = SPIF + WCOL; //��0 SPIF��WCOL��־

	CR = 1; //����PCA��
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

	P2 = 0xC0 & 0x7F; //����С����
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

void Digital_tube_four(float fangle) //��180.5��������������
{
	port_init();
	//P2�˿�����
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
	P2 = AscLed[ge] & 0x7F; //����С����
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

void PWM_test() //PWM���ܲ��Ժ���
{
	//������4
	//��㣺949-��4mA
	//�յ㣺3646-��20.1mA

	//	pwm = PWM_HIGH_MIN;		//pwm��ֵ
	pwm = 949;
	PWMn_init(pwm); //��ʼ��pwm

	while (1)
	{
		//		Delay_ms(6000);	//��ʱ
		//		pwm += 10;
		//		if(pwm >= 3646)	pwm = 3606;
		////		PWMn_SetHighReg(pwm);		//����PWM��ռ�ձ�
	}
}

void uart0_send(unsigned char c)
{
	B_TI = 0;
	SBUF = c; //ͨ�����ڷ���
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
	SCON = 0x50;				  //UART0 set as 10bit , UART0 RX enable sl:8λ�ɣ�����10λ
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
	AUXR |= (1 << 3);				   //����2�����ʼӱ� S2SMOD
	S2CON = (S2CON & 0x3f) | (1 << 6); //����2ģʽ1��8λUART��(2^S2SMOD / 32) * BRT�����
	S2CON |= 1 << 4;				   //����2����

	AUXR |= 1 << 4; //baudrate use BRT
	BRT = BRT_Reload;

	IE2 |= 1; //������2�ж�
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
	CR = 1; //��PCA
	send = 1;
	//	SPI_CS = 0;
	receive_data = SPDAT;
	SPSTAT = SPIF + WCOL; //��0 SPIF��WCOL��־
	//	uart0_send(SPI_RxBuffer[SPI_RxWr]);�жϲ��ܵ��ú���
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

void PrintString(unsigned char code *puts) //����һ���ַ���
{
	for (; *puts != 0; puts++)
		uart0_send(*puts); //����ֹͣ��0����
}
void PrintString2(unsigned char code *puts) //����һ���ַ���
{
	for (; *puts != 0; puts++)
		uart2_send(*puts); //����ֹͣ��0����
}
