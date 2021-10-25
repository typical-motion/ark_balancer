#include "rm2020_uart_2stm32.h"
#include "rm2020_uart_thread_init.h"
#include <semaphore.h>
#include <linux/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <uart_process_2/uart_send.h>
#include <uart_process_2/uart_receive.h>
#include <dirent.h>
#define PATHNAME "/dev/ttyUSB0"//串口名称
#define RandRate B921600//波特率(B开头)
#define SEND_MESSAGE_LENGTH 23//上传数据长度
#define RECEIVE_MESSAGE_LENGTH 17//下载数据长度
typedef  char u8;

extern ros::Publisher pub;//ros转发
extern uart_process_2::uart_receive uart_Re_data;//ros转发数据
extern uart_process_2::uart_send uart_Se_data;//ros转发数据

int UART_ID;//串口句柄
bool ERROR_UART = false;//串口错误标示
Date_message DOWN_DATA_AM;//上传数据
int DATA_DOWN_Am = SEND_MESSAGE_LENGTH;//上传数据长度
Date_message UP_DATA_RE;//下载数据
int UP_DATA_Re = RECEIVE_MESSAGE_LENGTH;//下载数据长度
bool set_uart_mode(speed_t speed, int vtime, int vmin);
bool INIT_UART();
extern bool predictingFrame;



float char_P_2Float(char* data)//16进制字符数组转化为浮点数
{
	float float_data;
	float_data = *(float*)data;
	//for(size_t i = 0; i < 4; i++)
	//{
		//printf("data[%d] = %x\n",i,data[i]);
	//}
	return float_data;
}

void send_message_AM(float xdata, float ydata, float zdata, float tdata, uint8_t Cmdata)//数据发送
{
	memset(&DOWN_DATA_AM,0,sizeof(DOWN_DATA_AM));
	DOWN_DATA_AM.FH_data.Head = 0xA5;
	DOWN_DATA_AM.FH_data.CmdID = 0x01;
	DOWN_DATA_AM.FH_data.Datalength = 17;
	DOWN_DATA_AM.Data.Am_data.x = xdata;
	DOWN_DATA_AM.Data.Am_data.y = ydata;
	DOWN_DATA_AM.Data.Am_data.z = zdata;
	DOWN_DATA_AM.Data.Am_data.Time_Interval = tdata;//延迟时间
	DOWN_DATA_AM.Data.Am_data.Goal_State = Cmdata;//命令码
	Append_CRC16_Check_Sum_Judge(( char *)&DOWN_DATA_AM, DATA_DOWN_Am);//CRC16校验位
}

void receive_message_RE(Date_message* RE_data,char* coordinate_num)
{
	memset(RE_data, 0, sizeof(RE_data));
	RE_data->FH_data.Head = coordinate_num[0];
	//printf("RE_data->FH_data.Head = %x\n",coordinate_num[0]);
	RE_data->FH_data.CmdID = coordinate_num[1];
	char char_y_p[2][4] = {};//定义存储float类型数组
	char _length[2] = {};//定义存储uint16_t类型数组
	size_t n = 0;//获取float时，控制字节移位
	_length[0] = coordinate_num[2];//获取length长度
	_length[1] = coordinate_num[3];
	for(size_t i = 0;i < 2;i++)
	{
		for(size_t j = 0;j < 4;j++)
		{
			char_y_p[i][j] = coordinate_num[n+4];
			//printf("coordinate_num[%d] = %x\n",j,coordinate_num[n+4]);
			n++;
		}
	}
	RE_data->FH_data.Datalength = *(uint16_t*)_length;
	RE_data->Data.Re_data.yaw = char_P_2Float(char_y_p[0]);
	RE_data->Data.Re_data.pitch = char_P_2Float(char_y_p[1]);
	RE_data->Data.Re_data.Mod_set_over = coordinate_num[12];
	RE_data->Data.Re_data.Red_Blue = coordinate_num[13];
	RE_data->Data.Re_data.shoot_speed_mod = coordinate_num[14];
	RE_data->CRC16 = Get_CRC16_Check_Sum(coordinate_num, UP_DATA_Re - 2,CRC16_INIT_Judge);
	//*RE_data.FH_data.Datalength = (uint16_t)(coordinate_num[2] << 8 &
}

bool set_uart_mode(speed_t speed, int vtime, int vmin)//串口终端设置
{
	struct termios newtio;

	if(UART_ID<= 0)
	{
		printf("set_uart_mode failed---fd=%d\n",UART_ID);
		return -1;
	}	
	bzero(&newtio,sizeof(newtio));	 //置0
	newtio.c_iflag |= IGNBRK|BRKINT;	//忽略break或break中断
	newtio.c_cflag |= CLOCAL|CREAD;  //过位掩码的方式激活本地连接和接受使能选项
	newtio.c_cflag &= ~CSIZE;		//设置数据位
	newtio.c_cflag |= CS8;			//数据位为8位
	newtio.c_cflag &= ~PARENB;		//无奇偶校验
	cfsetispeed(&newtio,speed);   //设置波特率
	cfsetospeed(&newtio,speed);  
	newtio.c_cflag &= ~CSTOPB;		//1//  设置1位停止位
	newtio.c_cc[VTIME] = vtime; //等待时间，单位百毫秒 （读）。
	newtio.c_cc[VMIN] = vmin;	//最小字节数 （读）。
	tcflush(UART_ID,TCIFLUSH);	//tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来

	if ((tcsetattr(UART_ID, TCSANOW, &newtio)) != 0)   //激活与终端相关的参数。TCSANOW:立即执行
	{
		printf("set_uart_mode failed\n");
		return false;
	}
	printf("set_uart_mode\n"); 
	return true;
}

bool INIT_UART()//串口初始化
{
	static bool cout_flag = true;//显示错误信息
	UART_ID = open(PATHNAME,O_RDWR | O_NOCTTY);//打开文件的方式打开串口
	if(UART_ID < 0)
	{
		close(UART_ID);
		if(cout_flag)
		{
			printf("open UART3 fail\n");
			cout_flag = false;
		}
		usleep(100000);
		return false;
	}
	cout_flag = true;
	printf("open UART3 success\n");
	set_uart_mode(RandRate, 0, RECEIVE_MESSAGE_LENGTH * 2);//串口终端设置
	return true;
}//初始化

void *thread_read(void *arg)//数据读取线程
{
	int ret;
	while(true)
	{
		if(ERROR_UART == false)
		{
			char coordinate_num_temp[RECEIVE_MESSAGE_LENGTH * 2] = {};//定义数据帧
			char coordinate_num[RECEIVE_MESSAGE_LENGTH];
			ret = read(UART_ID,coordinate_num_temp,sizeof(coordinate_num_temp));
			for(int i = 0; i < RECEIVE_MESSAGE_LENGTH * 2 - RECEIVE_MESSAGE_LENGTH; i++)
			{
				if((unsigned char)coordinate_num_temp[i] == 0xA5 && (unsigned char)coordinate_num_temp[i+1] == 0x05)
				{
					for(int j = 0; j < RECEIVE_MESSAGE_LENGTH; j++)
					{
						coordinate_num[j] = coordinate_num_temp[i+j];
					}
					break;
				}
			}
		
			//std::cout << coordinate_num << std::endl;
			if((unsigned char)coordinate_num[0] == 0xA5 && (unsigned char)coordinate_num[1] == 0x05)
			{
				//printf("debug\n");
				//for(size_t i = 0; i < UP_DATA_Re; i++)
				//{
				//	printf("%x ",coordinate_num[i]);
				//}
				//std::cout << std::endl;
				if(Verify_CRC16_Check_Sum_Judge(coordinate_num,UP_DATA_Re))
				{
					receive_message_RE(&UP_DATA_RE,coordinate_num);
					//printf("get_message:\n");
					/*printf("pitch:%.4f\t yaw:%.4f \t Mod:%x\n\t Red_Blue:%x\n\t shoot_speed_mod:%x\n\t",
					UP_DATA_RE.Data.Re_data.pitch,
					UP_DATA_RE.Data.Re_data.yaw,
					UP_DATA_RE.Data.Re_data.Mod_set_over,
					UP_DATA_RE.Data.Re_data.Red_Blue,
					UP_DATA_RE.Data.Re_data.shoot_speed_mod);*/
					uart_Re_data.yaw = UP_DATA_RE.Data.Re_data.yaw;
					uart_Re_data.pitch = UP_DATA_RE.Data.Re_data.pitch;
					uart_Re_data.mod = UP_DATA_RE.Data.Re_data.Mod_set_over;
					uart_Re_data.red_blue = UP_DATA_RE.Data.Re_data.Red_Blue;
					uart_Re_data.shoot_speed_mod = UP_DATA_RE.Data.Re_data.shoot_speed_mod;
					pub.publish(uart_Re_data);
				}
			}
		}
		/*
		else
		{
			uart_Re_data.yaw = 0;
			uart_Re_data.pitch = 0;
			uart_Re_data.mod = 0;
			uart_Re_data.red_blue = 0;
			uart_Re_data.shoot_speed_mod = 0;
			pub.publish(uart_Re_data);
		}
		*/
	}
}

void *thread_write(void* arg)
{
	int ret;
	while(true)
	{
		//std::cout << "debug" << std::endl;
		//send_message_AM(1.5, 1.5, 2.0,  0.01,0);
		if(ERROR_UART == false)
		{
			ret = write(UART_ID,(char*)&DOWN_DATA_AM,DATA_DOWN_Am);
			if(ret < 0)
			{
				close(UART_ID);
				ERROR_UART = true;
			}
		}
		predictingFrame = true;
		usleep(3000);
		if(uart_Se_data.predictLatency) {
			uart_Se_data.predictLatency = 0;
			double unitYaw = uart_Se_data.predictYaw - uart_Se_data.curYaw;
			double unitPitch = uart_Se_data.predictPitch - uart_Se_data.curPitch;
			unitYaw /= uart_Se_data.predictLatency / 3;
			unitPitch /= uart_Se_data.predictLatency / 3;
			for(int i = 0; i < uart_Se_data.predictLatency && predictingFrame; i+= 3) {
				send_message_AM(unitYaw, unitPitch, uart_Se_data.curDistance, uart_Se_data.time + 3, (unsigned char)uart_Se_data.attackFlag);
				write(UART_ID,(char*)&DOWN_DATA_AM,DATA_DOWN_Am);
				usleep(3000);
			}
		}
	}
}


