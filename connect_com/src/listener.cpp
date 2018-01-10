#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include  <tf/transform_broadcaster.h>
#include <iostream>
#include <pthread.h>
#include  <fcntl.h>
#include "connect_com/controlcan.h"
#include "stdio.h"
#include "string.h"
using namespace std;

VCI_BOARD_INFO pInfo;
#define VEL_CONTROL 0x10
#define LOCATION_RETURN 0x01
#define WIDTH_ROBOT  395
#define DIAMETER_WHEEL  170
#define ENCODER_COUNT  4096
#define PI 3.14

void cmd_vel_callback(const geometry_msgs::Twist &vel_cmd)
{
    ROS_INFO("I heard: [%f]  [%f]  [%f]  [%f]  [%f]  [%f]", vel_cmd.linear.x,vel_cmd.linear.y,vel_cmd.linear.z,vel_cmd.angular.x,vel_cmd.angular.y,vel_cmd.angular.z);
    printf(">>this is hello !\r\n");//指示程序已运行
    //里程计速度转换
    geometry_msgs::Twist twist = vel_cmd;
    double robot_speed = twist.linear.x;
    double vel_angle = twist.angular.z;//弧度制
    double right_vel = 0.0;
    double left_vel = 0.0;
    if(robot_speed==0){
    	//turning
    	right_vel = vel_angle*WIDTH_ROBOT/2;
        left_vel = (-1)*right_vel;
     }
     else if (vel_angle==0){
    	//forward or background
    	right_vel = left_vel = robot_speed;
     }
     else{
    	right_vel = robot_speed-vel_angle*WIDTH_ROBOT/2;
    	left_vel = robot_speed+vel_angle*WIDTH_ROBOT/2;
     }
    int right_vel_count = (int)(right_vel/(PI*DIAMETER_WHEEL)*ENCODER_COUNT);
    int left_vel_count = (int)(left_vel/(PI*DIAMETER_WHEEL)*ENCODER_COUNT);
    printf("the count of speed %d   %d",right_vel_count,left_vel_count);
    //右车轮需要发送的帧，结构体设置
	VCI_CAN_OBJ send_right[1];
	send_right[0].ID = 0;
	send_right[0].SendType = 0;
	send_right[0].RemoteFlag = 0;
	send_right[0].ExternFlag = 1;
	send_right[0].DataLen = 6;
	//左车轮需要发送的帧，结构体设置
	VCI_CAN_OBJ send_left[1];
	send_left[0].ID = 1;
	send_left[0].SendType = 0;
	send_left[0].RemoteFlag = 0;
	send_left[0].ExternFlag = 1;
	send_left[0].DataLen = 6;
	printf(">>yeah !\r\n");//指示程序已运行
    //右车轮速度指令
	send_right[0].Data[5] = VEL_CONTROL;
	send_right[0].Data[4] = LOCATION_RETURN;
	send_right[0].Data[3] = (BYTE)(0XFF&right_vel_count);
	send_right[0].Data[2] = (BYTE)(0XFF&right_vel_count>>8);
	send_right[0].Data[1] = (BYTE)(0XFF&right_vel_count>>16);
	send_right[0].Data[0] = (BYTE)(0XFF&right_vel_count>>24);
	//左车轮速度指令
	send_left[0].Data[5] = VEL_CONTROL;
	send_left[0].Data[4] = LOCATION_RETURN;
	send_left[0].Data[3] = (BYTE)(0XFF&left_vel_count);
	send_left[0].Data[2] = (BYTE)(0XFF&left_vel_count>>8);
	send_left[0].Data[1] = (BYTE)(0XFF&left_vel_count>>16);
	send_left[0].Data[0] = (BYTE)(0XFF&left_vel_count>>24);
	//右轮数据传输
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_right, 1) == 1){
    	printf("the right_wheel connect success!\n");
        }
    else
        {
        printf("the right_wheel connect failed!\n");
        }
    //左轮数据传输
    if(VCI_Transmit(VCI_USBCAN2, 0, 1, send_left, 1) == 1){
       	printf("the left_wheel connect success!\n");
         }
     else
         {
        printf("the left_wheel connect failed!\n");
         }
}

void *receive_func(void* param)  //接收线程。
{	int reclen = 0;
	VCI_CAN_OBJ rec[3000];
	int *run = (int*)param;
	int ind = 0;
	while((*run)&0x0f)
		{
			if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
			{
				for(int j=0;j<reclen;j++)
				{
					printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
					if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					for(int i = 0; i < rec[j].DataLen; i++)
						{
						  printf(" %02X", rec[j].Data[i]);
						 }
					printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					printf("\n");
				}
			}
			ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。
		}
	printf("run thread exit\n");//退出接收线程
	pthread_exit(0);
	}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_b");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    //打开设备
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    	{
    	    printf(">>open deivce1 success!\n");//打开设备成功
    	}else
    	{
    		printf(">>open deivce1 error!\n");
    		exit(1);
    	}
  	//初始化参数，严格参数二次开发函数库说明书。
  	VCI_INIT_CONFIG config;
  	config.AccCode=0;
  	config.AccMask=0xFFFFFFFF;
  	config.Filter=1;//接收所有帧
  	config.Timing0=0x03;/*波特率125 Kbps  0x03  0x1C*/
  	config.Timing1=0x1C;
  	config.Mode=0;//正常模式
    //初始化CAN2
  	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
  	  {
  	  	printf(">>Init CAN1 error\n");
  	  	VCI_CloseDevice(VCI_USBCAN2,0);
  	   }
  	  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  	  	{
  	  	printf(">>Start CAN1 error\n");
  	  	VCI_CloseDevice(VCI_USBCAN2,0);
  	   }
  	  	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
  	  	{
  	  	printf(">>Init can2 error\n");
  	  	VCI_CloseDevice(VCI_USBCAN2,0);
  	  	}
  	  	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
  	  	{
  	  	printf(">>Start can2 error\n");
  	  	VCI_CloseDevice(VCI_USBCAN2,0);
  	  	}
  	int m_run0=1;
  	pthread_t threadid;
  	int ret;
  	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
    //订阅/cmd_vel信号
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmd_vel_callback);
    ros::spin();
    while(n.ok())
    {
      ros::spin();
    }
     VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
     usleep(100000);//延时100ms。
  	 VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
  	 usleep(100000);//延时100ms。
  	 VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
     return 0;
}
