/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odom 发布的是编码器的数据
串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
（2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
/有了电量检测之后，判断除以100之后是电压，判断电压的数字，来启动语音的播报功能
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include "std_msgs/Int32MultiArray.h"
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using namespace std;
/*****************************************************************************/
#define PI 3.14159
serial::Serial my_serial;
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.31 ;    //两轮间距，单位是m
float encoder_ppr = 1024;
float wheel_radius = 0.06;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char data_head=0x3c;  //帧头
unsigned char data_cmd1=0x01;  //cmd1
unsigned char data_cmd2=0x54;  //cmd2
unsigned char data_cmd4=0x70;  //cmd2
unsigned char data_cmd3=0x42;  //cmd2
unsigned char data_ID=0x01;  //ID
unsigned char data_end=0x3e;  //帧尾
unsigned char data_length=0x0d;  //长度
unsigned char data_length2=0x0e;  //长度
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[16]={0};   //要发给串口的数据
unsigned char chat_data[15]={0};   //要发给串口的数据
std_msgs::Int32MultiArray face_msg;
short speed=0;
int localizaton_x=0;
int localizaton_y=0;
short yaw=0;
short yaw_speed=0;
float left_temp=0;
float right_temp=0;

string rec_buffer;  //串口数据接收变量

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular;


/************************************************************/
void Received_data()//send 42
{

//std::cout<<"chat"<<std::endl;
    chat_data[0]=data_head;
   //添加ID
    chat_data[1]=data_ID;
  //添加长度
    chat_data[2]=data_length;
   //添加命令
    chat_data[3]=data_cmd1;
    chat_data[4]=data_cmd3;
  //添加截止位置
    chat_data[14]=data_end;

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        chat_data[i+5]=0;
        chat_data[i+5+4]=0;
    }
unsigned char buf;
buf=0x00;

for(int i=0;i<13;++i)
{
buf+=chat_data[i];
}
buf=~buf+0x01;
chat_data[13]=buf;

for(int i=0;i<15;++i)
{

//std::cout<<"发送chat的数据"<<std::endl;
//std::cout<<std::hex<<(int)chat_data[i]<<std::endl;
}

 my_serial.write(chat_data,15);
//数组清零
}

void face_callback(const  std_msgs::Int32MultiArray &myMsg)//订阅/cmd_vel主题回调函数
{
      face_msg=myMsg;

}

void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
    speed_data[0]=data_head;
   //添加ID
    speed_data[1]=data_ID;
  //添加长度
    speed_data[2]=data_length2;
   //添加命令
    speed_data[3]=data_cmd1;
    speed_data[4]=data_cmd4;
  //添加截止位置
    speed_data[16]=data_end;
    

//这里添加判断，来修改运动模型
    left_speed_data.d  =(linear_temp- 0.5*angular_temp*D);// ;
    right_speed_data.d = (linear_temp+ 0.5*angular_temp*D) ;// ;
//添加运动状态的判断
    left_temp=left_speed_data.d;
    right_temp=right_speed_data.d;
    left_speed_data.d*=ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=ratio;//放大１０００倍，mm/s

    ROS_INFO("left:%f,righ:%f",left_temp,right_temp);

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i+5]=right_speed_data.data[i];
        speed_data[i+5+4]=left_speed_data.data[i];
    }

//如果是1表示遥控，超声波开启
 speed_data[13]=0x01;

//如果是0的话表示图像,超声波关闭
/*
if(face_msg.data[1]==1)
{
 speed_data[13]=0x00;
}
*/

//添加对运动状态的判断
//首先判断是不是有转向的问题
//来判断左右转
//再判断直行还是后退

 speed_data[14]=0x01;



unsigned char buf;
buf=0x00;
for(int i=0;i<15;++i)
{
buf+=speed_data[i];
}
buf=~buf+0x01;
speed_data[15]=buf;

for(int i=0;i<17;++i)
{
std::cout<<"发送的数据"<<std::endl;
std::cout<<std::hex<<(int)speed_data[i]<<std::endl;
}
// Received_data();

//if(my_serial.available()){

    my_serial.write(speed_data,17);
//}
}


int main(int argc, char **argv)
{

try{

     my_serial.setPort("/dev/ttyUSB0");
     my_serial.setBaudrate(115200);
     serial::Timeout to=serial::Timeout::simpleTimeout(1000);
     my_serial.setTimeout(to);
     my_serial.open();    //保存编码器的数据
}
catch (serial::IOException &e)
{
     ROS_ERROR_STREAM("unable to open port");
     return -1;
}

if(my_serial.isOpen())
{

ROS_INFO_STREAM("serial port initialized");

}

    ros::init(argc, argv, "laser_odom");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("cmd_vel", 50, callback); //订阅/cmd_vel主题
   // ros::Subscriber face_sub = n.subscribe("faceCoord", 50, face_callback); //订阅/cmd_vel主题
  // ros::Rate loop_rate(12);//设置发布的频率，这里的单位是HZ，也就是说，现在发布的频率的是10HZ
    int num=0;
ros::spin();
  /*  while(ros::ok())
    {
   //1\先发布42指令
        loop_rate.sleep();
         Received_data();// this is OK beacause the ros spinonce is normal 
        
//2\速度接受
     if(my_serial.available()){
         Received_data();
std::cout<<"hi"<<std::endl;
       rec_buffer =my_serial.read(32);    //获取串口发送来的数据
        std::cout<<rec_buffer.data()<<std::endl;
        const unsigned char *receive_data=(const unsigned char*)rec_buffer.c_str(); //保存串口发送来的数据 


if(receive_data[0]==0x3c)
{
    num++;
    std::cout<<"当前写入次数:"<<std::oct<<num<<std::endl;
///这里进行数据处理---通过receive_data这个数组来进行数据处理。
//打印接收到的数组
  for(int i=0;i<32;++i)
   {

           //std::cout<<"发送chat的数据"<<std::endl;
         std::cout<<std::hex<<(int)receive_data[i]<<std::endl;
   }


}



    }
       loop_rate.sleep();//周期休眠 控制发布的
        ros::spinOnce();//回调函数是否起作用的关键 
}

 */
    return 0;
}
