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
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include "std_msgs/Float64.h"
//#include <razor_imu_9dof/RazorImu.h>
#include <fstream>
#include <iostream>
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
#include <iostream>  
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using namespace std;
using namespace boost;
serial::Serial my_serial;
/*****************************************************************************/
#define PI 3.14159
#define PI_DIV_180 0.017453292519943
#define ENCODER_PPR 32000
float Ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.50 ;    //两轮间距，单位是m
//float encoder_ppr = 1024;
float wheel_radius = 0.125 ;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
string rec_buffer;  //串口数据接收变量
/****************************************************/
unsigned char data_head=0x3c;  //帧头
unsigned char data_cmd1=0x01;  //cmd1
unsigned char data_cmd2=0x54;  //cmd2 用于ROS数据通信
unsigned char data_cmd3=0x42;  //cmd3用于打开串口
unsigned char data_ID=0x01;  //ID
unsigned char data_end=0x3e;  //帧尾
unsigned char data_length=0x0d;  //长度 更改长度11
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[19]={0};   //要发给串口的数据
unsigned char chat_data[15]={0};   //要发给串口的数据
std_msgs::Float64 razor_yaw;
std_msgs::Float64 razor_angle;
int32_t prev_time_enc_left,prev_time_enc_right;
double current_time_enc_left,current_time_enc_right,delta_enc_left,delta_enc_right;
double current_time_x,current_time_y,delt_left_s,delt_right_s,delt_x,delt_y,x_vel,angle_vel;
//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    int it;
    unsigned char data[4];
}right_speed_data,left_speed_data,curr_rightQC,curr_leftQC;
//进行转化

/*
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular,Razor_yaw;

*/
/************************************************************/
void callback(const geometry_msgs::Twist & cmd_input )//订阅/cmd_vel主题回调函数
{
    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    //std::cout<<"cmd_vel_angular.z:"<<cmd_input.angular.z<<std::endl;
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
   //  std::cout<<"cmd_vel_angular.x:"<<cmd_input.linear.x<<std::endl;
   //进行速度限定，转角0.2 直线0.1
   if (angular_temp>0.2)
  {
   angular_temp=0.2;
  }
   if (angular_temp<-0.2)
  {
   angular_temp=-0.2;
  }

  if (linear_temp>0.2)
  {
    linear_temp=0.2;
  }
   if (linear_temp<-0.2)
  {
    linear_temp=-0.2;
  }
  //将需要写入的串口数据进行赋值
    speed_data[0]=data_head;
   //添加ID
    speed_data[1]=data_ID;
  //添加长度
    speed_data[2]=data_length;
   //添加命令
    speed_data[3]=data_cmd1;
    speed_data[4]=data_cmd2;
  //添加截止位置
   speed_data[14]=data_end;//18,14
   //将转换好的小车速度分量为左右轮速度cmd_input.linear.x
    left_speed_data.d  = linear_temp- 0.5*angular_temp*D ;// ;
    
    right_speed_data.d = linear_temp+ 0.5*angular_temp*D ;// ;

    //ROS_INFO("cmd_vel_angular.z:%f,cmd_vel_angular.x:%f, left_speed_data.d:%f,right_speed_data.d:%f",cmd_input.angular.z,cmd_input.linear.x,left_speed_data.d,right_speed_data.d);

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=Ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=Ratio;//放大１０００倍，mm/s

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i+5]=right_speed_data.data[i];
        speed_data[i+5+4]=left_speed_data.data[i];
    }
    unsigned char buf;
    buf=0x00;

    for(int i=0;i<13;++i)
    {
        buf+=speed_data[i];
    }
    buf=~buf+0x01;
    speed_data[13]=buf;

     /*std::cout<<"发送的数据"<<std::endl;
    for(int i=0;i<16;++i)
    {
    std::cout<<std::hex<<(int)speed_data[i]<<std::endl;
    }
    */
    //写入数据到串口
    my_serial.write(speed_data,15);
}


//这里send42才可以打开串口进行串口通信
void Received_data()//send 42
{

    chat_data[0]=data_head;
   //添加ID
    chat_data[1]=data_ID;
  //添加长度
    chat_data[2]=0x0d;
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
    my_serial.write(chat_data,15);
//数组清零
}

void callback_yaw(const std_msgs::Float64 razor_yaw_temp)
{
  razor_yaw=razor_yaw_temp;//这里的单位是弧度
  //确保在-pi~pi之间
  if(razor_yaw.data<=-PI)
  {
  razor_yaw.data+=2*PI;
  }
  else if(razor_yaw.data>PI)
  {
  razor_yaw.data-=2*PI;
  }
}
void callback_angle_vel(const std_msgs::Float64 razor_angle_temp)
{
  razor_angle=razor_angle_temp;//这里的单位是弧度
}

int main(int argc, char **argv)
{
//ROS相关定义

    ros::init(argc, argv, "encoder");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("/cmd_vel", 50, callback); //订阅/cmd_vel主题
    ros::Subscriber sub_razor = n.subscribe("/razor_imu", 50, callback_yaw); //订阅/cmd_vel主题
    ros::Subscriber sub_razor_angle = n.subscribe("/razor_angle_vel", 50, callback_angle_vel); //订阅/cmd_vel主题
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题

    //我打算在这个地方接收razor的数据，同时发布odom 这里接收的目的只是为了更好的来计算里程
    nav_msgs::Odometry odom_msg;;//定义里程计对象
    tf::TransformBroadcaster odom_broadcaster;//发布TF变换
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_tf;

//打开串口，配置串口
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
	//用于判断已经打开串口
    if(my_serial.isOpen())
     {
	ROS_INFO_STREAM("serial port initialized");
      }

   
    //协防差矩阵初始化
/*
    double var_len,var_angle;
    var_len=(50.0f/encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/encoder_ppr*2.0f*PI*wheel_radius);
    var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);
    //定义covariance矩阵，作用为解决Location和velocity的不同测量的不确定性
    float covariance[36] = {var_len,  0,     0,     0,     0,     0,  // covaFriance on gps_x
                            0,      var_len, 0,     0,     0,     0,  // covariance on gps_y
                            0,        0,    999,    0,     0,     0,  // covariance on gps_z
                            0,        0,     0,     999,   0,     0,  // large covariance on rot x
                            0,        0,     0,     0,     999,   0,  // large covariance on rot y
                            0,        0,     0,     0,     0,     var_angle};  // large covariance on rot z 
*/

    ros::Rate loop_rate(20);//设置发布的频率，这里的单位是HZ，也就是说，现在发布的频率的是10HZ--100ms,如果是10hz,会导致缓慢的问题
    int num=0;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();//进行初始化赋值
    last_time = ros::Time::now();
    //初始化
    current_time_enc_left=0;
    current_time_enc_right=0;
    prev_time_enc_left=0;
    prev_time_enc_right=0;
    while(ros::ok())
    {
       //1\先发布42指令
         Received_data();// this is OK beacause the ros spinonce is normal  
        //2\速度接受
     if(my_serial.available()){
         Received_data();
       rec_buffer =my_serial.read(13);    //获取串口发送来的数据
      // std::cout<<rec_buffer.data()<<std::endl;
       const unsigned char *receive_data=(const unsigned char*)rec_buffer.c_str(); //保存串口发送来的数据 
       unsigned char curr_leftQC_char,curr_rightQC_char;
      //如果开头是3C证明接受到的数据是正确的
		if(receive_data[0]==0x3c)
		{
   			 num++;
   			 std::cout<<"当前写入次数:"<<std::oct<<num<<std::endl;
                 /*int j=0;
                 for(int i=4;i>0;i--)//提取当前左右轮的编码器的数值
                 {
              		    curr_leftQC.data[i]=receive_data[j+5];
             		    curr_rightQC.data[i]=receive_data[j+4+5];
                            j++;               
                 }*/
                //计算部分
		//从串口得到左边编码器的脉冲数，和右编码器的脉冲数
                  curr_leftQC.data[0]=receive_data[5];
 		  curr_leftQC.data[1]=receive_data[6];
                  curr_leftQC.data[2]=receive_data[7];
                  curr_leftQC.data[3]=receive_data[8];

                  curr_rightQC.data[0]=receive_data[9];
                  curr_rightQC.data[1]=receive_data[10];
                  curr_rightQC.data[2]=receive_data[11];
                  curr_rightQC.data[3]=receive_data[12];
                  //std::cout<<"左编码器"<<std::dec<<curr_leftQC.it<<std::endl;//这里有个问题，是因为需要计算差值 这里的单位是QC
		  
                  current_time_enc_left=curr_leftQC.it;//一个int可以赋值double
		  current_time_enc_right=curr_rightQC.it;
		  current_time = ros::Time::now();
		  delta_enc_left=current_time_enc_left-prev_time_enc_left;
		  delta_enc_right=current_time_enc_right-prev_time_enc_right;
		  double delta_t=(current_time-last_time).toSec();//时间间隔,单位是秒
		  delt_left_s=delta_enc_left*PI*wheel_radius/ENCODER_PPR;//左轮的delt位移-- 单位是m
		  delt_right_s=delta_enc_right*PI*wheel_radius/ENCODER_PPR;//右边轮的delt位移---32000是脉冲数字
		 // delt_left_s=delt_left_s/100;//单位转化成m
		 // delt_right_s=delt_right_s/100;//单位转化成m
                 //这里的yaw需要做判断，然后将所有的数据打印到屏幕上面
                  delt_x=(delt_left_s+delt_left_s)/2*cos(razor_yaw.data);//不知道如何引入cos	  
		  delt_y=(delt_left_s+delt_left_s)/2*sin(razor_yaw.data);
		  current_time_x+=delt_x;
		  current_time_y+=delt_y;
                  //计算线速度和角速度
		  x_vel=(delt_right_s+delt_right_s)/2/delta_t;
		 // angle_vel=(delt_right_s-delt_right_s)/delta_t/D;编码器计算
		  angle_vel=razor_angle.data;
                  ROS_INFO("01-: current_time_enc_left：%f;delta_enc_left:%f",current_time_enc_left,delta_enc_left);
                  ROS_INFO("02-delt: delta_t：%f;delt_x:%f;delt_y:%f",delta_t,delt_x,delt_y);  		  // 发布坐标变换部分
                  ROS_INFO("03-: current_time_x：%f;current_time_y:%f;razor_yaw.data:%f",current_time_x,current_time_y,razor_yaw.data);  		  // 发布坐标变换部分
                 
                  odom_quat = tf::createQuaternionMsgFromYaw(razor_yaw.data);	
  		  odom_tf.header.stamp = current_time;
  		  odom_tf.header.frame_id = "odom";
                  odom_tf.child_frame_id = "base_link"; //base_link 
  		  odom_tf.transform.translation.x = current_time_x;
  		  odom_tf.transform.translation.y = current_time_y;
 		  odom_tf.transform.translation.z = 0.0;
 		  odom_tf.transform.rotation = odom_quat;
 		  odom_broadcaster.sendTransform(odom_tf);

                  //发布odom

 		  odom_msg.header.stamp = current_time;
		  odom_msg.header.frame_id = "odom";

  // Set the position
		  odom_msg.pose.pose.position.x = current_time_x;
		  odom_msg.pose.pose.position.y = current_time_y;
		  odom_msg.pose.pose.orientation = odom_quat;

  // Set the velocity
		  odom_msg.child_frame_id = "base_link"; //base_link
		  odom_msg.twist.twist.linear.x = x_vel;
		  odom_msg.twist.twist.linear.y = 0;
 		  odom_msg.twist.twist.angular.z = angle_vel;

 		  odom_pub.publish(odom_msg);

		//进行下一个循环
                  last_time=current_time;
       		  prev_time_enc_left=current_time_enc_left;
		  prev_time_enc_right=current_time_enc_right;                  
		  
         //主要的思路是，通过解算左右轮编码器的数值，通过航迹推演，来计算坐标--另外要注意单位
		/*	current_time_enc1=data.left_wheel;
			current_time_enc2=data.right_wheel;
			current_time = ros::Time::now();
			delta_enc1=current_time_enc1-prev_time_enc1;
			delta_enc2=current_time_enc2-prev_time_enc2;
			double delta_t=(current_time-last_time).toSec();//时间间隔
			delt_left=delt_leftQC*PI*D;//左轮的delt位移---这个地方需要看下计算过程
			delt_right=delt_rightQC*PI*D;//右边轮子的delt位移
			delt_s=(delt_left+delt_right)/2;//编码器计算的delt_s
			delt_encoder_yaw=(delt_right-delt_left)/D;//编码器的yaw
			encoder_yaw+=delt_encoder_yaw;
			delt_x=delt_s*cos((encoder_yaw+delt_encoder_yaw/2)*PI_DIV_180);//编码器的yaw
			delt_y=delt_s*sin((encoder_yaw+delt_encoder_yaw/2)*PI_DIV_180);//编码器的yaw
			locaton_x+=delt_x;
			locaton_y+=delt_y;
			locaton_v=delt_s/delt_t;
			location_w=delt_encoder_yaw/delt_t;*/
//然后现在假设我已经得到了左右轮的编码器的数值

		}
	}
        loop_rate.sleep();//周期休眠 控制发布的
        ros::spinOnce();//回调函数是否起作用的关键          
   }

    return 0;
}
