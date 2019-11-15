/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm

串口通信说明：
1.写入串口
（1）内容：左右轮速度，单位为mm/s
（2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
2.读取串口
（1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
（2）格式：２１字节，[Ｘ坐标４字节][ Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
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
/*****************************************************************************/
#define PI 3.14159
serial::Serial ser; 
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.576f ;    //两轮间距，单位是m
float encoder_ppr = 1024;
float wheel_radius = 0.125;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char data_head=0x3c;  //帧头
unsigned char data_cmd1=0x01;  //cmd1
unsigned char data_cmd2=0x66;  //cmd2
unsigned char data_ID=0x01;  //ID
unsigned char data_end=0x3e;  //帧尾
unsigned char data_length=0x07;  //长度
unsigned char speed_data_state_success=0x01;  //启动
unsigned char speed_data_state_fail=0x00;  //停止
unsigned char speed_data_vel_percent=0x10;  //速度转成十进制表示的速度百分比
unsigned char speed_data_orientation_front=0x01;  //前进
unsigned char speed_data_orientation_back=0x04;  //后退
unsigned char speed_data_orientation_left=0x02;  //左转
unsigned char speed_data_orientation_right=0x03;  //右转
unsigned char data_check_front=0x27;  //前进check
unsigned char data_check_back=0x24;  //后退check
unsigned char data_check_left=0x26;  //左转check
unsigned char data_check_right=0x25;  //右转check
unsigned char data_check_stop=0x34;  //停止的check还没有计算
//通过速度占比0%来让小车停止下来
//最终要给串口发送的数据
//数据的组成：帧头+ID+长度+cmd1+cmd2+数据+校验位+帧尾
unsigned char speed_data[9]={0};   //要发给串口的数据

serial::Serial my_serial;

void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    //string port("/dev/ttyUSB0");    //小车串口号
   // unsigned long baud = 115200;    //小车串口波特率
   // serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(100)); //配置串口

    


     angular_temp =10* cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    std::cout<<"cmd_vel_angular.z:"<< angular_temp <<std::endl;
    linear_temp  = 10*cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
    std::cout<<"cmd_vel_angular.x:"<< linear_temp  <<std::endl;
    //在这边需要进行速度的变换

    //添加帧头
    speed_data[0]=data_head;
   //添加ID
    speed_data[1]=data_ID;
  //添加长度
    speed_data[2]=data_length;
   //添加命令
     speed_data[3]=data_cmd1;
     speed_data[4]=data_cmd2;
  //添加截止位置
     speed_data[8]=data_end;
     speed_data[5]=speed_data_state_success;//后面如果有其他情况进行重新赋值
//这里要写到判断语句当中，只要判断上面发送的数值是一个什么，是一个大于0的  
 //添加数据根据一些东西，来判断这里到底
    if(linear_temp>0)
    {       
        //进行前进
         speed_data[6]=speed_data_vel_percent;
         speed_data[7]=speed_data_orientation_front;
        if( angular_temp>0)
         {
           //进行左转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_left;
           }

        else if  ( angular_temp<0)
           {
           //进行右转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_right;
           }

    }
      else if(linear_temp<0)
    {       
        //进行后退
         speed_data[6]=speed_data_vel_percent;
         speed_data[7]=speed_data_orientation_back;
        if( angular_temp>0)
         {
           //进行左转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_left;
           }
        else if( angular_temp<0)
           {
           //进行右转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_right;
           }
    
    }
    else //线速度==0 也就是发布停止的指令
    {       
        //进行停止
         speed_data[6]=speed_data_state_fail;//让第6位置的百分比为0,然后让小车停止。
         speed_data[7]=speed_data_orientation_back;
        if( angular_temp>0)
         {
           //进行左转 键盘控制的真实的右转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_left;
           }
        else if( angular_temp<0)
           {

           //进行右转 键盘控制真实的进行右转
           speed_data[6]=speed_data_vel_percent;
           speed_data[7]=speed_data_orientation_right;
           }

    }


/*
    //将转换好的小车速度分量为左右轮速度
    left_speed_data.d  = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=ratio;//放大１０００倍，mm/s

    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i]=right_speed_data.data[i];
        speed_data[i+4]=left_speed_data.data[i];
    }
    //在写入串口的左右轮速度数据后加入”/r/n“
*/
   //添加帧尾
    //写入数据到串口
/*
int i;
unsigned char temp;
for(i=0;i<5;i++)
{temp=speed_data[i];
speed_data_v[i]=speed_data[9-i];
speed_data_v[9-i]=temp;}
*/
/*
int i;
unsigned char temp;
for(i=0;i<4;i++)
{temp=speed_data_test[i];
speed_data_test_v[i]=speed_data_test[6-i];
speed_data_test_v[6-i]=temp;}



for(int i=0;i<7;++i)
{
std::cout<<std::hex<<(int)speed_data_test[i]<<std::endl;
}
    my_serial.write(speed_data_test,7);
*/
for(int i=0;i<9;++i)
{
std::cout<<std::hex<<(int)speed_data[i]<<std::endl;
}
 my_serial.write(speed_data,9);
}
//定义一个全局变量，进行赋值
int main(int argc, char **argv)
{
   // string port("/dev/ttyUSB0");//小车串口号
   // unsigned long baud = 115200;//小车串口波特率
   // serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口
     my_serial.setPort("/dev/ttyUSB0");
     my_serial.setBaudrate(115200);
     serial::Timeout to=serial::Timeout::simpleTimeout(1000);
     my_serial.setTimeout(to);
     my_serial.open();
    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("cmd_vel", 50, callback); //订阅/cmd_vel主题
    ros::spin();
   // ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题
/*
 //   geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry CarOdom;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量

    double var_len,var_angle;
    var_len=(50.0f/encoder_ppr*2.0f*PI*wheel_radius)*(50.0f/encoder_ppr*2.0f*PI*wheel_radius);
    var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);

    //定义covariance矩阵，作用为解决Location和velocity的不同测量的不确定性
    float covariance[36] = {var_len,  0,     0,     0,     0,     0,  // covariance on gps_x
                            0,      var_len, 0,     0,     0,     0,  // covariance on gps_y
                            0,        0,    999,    0,     0,     0,  // covariance on gps_z
                            0,        0,     0,     999,   0,     0,  // large covariance on rot x
                            0,        0,     0,     0,     999,   0,  // large covariance on rot y
                            0,        0,     0,     0,     0,     var_angle};  // large covariance on rot z 

    ros::Rate loop_rate(50);//设置周期休眠时间
    while(ros::ok())
    {
        rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据
        const char *receive_data=rec_buffer.data(); //保存串口发送来的数据
        if(rec_buffer.length()==21) //串口接收的数据长度正确就处理并发布里程计数据消息
        {
            for(int i=0;i<4;i++)//提取X，Y坐标，方向，线速度，角速度
            {
                position_x.data[i]=receive_data[i];
                position_y.data[i]=receive_data[i+4];
                oriention.data[i]=receive_data[i+8];
                vel_linear.data[i]=receive_data[i+12];
                vel_angular.data[i]=receive_data[i+16];
            }
            //将X，Y坐标，线速度缩小1000倍
            position_x.d/=1000; //m
            position_y.d/=1000; //m
            vel_linear.d/=1000; //m/s

            //里程计的偏航角需要转换成四元数才能发布
            odom_quat = tf::createQuaternionMsgFromYaw(oriention.d/180.0f*PI);//将偏航角转换成四元数

            //载入坐标（tf）变换时间戳
            CarOdom.header.stamp = ros::Time::now();
            //发布坐标变换的父坐标系
            CarOdom.header.frame_id = "odom";           
            CarOdom.pose.pose.position.x = position_x.d;     
            CarOdom.pose.pose.position.y = position_y.d;
            CarOdom.pose.pose.position.z = 0.0f;
            CarOdom.pose.pose.orientation = odom_quat;   
            //载入covariance矩阵
            for(int i = 0; i < 36; i++)
            {
             CarOdom.pose.covariance[i] = covariance[i];
            }   

            //里程计的子坐标系
            CarOdom.child_frame_id = "base_link";       
            //载入线速度和角速度
            CarOdom.twist.twist.linear.x = vel_linear.d*cos(oriention.d* PI / 180.0f);
            CarOdom.twist.twist.linear.y = vel_linear.d*sin(oriention.d* PI / 180.0f);
            CarOdom.twist.twist.angular.z = vel_angular.d;    
            for(int i = 0; i < 36; i++)
            {
             CarOdom.twist.covariance[i] = covariance[i];
            }  
            //发布里程计
            odom_pub.publish(CarOdom);


            // pub transform变
            static tf::TransformBroadcaster br;//定义tf对象
            tf::Quaternion q;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(position_x.d, position_y.d, 0.0));
            q.setRPY(0, 0, oriention.d/180*PI);
            transform.setRotation(q);
            //发布tf坐标变化
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

            ros::spinOnce();//周期执行
            loop_rate.sleep();//周期休眠
        }
        //程序周期性调用
        //ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到
    }
*/
    return 0;
}

