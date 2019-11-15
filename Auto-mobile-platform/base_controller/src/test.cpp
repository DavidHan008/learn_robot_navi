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
#include <base_controller/RazorImu.h>
#include <razor_imu_9dof/RazorImu.h>
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
#include <boost/asio.hpp>  
#include <boost/bind.hpp> 
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using namespace std;
using namespace boost;
/*****************************************************************************/
#define PI 3.14159
serial::Serial my_serial;
float Ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
float D = 0.34 ;    //两轮间距，单位是m
float encoder_ppr = 1024;
float wheel_radius = 0.06;
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
/****************************************************/
unsigned char data_head=0x3c;  //帧头
unsigned char data_cmd1=0x01;  //cmd1
unsigned char data_cmd2=0x54;  //cmd2
unsigned char data_cmd3=0x42;  //cmd2
unsigned char data_cmd4=0x72;  //reset
unsigned char data_ID=0x01;  //ID
unsigned char data_end=0x3e;  //帧尾
unsigned char data_length=0x11;  //长度 更改长度11
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[19]={0};   //要发给串口的数据
unsigned char chat_data[15]={0};   //要发给串口的数据
short speed=0;
int localizaton_x=0,location_ximu=0;
int localizaton_y=0,location_yimu=0;
short yaw=0,yawimu=0;
short yaw_speed=0;
float Razor_yaw_temp=0;
string rec_buffer;  //串口数据接收变量

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,position_x,position_y,oriention,vel_linear,vel_angular,Razor_yaw;


/************************************************************/
void callback(const geometry_msgs::Twist & cmd_input )//订阅/cmd_vel主题回调函数
{
   // string port("/dev/ttyUSB0");    //小车串口号
   // unsigned long baud = 115200;    //小车串口波特率
   // serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口
    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    //std::cout<<"cmd_vel_angular.z:"<<cmd_input.angular.z<<std::endl;
    linear_temp  = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
  //  std::cout<<"cmd_vel_angular.x:"<<cmd_input.linear.x<<std::endl;
    speed_data[0]=data_head;
   //添加ID
    speed_data[1]=data_ID;
  //添加长度
    speed_data[2]=data_length;
   //添加命令
    speed_data[3]=data_cmd1;
    speed_data[4]=data_cmd2;
  //添加截止位置
    speed_data[18]=data_end;//18,14
  

    //将转换好的小车速度分量为左右轮速度cmd_input.linear.x
    left_speed_data.d  = linear_temp- 0.5*angular_temp*D ;// ;
    
    right_speed_data.d = linear_temp+ 0.5*angular_temp*D ;// ;
    Razor_yaw.d= Razor_yaw_temp;

    //ROS_INFO("cmd_vel_angular.z:%f,cmd_vel_angular.x:%f, left_speed_data.d:%f,right_speed_data.d:%f",cmd_input.angular.z,cmd_input.linear.x,left_speed_data.d,right_speed_data.d);

    //存入数据到要发布的左右轮速度消息
    left_speed_data.d*=Ratio;   //放大１０００倍，mm/s
    right_speed_data.d*=Ratio;//放大１０００倍，mm/s
     Razor_yaw.d*=100; //放大１０００倍


    //ROS_INFO("left:%f,right:%f",left_speed_data.d,left_speed_data.d);



    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
    {
        speed_data[i+5]=right_speed_data.data[i];
        speed_data[i+5+4]=left_speed_data.data[i];
        speed_data[i+5+4+4]=Razor_yaw.data[i];
    }
unsigned char buf;
buf=0x00;

for(int i=0;i<17;++i)
{
buf+=speed_data[i];
}
buf=~buf+0x01;
speed_data[17]=buf;

//std::cout<<"发送的数据"<<std::endl;
/*
for(int i=0;i<19;++i)
{

std::cout<<std::hex<<(int)speed_data[i]<<std::endl;
}*/
    //在写入串口的左右轮速度数据后加入”/r/n“
    //写入数据到串口
    my_serial.write(speed_data,19);
//CHeckFLag=0;
}

void Received_data()//send 42
{

//std::cout<<"chat"<<std::endl;
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

for(int i=0;i<15;++i)
{

//std::cout<<"发送chat的数据"<<std::endl;
//std::cout<<std::hex<<(int)chat_data[i]<<std::endl;
}

 my_serial.write(chat_data,15);
//数组清零
}


void callback_yaw(const razor_imu_9dof::RazorImu Razor_imu)
{

  Razor_yaw_temp=Razor_imu.yaw;


}


int main(int argc, char **argv)
{

//ROS相关定义

    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::Subscriber sub = n.subscribe("cmd_vel", 50, callback); //订阅/cmd_vel主题
    ros::Subscriber sub_razor = n.subscribe("RazorImu", 50, callback_yaw); //订阅/cmd_vel主题
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    nav_msgs::Odometry CarOdom;//定义里程计对象
    geometry_msgs::Quaternion odom_quat; //四元数变量
    


//打开串口，配置串口


try{

    
     my_serial.setPort("/dev/ttyUSB1");
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

    double plot_x,plot_y,plot_yaw,plot_yaw_speed,plot_line_speed,plot_imu_x,plot_imu_y,plot_imu_yaw;
    ofstream fout_x_y;//x,y
    ofstream fout_yw_s;//yaw,yaw_speed
    ofstream fout_imu_x_y;//line_speed
    ofstream fout_imu_yw;//All
     fout_x_y.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
     fout_x_y.precision(4);  // 设置精度 4
     fout_yw_s.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
     fout_yw_s.precision(4);  // 设置精度 4 
     fout_imu_x_y.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
     fout_imu_x_y.precision(4);  // 设置精度 4 
     fout_imu_yw.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
     fout_imu_yw.precision(4);  // 设置精度4

    fout_x_y.open("x_y.txt",ios::trunc);//x,y
    fout_yw_s.open("yw.txt",ios::trunc);//yaw,yaw_speed
    fout_imu_x_y.open("imu_x_y.txt",ios::trunc);//line_speed
    fout_imu_yw.open("imu_yw.txt",ios::trunc);//All

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

   ros::Rate loop_rate(12);//设置发布的频率，这里的单位是HZ，也就是说，现在发布的频率的是10HZ
   int num=0;
    while(ros::ok())
    {
   //1\先发布42指令
        loop_rate.sleep();
         Received_data();// this is OK beacause the ros spinonce is normal 
        
//2\速度接受
     if(my_serial.available()){
         Received_data();
std::cout<<"hi"<<std::endl;
       rec_buffer =my_serial.read(29);    //获取串口发送来的数据


      // std::cout<<rec_buffer.data()<<std::endl;
        const unsigned char *receive_data=(const unsigned char*)rec_buffer.c_str(); //保存串口发送来的数据 


if(receive_data[0]==0x3c)

{
    num++;
    std::cout<<"当前写入次数:"<<std::oct<<num<<std::endl;
    /* for(int i=0;i<19;++i)
   {

           //std::cout<<"发送chat的数据"<<std::endl;
         std::cout<<std::hex<<(int)receive_data[i]<<std::endl;
    }
*/
   
 
       //计算速度
	int check=0;
            //判断速度正负 
	check = receive_data[6]&(0x01<<7);

	if(check==0)//zhengzhuan
	{
	 speed = receive_data[6]<<8|receive_data[5];
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		speed = ~(receive_data[6]<<8|receive_data[5])+1;
		speed = (-1)*speed;}
        //std::cout<<"speed:"<<std::dec<<speed<<std::endl;
        //计算local_x
	check = receive_data[10]&(0x01<<7);
	localizaton_x= receive_data[10]<<24|receive_data[9]<<16|receive_data[8]<<8|receive_data[7];
	if(check==0)//zhengzhuan
	{
	localizaton_x=localizaton_x;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		localizaton_x = ~(localizaton_x)+1;
		localizaton_x = (-1)*localizaton_x;}
       // std::cout<<"localizaton_x:"<<std::dec<<localizaton_x<<std::endl;
        //计算local_y
 	check = receive_data[14]&(0x01<<7);
	localizaton_y= receive_data[14]<<24|receive_data[13]<<16|receive_data[12]<<8|receive_data[11];
	if(check==0)//zhengzhuan
	{
	localizaton_y=localizaton_y;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		localizaton_y = ~(localizaton_y)+1;
		localizaton_y = (-1)*localizaton_y;}
       // std::cout<<"localizaton_y:"<<std::dec<<localizaton_y<<std::endl;
        //计算yaw
        check = receive_data[16]&(0x01<<7);
	yaw= receive_data[16]<<8|receive_data[15];
	if(check==0)//zhengzhuan
	{
	yaw =yaw;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		yaw = ~yaw+1;
		yaw = (-1)*yaw;}
       // std::cout<<"yaw:"<<std::dec<<yaw<<std::endl;
        //计算yaw_speed
        check = receive_data[18]&(0x01<<7);
	if(check==0)//zhengzhuan
	{
	yaw_speed= receive_data[18]<<8|receive_data[17];
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		yaw_speed = ~(receive_data[18]<<8|receive_data[17])+1;
		yaw_speed = (-1)*yaw_speed;}
        //std::cout<<"yaw_speed:"<<std::dec<<yaw_speed<<std::endl;

//	ximu

        check = receive_data[22]&(0x01<<7);
	location_ximu= receive_data[22]<<24|receive_data[21]<<16|receive_data[20]<<8|receive_data[19];
	if(check==0)//zhengzhuan
	{
	location_ximu=location_ximu;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		location_ximu = ~(location_ximu)+1;
		location_ximu = (-1)*location_ximu;}

//yimu
 check = receive_data[26]&(0x01<<7);
	location_yimu= receive_data[26]<<24|receive_data[25]<<16|receive_data[24]<<8|receive_data[23];
	if(check==0)//zhengzhuan
	{
	location_yimu=location_yimu;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		location_yimu = ~(location_yimu)+1;
		location_yimu = (-1)*location_yimu;}


//yaeimu
check = receive_data[28]&(0x01<<7);
	yawimu= receive_data[28]<<8|receive_data[27];
	if(check==0)//zhengzhuan
	{
	yawimu =yawimu;
	 //localizaton_x = ~(receive_data[6]<<8|receive_data[5])+1;
	}
	if(check ==0x80)//fanzhuan
		{//speed = receive_data[6]<<8|receive_data[5];
		yawimu = ~yawimu+1;
		yawimu = (-1)*yawimu;}
// 变换单位+做图
      
  //    std::cout<<"x:"<<localizaton_x<<std::endl;
      plot_x=localizaton_x;
      plot_x=0.0+plot_x/10000;//除以100单位是cm，除以10000,单位是m
    //  std::cout<<"localizaton_y:"<<localizaton_y<<std::endl;
     // std::cout<<"localizaton_x:"<<localizaton_x<<std::endl;
      plot_y=localizaton_y;//除以1000单位是mm，除以100000,单位是m
      plot_y=0.0+plot_y/10000;
      plot_yaw=yaw;
      plot_yaw=plot_yaw/100;//单位是度，但是ROS中需要的是弧度 

      plot_imu_x=location_ximu;
      plot_imu_x=3.0+plot_imu_x/10000;//除以100单位是cm，除以10000,单位是m
    //  std::cout<<"localizaton_y:"<<localizaton_y<<std::endl;
     // std::cout<<"localizaton_x:"<<localizaton_x<<std::endl;
      plot_imu_y=location_yimu;//除以1000单位是mm，除以100000,单位是m
      plot_imu_y=3.0+plot_imu_y/10000;
      plot_imu_yaw=yawimu;
      plot_imu_yaw=plot_imu_yaw/100;//单位是度，但是ROS中需要的是弧度 

   
      plot_yaw_speed=yaw_speed;
      plot_yaw_speed=plot_yaw_speed/10000000;//放大1000倍发送，单位是1000rad 然后
      plot_line_speed=speed;
      plot_line_speed=plot_line_speed/10000;//这里同x,y

      cout <<setiosflags(ios::fixed);  //只有在这项设置后，setprecision才是设置小数的
      std::cout<<setprecision(4) <<"plot_x:"<<plot_x<<";"<<"plot_y:"<<plot_y<<";"<<"yaw:"<<plot_yaw<<endl;
      std::cout<<setprecision(4) <<"plot_imu_x:"<<plot_imu_x<<";"<<"plot_imu_y:"<<plot_imu_y<<";"<<"plot_imu_yaw:"<<plot_imu_yaw<<endl;
 //保存txt
     fout_x_y<<plot_x<<" "<<plot_y<<endl;//x,y
     fout_yw_s<<(-1)*plot_yaw<<endl;//yaw,yaw_speed v
     fout_imu_x_y<<plot_imu_x<<" "<<plot_imu_y<<endl;//x,y
     fout_imu_yw<<(-1)*plot_imu_yaw<<endl;//yaw,yaw_speed v
    // fout_s<<speed<<endl;//line_speed
     //fout_A<<plot_x<<" "<<plot_y<<" "<<plot_yaw<<" "<<plot_yaw_speed<<" "<<plot_line_speed<<endl;//All

     
       //将航偏角转化成四元数
       //yaw除以100 是真正的数值。

            odom_quat = tf::createQuaternionMsgFromYaw(plot_yaw*3.1415926/180);//角度转弧度
            CarOdom.header.stamp = ros::Time::now();
            //发布坐标变换的父坐标系
//localization_x 除以100是cm,localization_y 除以100是cm
            CarOdom.header.frame_id = "odom";           
            CarOdom.pose.pose.position.x =plot_x;//单位m     
            CarOdom.pose.pose.position.y =plot_y;//单位m
            CarOdom.pose.pose.position.z = 0.0f;
            CarOdom.pose.pose.orientation = odom_quat;  
            
            //载入协方差矩阵  

             CarOdom.pose.covariance[0] = 0.1;
             CarOdom.pose.covariance[7] = 0.1;
             CarOdom.pose.covariance[14] = 0.1;
    	     CarOdom.pose.covariance[21] = 0.0025;
             CarOdom.pose.covariance[28] = 0.0025;
             CarOdom.pose.covariance[25] = 0.0025;
             CarOdom.pose.covariance = {0.1,  0.0,  0.0,  0.0,  0.0,  0.0,
                            0.0,  0.1,  0.0,  0.0,  0.0,  0.0,
                            0.0,  0.0,  0.1,  0.0,  0.0,  0.0,
                            0.0,  0.0,  0.0,  999,  0.0,  0.0,
                            0.0,  0.0,  0.0,  0.0,  999,  0.0,
                            0.0,  0.0,  0.0,  0.0,  0.0,  0.0025};


             CarOdom.child_frame_id = "base_link";       
            //载入线速度和角速度
            CarOdom.twist.twist.linear.x = plot_line_speed;
            CarOdom.twist.twist.linear.y =0;
            CarOdom.twist.twist.angular.z = plot_yaw_speed*3.1415926/180; //转化成弧度   
           //协方差

            CarOdom.twist.covariance[0] = 0.25;
            CarOdom.twist.covariance[7] = 0.25;
            CarOdom.twist.covariance[14] = 0.1;
            CarOdom.twist.covariance[21] = 0.02;
            CarOdom.twist.covariance[28] = 0.02;
            CarOdom.twist.covariance[25] = 0.02;
            CarOdom.twist.covariance = {0.25,  0.0,  0.0,  0.0,  0.0,  0.0,	
                            0.0,  0.25,  0.0,  0.0,  0.0,  0.0,
                            0.0,   0.0,  0.1,  0.0,  0.0,  0.0,
                            0.0,   0.0,  0.0,  999,  0.0,  0.0,
                            0.0,   0.0,  0.0,  0.0,  999,  0.0,
                            0.0,   0.0,  0.0,  0.0,  0.0,  0.02};
    
            //发布里程计
            odom_pub.publish(CarOdom);


            // pub transform

            static tf::TransformBroadcaster br;//定义tf对象
            tf::Quaternion q;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3( plot_x,plot_y , 0.0));
            q.setRPY(0, 0, plot_yaw*3.1415926/180);//转化成弧度
            transform.setRotation(q);
            //发布tf坐标变化
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}
}
        loop_rate.sleep();//周期休眠 控制发布的
        ros::spinOnce();//回调函数是否起作用的关键          
   }

       
   fout_x_y.close();//x,y
   fout_yw_s.close();//yaw,yaw_speed
   fout_imu_x_y.close();//line_speed
   fout_imu_yw.close();//All

    return 0;
}
