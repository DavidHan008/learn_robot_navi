//
// MIT License
//
// Copyright (c) 2017 MRSD Team D - LoCo
// The Robotics Institute, Carnegie Mellon University
// http://mrsdprojects.ri.cmu.edu/2016teamd/
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv){
  ros::init(argc,argv,"fake_imu_publisher");

  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);

  ros::Rate r(50);

  int i = 0;

  while(nh.ok()){
    //populate the imu message
    ros::Time imu_time = ros::Time::now();
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = imu_time;
    imu_data.header.frame_id = "imu_frame";

    geometry_msgs::Vector3 ang_vel;
    geometry_msgs::Vector3 lin_acc;

    ang_vel.x=0; ang_vel.y=0; ang_vel.z=0.0;
    lin_acc.x=-0.15; lin_acc.y=0.0; lin_acc.z=0;

    i++;
    if ((i%500)<250) { lin_acc.x=0.15;}

    imu_data.angular_velocity = ang_vel;
    imu_data.linear_acceleration = lin_acc;

    //Stuff we don't know
    geometry_msgs::Quaternion imu_orient;
    imu_orient.x = 0.0; imu_orient.y = 0; imu_orient.z = 0; imu_orient.w = 1.0;
    imu_data.orientation =  imu_orient;

    //Datasheet says "Zero-Rate Output Variation Over Temperature"=40deg/s
    //Total RMS noise = 0.38deg/s-rms
    imu_data.angular_velocity_covariance[0] = 40;
    imu_data.angular_velocity_covariance[4] = 40;
    imu_data.angular_velocity_covariance[8] = 40;

    //Noise(x-,y-Axes)<1.0, Noise(z-Axis)<1.5
    imu_data.linear_acceleration_covariance[0] = 1.0;
    imu_data.linear_acceleration_covariance[4] = 1.0;
    imu_data.linear_acceleration_covariance[8] = 1.5;

    imu_pub.publish(imu_data);
    //ROS_INFO("Published IMU data.");
    r.sleep();
  }

  return 0;

}
