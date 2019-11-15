#! /bin/bash

while date
do 
#检查网络链接每过一秒检查一次
    ping -c 1 192.168.1.189 > /dev/null 2>&1
    if [ $? -eq 0 ];then
        echo "right"
        source /opt/ros/indigo/setup.bash
        source ~/catkin_ws/devel/setup.bash	
        source ~/.bashrc
        export ROS_HOSTNAME=192.168.1.189
        export ROS_IP=192.168.1.189
        export ROS_MASTER_URI=http://192.168.1.189:11311 
        sleep 10 	
        #roslaunch /home/zgzn/catkin_ws/src/Auto-mobile-platform/base_controller/launch/sum.launch
    else
        echo "wrong"
       
#source /opt/ros/indigo/setup.bash
#source ~/catkin_ws/devel/setup.bash	
#source ~/.bashrc	
#roscore
    fi
    sleep 5 
done
