#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
  pub_p = rospy.Publisher('joystick', Float32MultiArray, queue_size=1)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    array = [0.0,1.0,1.0,1.0]
    joystick = Float32MultiArray(data=array)
    #也可以采用下面的形式赋值
    #left_top = Float32MultiArray()
    #left_top.data = [521,1314]
    #left_top.label = 'love'
    rospy.loginfo(joystick)
    pub_p.publish(joystick)
    rate.sleep()
 
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass


'''
for c++: 
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;
 
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
 
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        msg.data.push_back(1.0);//自己写的，可行
        msg.data.push_back(2.0);
        msg.data.push_back(3.0);
        msg.data.push_back(4.0);
 
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
//订阅
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
 
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f],[%f],[%f],[%f]", msg->data.at(0),msg->data.at(1),msg->data.at(2),msg->data.at(3));
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Array_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}

'''