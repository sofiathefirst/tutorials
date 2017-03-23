#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <std_msgs/Time.h>
#include <sstream>
#include <stdio.h>
  ros::Publisher hand_pub;
  ros::Publisher hand_pub1;
void gripper_release(int x,int y)
{

  ros::Time start_time=ros::Time::now();

  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::Time::now()-start_time<ros::Duration(0.0005))
  {

    std_msgs::UInt16 msg;  
    std_msgs::UInt16 msg1;
    msg.data=x;
    msg1.data=y;
    hand_pub.publish(msg);
    hand_pub1.publish(msg1);
  }
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"pub");

ros::NodeHandle n;
int i=1;
//ros::Publisher value_pub = n.advertise<std_msgs::UInt16>("value",1000);

//ros::rate loop_rate(10);
//ros::Publisher value_pub = n.advertise<std_msgs::UInt16>("value",1000);
  hand_pub = n.advertise<std_msgs::UInt16>("servo", 1);
  hand_pub1 = n.advertise<std_msgs::UInt16>("servo4", 1);
while(ros::ok())
{

ROS_INFO("The %d time loop",i);
ROS_INFO("start to grip");
   // std_msgs::UInt16 msg;
    //msg.data=3;
    //value_pub.publish(msg);
gripper_release(60,80);
sleep(3);
ROS_INFO("wait");
ROS_INFO("start to release");
gripper_release(1,1);
sleep(3);
i++;

}
}
