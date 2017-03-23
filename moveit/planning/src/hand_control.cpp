#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <sstream>
#include "std_msgs/UInt16.h"
#include <std_msgs/Time.h>
#include <math.h>
ros::Publisher value_pub; 

void grip_pick()
{

ros::Time start_time=ros::Time::now();

ros::Rate loop_rate(10);

int count = 0;
while (ros::Time::now()-start_time<ros::Duration(5))
  {

    std_msgs::UInt16 msg;
    msg.data=3;
    value_pub.publish(msg);
ROS_INFO("1111");

    loop_rate.sleep();
   // ++count;
  }
}

void gripper_release()
{

ros::Time start_time = ros::Time::now();

ros::Rate loop_rate(10);

int count = 0;
while (ros::Time::now()-start_time<ros::Duration(5))
  {

    std_msgs::UInt16 msg;
    msg.data=0;
    value_pub.publish(msg);

  
ROS_INFO("1111");
    loop_rate.sleep();
    //++count;
  }
}

 
void chatterCallback(geometry_msgs::Pose pose)
{
	ROS_INFO("I heard  cur: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);


  geometry_msgs::Pose final_pose, final_pose1;

    final_pose.orientation.x = 0.137257;//the first point 
    final_pose.orientation.y = 0.509602;
    final_pose.orientation.z = -0.003544;
    final_pose.orientation.w = 0.849384;
    final_pose.position.x = 0.360366;
    final_pose.position.y = 0.2996;
    final_pose.position.z = -0.11;

ROS_INFO("444");
/*
(fabs(pose.position.x - final_pose.position.x )<= 0.1  && fabs(pose.position.y - final_pose.position.y) <= 0.1  && fabs(pose.position.z - final_pose.position.z ) <= 0.1 && fabs(pose.orientation.x - final_pose.orientation.x) <=0.1 && fabs(pose.orientation.y -final_pose.orientation.y)<=0.1 && fabs(pose.orientation.z - final_pose.orientation.z)<=0.1 && fabs(pose.orientation.w - final_pose.orientation.w)<=0.1)*/
    if((fabs(pose.position.x - final_pose.position.x )<= 0.1  && fabs(pose.position.y - final_pose.position.y) <= 0.1  && fabs(pose.position.z - final_pose.position.z ) <= 0.1 && fabs(pose.orientation.x - final_pose.orientation.x) <=0.1 && fabs(pose.orientation.y -final_pose.orientation.y)<=0.1 && fabs(pose.orientation.z - final_pose.orientation.z)<=0.1 && fabs(pose.orientation.w - final_pose.orientation.w)<=0.1))
    {
 ROS_INFO("2222222");
       gripper_release();ROS_INFO("0000");
    }
    /*
    else if(   (fabs(pose.position.x - final_pose.position.x )<= 0.1  && fabs(pose.position.y - final_pose.position.y) <= 0.1))
	{
	gripper_release();	ROS_INFO("333");
	} */   

    else 
    {
       grip_pick();ROS_INFO("66");
    }

}




int main(int argc, char** argv)
{
   ros::init(argc, argv, "hand_control");
   ros::NodeHandle n;
//ROS_INFO("1111");
   ros::Subscriber sub = n.subscribe("/current_state",1000,chatterCallback);
value_pub = n.advertise<std_msgs::UInt16>("/value",1000);
ros::Rate loop_rate(10);

 //  ros::spin();
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

   return 0;

}


