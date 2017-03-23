#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_state_publisher");
  ros::NodeHandle n;
  moveit::planning_interface::MoveGroup group("manipulator");

  ros::Publisher state_pub = n.advertise<geometry_msgs::Pose>("/current_state", 1000);

  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
  {
    state_pub.publish(group.getCurrentPose().pose);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
