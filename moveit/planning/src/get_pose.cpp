#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

moveit::planning_interface::MoveGroup group("manipulator");

void chatterCallback(const geometry_msgs::Pose& pose)
{
  ROS_INFO("I heard: [%f,%f,%f,%f]", pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);

//  group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = pose.orientation.w;
  target_pose.position.x = pose.position.x;
  target_pose.position.y = pose.position.y;
  target_pose.position.z = pose.position.z;
//  group.setPoseTarget(target_pose);

//  group.move();
  sleep(2.0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  ros::Subscriber sub = n.subscribe("get_pose", 1000, chatterCallback);

/*  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());*/

  ros::spin();

  return 0;
}
