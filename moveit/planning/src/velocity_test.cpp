/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//////headers for gripper///////
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>
#include <stdio.h>
#include <sstream>


void robo_arm()
{
///**********************definition of robo_arm**************************///
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);

  moveit::planning_interface::MoveGroup group("manipulator");


  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  move_group_interface::MoveGroup::Plan my_plan;
//  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  
    bool success;
  double fraction = 0;
///***********************************************************///
  spinner.start();
  group.allowReplanning(true);
  kinematic_state = moveit::core::RobotStatePtr(group.getCurrentState());
  kinematic_state->setToDefaultValues(); 
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start_pose;
  ROS_INFO("start_pose");
  start_pose = group.getCurrentPose().pose;
  ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,
start_pose.orientation.w,start_pose.position.x,start_pose.position.y,start_pose.position.z);

  robot_state::RobotState start_state(*group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose);
  group.setStartState(start_state);

  waypoints.push_back(start_pose);
 
  geometry_msgs::Pose target_pose1;
ROS_INFO("4");
//  waypoints.push_back(target_pose1);
  target_pose1.orientation.x = -0.12;
  target_pose1.orientation.y = 0.71;
  target_pose1.orientation.z = 0.13;
  target_pose1.orientation.w = 0.68;
  target_pose1.position.x = 0.64;
  target_pose1.position.y = 0.35;
  target_pose1.position.z = 0.37;
//  target_pose1.position.z -= 0.2;
//  group.setPoseTarget(target_pose1); 
  waypoints.push_back(target_pose1);

  ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,
target_pose1.orientation.w,target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);

//  group.setPoseTargets(waypoints);
//  group.move();
//  waypoints.clear();
//  sleep(5.0);

ROS_INFO("5");
 //   waypoints.push_back(target_pose1);
  target_pose1.orientation.x = -0.12;
  target_pose1.orientation.y = 0.71;
  target_pose1.orientation.z = 0.13;
  target_pose1.orientation.w = 0.68;
  target_pose1.position.x = 0.74;
  target_pose1.position.y = 0.38;
  target_pose1.position.z = -0.03;
//  target_pose1.position.z -= 0.2;
//  group.setPoseTarget(target_pose1); 
  waypoints.push_back(target_pose1);

  ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,
target_pose1.orientation.w,target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);

  fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 2 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

  robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), "manipulator");
    success = trajectory_processing::isTrajectoryEmpty(trajectory);
    ROS_INFO("trajectory (goal) %s",success?"":"FAILED");
    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory);
 
   // Thrid create a IterativeParabolicTimeParameterization object
  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
  	success = iptp.computeTimeStamps(rt);
  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
   // Get RobotTrajectory_msg from RobotTrajectory

    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
  	my_plan.trajectory_ = trajectory;
 success = group.execute(my_plan);
ROS_INFO("finish robot_arm");
}






int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_test");
  /////////////////////////to hear the cv_position//////////////////////////////
  ros::NodeHandle n_handle; 


//////////////////////////////////////////////////////////
robo_arm();
 ros::spin();
// END_TUTORIAL


  return 0;
}
