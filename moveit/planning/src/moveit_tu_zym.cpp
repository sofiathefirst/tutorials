/* Author: zym */
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>


class MAYMOVE
{
	moveit::planning_interface::MoveGroup *group_ptr;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher display_publisher; 
	moveit_msgs::DisplayTrajectory display_trajectory;

	ros::NodeHandle nh_;
	public:
	MAYMOVE():nh_("~")
	{
		group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
		display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
		ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());	
		plannPose();
	}
	void plannPose()
	{
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation.w = 1.0;
		target_pose1.position.x = 0.28;
		target_pose1.position.y = -0.7;
		target_pose1.position.z = 1.0;
		group_ptr->setPoseTarget(target_pose1);
		
		moveit::planning_interface::MoveGroup::Plan my_plan;
		bool success = group_ptr->plan(my_plan);

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
		/* Sleep to give Rviz time to visualize the plan. */
		sleep(5.0);

		if (1)
		{
		  ROS_INFO("Visualizing plan 1 (again)");
		  display_trajectory.trajectory_start = my_plan.start_state_;
		  display_trajectory.trajectory.push_back(my_plan.trajectory_);
		  display_publisher.publish(display_trajectory);
		  /* Sleep to give Rviz time to visualize the plan. */
		  sleep(5.0);
		}
		group_ptr->move();
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grip_rect");
  ros::NodeHandle n;
  MAYMOVE littlemove;

  ros::spin();
  

  return 0;
}
