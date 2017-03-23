#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unistd.h>
#include <object_detect/twocamtargetpose.h>
const float PI=3.1415926;
moveit::planning_interface::MoveGroup *group;// = new moveit::planning_interface::MoveGroup("manipulator");


void calFixQright(const tf2::Vector3 positionThing, const tf2::Vector3 positionRobot, geometry_msgs::Quaternion &gq)
{
	ROS_INFO("\n\n*****calFixAngleQ****THING PISTION , ROBOT POSITION*********start******");
	tf2::Vector3 position2;
	tf2::Quaternion q;
	tf2::Vector3 OA(0,0,-1);
	double mangle;
	tf2::Vector3 maxis;

	//seeV3(positionThing);
	//seeV3(positionRobot);
	tf2::Vector3 level(0,-1,0);
	position2 = positionThing - positionRobot;
	position2.normalize();
	mangle= position2.angle(OA);
	
	//ROS_ERROR("TWO VECTOR ANGEL = %f",mangle*180/PI);
	maxis  =  OA.cross(position2);

	position2.setZ(0);
	position2.normalize();
	double fixAngle =  position2.angle(level) + 40*PI/180 ;

	tf2::Quaternion fixq,oaq(0.032218,0.706275,-0.031921,0.706483);//-0.414125,0.563932,0.434589,0.567110);//0.336918,0.635834,-0.320840,0.615844);//(0.664140,0.242730,-0.664140,0.242730);
	if(fabs(mangle-180)<0.01 ||fabs(mangle)<0.01)
	{
		gq.x = oaq.x();
		gq.y = oaq.y();
		gq.z = oaq.z();
		gq.w = oaq.w();
	}
	oaq.normalize();
	fixq.setRotation(-OA, fixAngle);
	
	fixq = fixq*oaq;	
	fixq.normalize();

	q.setRotation(maxis,mangle);//FOR FINALE LEFT
	q.normalize();
	q = q*fixq;	
	q.normalize();
	//seeQ(q);
	ROS_INFO( "  \n\n\ni calculate %.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f]'\n\n\n ",q.x(),q.y(),q.z(),q.w(),positionRobot.getX(),positionRobot.getY(),positionRobot.getZ());
	gq.x = q.x();
	gq.y = q.y();
	gq.z = q.z();
	gq.w = q.w();
	//ROS_INFO("-----calFixAngleQ----THING PISTION , ROBOT POSITION---_____end--");
	return ;

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorialpub");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  group = new moveit::planning_interface::MoveGroup("manipulator");

group->allowReplanning(true);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  move_group_interface::MoveGroup::Plan my_plan;
//  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::core::RobotStatePtr kinematic_state;

  kinematic_state = moveit::core::RobotStatePtr(group->getCurrentState());
  kinematic_state->setToDefaultValues(); 

  double fraction = 0;
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose curp;


  robot_state::RobotState start_state(*group->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group->getName());
  start_state.setFromIK(joint_model_group, curp);
  group->setStartState(start_state);
//  curp = group->getCurrentPose().pose;
 

//  group->setStartStateToCurrentState();
  
  ROS_INFO("1");
  



 // ur_modern_driver::VisualServo vsdata;
  ros::Rate loop_rate(10);



bool success;
  while (ros::ok())
  {

	ROS_ERROR("\ntoward to the right corner");

	geometry_msgs::Quaternion gq;
	std::vector<geometry_msgs::Pose> waypoints;
    	group->setStartStateToCurrentState();
    	geometry_msgs::Pose curp = group->getCurrentPose().pose;
	geometry_msgs::Pose wp;
	
	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.119688,0.342666,0.039582),gq);//1
	
	waypoints.push_back(curp);
	wp.position.x = -0.119688;
	wp.position.y = 0.342666;
	wp.position.z = 0.039582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);




	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.119688,0.342666,0.009582),gq);//2
	waypoints.push_back(curp);
	wp.position.x = -0.119688;
	wp.position.y = 0.342666;
	wp.position.z = 0.009582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt1(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);



	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.119688,0.342666,0.089582),gq);//3
	waypoints.push_back(curp);
	wp.position.x = -0.119688;
	wp.position.y = 0.342666;
	wp.position.z = 0.089582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt2(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success =group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.299033,0.298652,0.090067),gq);//4
	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.090067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt3(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.299033,0.298652,0.040067),gq);//5
	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.040067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt4(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success =group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.272385,0.623959,-0.022855),tf2::Vector3(-0.299033,0.298652,0.000067),gq);//6
	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.000067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt5(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	ROS_ERROR("\n\n\ntoward to the lefter corner");



	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.009688,0.542666,0.039582),gq);//1
	waypoints.push_back(curp);
	wp.position.x = -0.009688;
	wp.position.y = 0.542666;
	wp.position.z = 0.039582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt6(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.009688,0.542666,0.009582),gq);//2
	waypoints.push_back(curp);
	wp.position.x = -0.009688;
	wp.position.y = 0.542666;
	wp.position.z = 0.009582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt7(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success =group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.009688,0.542666,0.089582),gq);//3
	waypoints.push_back(curp);
	wp.position.x = -0.009688;
	wp.position.y = 0.542666;
	wp.position.z = 0.089582;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt8(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.299033,0.298652,0.090067),gq);//4
	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.090067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt9(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);

	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.299033,0.298652,0.040067),gq);//5
	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.040067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt10(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success =group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);


	calFixQright( tf2::Vector3(-0.379470,0.528077,-0.020979),tf2::Vector3(-0.299033,0.298652,0.000067),gq);//6

	waypoints.push_back(curp);
	wp.position.x = -0.299033;
	wp.position.y = 0.298652;
	wp.position.z = 0.000067;
	wp.orientation = gq;
	waypoints.push_back(wp);

	fraction = group->computeCartesianPath(waypoints,
						0.01,	//eef_step
						0.0,	//jump_thredshold
						trajectory);

	ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);
	if(!fraction)
	return false;
	robot_trajectory::RobotTrajectory rt11(kinematic_state->getRobotModel(),"manipulator");
	rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
	//trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	// Finally plan and execute the trajectory
	my_plan.trajectory_ = trajectory;
	success = group->execute(my_plan);
	ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
	waypoints.clear();
	
	curp = group->getCurrentPose().pose;
 
	ROS_INFO("I achieved: [%f,%f,%f,%f,%f,%f,%f]\n\n\n", curp.orientation.x,curp.orientation.y,curp.orientation.z,	
	curp.orientation.w,curp.position.x,curp.position.y,curp.position.z);
	sleep(1);
	ros::spinOnce();

	loop_rate.sleep();
   // ++count;
  }
  //vsMove( targetPose);
  return 0;
}
