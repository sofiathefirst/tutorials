/* Author: Sachin Chitta */
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
#include <object_detect/Check.h>
//////headers for gripper///////
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>
#include <stdio.h>
#include <sstream>
#include <object_detect/poseDepth.h>


moveit::planning_interface::MoveGroup *group;
geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
float error = 0.05;   //zliu7
//int angel1,angle2; //define two varies of the angle 
int angel1,angle2; //define two varies of the angle
int number;
class GRIPCV
{
  ros::NodeHandle nh_;
  ros::Publisher tar_state_pub;
  ros::Publisher display_publisher;
  ros::Publisher planning_scene_diff_publisher;
  ros::Subscriber cv_position_sub;
  ros::Subscriber pose_target_;
  //ros::Subscriber  servo_angle; //
 
  ros::Publisher hand_pub;
  ros::Publisher hand_pub1;
  ros::ServiceClient client;
  object_detect::Check srv;
//the next two line is add by zhang yu mei 20160120, for service request , thus can solve the problem of data delay. 
  ros::ServiceClient pos_pose_client;
  object_detect::poseDepth srvPositionPose;
  geometry_msgs::Pose thingpose, boxpose;
  bool thingposeflag;
 // boxpose
 int gwidth;
  double xr;
  double yr;
  int gplacecnt;

  moveit::planning_interface::MoveGroup *group_ptr;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  robot_model::RobotModelPtr model;

  moveit_msgs::PlanningScene planning_scene;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  public:
  GRIPCV():
	nh_("~")
  {
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::poseDepth>("/srvPoseDepth");
 thingposeflag=false;
 // boxpose
   xr=0.684106;
   yr=0.729383;
   gplacecnt=0;
   boxpose.orientation.x = -0.6;
   boxpose.orientation.y = 0.2;
   boxpose.orientation.z = 0;
   boxpose.orientation.w = 0.693939;
   boxpose.position.x = 0.161964;
   boxpose.position.y = -0.689242;
   boxpose.position.z = 0.131002;
//[0.693939,0.161964,-0.689242,0.131002,-0.6,0.2,-0.0]]
   

    hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000);
    hand_pub1 = nh_.advertise<std_msgs::UInt16>("/servo4", 1000);

    tar_state_pub = nh_.advertise<geometry_msgs::Pose>("/target_state", 1, true);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    //servo_angle = nh_.subscribe("short_extent",1,&GRIPCV::shortExtentCallback,this);//width of contour
   // hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000,true);
    client = nh_.serviceClient<object_detect::Check>("/check_position");
    group_ptr->setPlanningTime(3.0);
    group_ptr->allowReplanning(true);
    kinematic_state = moveit::core::RobotStatePtr(group_ptr->getCurrentState());
    kinematic_state->setToDefaultValues();
   
    
    // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());

//  robot_model::RobotModelPtr model_;
  model.reset(new robot_model::RobotModel(group_ptr->getCurrentState()->getRobotModel()->getURDF(),        group_ptr->getCurrentState()->getRobotModel()->getSRDF()));
//the next service related code written by zhang yumei 20160120, 
//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! return old place!");
	robo_arm();
	}
	 gwidth =  srvPositionPose.response.width;
	 angle2 = (int)(-0.01934*gwidth*gwidth+0.19408*gwidth+102.89+8);
	 ROS_WARN("WIDTH = %d,angle2=%d ",gwidth,angle2);
	 movetopose(srvPositionPose.response.pose);
	 
}

  void robo_arm(int number  = 10)
  {
	ROS_INFO("*************20160203START TO PULL BACK**************88");
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	float fraction = 0.0;
	std::vector<geometry_msgs::Pose> waypoints;
	kinematic_state = group_ptr->getCurrentState();
	if (thingposeflag)
	{
		thingpose.position.z+=0.2;
		waypoints.push_back(thingpose);
	//thingposeflag=false;
	}
	if(number > 90)
	{
		geometry_msgs::Pose midpose;
		midpose.position.x =  0.401205;
		midpose.position.y = 0.511234;
		midpose.position.z = 0.2;

		midpose.orientation =  boxpose.orientation;

		waypoints.push_back(midpose);
		//0.401205, 0.511234, 0.100787
		midpose.position.x =  0.633569;
		midpose.position.y = 0.309136;
		midpose.position.z = 0.2;
		waypoints.push_back(midpose);
	//0.633569, 0.309136, 0.095324,
	}
	//waypoints.push_back(thingpose);
        
	boxpose.orientation.x = -0.6+0.08*xr*gplacecnt;
	boxpose.orientation.y = 0.2+0.08*yr*gplacecnt;
        gplacecnt++;
	if(boxpose.orientation.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
	boxpose.position.z += 0.1;
	waypoints.push_back(boxpose);
	boxpose.position.z -= 0.1;
	waypoints.push_back(boxpose);

	bool compute = false;

	fraction = group_ptr->computeCartesianPath(waypoints,
		       0.01,  // eef_step
		       0.0,   // jump_threshold
		       trajectory);

	ROS_INFO("Visualizing plan 2 (cartesian path) (%.2f%% acheived)",
	fraction * 100.0);

	robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), "manipulator");
	bool success = trajectory_processing::isTrajectoryEmpty(trajectory);
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

	success = group_ptr->execute(my_plan);
	sleep(1);
	ROS_INFO("angle release!");
	gripper_release(1,1);


	//the next service related code written by zhang yumei 20160120, 
	//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(0.1);
	ROS_INFO("no object ! waiting the target object position data");
	}
	gwidth =  srvPositionPose.response.width;
	angle2 = (int)(-0.01934*gwidth*gwidth+0.19408*gwidth+102.89+8);
	ROS_WARN("WIDTH = %d ",gwidth);
	ROS_WARN("angle = %d ",angle2);
	movetopose(srvPositionPose.response.pose);
	ROS_INFO("finish robot_arm");
  }
//-0.680760,-0.725815,0.098787
	void movetopose(const geometry_msgs::Pose& pose)
	{
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);
		thingpose = pose;	
		move_group_interface::MoveGroup::Plan my_plan;
		moveit_msgs::RobotTrajectory trajectory;
		std::vector<geometry_msgs::Pose> waypoints;

		waypoints.push_back(boxpose);
		thingposeflag = true;

		if(number > 90)
		{
			geometry_msgs::Pose midpose;

			midpose.position.x =  0.633569;
			midpose.position.y = 0.309136;
			midpose.position.z = 0.2;

			midpose.orientation =  boxpose.orientation;

			waypoints.push_back(midpose);
			midpose.position.x =  0.401205;
			midpose.position.y = 0.511234;
			midpose.position.z = 0.2;
			waypoints.push_back(midpose);

		}

		waypoints.push_back(thingpose);
		thingpose.position.z-=0.1;
		waypoints.push_back(thingpose);
		target = thingpose;
		ROS_INFO("version 2START TO SEND TARGET");
		std_msgs::Time squeeze_time;
		squeeze_time.data=ros::Time::now();
		kinematic_state = group_ptr->getCurrentState();
		while(ros::Time::now() - squeeze_time.data < ros::Duration(1))
		{
			tar_state_pub.publish(target);
		}
		ROS_INFO("START TO COMPUTING PATH");

		float fraction = group_ptr->computeCartesianPath(waypoints,
			       0.01,  // eef_step
			       0.0,   // jump_threshold
			       trajectory);

		ROS_INFO("Visualizing plan 'start_trajectory' (cartesian path) (%.2f%% acheived)",
		fraction * 100.0);

		robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(),"manipulator");
		rt.setRobotTrajectoryMsg(*(kinematic_state), trajectory);
		bool success = trajectory_processing::isTrajectoryEmpty(trajectory);
		ROS_INFO("trajectory (goal) %s",success?"":"FAILED");

		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		success = iptp.computeTimeStamps(rt);
		ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");  

		rt.getRobotTrajectoryMsg(trajectory);
		my_plan.trajectory_ = trajectory;

		ROS_INFO("Visualizing plan 1 (cartesian path) (%.2f%% acheived)",
		fraction * 100.0);
		success = group_ptr->execute(my_plan);
		sleep(1);
		gripper_release(10,angle2);
		sleep(1);
		robo_arm(number);

	}

	void gripper_release(int x,int y)
	{
	    
		ros::Time start_time=ros::Time::now();

		ros::Rate loop_rate(100);

		int count = 0;
		while (ros::Time::now()-start_time<ros::Duration(1))
		{
			std_msgs::UInt16 msg;  
			std_msgs::UInt16 msg1;
			msg.data=x;
			msg1.data=y;
			hand_pub.publish(msg);
			hand_pub1.publish(msg1);

			loop_rate.sleep();
			++count;
		}
		ROS_INFO("I release la~");
	}
}; 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grip_rect");
  ros::NodeHandle n;
  GRIPCV grip;
  ros::spin();
  return 0;
}
