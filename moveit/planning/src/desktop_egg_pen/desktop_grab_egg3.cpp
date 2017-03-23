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
#include <object_detect/desktopData.h>
#include <std_msgs/Time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Matrix3x3.h>
const tf2::Quaternion midq(0.5,0.5,0.5,-0.5);//x,y,z,w
const tf2::Matrix3x3 MID_MATRIX(midq);
geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
const float FINGERLEN = 0.30;
const float FINGERLEN2 = 0.13;
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
  object_detect::desktopData srvPositionPose;
  geometry_msgs::Pose thingpose, homepose,midpose;
  bool thingposeflag,midflag;
 // homepose
 int gwidth;
  double xr;
  double yr;
  int gplacecnt;
  int failcnt = 0;
  moveit::planning_interface::MoveGroup *group_ptr;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  robot_model::RobotModelPtr model;

  moveit_msgs::PlanningScene planning_scene;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  public:


int euler2matrix2quaternion(double z , geometry_msgs::Quaternion &gq)
{
	tf2::Matrix3x3 eulerMatrix;

	eulerMatrix.setRPY(3.1415926,0,z);
		
	eulerMatrix = eulerMatrix * MID_MATRIX ;
	tf2::Quaternion tq;

	eulerMatrix.getRotation(tq);
	ROS_INFO("eulQ:    X=%f,Y=%f,Z=%f,W=%f",tq.x(),tq.y(),tq.z(),tq.w());
	gq.x = tq.x();
	gq.y = tq.y();
	gq.z = tq.z();
	gq.w = tq.w();
	return 0;
}

  GRIPCV():
	nh_("~")
  {
ros::AsyncSpinner spinner(1);
	spinner.start();
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::desktopData>("/srvDesktopData");
 thingposeflag=false;
midflag = false;
 // homepose-0.673735,0.738973,0
   xr=-0.673735;
   yr=-0.738973;
   gplacecnt=0;

//-I heard: [-0.556427,0.484032,0.519925,0.431022,-0.477447,0.640033,0.187705]]

	target.orientation.x=0.70207;
	target.orientation.y=0.0600884;
	target.orientation.z=-0.708811;
	target.orientation.w=0.0327785;
 

homepose.orientation.x=-0.00820612;
homepose.orientation.y=0.707095;
homepose.orientation.z=0.0158262;
homepose.orientation.w=0.706894;
homepose.position.x=-0.0106477;
homepose.position.y=0.646863;
homepose.position.z=0.140246;


    hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 10);
    hand_pub1 = nh_.advertise<std_msgs::UInt16>("/servo4", 10);

    tar_state_pub = nh_.advertise<geometry_msgs::Pose>("/target_state", 1, true);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
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
gripper_release(10,10);
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! return old place!");
	robo_arm();
	}
	 //gwidth =  srvPositionPose.response.width;
	 angle2 = (int)(-0.01934*gwidth*gwidth+0.19408*gwidth+102.89+8);
	 ROS_WARN("WIDTH = %d,angle2=%f ",gwidth,srvPositionPose.response.angle);
	 //target = group_ptr->getCurrentPose().pose; ROS_WARN("WIDTH ");
	target.position = srvPositionPose.response.position;
	euler2matrix2quaternion(-srvPositionPose.response.angle+1.8349, target.orientation );
	ROS_WARN("WIDTH ");
	fit(0, target);
	 movetopose(target);	 
}
	void fit(double angle ,geometry_msgs::Pose &mypose)
	{
		angle = angle;
		tf2::Matrix3x3 m (tf2::Quaternion ( tf2::Vector3(0,0,-1), angle));
		tf2::Vector3 v(-1,1,0), vp(mypose.position.x, mypose.position.y, mypose.position.z);
		v = v * m;
		v.normalize();
		ROS_WARN("%f,%f,%f,angle =%f",v.getX(),v.getY(), v.getZ(), angle*180./3.1415926);
		ROS_WARN("before 0.02%f,%f,%f",vp.getX(),vp.getY(), vp.getZ());
		vp = vp + 0.02*v;
		mypose.position.x = vp.getX(); 
		mypose.position.y=vp.getY(); 
		mypose.position.z=vp.getZ();

		ROS_WARN("after %f,%f,%f",vp.getX(),vp.getY(), vp.getZ());
	}
  void robo_arm(int number  = 10)
  {
	ROS_INFO("************* TO PULL BACK**************88");
	ros::Time start_time=ros::Time::now();
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	float fraction = 0.0;
	std::vector<geometry_msgs::Pose> waypoints;
	 geometry_msgs::Pose mypose;
	kinematic_state = group_ptr->getCurrentState();




	if (thingposeflag)
	{
		mypose = thingpose;
		mypose.position.z +=0.1;
		waypoints.push_back(mypose);
	}
	
	mypose.orientation.x=0.0401333;
	mypose.orientation.y=0.721599;
	mypose.orientation.z=-0.00288368;
	mypose.orientation.w=0.691141;
	mypose.position.x=0.0645235;
	mypose.position.y=0.707698;
	mypose.position.z=-0.170141;

	mypose.position.z = 0.25;
	waypoints.push_back(mypose);
	mypose.position.z = -0.160141;
	waypoints.push_back(mypose);

	bool compute = false;
	while(fraction<0.99)
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
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("cur backtohome pose  [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

	ros::Duration dur2=ros::Time::now() - start_time;
	ROS_INFO("The  backtohome plan move time is:%f",dur2.toSec());

	//sleep(1);
	ROS_INFO("angle release!");
	ros::Time gripperstart=ros::Time::now();

	gripper_release(30,30);
	dur2=ros::Time::now() - gripperstart;
	ROS_INFO("The  backtohome plan+gripper move time is:%f",dur2.toSec());

	sleep(1);
/*
	waypoints.clear();
	homepose.position.z = FINGERLEN;
	exec_path_points(waypoints);

	gripper_release(1,1);*/
	//the next service related code written by zhang yumei 20160120, 
	//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object

	gripperstart=ros::Time::now();
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(0.5);
	ROS_INFO("no object ! waiting the target object position data");
	}
	dur2=ros::Time::now() - gripperstart;
	ROS_INFO("The  srvtime plan+gripper move time is:%f",dur2.toSec());

	gwidth =  srvPositionPose.response.width;
	angle2 = (int)(-0.01934*gwidth*gwidth+0.19408*gwidth+102.89+8);
	ROS_WARN("WIDTH = %d ",gwidth);
	ROS_WARN("angle = %d ",angle2);
	 //target = group_ptr->getCurrentPose().pose;

	target.orientation.x=0.70207;
	target.orientation.y=0.0600884;
	target.orientation.z=-0.708811;
	target.orientation.w=0.0327785;

	target.position = srvPositionPose.response.position;
	euler2matrix2quaternion(-srvPositionPose.response.angle+1.8349, target.orientation );
		fit(-srvPositionPose.response.angle, target);
	movetopose(target);
	ROS_INFO("finish robot_arm");
  }

	void movetopose( geometry_msgs::Pose& pose)
	{
		pose.position.z = 0.15;//0.112;
		gripper_release(10,85);
		ROS_INFO("************* TO pick objs**************88");
		if(gplacecnt>9) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
  		ros::Time start_time=ros::Time::now();
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,FINGERLEN2);
		thingpose = pose;	
		move_group_interface::MoveGroup::Plan my_plan;
		moveit_msgs::RobotTrajectory trajectory;
		std::vector<geometry_msgs::Pose> waypoints;
		homepose.position.z = FINGERLEN;
		waypoints.push_back(homepose);
		thingposeflag = true;
		midflag = false;
		
		
		thingpose.position.z +=0.1;
		waypoints.push_back(thingpose);
		thingpose.position.z -=0.1;
		waypoints.push_back(thingpose);
		target = thingpose;
		
		std_msgs::Time squeeze_time;
		squeeze_time.data=ros::Time::now();
		kinematic_state = group_ptr->getCurrentState();
		
		ROS_INFO("START TO COMPUTING PATH");
		float fraction=0;
		while(fraction<0.99)
		 fraction = group_ptr->computeCartesianPath(waypoints,
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
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

		ros::Duration dur2=ros::Time::now() - start_time;
  		ROS_INFO("The  movetopose plan move time is:%f",dur2.toSec());

		sleep(1);
		ros::Time gripperstart=ros::Time::now();
		gripper_release(10,145);
		dur2=ros::Time::now() - gripperstart;
		ROS_INFO("The  movetopose plan+gripper move time is:%f",dur2.toSec());
		sleep(1);
		//gripper_release(20,150);
		//sleep(1);
		robo_arm(number);

	}
void exec_path_points(std::vector<geometry_msgs::Pose> waypoints)
{
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	std_msgs::Time squeeze_time;
	squeeze_time.data=ros::Time::now();
	kinematic_state = group_ptr->getCurrentState();
	while(ros::Time::now() - squeeze_time.data < ros::Duration(1))
	{
		tar_state_pub.publish(target);
	}
	ROS_INFO("START TO COMPUTING PATH");

	float fraction =0;
	while(fraction<0.99)
	fraction= group_ptr->computeCartesianPath(waypoints,
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
}
	void gripper_release(int x,int y)
	{
	    
		ros::Time start_time=ros::Time::now();

		ros::Rate loop_rate(100);

		int count = 0;
		while (ros::Time::now()-start_time<ros::Duration(0.1))
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

/*near I heard: [-0.609205,0.243008,0.699562,0.283601,0.358745,0.521433,-0.139971]]
mid I heard: [0.645851,0.100631,-0.738165,0.166920,-0.362077,0.490982,-0.166557]]
far[-I heard: [0.538673,0.370231,-0.593198,0.469975,-0.609540,0.024454,-0.166550]]

axis:
point1;[0.058810,0.717137,-0.134719,0.681254,-0.129474,-0.787660,-0.182585]]
point2； [0.452482,0.559384,-0.485789,0.496346,-0.690555,-0.172250,-0.174608]]
vector = -0.673735,0.738973,0

start:I heard: [-0.525935,0.482451,0.548879,0.435162,0.653862,0.193842,-0.259047]]
about end:I heard: [-0.692085,0.174264,0.689792,0.121809,0.195921,0.653293,-0.259034]]


far I heard: [0.632103,0.233803,-0.696879,0.245238,-0.678733,0.040766,0.007015]]
near:-I heard: [0.631420,0.235132,-0.696608,0.246493,0.532600,0.147355,0.024777]]

*/
  ros::init(argc, argv, "desktop_grab");
  ros::NodeHandle n;
  GRIPCV grip;
  ros::spin();
  return 0;
}
