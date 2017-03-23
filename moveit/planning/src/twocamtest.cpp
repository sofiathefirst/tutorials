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
#include <std_msgs/Time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
moveit::planning_interface::MoveGroup *group;
geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
const float FINGERLEN = 0.36;
const float FINGERLEN2 = 0.18;
const tf2::Quaternion midq(0.5,0.5,0.5,-0.5);//x,y,z,w
const tf2::Matrix3x3 MID_MATRIX(midq);
geometry_msgs::Quaternion gq;

static const double PI = 3.1415926;
float error = 0.05;   //zliu7
//int angel1,angle2; //define two varies of the angle 
int angel1,angle2; //define two varies of the angle
int number;
float fx,fy,fz;
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
euler2matrix2quaternion(0,0,0);	
ros::AsyncSpinner spinner(1);
	spinner.start();
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::poseDepth>("/srvposeDepth");
 thingposeflag=false;
 // boxpose-0.673735,0.738973,0
   xr=-0.673735;
   yr=0.738973;
   gplacecnt=0;
//-0.105177,0.699311,0.105112,0.699180
  /* boxpose.orientation.x = -0.543554;
   boxpose.orientation.y = 0.452293;
   boxpose.orientation.z = 0.543578;
   boxpose.orientation.w = 0.452221;
*///0.392424,0.377517,0.131115
   boxpose.orientation.x = -0.105177;
   boxpose.orientation.y = 0.699311;
   boxpose.orientation.z = 0.105112;
   boxpose.orientation.w = 0.699180;
   boxpose.position.x = 0.392424;
   boxpose.position.y = 0.377517;
   boxpose.position.z = 0 + FINGERLEN;

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

	
	robo_arm();

}

int euler2matrix2quaternion(double x,double y,double z)
{
	tf2::Matrix3x3 eulerMatrix;
	x = -x;
	y = -y;
	eulerMatrix.setRPY(x,y,z);
		
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
void movetopose(const geometry_msgs::Pose& pose)
	{
		//gripper_release(1,1);
		ROS_INFO("************* TO pick objs**************88");
		if(gplacecnt>9) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
  		ros::Time start_time=ros::Time::now();
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,FINGERLEN2);
		thingpose = pose;	
		move_group_interface::MoveGroup::Plan my_plan;
		moveit_msgs::RobotTrajectory trajectory;
		std::vector<geometry_msgs::Pose> waypoints;
		boxpose.position.z = FINGERLEN;
		waypoints.push_back(boxpose);
		thingposeflag = true;

		geometry_msgs::Pose midp;
		
		thingpose.position.z =FINGERLEN;
		waypoints.push_back(thingpose);
		thingpose.position.z =FINGERLEN2;
		waypoints.push_back(thingpose);
		target = thingpose;
		
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
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

		ros::Duration dur2=ros::Time::now() - start_time;
  		ROS_INFO("The  movetopose plan move time is:%f",dur2.toSec());

		//sleep(1);
		ros::Time gripperstart=ros::Time::now();
		gripper_release(40,120);
		dur2=ros::Time::now() - gripperstart;
		ROS_INFO("The  movetopose plan+gripper move time is:%f",dur2.toSec());


	}
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
}
void seeMatrix(tf2::Matrix3x3 m)
{
	seeV3(m.getRow(0));
	seeV3(m.getRow(1));
	seeV3(m.getRow(2));
}
  void robo_arm(int number  = 10)
  {
	ROS_INFO("************* TO PULL BACK**************88");
	ros::Time start_time=ros::Time::now();
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	float fraction = 0.0;
	std::vector<geometry_msgs::Pose> waypoints;
	 
	kinematic_state = group_ptr->getCurrentState();

geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("cur  pose  [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

	tf2::Vector3 positioncal(fx,fy,fz);//rewrite this position
	seeV3(positioncal);
	curpose.position.x = positioncal.getX();
	curpose.position.y = positioncal.getY();
	curpose.position.z = positioncal.getZ();

ROS_INFO("cur  pose  [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

	/*curpose.orientation.x = -0.5;
	curpose.orientation.y = -0.5;
curpose.orientation.z = -0.5;
curpose.orientation.w = 0.5;
*/
tf2::Matrix3x3 m(tf2::Quaternion(0.5,0.5,0.5,-0.5)),m1(tf2::Quaternion(-0.5,-0.5,-0.5,0.5));
seeMatrix(m);
seeMatrix(m1);
	waypoints.push_back(curpose);
	
	
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
	 curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("cur  pose  [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);
ros::shutdown();
	
	ros::Duration dur2=ros::Time::now() - start_time;
	ROS_INFO("The  backtohome plan move time is:%f",dur2.toSec());

	//sleep(1);
	ROS_INFO("angle release!");
	ros::Time gripperstart=ros::Time::now();
	//gripper_release(25,50);
	gripper_release(1,1);
	dur2=ros::Time::now() - gripperstart;
	ROS_INFO("The  backtohome plan+gripper move time is:%f",dur2.toSec());

	
  }

	

	void gripper_release(int x,int y)
	{
	    
		ros::Time start_time=ros::Time::now();

		ros::Rate loop_rate(100);

		int count = 0;
		while (ros::Time::now()-start_time<ros::Duration(0.5))
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
  ros::init(argc, argv, "grip_rect");
  ros::NodeHandle n;
float nums[3];
char fnum[30];
int j = 0,numsi = 0;
ROS_ERROR("%s",argv[1]);
for(int i =0 ;argv[1][i]!=0;i++)
{
	fnum[j++]=argv[1][i];
	if(argv[1][i]==',' )
	{
	fnum[j++]=0;j=0;nums[numsi++]=atof(fnum);
	ROS_INFO("%d",numsi);}
}
fx = nums[0]/1000;
fy = nums[1]/1000;
fz = nums[2]/1000;

  GRIPCV grip;
  ros::spin();
  return 0;
}
