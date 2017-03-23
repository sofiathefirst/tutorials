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
#include <cmath>
#include <object_detect/cabinetPose.h>
#include <std_msgs/Time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Quaternion.h>
moveit::planning_interface::MoveGroup *group;
geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
const float FINGERLEN = 0.26;
const float FINGERLEN2 = -0.021;
const float homez = -0.064;
const tf2::Quaternion midq(0.5,0.5,0.5,-0.5);//x,y,z,w
const tf2::Matrix3x3 MID_MATRIX(midq);


static const double PI = 3.1415926;
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
  object_detect::cabinetPose srvPositionPose;
  geometry_msgs::Pose pickp, homep,midp;
  bool pickpflag;
 // homep
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
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2::Vector3 :X=%f,%f,%f",-v.getX(),-v.getY(),v.getZ());
}
void calcuraxisq(tf2::Quaternion &q, geometry_msgs::Quaternion &gq)
{
	tf2::Vector3 oz(0,0,1),axisq;
	tf2::Quaternion qoz(0.5,0.5,0.5,-0.5);

	tf2::Quaternion qmid= q*qoz.inverse();
	tf2::Matrix3x3 mmid(qmid);	
	axisq= mmid*oz;
	seeV3(axisq);
	q = tf2::Quaternion(axisq,-PI/2)*q;
	gq.x = q.x();
	gq.y = q.y();
	gq.z = q.z();
	gq.w = q.w();
}

void calcuraxisq(tf2::Vector3 axisq,const tf2::Quaternion curq ,geometry_msgs::Quaternion &gq,double angle=-PI/2)
{
	tf2::Quaternion q = curq;
	q = tf2::Quaternion(axisq,angle)*curq;
	gq.x = q.x();
	gq.y = q.y();
	gq.z = q.z();
	gq.w = q.w();
}


void calcuraxis(const tf2::Quaternion q,tf2::Vector3 &axisq)
{
	tf2::Vector3 oz(0,0,1);
	tf2::Quaternion qoz(0.5,0.5,0.5,-0.5);

	tf2::Quaternion qmid= q*qoz.inverse();
	tf2::Matrix3x3 mmid(qmid);	
	axisq= mmid*oz;
	seeV3(axisq);
}

  GRIPCV():
	nh_("~")
  {
ros::AsyncSpinner spinner(1);
	spinner.start();
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::cabinetPose>("/srvbottlePose");
 pickpflag=true;
	xr=0.673735;
   	yr=0.738973;
	pickp.position.x = -0.507;
	pickp.position.y = -0.326;
	pickp.position.z = 0.115;
	euler2matrix2quaternion(-1.6496,0.4214,-0.7630,pickp.orientation);

	midp.position.x = -0.595;
	midp.position.y = 0.178;
	midp.position.z = 0.129;
	euler2matrix2quaternion(-1.5308,0.8548,-0.6488,midp.orientation);

	homep.position.x = -0.157;
	homep.position.y = 0.493;
	homep.position.z = -0.064;
	euler2matrix2quaternion(-1.6496,0.85421448,-2.3615,midp.orientation);



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
	

	init_ur5();

	ROS_ERROR("SHUTING DOWN.....");

	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(0.5);
	ROS_INFO("no object ! waiting the target object position data");
	}
	
	pick_water2(srvPositionPose.response.pose);
	ROS_INFO("finish robot_arm");


}

bool base_joint_move_degree(const geometry_msgs::Pose targetp)
{
	geometry_msgs::Pose curpose=group_ptr->getCurrentPose().pose;
    ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);
	tf2::Vector3 curxy(curpose.position.x,curpose.position.y,0), tarxy(targetp.position.x,targetp.position.y,0);
	seeV3(curxy);	
	double angle = curxy.angle(tarxy);
	ROS_ERROR("angle:%f",angle);
	tf2::Vector3 crossv = curxy.cross(tarxy);
	seeV3(crossv);
	seeV3(tarxy);
	if(crossv.getZ()<0) angle = -angle; 
	ROS_ERROR("%f",angle*180/PI);
	bool s=false;
	int failcnt = 0;
	while(!s)
	{s=joint_move(0,angle);sleep(1);failcnt++;if(failcnt>5) ros::shutdown();}
	//
	return s;
}

bool wrist2_joint_move_degree(const geometry_msgs::Pose targetp)
{
	geometry_msgs::Pose curpose=group_ptr->getCurrentPose().pose;
    ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);
	tf2::Vector3 curxy(curpose.position.x,curpose.position.y,0), tarxy(targetp.position.x,targetp.position.y,0);
	seeV3(curxy);	
	double angle = curxy.angle(tarxy);
	ROS_ERROR("angle:%f",angle);
	tf2::Vector3 crossv = curxy.cross(tarxy);
	seeV3(crossv);
	seeV3(tarxy);
	if(crossv.getZ()<0) angle = -angle; 
	ROS_ERROR("%f",angle*180/PI);
	bool s=false;
	int failcnt = 0;
	while(!s)
	{s=joint_move(4,angle);sleep(1);failcnt++;if(failcnt>5) ros::shutdown();}
	//
	return s;
}

void pourwater(const geometry_msgs::Pose midp)//midp is the pose start to pour water
{
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose midpose=midp;

	tf2::Quaternion qq(midp.orientation.x,midp.orientation.y,midp.orientation.z,midp.orientation.w);
	tf2::Vector3 axisq;
	calcuraxis(qq,axisq);
	geometry_msgs::Quaternion gq;
	for(int i = 30 ;i <181; i+=30)
	{	ROS_WARN("POURWATER I= %d",i);
		waypoints.clear();
		calcuraxisq(axisq,qq,midpose.orientation,-i*PI/180);
		waypoints.push_back(midpose);
		exec_path_points(waypoints);
		//if(i>70)
		sleep(1);
	}
	waypoints.clear();
	calcuraxisq(axisq,qq,midpose.orientation,-PI/2);
	waypoints.push_back(midpose);
	waypoints.push_back(midp);
	exec_path_points(waypoints);

}

void pourwater_joint()//midp is the pose start to pour water
{

joint_move(5,-45/180.*PI);
sleep(0.5);
joint_move(5,-20/180.*PI);
sleep(0.5);
joint_move(5,-25/180.*PI);
sleep(0.5);
joint_move(5,-25/180.*PI);
sleep(0.5);
joint_move(5,-20/180.*PI);
sleep(0.5);
//joint_move(5,-45/180.*PI);
//sleep(1);
joint_move(5,PI/4);
joint_move(5,PI/2);

}
int euler2matrix2quaternion(double x,double y,double z,geometry_msgs::Quaternion &gq)
{
	tf2::Matrix3x3 eulerMatrix;
	x = -x;
	y = -y;
	eulerMatrix.setRPY(x,y,z);
		
	eulerMatrix = eulerMatrix * MID_MATRIX ;
	tf2::Quaternion tq;

	eulerMatrix.getRotation(tq);
	ROS_WARN("eulQ: x,y,z%f,%f,%f,   X=%f,%f,%f,%f",x,y,z,tq.x(),tq.y(),tq.z(),tq.w());
	gq.x = tq.x();
	gq.y = tq.y();
	gq.z = tq.z();
	gq.w = tq.w();
	return 0;
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
}

bool joint_move(int joint_index, double degree)
{
move_group_interface::MoveGroup::Plan my_plan;
std::vector<double> group_variable_values;
group_ptr->getCurrentState()->copyJointGroupPositions(group_ptr->getCurrentState()->getRobotModel()->getJointModelGroup(group_ptr->getName()), group_variable_values);
ROS_INFO("%f,%f,%f,%f,%f,%f",group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4],group_variable_values[5]);

group_variable_values[joint_index] = group_variable_values[joint_index]+degree;
ROS_INFO(":%f,%d",group_variable_values[joint_index]*180/PI,joint_index);
group_ptr->setJointValueTarget(group_variable_values);
bool success = group_ptr->plan(my_plan);
//group_ptr->move();
success = group_ptr->execute(my_plan);
ROS_INFO("joint move success%d",(int)success);
return success;
}

void pose_goal_move()
{
geometry_msgs::Pose curpose=group_ptr->getCurrentPose().pose;
    ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

curpose.position.x = -0.25;
curpose.position.y = 0.32;
curpose.position.z = 0.2;
group_ptr->setPoseTarget(curpose);
moveit::planning_interface::MoveGroup::Plan my_plan;
bool success = group_ptr->plan(my_plan);
success = group_ptr->execute(my_plan);
}
void pose_goal_move(geometry_msgs::Pose curpose)
{

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(curpose);
	exec_path_points(waypoints);
	 
}
void init_ur5()
{
move_group_interface::MoveGroup::Plan my_plan;
std::vector<double> group_variable_values;
group_ptr->getCurrentState()->copyJointGroupPositions(group_ptr->getCurrentState()->getRobotModel()->getJointModelGroup(group_ptr->getName()), group_variable_values);
ROS_ERROR("%f,%f,%f,%f,%f,%f",group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4],group_variable_values[5]);

group_variable_values[0] = -1.10444433182;     //-62.91*
group_variable_values[1] = -2.35619445;     //-106.96*
group_variable_values[2] = -1.5707963;     //-108.28*
group_variable_values[3] = -2.35619445;     //-149.27*
group_variable_values[4] = -1.5707963;     //-109.67*
group_variable_values[5] = 2.12232022;      //202.80*  modified 121.6*
group_ptr->setJointValueTarget(group_variable_values);
bool success = group_ptr->plan(my_plan);
//group_ptr->move();
success = group_ptr->execute(my_plan);
ROS_INFO("init_ur5() FINISHED");

}

void water2home2()
{
	init_ur5();
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	curpose.position.z = homez;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(curpose);

	//waypoints.push_back(homep);
	exec_path_points(waypoints);

pourwater_joint();
gripper_release(15,15);  //
srvPositionPose.request.flag=true;
while(!pos_pose_client.call(srvPositionPose))
{
sleep(0.5);
ROS_INFO("no object! waiting the target object position data");
}

}
void pick_water2(const geometry_msgs::Pose& pose,bool home=true)
  {
	init_ur5();
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	curpose.position.z = FINGERLEN;

	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(curpose);

	//waypoints.push_back(homep);
	exec_path_points(waypoints);

	ROS_INFO("************* TO pick water2**************88");
	 curpose =  group_ptr->getCurrentPose().pose;
ROS_INFO("cur pickup pose : [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);


	//if(gplacecnt>9) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
	ros::Time start_time=ros::Time::now();
	ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,FINGERLEN2);
	//geometry_msgs::Pose pickpose=pickp;	
	pickp.position = pose.position;	
	pickp.position.x += 0.13*xr;
	pickp.position.y += 0.13*yr;
double radius_cur = curpose.position.x *curpose.position.x+curpose.position.y *curpose.position.y;
double radius_tar = pickp.position.x*pickp.position.x+pickp.position.y*pickp.position.y;
if(radius_cur > radius_tar)
{	
	tf2::Vector3 v(curpose.position.x ,curpose.position.y, 0);
	v = sqrt(radius_tar) * v.normalize();
	curpose.position.x = v.getX();
	curpose.position.y = v.getY();
	curpose.position.z = FINGERLEN;
	pose_goal_move(curpose);
}

	base_joint_move_degree(pickp);
	wrist2_joint_move_degree(pickp);
	waypoints.clear();
        curpose =  group_ptr->getCurrentPose().pose;
        curpose.position.z =FINGERLEN2;

	waypoints.push_back(curpose);
	pickp.orientation=curpose.orientation;
        pickp.position.z =FINGERLEN2;
	waypoints.push_back(pickp);

/*



	waypoints.push_back(pickp);*/
	
	//std::vector<geometry_msgs::Pose> waypoints;


	
	exec_path_points( waypoints);


	ros::Duration dur2=ros::Time::now() - start_time;
	ROS_INFO("The  pick_water2 plan move time is:%f",dur2.toSec());

	//sleep(1);
	ros::Time gripperstart=ros::Time::now();
	gripper_release(40,120);
	dur2=ros::Time::now() - gripperstart;
	ROS_INFO("The  pick_water2 plan+gripper move time is:%f",dur2.toSec());
	//sleep(1);
//	if (home)
	water2home2();
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
  ros::init(argc, argv, "water_bottle");
  ros::NodeHandle n;
  GRIPCV grip;
  ros::spin();
  return 0;
}

