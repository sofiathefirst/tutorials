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
#include <object_detect/cam2position.h>
//////headers for gripper///////
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>
#include <stdio.h>
#include <sstream>
#include <object_detect/poseDepth.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unistd.h>
#include <object_detect/cam2position.h>
#include <std_msgs/Time.h>
#include<thread>
#include<mutex>
#include<string.h> 
#include<stdlib.h> 
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#define PORT 6666
#define BUFLEN 1024
std::mutex mtx;
std::mutex datarecvmtx;
bool datarecv = false;
float data[3] = {0,0,0};
const float PI=3.1415926;

moveit::planning_interface::MoveGroup *group;// = new moveit::planning_interface::MoveGroup("manipulator");
ros::Publisher vspub;
ros::Publisher vs_start_pub;
const double DIST_TH = 0.005;
//double MAX_SPEED = 0.1;
const int RATION = 2;
double MIN_SPEED = DIST_TH;

geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
const float KINECT_DH = 0.3;
const float FINGER_DH = 0.2;
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

  ros::ServiceClient pose2cam_client;
  object_detect::cam2position srv2camPose;


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
  

  int sign (double x)
{
	if (x>0) return 1;
	return -1;
}
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	//ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
}
void cam_Zaxis2(tf2::Vector3 pcur,tf2::Quaternion qx,tf2::Vector3 &oz,tf2::Vector3 &ox,tf2::Vector3 &oy)//qx is the current q, then get the end current vector 
{
/*
1.测一组，ox1,oy1,oz1,并得到q1
2.求得oz,得到q2.
1->2 得qmid=q2*q1.inverse();
ox = mmid * ox1 
oy = mmid * oy1
oz = mmid * oz1
*/
	tf2::Vector3 oa(0,0,1);
	//0.664140,0.242730,-0.664140,0.242730 
	tf2::Quaternion qa(-0.685224,-0.137105,-0.696621,0.162449);//0.461825,-0.552688,0.439368,0.536852);//-0.706731,0.000540,-0.707481,0.001452),;
	tf2::Quaternion qmid= qx*qa.inverse();
	tf2::Matrix3x3 mmid(qmid);	
	oz= mmid*oa;
	//oz = ox;
	//seeV3(oz);

	oa.setValue(-1,0,0);
	oa  = tf2::Matrix3x3( tf2::Quaternion(tf2::Vector3(0,0,1),10*PI/180)) * oa;

	ox= mmid*oa;
	//oz = ox;
	//seeV3(ox);

	oa.setValue(0,-1,0);
	oa  = tf2::Matrix3x3(tf2::Quaternion(tf2::Vector3(0,0,1),10*PI/180)) * oa;
	oy= mmid*oa;
	//oz = oy;
	//seeV3(oy);
	
	//ox = -ox;
	//oy = -oy;
}

void cam3D2rosbyz( tf2::Quaternion qcur , tf2::Vector3 curp, tf2::Vector3 cam3d, tf2::Vector3 &cam_ros3d)
{	tf2::Vector3 oz,ox,oy;
	//cam_Zaxis(qcur, oz);
	cam_Zaxis2(curp,qcur, oz,ox,oy);
	cam3d =cam3d/1000;
	oz.normalize();
	ox.normalize();
	oy.normalize();
	cam_ros3d = curp +( cam3d.getZ())*oz;
	//ROS_WARN("after cam 3d to ros 3d");
	seeV3(cam_ros3d);

	cam_ros3d = cam_ros3d +( cam3d.getX())*ox;

	cam_ros3d = cam_ros3d +( cam3d.getY())*oy;
	seeV3(cam_ros3d);
	//ROS_WARN("after cam 3d to ros 3d,seeV3(cam_ros3d + end);\n");
}


void getfitspeed(geometry_msgs::Vector3 &vsdata,int ration, double min_speed)
{
	double vs3[]={vsdata.x,vsdata.y,vsdata.z};
	for( int i =0 ; i < 3; i++)
		{vs3[i]=vs3[i]/ration; if( vs3[i] < 0) vs3[i] = -vs3[i];}
	int ms = vs3[0]>vs3[1]? 0:1;
	ms = vs3[ms]> vs3[2]? ms:2;
	
	if (vs3[ms] < min_speed && vs3[ms]>0)	
	{
		double r = min_speed / vs3[ms] ;
		for( int i =0 ; i < 3; i++)
		{vs3[i]=vs3[i]*r;}
		
	}
	vsdata.x = vs3[0] * sign(vsdata.x);
	vsdata.y = vs3[1] * sign(vsdata.y);
	vsdata.z = vs3[2] * sign(vsdata.z);		
	
}
void vsMove( geometry_msgs::Pose targetPose,double dist_th,int ration, double min_speed)
{
	bool arrived=false;
	int cnt  = 0 ;
	geometry_msgs::Twist vsdata;
	geometry_msgs::Pose curPose;

	//curPose = group->getCurrentPose().pose;
	//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	while(!arrived)
	{
		curPose = group->getCurrentPose().pose;
	
		tf2::Vector3 dist3d(targetPose.position.x - curPose.position.x, targetPose.position.y - curPose.position.y,targetPose.position.z - curPose.position.z)	;
		if (dist3d.length() < dist_th )	
		{
			arrived = true; 
			//ROS_INFO("vs POSE : %f , %f,%f,%f , %f,%f",vsdata.angular.x,vsdata.angular.y,vsdata.angular.z,vsdata.linear.x,vsdata.linear.y,vsdata.linear.z); 	
			 break;
		}	
		
		vsdata.linear.x =  -dist3d.getX();
		vsdata.linear.y =  -dist3d.getY();
		vsdata.linear.z =  dist3d.getZ();
		getfitspeed(vsdata.linear,ration,min_speed);

 		vspub.publish(vsdata);
		usleep(100000);
		
	}
	curPose = group->getCurrentPose().pose;
	//ROS_INFO("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	//ROS_INFO("target POSE : %f , %f,%f",targetPose.position.x,targetPose.position.y,targetPose.position.z);
}



  GRIPCV():
	nh_("~")
  {
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::poseDepth>("/srvPoseDepth");
pose2cam_client = nh_.serviceClient<object_detect::poseDepth>("/srvtwocamPosition");

 thingposeflag=false;
 // boxpose
   xr=-0.673735;
   yr=0.738973;
   gplacecnt=0;

   boxpose.orientation.x = 0.693951;
   boxpose.orientation.y = 0.161983;
   boxpose.orientation.z = -0.689222;
   boxpose.orientation.w = 0.131024;
   boxpose.position.x = -0.6;
   boxpose.position.y = 0.2;
   boxpose.position.z = 0;
//[0.693939,0.161964,-0.689242,0.131002,-0.6,0.2,-0.0]]
   

    hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000);
    hand_pub1 = nh_.advertise<std_msgs::UInt16>("/servo4", 1000);
    vspub = nh_.advertise<geometry_msgs::Twist> ("/do_vs_vel", 1);
    vs_start_pub = nh_.advertise<std_msgs::Bool>("/do_vs_info", 1, true);
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
			
        geometry_msgs::Pose midp;

	midp.position.x =-0.230233;
	midp.position.y=0.607181;
	midp.position.z=-0;
	midp.orientation = boxpose.orientation;
	waypoints.push_back(midp);


	boxpose.position.x = -0.6+0.08*xr*gplacecnt;
	boxpose.position.y = 0.2+0.08*yr*gplacecnt;
        gplacecnt++;
	ROS_WARN("gplacecnt,position%d,%f,%f,%f",gplacecnt,boxpose.position.x,boxpose.position.y,boxpose.position.z);
	//if(boxpose.position.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
	boxpose.position.z += 0.05;
	waypoints.push_back(boxpose);
	boxpose.position.z -= 0.05;
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
 void dovs()
{
	
        ROS_INFO("A");
	srv2camPose.request.flag2=true;
	while(!pos_pose_client.call(srv2camPose))
	{
	sleep(0.1);
	ROS_INFO("no object ! ");
	
	}
			
	    
	tf2::Vector3 cam3d(srv2camPose.response.pose2.position.x, srv2camPose.response.pose2.position.y,srv2camPose.response.pose2.position.z);
	seeV3(cam3d);
	tf2::Vector3 oricam3d = cam3d,ration= 80*cam3d.normalized();	
	ROS_ERROR("TWO CAM3D:");	
	seeV3(oricam3d);
	cam3d=oricam3d - ration;
	seeV3(cam3d);

	geometry_msgs::Pose target_pose1 = group->getCurrentPose().pose,targetPose;

	tf2::Quaternion qcur(target_pose1.orientation.x, target_pose1.orientation.y,target_pose1.orientation.z, target_pose1.orientation.w) ;
	tf2::Vector3 curp(target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);
	tf2::Vector3 cam_ros3d;
	cam3D2rosbyz(qcur,curp,cam3d,cam_ros3d);
	
	targetPose.position.x = cam_ros3d.getX();
	targetPose.position.y = cam_ros3d.getY();
	targetPose.position.z = cam_ros3d.getZ();

	 ROS_ERROR("I CALCULATE TARGETT: [%f,%f,%f]", targetPose.position.x,targetPose.position.y,targetPose.position.z);

	std_msgs::Bool vs_start;
	vs_start.data = true;
	vs_start_pub.publish(vs_start);



	vsMove( targetPose,DIST_TH*50,4, MIN_SPEED *50);
	vsMove( targetPose,DIST_TH*10,4, MIN_SPEED *30);
	vsMove( targetPose,DIST_TH,2, MIN_SPEED *10);
	vsMove( targetPose,DIST_TH,1, MIN_SPEED *10);

	std_msgs::Bool vs_stop;
	vs_stop.data = false;
	vs_start_pub.publish(vs_stop);

	ROS_ERROR("vs POSE : moved to target"); 	


}
//-0.680760,-0.725815,0.098787
	void movetopose(const geometry_msgs::Pose& pose)
	{
		if(boxpose.position.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);
		thingpose = pose;
		thingpose.position.z += KINECT_DH;	
		move_group_interface::MoveGroup::Plan my_plan;
		moveit_msgs::RobotTrajectory trajectory;
		std::vector<geometry_msgs::Pose> waypoints;

		waypoints.push_back(boxpose);
		thingposeflag = true;

		geometry_msgs::Pose midp;

		midp.position.x =-0.230233;
		midp.position.y=0.607181;
		midp.position.z=-0;
		midp.orientation = boxpose.orientation;
		waypoints.push_back(midp);
		
		thingpose.position.z+=0.02;
		waypoints.push_back(thingpose);
		thingpose.position.z-=0.02;
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


		dovs();

		robo_arm(number);
		

	}
	
/*

上：
   xr=0.684106;
   yr=0.729383;

right:

0.683017,-0.730403

*/
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
  ros::init(argc, argv, "desktopvs");
  ros::NodeHandle n;
  GRIPCV grip;
  ros::spin();
  return 0;
}
