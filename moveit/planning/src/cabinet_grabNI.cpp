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
#include <object_detect/cabinetPose.h>
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
#define PORT 6664
#define BUFLEN 1024
std::mutex mtx;
std::mutex datarecvmtx;

bool datarecv = false;
float data[3] = {0,0,0};
const float PI=3.1415926;

//moveit::planning_interface::MoveGroup *group_ptr;// = new moveit::planning_interface::MoveGroup("manipulator");

ros::Publisher vspub;
ros::Publisher vs_start_pub;
const double DIST_TH = 0.005;
//double MAX_SPEED = 0.1;
//const int RATION = 2;
//double MIN_SPEED = DIST_TH;

geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
const float KINECT_DH = 0.3;
const float FINGER_DH = 0.2;
float error = 0.05;   //zliu7
//int angel1,angle2; //define two varies of the angle 
int angel1,angle2; //define two varies of the angle
int number;


const double DIST_THQ = 0.005;
//double MAX_SPEED = 0.1;
const int RATION = 2;
double MIN_SPEED = DIST_TH*5;
double MIN_SPEEDQ = DIST_THQ*5;

double MAX_SPEED = DIST_TH*20;
double MAX_SPEEDQ = DIST_THQ*20;

void visualServoingThread(){
		//geometry_msgs::Twist data;
		struct sockaddr_in si_me, si_other;

		int s, i, recv_len;
		socklen_t slen = (socklen_t)sizeof(si_other); 
		float buf[BUFLEN/sizeof(float)];

		if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		{
			perror("socket");
			return;
		}

		memset((char *) &si_me, 0, sizeof(si_me));

		si_me.sin_family = AF_INET;
		si_me.sin_port = htons(PORT);
		si_me.sin_addr.s_addr = htonl(INADDR_ANY);

		int opt = 1;  
		setsockopt( s, SOL_SOCKET,SO_REUSEADDR, (const void *)&opt, sizeof(opt) );

		if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
		{
			perror("bind");
			return;
		}

		printf("waiting for visual servoing data...\n");

		while(1)
		{
			if ((recv_len = recvfrom(s, (char*)buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
			{
			    perror("recvfrom()");
			    return;
			}
		 
			//printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
			//printf("Data: %4.2f,%4.2f,%4.2f\n" , buf[0],buf[1],buf[2]);
			mtx.lock();
			data[0] = buf[0];
			data[1] = buf[1];
			data[2] = buf[2];
			mtx.unlock();

			datarecvmtx.lock();
			datarecv = true;
			datarecvmtx.unlock();
		}
		close(s);
	}

class GRIPCV
{
  ros::NodeHandle nh_;
  ros::Publisher tar_state_pub;
  ros::Publisher display_publisher;
  ros::Publisher planning_scene_diff_publisher;
  ros::Subscriber objectsub;
  //ros::Subscriber  servo_angle; //
  double homez;
  ros::Publisher hand_pub;
  ros::Publisher hand_pub1;
  ros::ServiceClient client;
  object_detect::Check srv;
//the next two line is add by zhang yu mei 20160120, for service request , thus can solve the problem of data delay. 
  ros::ServiceClient pos_pose_client;
  object_detect::cabinetPose srvPositionPose;
  geometry_msgs::Pose thingpose, homepose,midp;
  geometry_msgs::Quaternion thingori, homeori,midori;
  bool thingposeflag;
 // homepose
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
ros::AsyncSpinner spinner(1);
	spinner.start();
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::cabinetPose>("/srvcabinetPose");
 thingposeflag=false;
 // homepose
   xr=0.684106;
   yr=0.729383;
   homez = -0.11;
   gplacecnt=0;
//[0.482464,-0.072558,-0.849932,0.198945,-0.427961,-0.590100,-0.012965]]
// heard: [0.355676,0.186971,-0.493461,0.771384,0.871048,-0.266650,-0.010617]]
//0.399889,0.346052,-0.475994,0.702685
   thingori.x = 0.399889;
   thingori.y = 0.346052;
   thingori.z = -0.475994;
   thingori.w = 0.702685;

//0.138503,0.510374,-0.002571,0.848722
   midori.x= 0.138503;
   midori.y=0.510374;
   midori.z=-0.002571;
   midori.w=0.848722;

//I heard: [0.334764,0.408009,-0.352763,0.772671,0.463108,-0.072104,-0.114081]]

	midp.position.x =0.621;
	midp.position.y=0;
	midp.position.z=-0.114081;
	midp.orientation.x =0.463108;
	midp.orientation.y =-0.072104;
	midp.orientation.z =-0.352763;
	midp.orientation.w =0.772671;
//home  [-0.479717,0.584006,0.342566,0.558083,0.616583,-0.052754,-0.208787]]


//-I heard: [-0.354083,0.693537,0.156041,0.607686,0.158831,0.740143,-0.165744]]

	homepose.orientation.x=-0.354083;
	homepose.orientation.y=0.693537;
	homepose.orientation.z=0.156041;
	homepose.orientation.w=0.607686;
   homepose.position.x = 0;
   homepose.position.y = 0.634084;
   homepose.position.z = -0.325310;
//[0.693939,0.161964,-0.689242,0.131002,-0.6,0.2,-0.0]]
   

    hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000);
    hand_pub1 = nh_.advertise<std_msgs::UInt16>("/servo4", 1000);
    vspub = nh_.advertise<geometry_msgs::Twist> ("/do_vs_vel", 1);
    vs_start_pub = nh_.advertise<std_msgs::Bool>("/do_vs_info", 1, true);
    tar_state_pub = nh_.advertise<geometry_msgs::Pose>("/target_state", 1, true);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    objectsub = nh_.subscribe ("/cab_pose_pubni", 1, &GRIPCV::movetopose, this);
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


//[132][-I heard: [-0.255618,-0.883609,0.377165,0.107894,0.227001,0.535995,0.000598]]
geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("I heard  cur: [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);


	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! return old place!");
	robo_arm();
	}
	// gwidth =  srvPositionPose.response.width;
	// angle2 = (int)(-0.01934*gwidth*gwidth+0.19408*gwidth+102.89+8);
	// ROS_WARN("WIDTH = %d,angle2=%d ",gwidth,angle2);
	 movetopose(srvPositionPose.response.pose);
	 
}

 void robo_arm(int number  = 10)
  {
	ROS_INFO("*************232START TO PULL BACK**************88");
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	float fraction = 0.0;
	std::vector<geometry_msgs::Pose> waypoints;
	kinematic_state = group_ptr->getCurrentState();
	if (thingposeflag)
	{
		thingpose.position.z+=0.02;
		homepose.position.z = thingpose.position.z;
		midp.position.z = thingpose.position.z;
		waypoints.push_back(thingpose);
	//thingposeflag=false;
	}
	/*		
        geometry_msgs::Pose midp;

	midp.position.x =-0.230233;
	midp.position.y=0.607181;
	midp.position.z=-0;
	midp.orientation = homepose.orientation;*/
	waypoints.push_back(midp);


	//homepose.position.z += 0.02;
	waypoints.push_back(homepose);
	homepose.position.z -= homez;
	waypoints.push_back(homepose);

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


  }

void movetopose( const geometry_msgs::Pose& pose)
	{
		//if(homepose.position.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);
		thingpose = pose;

		thingpose.orientation = thingori;
		//thingpose.position.x += 0.20*xr;
 		//thingpose.position.y += 0.20*yr;
		thingpose.position.z +=0.04;
		homepose.position.z = thingpose.position.z;
		midp.position.z = thingpose.position.z;
		//thingpose.position.z += KINECT_DH;	
		move_group_interface::MoveGroup::Plan my_plan;
		moveit_msgs::RobotTrajectory trajectory;
		std::vector<geometry_msgs::Pose> waypoints;

		waypoints.push_back(homepose);
		thingposeflag = true;

		
		waypoints.push_back(midp);
		
		//thingpose.position.z+=0.04;
		waypoints.push_back(thingpose);
		//thingpose.position.z-=0.04;
		//waypoints.push_back(thingpose);
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

		gripper_release(10,49);


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
  std::thread t(visualServoingThread);
  t.detach();
  ros::init(argc, argv, "cabinetvs2machines");
  ros::NodeHandle n;

  GRIPCV grip;
  ros::spin();
  return 0;
}
