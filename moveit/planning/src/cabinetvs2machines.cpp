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
#include<fstream>
#include<string.h> 
#include<stdlib.h> 
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#define PORT 6664
#define BUFLEN 1024
std::mutex mtx;
std::mutex datarecvmtx;
using namespace std;
bool datarecv = false;
float data[3] = {0,0,0};
const float PI=3.1415926;
ofstream out("a.txt");  
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
  
int sign (double x)
{
	if (x>0) return 1;
	return -1;
}
void getfitspeed(geometry_msgs::Vector3 &vsdata,int ration , double min_speed , double max_speed)
{
	double vs3[]={fabs(vsdata.x)/ration,fabs(vsdata.y)/ration,fabs(vsdata.z)/ration};
	int ms = vs3[0]>vs3[1]? 0:1;
	ms = vs3[ms]> vs3[2]? ms:2;
	
	if (vs3[ms] < min_speed && vs3[ms]>0)	
	{
		double r = min_speed / vs3[ms] ;
		for( int i =0 ; i < 3; i++)
		{vs3[i]=vs3[i]*r; }
		
	}

	else if (vs3[ms] > max_speed && vs3[ms]>0)	
	{
		double r = max_speed / vs3[ms] ;
		for( int i =0 ; i < 3; i++)
		{vs3[i]=vs3[i]*r; }
		
	}
	vsdata.x = vs3[0] * sign(vsdata.x);
	vsdata.y = vs3[1] * sign(vsdata.y);
	vsdata.z = vs3[2] * sign(vsdata.z);		
	
}
double dist_q(tf2::Quaternion q1, tf2::Quaternion q2)
{
	double d = fabs(q1.x() - q2.x())+fabs(q1.y() - q2.y())+fabs(q1.z() - q2.z())+fabs(q1.w() - q2.w());
	ROS_ERROR("dist_q%f,%f",d, q1.x() - q2.x());	
	return d;
}

//ofstream fout("out.txt");

bool cab3d2ros3d(tf2::Quaternion qcur , tf2::Vector3 Te, tf2::Vector3 Pcam,  tf2::Vector3 &cam_ros3d)
{/*

double _Rce[9] = {0.0509191640529953,	-0.00203772960867718,	0.999332894303202,
0.983977219728517,	-0.0519564219034156,	-0.0500547081566857,
0.0513454490584332,	0.985314729420426,	-0.00101231432095818};


	double  _Tce[3] = { 16.0201107489674,4.36122815746632,-66.3449862536716};


*/
	Pcam=Pcam/1000;
	tf2::Matrix3x3 Rce(0.0509191640529953,	-0.00203772960867718,	0.999332894303202,
0.983977219728517,	-0.0519564219034156,	-0.0500547081566857,
0.0513454490584332,	0.985314729420426,	-0.00101231432095818), Re(qcur);// = cv::Mat(3, 3, CV_64F, _Rce);
Re = Re.transpose();
tf2::Vector3 v= Re.getRow(0);
   if (out.is_open()) {
out<<"rotation after transpose:\n";
out<<v.getX()<<","<<v.getY()<<","<<v.getZ()<<",\n";
 v= Re.getRow(1);
out<<v.getX()<<","<<v.getY()<<","<<v.getZ()<<",\n";
 v= Re.getRow(2);
out<<v.getX()<<","<<v.getY()<<","<<v.getZ()<<",\n";
out<<"position  :\n";
v = Te;
out<<v.getX()<<","<<v.getY()<<","<<v.getZ()<<",\n";}
	tf2::Vector3 Tce(16.0201107489674,4.36122815746632,-66.3449862536716 ),Pce;
	Tce = Tce/1000;
	Pce = Rce*(Pcam - Tce);
	cam_ros3d = Re*Pce + Te;
	//cam_ros3d = cam_ros3d/1000;
		
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
	//0.513671,-0.503896,0.495232,0.486801
	tf2::Quaternion qa(0.513671,-0.503896,0.495232,0.486801);//0.534944,-0.481214,0.517296,0.463328);//0.461825,-0.552688,0.439368,0.536852);//-0.706731,0.000540,-0.707481,0.001452),;
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

	//curPose = group_ptr->getCurrentPose().pose;
	//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	while(!arrived)
	{
		curPose = group_ptr ->getCurrentPose().pose;
	
		//tf2::Vector3 dist3d(targetPose.position.x - curPose.position.x, targetPose.position.y - curPose.position.y,targetPose.position.z - curPose.position.z)	;
		tf2::Vector3 dist3d ( targetPose.position.x  , targetPose.position.y  ,targetPose.position.z);
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
	curPose =  group_ptr->getCurrentPose().pose;
	//ROS_INFO("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	//ROS_INFO("target POSE : %f , %f,%f",targetPose.position.x,targetPose.position.y,targetPose.position.z);
}


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
   gplacecnt=0;
//-0.359528,0.868245,0.114133,0.322281

   thingori.x = -0.359528;
   thingori.y = 0.868245;
   thingori.z = 0.114133;
   thingori.w = 0.322281;

//-0.383983,0.922893,0.004369,0.028405
   midori.x= -0.383983;
   midori.y=0.922893;
   midori.z=0.004369;
   midori.w=0.028405;

//-I heard: [0.086622,0.751004,-0.163522,0.633838,-0.041269,-0.483185,-0.068488]]

	midp.position.x =-0.21;
	midp.position.y=-0.48;
	midp.position.z=-0.068488;
	midp.orientation.x =0.086622;
	midp.orientation.y =0.751004;
	midp.orientation.z =-0.163522;
	midp.orientation.w =0.633838;
//home  [-0.479717,0.584006,0.342566,0.558083,0.616583,-0.052754,-0.208787]]


//-0.986623,0.158253,-0.018142,0.034665,0.501173,-0.157755,-0.056746
	homepose.orientation.x=-0.479717;
	homepose.orientation.y=0.584006;
	homepose.orientation.z=0.342566;
	homepose.orientation.w=0.558083;
   homepose.position.x = 0.633634;
   homepose.position.y = -0.046529;
   homepose.position.z = -0.070403;
//[0.693939,0.161964,-0.689242,0.131002,-0.6,0.2,-0.0]]
   

    hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000);
    hand_pub1 = nh_.advertise<std_msgs::UInt16>("/servo4", 1000);
    vspub = nh_.advertise<geometry_msgs::Twist> ("/do_vs_vel", 1);
    vs_start_pub = nh_.advertise<std_msgs::Bool>("/do_vs_info", 1, true);
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


	//homepose.position.x = -0.6+0.08*xr*gplacecnt;
	//homepose.position.y = 0.2+0.08*yr*gplacecnt;
        //gplacecnt++;
	ROS_WARN("gplacecnt,position%d,%f,%f,%f",gplacecnt,homepose.position.x,homepose.position.y,homepose.position.z);
	//if(homepose.position.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
	homepose.position.z += 0.02;
	waypoints.push_back(homepose);
	homepose.position.z -= 0.02;
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


	//the next service related code written by zhang yumei 20160120, 
	//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(0.1);
	ROS_INFO("no object ! waiting the target object position data");
	robo_arm();
	}

	movetopose(srvPositionPose.response.pose);
	ROS_INFO("finish robot_arm");
  }
void vsMove( geometry_msgs::Pose targetPose,int ration , double dist_th ,double dist_thq, double min_speedp, double max_speedp,double min_speedq, double max_speedq)
{
	bool parrived=false,qarrived=false;
	int cnt  = 0 ;
	geometry_msgs::Twist vsdata;
	geometry_msgs::Pose curPose;

	//curPose = group->getCurrentPose().pose;
	ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	tf2::Quaternion tarq( targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w);
	while(!parrived||!qarrived)
	{
		curPose = group_ptr->getCurrentPose().pose;
		//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
		//geometry_msgs::Twist vsdata;
		
		tf2::Quaternion curq( curPose.orientation.x,curPose.orientation.y,curPose.orientation.z,curPose.orientation.w);
		tf2::Quaternion midq = tarq * curq.inverse();
		tf2::Matrix3x3 eulerMatrix(midq);
		double rx,ry,rz,drx,dry,drz;
		eulerMatrix.getRPY(drx,dry,drz,1);
		//ROS_INFO("DrX,%f,%f,%f",drx,dry,drz);
		vsdata.angular.x = 0; 
		vsdata.angular.y = 0;
		vsdata.angular.z = 0;
		qarrived = true;/*
		if(dist_q(curq,tarq) > dist_thq)
		{
			vsdata.angular.x = -drx; 
			vsdata.angular.y = -dry;
			vsdata.angular.z = drz;
		}
		else
		{
			vsdata.angular.x = 0; 
			vsdata.angular.y = 0;
			vsdata.angular.z = 0;
			qarrived = true;
			ROS_ERROR("qarrived = true;"); 
		
		}*/
		tf2::Vector3 dist3d(targetPose.position.x - curPose.position.x, targetPose.position.y - curPose.position.y,targetPose.position.z - curPose.position.z)	;
		if (dist3d.length() < dist_th )	
		{
			parrived = true; 
			//ROS_ERROR("vs POSE : %f , %f,%f,%f , %f,%f",vsdata.angular.x,vsdata.angular.y,vsdata.angular.z,vsdata.linear.x,vsdata.linear.y,vsdata.linear.z); 
		
		 //break;
		}	
				//ROS_WARN("dist 3d POSE : %f , %f,%f",dist3d.getX(),dist3d.getY(),dist3d.getZ());		
		//dist3d.normalize();
		
		vsdata.linear.x =  -dist3d.getX();
		vsdata.linear.y =  -dist3d.getY();
		vsdata.linear.z =  dist3d.getZ();
		if (!parrived)
		getfitspeed(vsdata.linear,ration, min_speedp, max_speedp);
		if (!qarrived)
		getfitspeed(vsdata.angular,ration , min_speedq, max_speedq);
 		
		vspub.publish(vsdata);
		usleep(200000);
		cnt++;
		//if (cnt >4000) break;
		
	}
	curPose = group_ptr->getCurrentPose().pose;
	
	ROS_WARN("xyz CUR POSE : %f , %f,%f,%f , %f,%f,%f",curPose.orientation.x,curPose.orientation.y,curPose.orientation.z,curPose.orientation.w,curPose.position.x,curPose.position.y,curPose.position.z);
	ROS_WARN("xyz target POSE : %f , %f,%f,%f , %f,%f,%f",targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w,targetPose.position.x,targetPose.position.y,targetPose.position.z);
}

 void dovs()
{
ROS_INFO("1");
	tf2::Vector3 cam3d;
	while( data [2] <0.0001 )
	{	
		 ROS_INFO("1");
		usleep(1000000);
		std_msgs::Bool vs_stop;
		vs_stop.data = false;
		vs_start_pub.publish(vs_stop);
	}
	while(true)
	{
		if(datarecv && data [2] >50 && data[2]<500)
		{
			mtx.lock();
			cam3d.setValue(data[0],data[1],data[2]);
			mtx.unlock(); 

			datarecvmtx.lock();
			datarecv = false;
			datarecvmtx.unlock();
			ROS_ERROR("cam data :X=%f,%f,%f",cam3d.getX(),cam3d.getY(),cam3d.getZ());
			break;
		}
	}	

	//
ROS_INFO("1");
	tf2::Vector3 oricam3d = cam3d,ration= 0*cam3d.normalized();	
	ROS_ERROR("TWO CAM3D:");	
	seeV3(oricam3d);
	cam3d=oricam3d - ration;
	seeV3(cam3d);
ROS_INFO("1");
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

	geometry_msgs::Pose targetPose;ROS_INFO("1");
	tf2::Quaternion qcur(curpose.orientation.x, curpose.orientation.y,curpose.orientation.z, curpose.orientation.w) ;
	tf2::Vector3 curp(curpose.position.x,curpose.position.y,curpose.position.z);
	tf2::Vector3 cam_ros3d;
	cab3d2ros3d(qcur,curp,cam3d,cam_ros3d);
	//cam3D2rosbyz(qcur,curp,cam3d,cam_ros3d);
	ROS_WARN("IN DOVS%f,%f,%f",cam_ros3d.getX(),cam_ros3d.getY(),cam_ros3d.getZ());
	targetPose.position.x = cam_ros3d.getX();
	targetPose.position.y = cam_ros3d.getY();
	targetPose.position.z = cam_ros3d.getZ();
	//targetPose.position.z += 
	 //ROS_ERROR("I CALCULATE TARGETT: [%f,%f,%f]", targetPose.position.x,targetPose.position.y,targetPose.position.z);

	std_msgs::Bool vs_start;
	vs_start.data = true;
	vs_start_pub.publish(vs_start);
	
	//time.data=ros::Time::now();
	//targetPose.position.z+=0.1;
	//vsMove( targetPose,DIST_TH*50,20, MIN_SPEED *50);
	//vsMove( targetPose,DIST_TH*10,4, MIN_SPEED *30);
	//vsMove( targetPose,DIST_TH,2, MIN_SPEED *10);
	//vsMove( targetPose,DIST_TH,1, MIN_SPEED *10);
vsMove( targetPose,10, DIST_TH,DIST_THQ, MIN_SPEED , MAX_SPEED, MIN_SPEEDQ , MAX_SPEEDQ);

	std_msgs::Bool vs_stop;
	vs_stop.data = false;
	vs_start_pub.publish(vs_stop);	
	ROS_ERROR("vs POSE : moved to target"); 

}

 void dovs2()
{
ROS_INFO("1");
	tf2::Vector3 cam3d;
	while( data [2] <0.0001 )
	{	
		
		usleep(1000000);
		std_msgs::Bool vs_stop;
		vs_stop.data = false;
		vs_start_pub.publish(vs_stop);
	}
	while(true)
	{
		if(datarecv && data [2] >50 && data[2]<500)
		{
			mtx.lock();
			cam3d.setValue(data[0],data[1],data[2]);
			mtx.unlock(); 

			datarecvmtx.lock();
			datarecv = false;
			datarecvmtx.unlock();
			ROS_ERROR("cam data :X=%f,%f,%f",cam3d.getX(),cam3d.getY(),cam3d.getZ());
			break;
		}
	}	

	//
ROS_INFO("1");
	tf2::Vector3 oricam3d = cam3d,ration= 0*cam3d.normalized();	
	ROS_ERROR("TWO CAM3D:");	
	seeV3(oricam3d);
	cam3d=oricam3d - ration;
	seeV3(cam3d);
ROS_INFO("1");
	geometry_msgs::Pose curpose =  group_ptr->getCurrentPose().pose;
	ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", curpose.orientation.x,curpose.orientation.y,curpose.orientation.z,curpose.orientation.w,curpose.position.x,curpose.position.y,curpose.position.z);

	geometry_msgs::Pose targetPose;ROS_INFO("1");
	tf2::Quaternion qcur(curpose.orientation.x, curpose.orientation.y,curpose.orientation.z, curpose.orientation.w) ;
	tf2::Vector3 curp(curpose.position.x,curpose.position.y,curpose.position.z);
	tf2::Vector3 cam_ros3d;
	//cab3d2ros3d(qcur,curp,cam3d,cam_ros3d);
	cam3D2rosbyz(qcur,curp,cam3d,cam_ros3d);
	ROS_WARN("IN DOVS%f,%f,%f",cam_ros3d.getX(),cam_ros3d.getY(),cam_ros3d.getZ());
	targetPose.position.x = cam_ros3d.getX();
	targetPose.position.y = cam_ros3d.getY();
	targetPose.position.z = cam_ros3d.getZ();
	
	move_group_interface::MoveGroup::Plan my_plan;
	moveit_msgs::RobotTrajectory trajectory;
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose midp,finalp = targetPose;
	midp.orientation = thingori;
/*
targetPose.orientation.x=0.528453;
targetPose.orientation.x=0.846535;
targetPose.orientation.x=-0.050229;
targetPose.orientation.x=0.039907;*/

	targetPose.orientation = thingori;
	targetPose.position.x +=    0.10* xr;
	targetPose.position.y +=   0.10*yr; 
 
	targetPose.position.z +=0.1;
	waypoints.push_back(targetPose);
	targetPose.position.x -=    0.076* xr;
	targetPose.position.y -=   0.076*yr; 
	
	waypoints.push_back(targetPose);
	targetPose.position.z -=0.045;
	waypoints.push_back(targetPose);


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
		gripper_release(10,49);
		sleep(1);
ros::shutdown();

}
void movetopose(const geometry_msgs::Pose& pose)
	{
		//if(homepose.position.x >-0.17) {ROS_WARN("FINISHED! SHUTDOWN");ros::shutdown();exit(-1);}
		ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);
		thingpose = pose;
		thingpose.orientation = midori;//thingori;
		//thingpose.position.x += 0.20*xr;
 		//thingpose.position.y += 0.20*yr;
		thingpose.position.z +=0.04;
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



		//dovs2();

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
