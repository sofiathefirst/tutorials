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
#include <object_detect/cam2position.h>
const float PI=3.1415926;
moveit::planning_interface::MoveGroup *group;// = new moveit::planning_interface::MoveGroup("manipulator");
ros::Publisher vspub;
ros::Publisher vs_start_pub;
const double DIST_TH = 0.005;
//double MAX_SPEED = 0.1;
const int RATION = 2;
double MIN_SPEED = DIST_TH;
int sign (double x)
{
	if (x>0) return 1;
	return -1;
}
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
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
	tf2::Quaternion qa(0.461825,-0.552688,0.439368,0.536852);//-0.706731,0.000540,-0.707481,0.001452),;
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
	cam_ros3d = curp +( cam3d.getZ()-0.02)*oz;
	ROS_WARN("after cam 3d to ros 3d");
	seeV3(cam_ros3d);

	cam_ros3d = cam_ros3d +( cam3d.getX()-0.015)*ox;

	cam_ros3d = cam_ros3d +( cam3d.getY())*oy;
	seeV3(cam_ros3d);
	ROS_WARN("after cam 3d to ros 3d,seeV3(cam_ros3d + end);\n");
}


void getfitspeed(geometry_msgs::Vector3 &vsdata,int ration, int min_speed)
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
void vsMove( geometry_msgs::Pose targetPose,double dist_th,int ration, int min_speed)
{
	bool arrived=false;
	int cnt  = 0 ;
	geometry_msgs::Twist vsdata;
	geometry_msgs::Pose curPose;

	curPose = group->getCurrentPose().pose;
	//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
	while(!arrived)
	{
		curPose = group->getCurrentPose().pose;
		//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
		//geometry_msgs::Twist vsdata;
		
		tf2::Vector3 dist3d(targetPose.position.x - curPose.position.x, targetPose.position.y - curPose.position.y,targetPose.position.z - curPose.position.z)	;
		if (dist3d.length() < dist_th )	
		{
		arrived = true; 
		ROS_INFO("vs POSE : %f , %f,%f,%f , %f,%f",vsdata.angular.x,vsdata.angular.y,vsdata.angular.z,vsdata.linear.x,vsdata.linear.y,vsdata.linear.z); 
		
		 break;
		}	
				//ROS_WARN("dist 3d POSE : %f , %f,%f",dist3d.getX(),dist3d.getY(),dist3d.getZ());		
		//dist3d.normalize();
		
		vsdata.linear.x =  -dist3d.getX();
		vsdata.linear.y =  -dist3d.getY();
		vsdata.linear.z =  dist3d.getZ();
		getfitspeed(vsdata.linear,ration,min_speed);
		//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
		//ROS_ERROR("vs POSE : %f , %f,%f,%f , %f,%f",vsdata.angular.x,vsdata.angular.y,vsdata.angular.z,vsdata.linear.x,vsdata.linear.y,vsdata.linear.z); 
		
//time.data=ros::Time::now();

 // while(ros::Time::now() - time.data > ros::Duration(0.05))
  {
 vspub.publish(vsdata);

    
  }
		usleep(100000);
		cnt++;
		//if (cnt >4000) break;
		
	}
		curPose = group->getCurrentPose().pose;
		ROS_INFO("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
		ROS_INFO("target POSE : %f , %f,%f",targetPose.position.x,targetPose.position.y,targetPose.position.z);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorialpub");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  vspub = nh.advertise<geometry_msgs::Twist> ("/do_vs_vel", 1);
  vs_start_pub = nh.advertise<std_msgs::Bool>("/do_vs_info", 1,true);
  group = new moveit::planning_interface::MoveGroup("manipulator");

group->allowReplanning(true);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
   ros::ServiceClient   pos_pose_client = nh.serviceClient<object_detect::cam2position>("/srvtwocamPosition");
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  move_group_interface::MoveGroup::Plan my_plan;
//  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::core::RobotStatePtr kinematic_state;

  kinematic_state = moveit::core::RobotStatePtr(group->getCurrentState());
  kinematic_state->setToDefaultValues(); 
  
    bool success;
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
  geometry_msgs::Pose start_pose;


  robot_state::RobotState start_state(*group->getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group->getName());
  start_state.setFromIK(joint_model_group, start_pose);
  group->setStartState(start_state);
//  start_pose = group->getCurrentPose().pose;
 

//  group->setStartStateToCurrentState();
  
  ROS_INFO("1");
  geometry_msgs::Pose target_pose1 = group->getCurrentPose().pose;

  
  ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,
target_pose1.orientation.w,target_pose1.position.x,target_pose1.position.y,target_pose1.position.z);





 // ur_modern_driver::VisualServo vsdata;
  ros::Rate loop_rate(10);




  geometry_msgs::Pose targetPose;
//120113,0.431763,0.146073, 0.120113 , 0.213515,0.467090
/*
  targetPose.position.x = 0.333981;
  targetPose.position.y = 0.213515;
  targetPose.position.z = 0.467090;



  targetPose.position.x = 0.120113;
  targetPose.position.y = 0.431763;
  targetPose.position.z = 0.146073;

 // ur_modern_driver::VisualServo vszdata;
//-0.453431,-0.571659,-0.021811

 targetPose.position.x = -0.453431;
  targetPose.position.y = -0.571659;
  targetPose.position.z = -0.021811;
*/

//0.453812,-0.531110,0.290565


 	
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
 // int count = 0;
object_detect::cam2position srvPositionPose;
  while (ros::ok())
  {

	srvPositionPose.request.flag2=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! return old place!");
	
	}
	    
	tf2::Vector3 cam3d(srvPositionPose.response.position2.linear.x,srvPositionPose.response.position2.linear.y,srvPositionPose.response.position2.linear.z);
	tf2::Vector3 oricam3d = cam3d,ration= 0.05*cam3d.normalized();	
	ROS_ERROR("TWO CAM3D:");	
	seeV3(cam3d);
	cam3d=oricam3d - ration;

	target_pose1 = group->getCurrentPose().pose;

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

	vsMove( targetPose,DIST_TH*50,8, MIN_SPEED *50);
	vsMove( targetPose,DIST_TH*10,8, MIN_SPEED *30);
	vsMove( targetPose,DIST_TH,4, MIN_SPEED *10);
	vsMove( targetPose,DIST_TH,2, MIN_SPEED *10);
	std_msgs::Bool vs_stop;
	vs_stop.data = false;
	vs_start_pub.publish(vs_stop);
	ROS_ERROR("vs POSE : moved to target"); 	
	ros::spinOnce();

	loop_rate.sleep();
   // ++count;
  }
  //vsMove( targetPose);
  return 0;
}
