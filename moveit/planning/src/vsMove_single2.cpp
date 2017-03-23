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
#include <tf2/LinearMath/Matrix3x3.h>
#include <unistd.h>
#include <math.h>
const float PI=3.1415926;
moveit::planning_interface::MoveGroup *group;// = new moveit::planning_interface::MoveGroup("manipulator");
ros::Publisher vspub;
ros::Publisher vs_start_pub;
const double DIST_TH = 0.005;
const double DIST_THQ = 0.005;
//double MAX_SPEED = 0.1;
const int RATION = 2;
double MIN_SPEED = DIST_TH*5;
double MIN_SPEEDQ = DIST_THQ*5;

double MAX_SPEED = DIST_TH*20;
double MAX_SPEEDQ = DIST_THQ*20;
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
void calFixAngleQ(const tf2::Vector3 positionThing, const tf2::Vector3 positionRobot, tf2::Quaternion &gq)
{
	ROS_INFO("\n\n*****calFixAngleQ****THING PISTION , ROBOT POSITION*********start******");
	tf2::Vector3 position2;
	tf2::Quaternion q;
	tf2::Vector3 OA(0,0,-1);
	double mangle;
	tf2::Vector3 maxis;

	position2 = positionThing - positionRobot;
	position2.normalize();
	mangle= position2.angle(OA);
	ROS_INFO("TWO VECTOR ANGEL = %f",mangle*180/PI);
	maxis  =  OA.cross(position2);

	
	tf2::Quaternion oaq(-0.500089,0.512233,0.500147,0.487219);//-0.414125,0.563932,0.434589,0.567110);//0.336918,0.635834,-0.320840,0.615844);//(0.664140,0.242730,-0.664140,0.242730);
	oaq.normalize();

	q.setRotation(maxis, mangle);
	q.normalize();
	
	tf2::Matrix3x3 eulerMatrix(q);
	double rx,ry,rz,drx,dry,drz;
	eulerMatrix.getRPY(drx,dry,drz);
	ROS_INFO("DrX,%f,%f,%f",drx,dry,drz);
	
	ROS_INFO("-----calFixAngleQ----THING PISTION , ROBOT POSITION---_____end--");
	return ;

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
		curPose = group->getCurrentPose().pose;
		//ROS_WARN("xyz CUR POSE : %f , %f,%f",curPose.position.x,curPose.position.y,curPose.position.z);
		//geometry_msgs::Twist vsdata;
		
		tf2::Quaternion curq( curPose.orientation.x,curPose.orientation.y,curPose.orientation.z,curPose.orientation.w);
		tf2::Quaternion midq = tarq * curq.inverse();
		tf2::Matrix3x3 eulerMatrix(midq);
		double rx,ry,rz,drx,dry,drz;
		eulerMatrix.getRPY(drx,dry,drz,1);
		//ROS_INFO("DrX,%f,%f,%f",drx,dry,drz);

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
		
		}
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
	curPose = group->getCurrentPose().pose;
	
	ROS_WARN("xyz CUR POSE : %f , %f,%f,%f , %f,%f,%f",curPose.orientation.x,curPose.orientation.y,curPose.orientation.z,curPose.orientation.w,curPose.position.x,curPose.position.y,curPose.position.z);
	ROS_WARN("xyz target POSE : %f , %f,%f,%f , %f,%f,%f",targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w,targetPose.position.x,targetPose.position.y,targetPose.position.z);
}
//start I heard: [-0.500089,0.512233,0.500147,0.487219,0.120113,0.431763,0.146073]
//end I heard: [-0.349655,-0.470457,0.600847,0.543503,-0.224555,0.285898,0.545607]

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




  geometry_msgs::Pose start1,end1,startreal,endreal,endreal1;

//start I heard: [-0.500089,0.512233,0.500147,0.487219,0.120113,0.431763,0.146073]
//end I heard: [-0.349655,-0.470457,0.600847,0.543503,-0.224555,0.285898,0.545607]

start1.position.x = 0.120113;
start1.position.y = 0.431763;
start1.position.z = 0.146073;
start1.orientation.x = -0.500089;
start1.orientation.y = 0.512233;
start1.orientation.z = 0.500147;
start1.orientation.w = 0.487219;
//real:I heard: [-0.088320,0.018013,0.904410,0.417035,-0.543136,0.216047,-0.016239]]
//real:-I heard: [-0.013544,0.268312,0.857205,0.439346,-0.507351,0.292660,-0.053283]]
//real:-I heard: [0.208205,0.233230,0.759993,0.569794,-0.420654,0.380126,0.045389]]

startreal.position.x = -0.543136;
startreal.position.y = 0.216047;
startreal.position.z = -0.016239;
startreal.orientation.x = -0.088320;
startreal.orientation.y = 0.018013;
startreal.orientation.z = 0.904410;
startreal.orientation.w = 0.417035;
//
/*
end1.position.x = -0.224555;
end1.position.y = 0.285898;
end1.position.z = 0.545607;
*/
end1.position.x = -0.481732;
end1.position.y = 0.177044;
end1.position.z = 0.593711;

end1.orientation.x = -0.349655;
end1.orientation.y = -0.470457;
end1.orientation.z = 0.600847;
end1.orientation.w = 0.543503;


endreal.position.x = -0.507351;
endreal.position.y = 0.292660;
endreal.position.z = -0.053283;

endreal.orientation.x = -0.013544;
endreal.orientation.y = 0.268312;
endreal.orientation.z = 0.857205;
endreal.orientation.w = 0.439346;

endreal1.position.x = -0.420654;
endreal1.position.y = 0.380126;
endreal1.position.z = 0.045389;

endreal1.orientation.x = 0.208205;
endreal1.orientation.y = 0.233230;
endreal1.orientation.z = 0.759993;
endreal1.orientation.w = 0.569794;
std_msgs::Bool vs_start;
vs_start.data = true;
vs_start_pub.publish(vs_start);
ros::spinOnce();

vsMove( endreal1,10, DIST_TH,DIST_THQ, MIN_SPEED , MAX_SPEED, MIN_SPEEDQ , MAX_SPEEDQ);

std_msgs::Bool vs_stop;
vs_stop.data = false;
vs_start_pub.publish(vs_stop);
ROS_ERROR("vs POSE : moved to target"); 
 	
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
 // int count = 0;
  //while (true)
  {
 		
    ros::spinOnce();

    loop_rate.sleep();
   // ++count;
  }
  //vsMove( targetPose);
  return 0;
}
