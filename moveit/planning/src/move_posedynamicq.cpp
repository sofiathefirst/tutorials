#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <object_detect/PointVector.h>
#include <object_detect/CheckBox.h>
#include <object_detect/CheckSide.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
using namespace Eigen;
const float PI=3.1415926;
class MovePose
{
  ros::NodeHandle nh_;
  ros::Subscriber pose_target_;
  ros::Subscriber object_center_target;   //subscriber for receiving object 2d data  author zxx
  ros::Subscriber object_corner_left_target;   //subscriber for receiving object's left corner 3d data  author zxx
  ros::Subscriber object_corner_right_target;   //subscriber for receiving object's right corner 3d data  author zxx
  ros::Subscriber object_corner_left_2d_target;   //subscriber for receiving object's left corner 2d data  author zxx
  ros::Subscriber object_corner_right_2d_target;   //subscriber for receiving object's right corner 2d data  author zxx
  ros::Subscriber orientation_target;   //subscriber for receiving orientaion data  author zxx
  ros::Publisher display_publisher;
  moveit::planning_interface::MoveGroup *group_ptr;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  object_detect::PointVector pointsVector, leftCorner, rightCorner, right2d, left2d;   //zxx
  ros::ServiceClient client_position;      //zxx
  ros::ServiceClient client_rightside;      //zxx
  ros::ServiceClient client_leftside;      //zxx
  object_detect::CheckBox srv_position;       //zxx
  object_detect::CheckSide srv_rightside;       //zxx
  object_detect::CheckSide srv_leftside;       //zxx
  Matrix<float,4,4> mat_kinect;             //zxx
  geometry_msgs::Quaternion orientation_left;          //zxx
  geometry_msgs::Quaternion orientation_right;          //zxx
  bool left_side;
  ros::Time pick_time1,pick_time2,place_time;
  public:
  MovePose():
	nh_("~")
  {
    left_side = false;
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
    object_center_target = nh_.subscribe("/object_center_2d_pub", 1, &MovePose::GetCenterData, this);  //zxx
    object_corner_left_target = nh_.subscribe("/object_corner_left_pub", 1, &MovePose::GetCornerLeftData, this);  //zxx
    object_corner_right_target = nh_.subscribe("/object_corner_right_pub", 1, &MovePose::ObjectTransform, this);  //zxx
    object_corner_left_2d_target = nh_.subscribe("/object_corner_left_2d_pub", 1, &MovePose::GetCornerLeft2dData, this);  //zxx
    object_corner_right_2d_target = nh_.subscribe("/object_corner_right_2d_pub", 1, &MovePose::GetCornerRight2dData, this);  //zxx
//    object_corner_target = nh_.subscribe("/object_orientation_pub", 1, &MovePose::OrientationCallBack, this);  //zxx
//    pose_target_ = nh_.subscribe("get_pose", 1, &MovePose::chatterCallback, this);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    client_position = nh_.serviceClient<object_detect::CheckBox>("/box_check_position");  //zxx
    client_rightside = nh_.serviceClient<object_detect::CheckSide>("/box_check_rightside");  //zxx
    client_leftside = nh_.serviceClient<object_detect::CheckSide>("/box_check_leftside");  //zxx
    group_ptr->allowReplanning(true);
    kinematic_state = moveit::core::RobotStatePtr(group_ptr->getCurrentState());
    kinematic_state->setToDefaultValues();

    mat_kinect <<  0.535519534816726,-0.0569637060266108,-0.842599527667439,926.144010217435,
                   0.778090617159524,0.0441852590652739,0.626596085504566,-54.3966053417780,
                   0.00317945815808944,0.998981788242043,-0.0450030866335965,308.630797693139,
                   0,0,0,1;

//I heard: [-0.586118,-0.302051,0.633121,0.405448,-0.700445,0.228948,0.448364]
//-0.001655,0.488514,0.527282,0.695216
    orientation_left.x = -0.001655;
    orientation_left.y = 0.488514;
    orientation_left.z = 0.527282;
    orientation_left.w = 0.695216;
//I heard: [-0.294583,0.302412,0.881365,0.212046,-0.556905,0.432729,0.454853]
    orientation_right.x = -0.294583;
    orientation_right.y = 0.302412;
    orientation_right.z = 0.881365;
    orientation_right.w = 0.212046;

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());
  }
//zxx
  bool isAvailable(geometry_msgs::Point point)
  {
    if(isnan(point.x) || isnan(point.y) || isnan(point.z))
      return false;
    else return true;
  }
//zxx
  void GetCenterData(const object_detect::PointVector& pVector)
  {
    pointsVector = pVector;
  }

//zxx
  void GetCornerLeftData(const object_detect::PointVector& pVector)
  {
    leftCorner = pVector;
  }

//zxx
  bool checkSide(int index, geometry_msgs::Pose& object)
  {
    //call the service to check
    srv_rightside.request.input = right2d.points[index];
    srv_leftside.request.input = left2d.points[index];
    client_leftside.call(srv_leftside);
    client_rightside.call(srv_rightside);
    int right = srv_rightside.response.output.data;
    int left = srv_leftside.response.output.data;
    ROS_INFO("LEFT:%d ,RIGHT:%d",left,right);
    if(right >= 0&& left>= 0)
    {
      if(right >= left)
      {
        object.orientation.x = orientation_right.x;
        object.orientation.y = orientation_right.y;
        object.orientation.z = orientation_right.z;
        object.orientation.w = orientation_right.w;
        object.position.x = rightCorner.points[index].x;
        object.position.y = rightCorner.points[index].y;
        object.position.z = rightCorner.points[index].z;
    ROS_INFO("CHOOSING RIGHT SIDE,ORIG X:%f ,ORIG Y:%f",right2d.points[index].x,right2d.points[index].y);
        left_side = false;
      }
      else
      {
        object.orientation.x = orientation_left.x;
        object.orientation.y = orientation_left.y;
        object.orientation.z = orientation_left.z;
        object.orientation.w = orientation_left.w;
        object.position.x = leftCorner.points[index].x;
        object.position.y = leftCorner.points[index].y;
        object.position.z = leftCorner.points[index].z;
    ROS_INFO("CHOOSING LEFT SIDE,ORIG X:%f ,ORIG Y:%f",left2d.points[index].x,left2d.points[index].y);
        left_side = true;
      } 
    }
    else if((right == -1 && left == -1) || (right == -1 && left >= 0))
    {
      
      object.orientation.x = orientation_left.x;
      object.orientation.y = orientation_left.y;
      object.orientation.z = orientation_left.z;
      object.orientation.w = orientation_left.w;
      object.position.x = leftCorner.points[index].x;
      object.position.y = leftCorner.points[index].y;
      object.position.z = leftCorner.points[index].z;
    ROS_INFO("CHOOSING LEFT SIDE,ORIG X:%f ,ORIG Y:%f",left2d.points[index].x,left2d.points[index].y);
      left_side = true;
    }
    else if(left== -1 && right >= 0)
    {
      object.orientation.x = orientation_right.x;
      object.orientation.y = orientation_right.y;
      object.orientation.z = orientation_right.z;
      object.orientation.w = orientation_right.w;
      object.position.x = rightCorner.points[index].x;
      object.position.y = rightCorner.points[index].y;
      object.position.z = rightCorner.points[index].z;
    ROS_INFO("CHOOSING RIGHT SIDE,ORIG X:%f ,ORIG Y:%f",right2d.points[index].x,right2d.points[index].y);
      left_side = false;
    }
    return true;
/*    if (client_side.call(srv_side))
    {
      if(srv_side.response.output.data == 0 || srv_side.response.output.data == 2)
      {
        object.orientation.x = orientation_right.x;
        object.orientation.y = orientation_right.y;
        object.orientation.z = orientation_right.z;
        object.orientation.w = orientation_right.w;
        object.position.x = rightCorner.points[index].x;
        object.position.y = rightCorner.points[index].y;
        object.position.z = rightCorner.points[index].z;
        left_side = false;
      }
      else if(srv_side.response.output.data == 1)
      {
        object.orientation.x = orientation_left.x;
        object.orientation.y = orientation_left.y;
        object.orientation.z = orientation_left.z;
        object.orientation.w = orientation_left.w;
        object.position.x = leftCorner.points[index].x;
        object.position.y = leftCorner.points[index].y;
        object.position.z = leftCorner.points[index].z;
        left_side = true;
      }
      return true;
    }
    else
    {
      return false;
    }*/
  }
//zxx
  bool checkObject(geometry_msgs::Pose& pose)
  {
    //call the service to check
    srv_position.request.input.x = pose.position.x;
    srv_position.request.input.y = pose.position.y;
    srv_position.request.input.z = pose.position.z;
    if (client_position.call(srv_position))
    {
      if(srv_position.response.output.x != 100)
      {
        pose.position.x = srv_position.response.output.x;
        pose.position.y = srv_position.response.output.y;
        pose.position.z = srv_position.response.output.z;
      }
      return true;
    }
    else
    {
      return false;
    }
  }
//zxx
  bool matrix_trans(geometry_msgs::Pose& pose)
  {
    Matrix<float,4,1> Pr;
    Matrix<float,4,1> Pc;
    Pc << pose.position.x*1000,
          pose.position.y*1000,
          pose.position.z*1000,
                    1;

   Pr = mat_kinect *Pc;

   pose.position.x = Pr(0,0)/1000;
   pose.position.y = Pr(1,0)/1000;
   pose.position.z = Pr(2,0)/1000;
   if(isnan(pose.position.x) || isnan(pose.position.y) || isnan(pose.position.z))
     return false;
   else 
     ROS_INFO("rotation:%f,%f,%f",pose.position.x,pose.position.y,pose.position.z);
   return true;
  }
//zxx
  bool obj_trans(geometry_msgs::Pose& pose)
  {
    if(pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0)
      return false;
    if(!matrix_trans(pose))
       return false;
    return true;
  }
//zxx
  int DataProcessing(geometry_msgs::Point point, int index, geometry_msgs::Pose& pose)
  {
    int num = -1;
    if(point.x == 0 && point.y == 0 && isAvailable(rightCorner.points[index]) && isAvailable(leftCorner.points[index]))
    {
      num = 1;
    }
    else
    {
      if(!checkSide(index, pose))
      {
        num = 4;
      }
//      else if(!checkObject(pose))
//        num = 2;
      else if(!obj_trans(pose))
        num = 3;
      else
      {
        num = 0;
      }
    }
    return num;
  }
//int zymleft=0;
//zym 20160317
void seeV3(tf2::Vector3 v,int left=0)
{
	//v.normalize();
	//ROS_ERROR("tf2:ad:Vector3 :X=%f,%f,%f,left = %d",v.getX(),v.getY(),v.getZ(),(int)left);
}
void seeQ(tf2::Quaternion q)
{
	ROS_INFO("euler Q:X=%f,%f,%.10f,%.10f",q.x(),q.y(),q.z(),q.w());
}
void calFixAngleQ(const tf2::Vector3 positionThing, const tf2::Vector3 positionRobot, geometry_msgs::Quaternion &gq)
{
	ROS_INFO("\n\n*****calFixAngleQ****THING PISTION , ROBOT POSITION*********start******");
	tf2::Vector3 position2;
	tf2::Quaternion q;
	tf2::Vector3 OA(0,0,-1);
	double mangle;
	tf2::Vector3 maxis;

	seeV3(positionThing);
	seeV3(positionRobot);
	tf2::Vector3 level(0,-1,0);
	position2 = positionThing - positionRobot;
	position2.normalize();
	mangle= position2.angle(OA);
	ROS_INFO("TWO VECTOR ANGEL = %f",mangle*180/PI);
	maxis  =  OA.cross(position2);

	position2.setZ(0);
	position2.normalize();
	double fixAngle =  position2.angle(level) + 40*PI/180 ;//here 40 for try, 

	tf2::Quaternion fixq,oaq(0.032218,0.706275,-0.031921,0.706483);//-0.414125,0.563932,0.434589,0.567110);//0.336918,0.635834,-0.320840,0.615844);//(0.664140,0.242730,-0.664140,0.242730);
	oaq.normalize();
	fixq.setRotation(-OA, fixAngle);
	
	fixq = fixq*oaq;	
	fixq.normalize();

	q.setRotation(maxis, -mangle);
	q.normalize();
	q = q*fixq;	
	q.normalize();
	seeQ(q);
	gq.x = q.x();
	gq.y = q.y();
	gq.z = q.z();
	gq.w = q.w();
	ROS_INFO("-----calFixAngleQ----THING PISTION , ROBOT POSITION---_____end--");
	return ;

}

//zxx
  bool BackInitPosition(geometry_msgs::Pose pose)
  {
/*
last:I heard: [0.658973,0.204849,-0.677539,0.254425,-0.455334,-0.570633,0.026524]
*/
    geometry_msgs::Pose start_pose, target_pose1,target_pose2,target_pose3;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();
    start_pose = this->group_ptr->getCurrentPose().pose;
    target_pose1 = pose;

    target_pose1.position.x = -0.854562-0.03;
    target_pose1.position.y = 0.081640-0.03;
    target_pose1.position.z = 0.010846+0.09;
    waypoints.push_back(target_pose1);

    target_pose1.position.x = -0.854562+0.12;
    target_pose1.position.y = 0.081640-0.12;
    target_pose1.position.z = 0.010846+0.09;
    waypoints.push_back(target_pose1);

    target_pose2.orientation.x= 0.658973;
    target_pose2.orientation.y= 0.204849;
    target_pose2.orientation.z= -0.677539;
    target_pose2.orientation.w= 0.397951;
    target_pose2.position.x = -0.45;
    target_pose2.position.y = -0.57;
    target_pose2.position.z = 0.05;
//rostopic pub /move_pose/get_pose geometry_msgs/Pose '[-0.744562,-0.031640,0.010846]' '[0.658973,0.204849,-0.677539,0.397951]'
//rostopic pub /move_pose/get_pose geometry_msgs/Pose '[-0.45,-0.57,0.05]' '[0.658973,0.204849,-0.677539,0.397951]'
    waypoints.push_back(target_pose2);

    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
	if(!fraction)
		return false;
    robot_trajectory::RobotTrajectory rt(this->kinematic_state->getRobotModel(),"manipulator");
    rt.setRobotTrajectoryMsg(*(this->kinematic_state), trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
   // Get RobotTrajectory_msg from RobotTrajectory

    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
    my_plan.trajectory_ = trajectory;
    success = this->group_ptr->execute(my_plan);
    ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
    
//    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
    sleep(3);
    return true;
  }
//zxx
  bool PlaceObject(geometry_msgs::Pose pose)
  {
/*
last:I heard: [0.658973,0.204849,-0.677539,0.254425,-0.455334,-0.570633,0.026524]
*/
    geometry_msgs::Pose start_pose, target_pose1,target_pose2,target_pose3;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();
    start_pose = this->group_ptr->getCurrentPose().pose;
    target_pose2 = pose;
//    waypoints.push_back(start_pose);
//B closet: left I heard: [-0.003577,0.488623,0.528194,0.694439,-0.854562,0.081640,0.010846]
//               I heard: [-0.294567,0.303065,0.881088,0.212287,-0.828615,0.087779,0.013787]
    if(left_side == true)
    {
      pose.orientation.x = 0.062712;
      pose.orientation.y = 0.456148;
      pose.orientation.z = 0.759288;
      pose.orientation.w = 0.541021;
    }
    else 
    {
      pose.orientation.x = -0.248754;
      pose.orientation.y = 0.307697;
      pose.orientation.z = 0.899740;
      pose.orientation.w = 0.196943;
    }
    pose.position.x += 0.05;
    pose.position.y -= 0.05;
    pose.position.z += 0.04;
    waypoints.push_back(pose);

    if(left_side == true)
    {
	//zymleft = 1;
      pose.orientation.x = orientation_left.x;
      pose.orientation.y = orientation_left.y;
      pose.orientation.z = orientation_left.z;
      pose.orientation.w = orientation_left.w;
    }
    else 
    {//zymleft = 0;
      pose.orientation.x = orientation_right.x;
      pose.orientation.y = orientation_right.y;
      pose.orientation.z = orientation_right.z;
      pose.orientation.w = orientation_right.w;
    }
    pose.position.x += 0.05;
    pose.position.y -= 0.05; 
    pose.position.z += 0.05;
    //insert the last position//
//   rostopic pub /move_pose/get_pose geometry_msgs/Pose '[-0.45,-0.57,0.013787]' '[-0.294567,0.303065,0.881088,0.212287]'
    waypoints.push_back(pose);

//[0.211465,0.293361,0.759288,0.541021,-0.423678,0.512445,0.064716]left point
//I heard: [-0.100417,0.081471,0.913671,0.385337,-0.429042,0.539974,0.059461]right point
//I heard: [-0.319912,0.302364,0.875864,0.197724,-0.348277,0.130509,0.167887]
//I heard: [0.151415,0.476072,0.454971,0.737177,-0.163332,0.368741,0.154044]
    pose.position.x = -0.530851;
    pose.position.y = 0.063105;
    pose.position.z = 0.010877;
    waypoints.push_back(pose);

    target_pose1.orientation.x= pose.orientation.x;
    target_pose1.orientation.y= pose.orientation.y;
    target_pose1.orientation.z= pose.orientation.z;
    target_pose1.orientation.w= pose.orientation.w;
    target_pose1.position.x = -0.854562+0.15;
    target_pose1.position.y = 0.081640-0.15;
    target_pose1.position.z = target_pose2.position.z + 0.11;
    waypoints.push_back(target_pose1);

    target_pose1.position.x = -0.854562-0.03;
    target_pose1.position.y = 0.081640-0.03;
    target_pose1.position.z = target_pose2.position.z + 0.02;
    waypoints.push_back(target_pose1);
    
/*    target_pose2.orientation.x= 0.658973;
    target_pose2.orientation.y= 0.204849;
    target_pose2.orientation.z= -0.677539;
    target_pose2.orientation.w= 0.397951;
    target_pose2.position.x = -0.45;
    target_pose2.position.y = -0.57;
    target_pose2.position.z = 0.05;

    waypoints.push_back(target_pose2);*/

    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
	if(!fraction)
		return false;
    robot_trajectory::RobotTrajectory rt(this->kinematic_state->getRobotModel(),"manipulator");
    rt.setRobotTrajectoryMsg(*(this->kinematic_state), trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
   // Get RobotTrajectory_msg from RobotTrajectory

    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
    my_plan.trajectory_ = trajectory;
    success = this->group_ptr->execute(my_plan);
    ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
    
//    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
    sleep(3);
    return true;
  }
//zxx
  bool PickObject(geometry_msgs::Pose& target_pose)
  {
//I heard: [-0.697277,0.082292,0.710696,0.044085,-0.530253,-0.063105,0.010877]
    geometry_msgs::Pose pose,pose1;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();
//    pose = this->group_ptr->getCurrentPose().pose;
//    waypoints.push_back(pose);
    pose = target_pose;
//I heard: [-0.126086,0.722910,0.050704,0.677446,-0.380851,0.249190,0.074685]
    pose1.orientation.x = -0.697277;
    pose1.orientation.y = 0.082292;
    pose1.orientation.z = 0.710696;
    pose1.orientation.w = 0.044085;
    pose1.position.x = -0.530253;
    pose1.position.y = -0.063105;
    pose1.position.z = 0.010877;
    waypoints.push_back(pose1);
	
    pose1.orientation.x = -0.697277;
    pose1.orientation.y = 0.082292;
    pose1.orientation.z = 0.710696;
    pose1.orientation.w = 0.044085;
     tf2::Vector3 tpv3(target_pose.position.x,target_pose.position.y,target_pose.position.z );
	//seeV3(tpv3,zymleft);
	ROS_ERROR("targetpose%f,%f,%f,left=%d ",target_pose.position.x,target_pose.position.y,target_pose.position.z,(int)left_side);
//calFixAngleQ( tf2::Vector3 (target_pose.position.x,target_pose.position.y,target_pose.position.z ),  tf2::Vector3 (-0.380851,0.249190,0.074685), pose1.orientation);

    pose1.position.x = -0.380851;
    pose1.position.y = 0.249190;
    pose1.position.z = 0.074685;
if(pose.position.z >0.35)
{
//0.582340,0.313830,-0.713946,0.229505,-0.554827,-0.149314,0.363640]
pose1.orientation.x = -0.697277;
    pose1.orientation.y = 0.082292;
    pose1.orientation.z = 0.710696;
    pose1.orientation.w = 0.044085;
    pose1.position.x = -0.554827;
    pose1.position.y = -0.149314;
    pose1.position.z = 0.363640;
    waypoints.push_back(pose1);
}
    waypoints.push_back(pose1);
    if(left_side == false)
    {
      pose.position.x += 0.12;
      pose.position.y -= 0.10;
      pose.position.z += 0.05;
      waypoints.push_back(pose);
/*      pose.position.x += 0.014;
      pose.position.y += 0.014;
      waypoints.push_back(pose);*/
      pose.position.x -= 0.095;
      pose.position.y += 0.095;
      pose.position.z -= 0.012;
      waypoints.push_back(pose);
      pose.position.x -= 0.022;
      pose.position.y -= 0.022;
      pose.position.z -= 0.065;
      waypoints.push_back(pose);
/*      pose.position.x -= 0.02;   //lzc
      pose.position.y -= 0.02;   //lzc
      waypoints.push_back(pose);
      pose.position.x -= 0.025;
      pose.position.y += 0.025; 
      pose.position.z -= 0.02;   //lzc
      waypoints.push_back(pose);
      pose.position.z -= 0.02;    //lzc
      waypoints.push_back(pose);
      pose.position.z += 0.03;    //lzc
      waypoints.push_back(pose);;*/
    }
    if(left_side == true)
    {
      pose.position.x += 0.13;
      pose.position.y -= 0.08;
      pose.position.z += 0.05;
      waypoints.push_back(pose);
      pose.position.x -= 0.044;    //lzc: original 0.024
      pose.position.y -= 0.044;    //lzc: original 0.024
      waypoints.push_back(pose);
      pose.position.x -= 0.09;
      pose.position.y += 0.09;
      pose.position.z -= 0.022;
      waypoints.push_back(pose);
/*      pose.position.x += 0.035;    //lzc: 
      pose.position.y += 0.035;    //lzc: 
      waypoints.push_back(pose);*/
      pose.position.x -= 0.017;
      pose.position.y += 0.017;      
//      pose.position.z -= 0.02;    //lzc
      waypoints.push_back(pose);
      pose.position.z -= 0.05;    //lzc
      waypoints.push_back(pose);
    }
    target_pose = pose;
//    waypoints.push_back(target_pose);
    waypoints.push_back(pose);
    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
	if(!fraction)
		return false;
    robot_trajectory::RobotTrajectory rt(this->kinematic_state->getRobotModel(),"manipulator");
    rt.setRobotTrajectoryMsg(*(this->kinematic_state), trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
   // Get RobotTrajectory_msg from RobotTrajectory
    pick_time2 = ros::Time::now();
    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
    my_plan.trajectory_ = trajectory;
    success = this->group_ptr->execute(my_plan);
    ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
    
    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
    sleep(3);
    return true;
  }
//zxx
  void ObjectTransform(const object_detect::PointVector& pVector)
  {
     //transform the camera_frame_base data to robot_frame_base data
     //trabsform the point data to pose data
     //transform the object positions to target positions
     //check if the objects still here
     //if here send the positions as target positions
     //Pick up the objects
     rightCorner = pVector;

     pick_time1 = ros::Time::now();
     ROS_INFO("Hearing the data");
     if(!pointsVector.points.size() || !leftCorner.points.size() || !rightCorner.points.size())
     {
       ROS_INFO("Could not get enough information");
       return;
     }
     int errorNum = -1;
     ros::Duration dur1,dur2,dur3;
     geometry_msgs::Pose pose;
     for(int i = 0; i < pVector.points.size();i++)
     {
       errorNum = DataProcessing(pVector.points.at(i), i, pose);
       switch (errorNum)
       {
         case 0:
         {
           ROS_INFO("Processing Sucess,and Executing data is %f,%f,%f,%f,%f,%f,%f", pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);
           if(PickObject(pose))
		   {
			   ROS_INFO("Pick Path Planning Success");
                           dur1 = ros::Time::now() - pick_time1;
                           dur2 = ros::Time::now() - pick_time2;
                           ROS_INFO("PICK TIME: %f, %f",dur1.toSec(),dur2.toSec());
                           place_time = ros::Time::now();
			   if(PlaceObject(pose))
			   {
                                   dur3 = ros::Time::now() - place_time;
                                   ROS_INFO("Place TIME: %f",dur3.toSec());                           
				   ROS_INFO("Place Path Planning Success");
                                   BackInitPosition(pose);
                                   dur3 = ros::Time::now() - place_time;
                                   ROS_INFO("Place TIME: %f",dur3.toSec());  

			   }
			   else
			   {
			//	   ROS_INFO("Pick Path Planning Success");
			   }
		   }
		   else
		   {
			   ROS_INFO("Place Path Planning Success");
		   }
           break;
         }
         case 1:
         {
           ROS_INFO("Input Data is Empty or Unavailable, Processing fail");
           break;
         }
         case 2:
         {
           ROS_INFO("Calling Check Positon Service fail, Processing fail");
           break;
         }
         case 3:
         {
           ROS_INFO("Data Transform fail, Processing fail");
           break;
         }
         case 4:
         {
             ROS_INFO("Calling Check Side Service fail, Processing fail");
           break;
         }
         default:ROS_INFO("Default, Processing fail");
       }
     }
  }
//zxx
  void GetCornerLeft2dData(const object_detect::PointVector leftVector)
  {
    left2d = leftVector;
  }
//zxx
  void GetCornerRight2dData(const object_detect::PointVector rightVector)
  {
    right2d = rightVector;
  }
//zxx
  void OrientationCallBack(const geometry_msgs::Quaternion qua)
  {
    orientation_left = qua;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_pose");
  MovePose move;
  ros::spin();

  return 0;
}
