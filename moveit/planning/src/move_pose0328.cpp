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
#include <object_detect/PointCorner.h>
#include <object_detect/CheckBox.h>
#include <object_detect/CheckSide.h>
#include <geometry_msgs/Quaternion.h>

using namespace Eigen;
class MovePose
{
  ros::NodeHandle nh_;
  ros::Subscriber pose_target_;
  ros::Subscriber object_center_target;   //subscriber for receiving object 2d data  author zxx
  ros::Subscriber object_corner_target;   //subscriber for receiving object's left corner 3d data  author zxx
//  ros::Subscriber object_corner_right_target;   //subscriber for receiving object's right corner 3d data  author zxx
//  ros::Subscriber object_corner_left_2d_target;   //subscriber for receiving object's left corner 2d data  author zxx
//  ros::Subscriber object_corner_right_2d_target;   //subscriber for receiving object's right corner 2d data  author zxx
  ros::Subscriber orientation_target;   //subscriber for receiving orientaion data  author zxx
  ros::Publisher display_publisher;
  moveit::planning_interface::MoveGroup *group_ptr;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  object_detect::PointVector pointsVector;   //zxx
  object_detect::PointCorner corners;   //zxx
  ros::ServiceClient client_position;      //zxx
  object_detect::CheckBox srv_position;       //zxx
  object_detect::CheckSide srv_rightside;       //zxx
  object_detect::CheckSide srv_leftside;       //zxx
  Matrix<float,4,4> mat_kinect;             //zxx
  geometry_msgs::Quaternion orientation;          //zxx
  bool left_side;
  int count;
  public:
  MovePose():
	nh_("~")
  {
    left_side = false;
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
    object_center_target = nh_.subscribe("/object_center_pub", 1, &MovePose::ObjectTransform, this);  //zxx
    object_corner_target = nh_.subscribe("/object_corner_pub", 1, &MovePose::GetCornerData, this);  //zxx
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    client_position = nh_.serviceClient<object_detect::CheckBox>("/box_check_position");  //zxx

    group_ptr->allowReplanning(true);
    kinematic_state = moveit::core::RobotStatePtr(group_ptr->getCurrentState());
    kinematic_state->setToDefaultValues();
 
  mat_kinect << 0.660196904125966, -0.213597975810214, -0.720080518075770, 963.612453142766,
	        0.779321726349541,  0.0801567126511229, 0.621476104332690,-552.302186532336,
	       -0.00800636751307271,0.975890309290152, -0.218114654053358, 897.936993543045,
	        0,0,0,1;
//I heard: [-0.586118,-0.302051,0.633121,0.405448,-0.700445,0.228948,0.448364]

    orientation.x = -0.561165;
    orientation.y = -0.258624;
    orientation.z = 0.761804;
    orientation.w = 0.194583;

    count = 0;

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());
  }

//zxx
  void GetCornerData(const object_detect::PointCorner& pVector)
  {
    corners = pVector;
  }

//zxx
  bool checkObject(geometry_msgs::Point& point)
  {
    //call the service to check
    srv_position.request.input = point;

    if (client_position.call(srv_position))
    {
      if(srv_position.response.output.x != 0)
        return true;
    }
    else
    {
      return false;
    }
  }
//zxx
  void matrix_trans(geometry_msgs::Pose& pose)
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
   ROS_INFO("rotation:%f,%f,%f",pose.position.x,pose.position.y,pose.position.z);
  }
//zxx
  bool obj_trans(geometry_msgs::Point point,geometry_msgs::Pose& pose)
  {
    if(point.x == 0 && point.y == 0 && point.z == 0)
      return false;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    matrix_trans(pose); 
    pose.orientation = orientation;
    return true;
  }
//zxx
  int DataProcessing(geometry_msgs::Point point, int index, geometry_msgs::Pose& pose)
  {ROS_INFO("orig rotation:%f,%f,%f",point.x,point.y,point.z);
    int num = -1;
    if(point.x == 0 && point.y == 0)
    {
      num = 1;
    }
    else
    {
      if(!checkObject(point))
        num = 2;
      else if(!obj_trans(corners.points.at(index),pose))
        num = 3;
      else
      {
        num = 0;
      }
    }
    return num;
  }
//zxx
  float BackInitPosition(geometry_msgs::Pose& target_pose)
  {
    geometry_msgs::Pose pose,target_pose2;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();
//    pose = this->group_ptr->getCurrentPose().pose;
//    waypoints.push_back(pose);
    pose = target_pose;
    pose.position.z += 0.03;
    waypoints.push_back(pose);
    pose.position.x += 0.16;
    pose.position.y -= 0.16;
//    pose.position.z -= 0.08;
    waypoints.push_back(pose);

    pose.position.x += 0.16;
    pose.position.y -= 0.16;
//    pose.position.z -= 0.08;
    waypoints.push_back(pose);

    pose.orientation = orientation;
    pose.position.x = 0.489018;
    pose.position.y = 0.386114;
    pose.position.z = 0.16004;

//    waypoints.push_back(target_pose);

    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

    if(fraction == 0)
      return fraction;
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
    
    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
    sleep(3);
    return fraction;
  }
//zxx
  float PlaceObject(geometry_msgs::Pose& pose)
  {
/*
last:I heard: [0.658973,0.204849,-0.677539,0.254425,-0.455334,-0.570633,0.026524]

   [-I heard: [-0.561165,-0.258624,0.761804,0.194583,-0.508922,0.555263,0.491085]]
[-I heard: [-0.309618,0.366667,0.795010,0.371014,-0.764025,0.300378,0.490748]]
[-I heard: [-0.508210,0.444116,0.553487,0.487991,-0.386159,0.242182,0.021647]]//zhongjian*/
    geometry_msgs::Pose target_pose,target_pose2,target_pose3;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();

    target_pose = pose;
    target_pose.position.x -= 0.15;
    target_pose.position.y += 0.15;
    target_pose.position.z += 0.5;
    waypoints.push_back(pose);
//    waypoints.push_back(pose);

    target_pose2.orientation = orientation;
    target_pose2.position.x = -0.386159;
    target_pose2.position.y =  0.242182;
    target_pose2.position.z =  0.242182;
    waypoints.push_back(target_pose2);

    target_pose2.orientation = orientation;
    target_pose2.position.x = -0.608922+0.12;
    target_pose2.position.y =  0.455263-0.12;
    target_pose2.position.z =  0.491085+0.03;
    waypoints.push_back(target_pose2);

    target_pose2.orientation = orientation;
    target_pose2.position.x = -0.608922;
    target_pose2.position.y =  0.455263;
    target_pose2.position.z =  0.491085;
    waypoints.push_back(target_pose2);

    pose = target_pose2;

    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

    if(fraction == 0)
      return fraction;

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
    return fraction;
  }
//zxx
  float PickObject(geometry_msgs::Pose& target_pose)
  {
//I heard: [-0.697277,0.082292,0.710696,0.044085,-0.530253,-0.063105,0.010877]
    geometry_msgs::Pose pose,pose1;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    bool complete = false;
    group_ptr->setStartStateToCurrentState();
    pose = target_pose;

    pose1.orientation.x = -0.697277;
    pose1.orientation.y = 0.082292;
    pose1.orientation.z = 0.710696;
    pose1.orientation.w = 0.044085;
    pose1.position.x = -0.530253;
    pose1.position.y = -0.063105;
    pose1.position.z = 0.010877;
    waypoints.push_back(pose1);

    if(left_side == true)
    {
      pose.position.x -= 0.15;
      pose.position.y += 0.15;
      pose.position.z += 0.012;
      waypoints.push_back(pose);
      pose.position.x += 0.06;
      pose.position.y += 0.06;
//    pose.position.z -= 0.08;
      waypoints.push_back(pose);
      pose.position.x += 0.15;
      pose.position.y -= 0.15;
      pose.position.z -= 0.08;
      waypoints.push_back(pose);
      pose.position.z -= 0.3;
      waypoints.push_back(pose);
    }

    if(left_side == false)
    {
      pose.position.x -= 0.15;
      pose.position.y += 0.15;
      pose.position.z += 0.012;
      waypoints.push_back(pose);
      pose.position.x -= 0.06;
      pose.position.y -= 0.06;
//    pose.position.z -= 0.08;
      waypoints.push_back(pose);
      pose.position.x += 0.15;
      pose.position.y -= 0.15;
      pose.position.z -= 0.08;
      waypoints.push_back(pose);
      pose.position.z -= 0.3;
      waypoints.push_back(pose);
    }
    target_pose = pose;

    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory);
    
    ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

    if(fraction == 0)
      return fraction;

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
    
    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
    sleep(3);
    return fraction;
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
     pointsVector = pVector;
     ROS_INFO("Hearing the data");
     if(!pointsVector.points.size() || !corners.points.size())
     {
       ROS_INFO("Could not get enough information");
       return;
     }
     int errorNum = -1;
     geometry_msgs::Pose pose;
     for(int i = 0; i < pointsVector.points.size();i++)
     {
       errorNum = DataProcessing(pointsVector.points.at(i), i, pose);
       switch (errorNum)
       {
         case 0:
         {
           left_side = corners.flags.at(i).data;
           if(PickObject(pose) == 1)
             count++;
        //   if(PlaceObject(pose) != 1)
             count--;
           BackInitPosition(pose);
           break;
         }
         case 1:
         {
           ROS_INFO("Input Data is Empty, Processing fail");
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_pose");
  MovePose move;
  ros::spin();

  return 0;
}
