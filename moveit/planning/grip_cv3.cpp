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
//////headers for gripper///////
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Time.h>
#include <stdio.h>
#include <sstream>



moveit::planning_interface::MoveGroup *group;
geometry_msgs::Pose target;
float error = 0.05;   //zliu7
class GRIPCV
{
  ros::NodeHandle nh_;
  ros::Publisher tar_state_pub;
  ros::Publisher display_publisher;
  ros::Publisher planning_scene_diff_publisher;
  ros::Subscriber cv_position_sub;
  ros::Publisher hand_pub;
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
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
    cv_position_sub = nh_.subscribe("/cv_position", 1, &GRIPCV::cv_position_Cb, this);
    tar_state_pub = nh_.advertise<geometry_msgs::Pose>("/target_state", 1, true);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    hand_pub = nh_.advertise<std_msgs::UInt16>("servo", 1);    //zliu7  "1000"

    group_ptr->setPlanningTime(3.0);
    group_ptr->allowReplanning(true);
    kinematic_state = moveit::core::RobotStatePtr(group_ptr->getCurrentState());
    kinematic_state->setToDefaultValues();
    resetPose(target);
    
    // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());

//  robot_model::RobotModelPtr model_;
  model.reset(new robot_model::RobotModel(group_ptr->getCurrentState()->getRobotModel()->getURDF(),        group_ptr->getCurrentState()->getRobotModel()->getSRDF()));
  }
  
  bool TargetCheck(geometry_msgs::Pose target_pose)
  {
      float e[7] = {0};
      e[0] = group_ptr->getCurrentPose().pose.orientation.x - target_pose.orientation.x;
      e[1] = group_ptr->getCurrentPose().pose.orientation.y - target_pose.orientation.y;
      e[2] = group_ptr->getCurrentPose().pose.orientation.z - target_pose.orientation.z;
      e[3] = group_ptr->getCurrentPose().pose.orientation.w - target_pose.orientation.w;
      e[4] = group_ptr->getCurrentPose().pose.position.x - target_pose.position.x;
      e[5] = group_ptr->getCurrentPose().pose.position.y - target_pose.position.y;
      e[6] = group_ptr->getCurrentPose().pose.position.z - target_pose.position.z;

      for(int i = 0; i < 7; i++)
      {
        if(!(e[i] > error || e[i] < -error))
          return false;
      }
      
      return true;
  }
  void resetPose(geometry_msgs::Pose target_pose)
  {
      target_pose.orientation.x = 0;
      target_pose.orientation.y = 0;
      target_pose.orientation.z = 0;
      target_pose.orientation.w = 0;
      target_pose.position.x = 0;
      target_pose.position.y = 0;
      target_pose.position.z = 0;
  }
  geometry_msgs::Pose CurrentPose()
  {
    return group_ptr->getCurrentPose().pose;
  }

  void robo_arm()
  {
    ROS_INFO("START TO PULL BACK");
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose;
    start_pose = group_ptr->getCurrentPose().pose;

    waypoints.push_back(start_pose);
 
    geometry_msgs::Pose target_pose1;
//  waypoints.push_back(target_pose1);

    target_pose1.orientation.x = -0.239494;
    target_pose1.orientation.y = 0.682661;
    target_pose1.orientation.z = 0.220907;
    target_pose1.orientation.w = 0.654077;
    target_pose1.position.x = 0.435729;
    target_pose1.position.y = 0.472243;
    target_pose1.position.z = 0.440597;

    waypoints.push_back(target_pose1);

 //   waypoints.push_back(target_pose1);
    target_pose1.orientation.x = -0.239494;
    target_pose1.orientation.y = 0.682661;
    target_pose1.orientation.z = 0.220907;
    target_pose1.orientation.w = 0.654077;
    target_pose1.position.x = 0.435729;
    target_pose1.position.y = 0.472243;
    target_pose1.position.z = 0.240597;

    waypoints.push_back(target_pose1);
    target = target_pose1;

    float fraction = group_ptr->computeCartesianPath(waypoints,
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
   ROS_INFO("finish robot_arm");
   sleep(0.5);
  }
  void getTargetPosition(int number)
  {
    ROS_INFO("number is %d)",number);
if (number==49)
{    ros::shutdown();}        //zliu7
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    geometry_msgs::Pose start_pose, target_pose0, target_pose1;
    std::vector<geometry_msgs::Pose> waypoints;

    start_pose = group_ptr->getCurrentPose().pose;
    waypoints.push_back(start_pose);

    target_pose0.orientation.x = 0.727432;
    target_pose0.orientation.y = 0.000533;
    target_pose0.orientation.z = -0.686147;
    target_pose0.orientation.w = 0.006733;
    target_pose0.position.x = 0.052658;
    target_pose0.position.y = 0.624502;
    target_pose0.position.z = 0.359414;

    waypoints.push_back(target_pose0);
/////////////////////////pick_position_choices///////////////////////////////////////////////
   switch(number)
   {
    case 1:
    ROS_INFO("region number is 1");
     {
       target_pose0.orientation.x = 0.507844;
       target_pose0.orientation.y = 0.512758;
       target_pose0.orientation.z = -0.446485;
       target_pose0.orientation.w = 0.528985;
       target_pose0.position.x = -0.683588;
       target_pose0.position.y = -0.056049;
       target_pose0.position.z = 0.169229;
     }
      break;
    case 2:
    ROS_INFO("region number is 2");
     {
        target_pose0.orientation.x = 0.512654;
        target_pose0.orientation.y = 0.492457;
        target_pose0.orientation.z = -0.476485;
        target_pose0.orientation.w = 0.515365;
        target_pose0.position.x = -0.769702;
        target_pose0.position.y = 0.001196;
        target_pose0.position.z = 0.160482;}
     break;
    case 3:
    ROS_INFO("region number is 3");
     {
       target_pose0.orientation.x = 0.512654;
       target_pose0.orientation.y = 0.492457;
       target_pose0.orientation.z = -0.476485;
       target_pose0.orientation.w = 0.515365;
       target_pose0.position.x = -0.827457;
       target_pose0.position.y = 0.092285;
       target_pose0.position.z = 0.160482;}
    break;
    case 4:
    ROS_INFO("region number is 4");
    {
      target_pose0.orientation.x = 0.512654;
      target_pose0.orientation.y = 0.492457;
      target_pose0.orientation.z = -0.476485;
      target_pose0.orientation.w = 0.515365;
      target_pose0.position.x = -0.882397;
      target_pose0.position.y = 0.166527;
      target_pose0.position.z = 0.160482;}
    break;
  case 5:
   ROS_INFO("region number is 5");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.627950;
     target_pose0.position.y = 0.028137;
     target_pose0.position.z = 0.160482;}
   break;
  case 6:
   ROS_INFO("region number is 6");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.691047;
     target_pose0.position.y = 0.077501;
     target_pose0.position.z = 0.160482;}
   break;
  case 7:
   ROS_INFO("region number is 7");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.752627;
     target_pose0.position.y = 0.165556;
     target_pose0.position.z = 0.160482;}
    break;
  case 8:
   ROS_INFO("region number is 8");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.822862;
     target_pose0.position.y = 0.233591;
     target_pose0.position.z = 0.160482;}
   break;
  case 9:
   ROS_INFO("region number is 9");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.564816;
     target_pose0.position.y = 0.110791;
     target_pose0.position.z = 0.160482;}
   break;
   case 10:
   ROS_INFO("region number is 10");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.613242;
     target_pose0.position.y = 0.165149;
     target_pose0.position.z = 0.160482;}
   break;
   case 11:
  ROS_INFO("region number is 11");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.673585;
     target_pose0.position.y = 0.253191;
     target_pose0.position.z = 0.160482;}
   break;

   case 12:
   ROS_INFO("region number is 12");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.745037;
     target_pose0.position.y = 0.316772;
     target_pose0.position.z = 0.160482;}
   break;

   case 13:
   ROS_INFO("region number is 13");
   {
     target_pose0.orientation.x = 0.512654;
     target_pose0.orientation.y = 0.492457;
     target_pose0.orientation.z = -0.476485;
     target_pose0.orientation.w = 0.515365;
     target_pose0.position.x = -0.452382;
     target_pose0.position.y = 0.185003;
     target_pose0.position.z = 0.160482;}
   break;

   case 14:
   ROS_INFO("region number is 14");
   {
     target_pose0.orientation.x = 0.602899;
     target_pose0.orientation.y = 0.356418;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.546517;
     target_pose0.position.y = 0.236324;
     target_pose0.position.z = 0.159414;}
   break;
   case 15:
   ROS_INFO("region number is 15");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x =-0.588250;
     target_pose0.position.y = 0.303758;
     target_pose0.position.z = 0.159414;}
   break;
   case 16:
   ROS_INFO("region number is 16");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.673969;
     target_pose0.position.y = 0.368010;
     target_pose0.position.z = 0.159414;}
    break;
   case 17:
   ROS_INFO("region number is 17");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.397272;
     target_pose0.position.y = 0.236959;
     target_pose0.position.z = 0.159414;}
   break;
   case 18:
   ROS_INFO("region number is 18");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.453046;
     target_pose0.position.y = 0.301415;
     target_pose0.position.z = 0.159414;}
   break;
   case 19:
   ROS_INFO("region number is 19");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.511637;
     target_pose0.position.y = 0.360980;
     target_pose0.position.z = 0.159414;}
   break;
   case 20:
   ROS_INFO("region number is 20");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.606802;
     target_pose0.position.y = 0.444754;
     target_pose0.position.z = 0.159414;}
   break;
   case 21:
   ROS_INFO("region number is 21");
  {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.364137;
     target_pose0.position.y =0.293931;
     target_pose0.position.z = 0.159414;}
   break;

   case 22:
   ROS_INFO("region number is 22");
   {
     target_pose0.orientation.x = 0.727432;
     target_pose0.orientation.y = 0.000533;
     target_pose0.orientation.z = -0.686147;
     target_pose0.orientation.w = 0.006733;
     target_pose0.position.x = -0.405417;
     target_pose0.position.y = 0.362528;
     target_pose0.position.z = 0.159414;}
   break;
   case 23:
   ROS_INFO("region number is 23");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.449169;
  target_pose0.position.y =  0.421137;
  target_pose0.position.z = 0.159414;}
break;
   case 24:
ROS_INFO("region number is 24");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.525099;
  target_pose0.position.y = 0.481579;
  target_pose0.position.z = 0.159414;}
break;
 case 25:
ROS_INFO("region number is 25");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.267721;
  target_pose0.position.y = 0.350164;
  target_pose0.position.z = 0.159414;}
break;
 case 26:
ROS_INFO("region number is 26");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.320128;
  target_pose0.position.y = 0.408632;
  target_pose0.position.z = 0.159414;}
break;
 case 27:
ROS_INFO("region number is 27");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.393979;
  target_pose0.position.y = 0.490341;
  target_pose0.position.z = 0.159414;}
break;
 case 28:
ROS_INFO("region number is 28");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.457486;
  target_pose0.position.y = 0.564841;
  target_pose0.position.z = 0.53767;}
break;
 case 29:
ROS_INFO("region number is 29");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.183576;
  target_pose0.position.y = 0.436186;
  target_pose0.position.z = 0.159414;}
break;
  case 30:
ROS_INFO("region number is 30");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.250219;
  target_pose0.position.y =0.503831;
  target_pose0.position.z = 0.159414;}
break;
 case 31:
ROS_INFO("region number is 31");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.304597;
  target_pose0.position.y = 0.555553;
  target_pose0.position.z = 0.159414;}
break;

     case 32:
ROS_INFO("region number is 32");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.376969;
  target_pose0.position.y =0.627713;
  target_pose0.position.z = 0.159414;}
break;
 case 33:
ROS_INFO("region number is 33");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.111549;
  target_pose0.position.y = 0.526764;
  target_pose0.position.z = 0.159414;}
break;
   case 34:
ROS_INFO("region number is 34");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.168368;
  target_pose0.position.y =0.560281;
  target_pose0.position.z = 0.159414;}
break;
 case 35:
ROS_INFO("region number is 35");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.233896;
  target_pose0.position.y = 0.633942;
  target_pose0.position.z = 0.159414;}
break;
 case 36:
ROS_INFO("region number is 36");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.292756;
  target_pose0.position.y = 0.694352;
  target_pose0.position.z = 0.159414;}
break;
 case 37:
ROS_INFO("region number is 37");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.024297;
  target_pose0.position.y = 0.589113;
  target_pose0.position.z = 0.159414;}
break;
 case 38:
ROS_INFO("region number is 38");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.088627;
  target_pose0.position.y = 0.633906;
  target_pose0.position.z = 0.159414;}
break;
 case 39:
ROS_INFO("region number is 39");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.155018;
  target_pose0.position.y = 0.676999;
  target_pose0.position.z = 0.159414;}
break;
  case 40:
ROS_INFO("region number is 40");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.229342;
  target_pose0.position.y = 0.764232;
  target_pose0.position.z = 0.159414;}
break;
 case 41:
ROS_INFO("region number is 41");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.046146;
  target_pose0.position.y = 0.637606;
  target_pose0.position.z = 0.159414;}
break;

     case 42:
ROS_INFO("region number is 42");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.009425;
  target_pose0.position.y = 0.692584;
  target_pose0.position.z = 0.159414;}
break;
 case 43:
ROS_INFO("region number is 43");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.078524;
  target_pose0.position.y = 0.762469;
  target_pose0.position.z = 0.159414;}
break;
   case 44:
ROS_INFO("region number is 44");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.148140;
  target_pose0.position.y = 0.831043;
  target_pose0.position.z = 0.159414;}
break;
 case 45:
ROS_INFO("region number is 45");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.138174;
  target_pose0.position.y = 0.721717;
  target_pose0.position.z = 0.159414;}
break;
 case 46:
ROS_INFO("region number is 46");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.068142;
  target_pose0.position.y = 0.780575;
  target_pose0.position.z = 0.159414;}
break;
 case 47:
ROS_INFO("region number is 47");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.002522;
  target_pose0.position.y = 0.835814;
  target_pose0.position.z = 0.159414;}
break;
 case 48:
ROS_INFO("region number is 48");
  {target_pose0.orientation.x = 0.727432;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = -0.080989;
  target_pose0.position.y = 0.881881;
  target_pose0.position.z = 0.159414;}
break;
 case 49:
ROS_INFO("region number is 49");
  {target_pose0.orientation.x = -0.674416;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = 0.725754;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.221192;
  target_pose0.position.y = 0.760057;
  target_pose0.position.z = 0.159414;}
break;
  case 50:
ROS_INFO("region number is 50");
  {target_pose0.orientation.x = -0.677166;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.137612 ;
  target_pose0.position.y = 0.821663;
  target_pose0.position.z = 0.159414;}
break;
  case 51:
ROS_INFO("region number is 51");
  {target_pose0.orientation.x = -0.682886;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.065263;
  target_pose0.position.y = 0.877016;
  target_pose0.position.z = 0.159414;}
break;
 case 52:
ROS_INFO("region number is 52");
  {target_pose0.orientation.x = -0.682886;
  target_pose0.orientation.y = 0.000533;
  target_pose0.orientation.z = -0.686147;
  target_pose0.orientation.w = 0.006733;
  target_pose0.position.x = 0.065263;
  target_pose0.position.y =0.877016;
  target_pose0.position.z = 0.159414;}
break;
 
 }
////////////////////////////////////////////////////////////////////////////////////////////////
  
  waypoints.push_back(target_pose0);
  target = target_pose0;
  ROS_INFO("START TO SEND TARGET");
  std_msgs::Time squeeze_time;
  squeeze_time.data=ros::Time::now();
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

//  while(TargetCheck(target_pose0) == false);
  sleep(2);
  robo_arm();

}

  void cv_position_Cb(const geometry_msgs::Twist& cv_p)
  { 
    int number=0;

    ROS_INFO("cv_p.linear.x is %.2f,cv_p.linear.y is %.2f)",cv_p.linear.x,cv_p.linear.y);

    if ((cv_p.linear.x>26.5)&&(cv_p.linear.x<74.5)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42.5))
    {
      number=49;
    }
  else if ((cv_p.linear.x>27.5)&&(cv_p.linear.x<77)&&(cv_p.linear.y>42.5)&&(cv_p.linear.y<87.5))
    {
      number=50;
    } 
  else if ((cv_p.linear.x>28.5)&&(cv_p.linear.x<77)&&(cv_p.linear.y>88.5)&&(cv_p.linear.y<132))
    {
      number=51;
    }
  else if ((cv_p.linear.x>29.5)&&(cv_p.linear.x<81.5)&&(cv_p.linear.y>132)&&(cv_p.linear.y<174))
    {
      number=52;
    }
  else if ((cv_p.linear.x>75)&&(cv_p.linear.x<118)&&(cv_p.linear.y>10)&&(cv_p.linear.y<43))
    {
      number=45;
    }
  else if ((cv_p.linear.x>78)&&(cv_p.linear.x<122.5)&&(cv_p.linear.y>42.5)&&(cv_p.linear.y<88))
    {
      number=46;
    }
  else if ((cv_p.linear.x>79)&&(cv_p.linear.x<122.5)&&(cv_p.linear.y>87.5)&&(cv_p.linear.y<128))
    {
      number=47;
    }
  else if ((cv_p.linear.x>81.5)&&(cv_p.linear.x<124.5)&&(cv_p.linear.y>132)&&(cv_p.linear.y<174))
    {
      number=48;
    }
  else if ((cv_p.linear.x>118)&&(cv_p.linear.x<162.5)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42.5))
    {
      number=41;
    }
  else if ((cv_p.linear.x>121)&&(cv_p.linear.x<165)&&(cv_p.linear.y>43)&&(cv_p.linear.y<88.5))
    {
      number=42;
    }
else if ((cv_p.linear.x>122.5)&&(cv_p.linear.x<166)&&(cv_p.linear.y>88.5)&&(cv_p.linear.y<132))
    {
      number=43;
    } 
  else if ((cv_p.linear.x>124.5)&&(cv_p.linear.x<166)&&(cv_p.linear.y>132)&&(cv_p.linear.y<173))
    {
      number=44;
    }
  else if ((cv_p.linear.x>162.5)&&(cv_p.linear.x<204)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=37;
    }
  else if ((cv_p.linear.x>165)&&(cv_p.linear.x<209)&&(cv_p.linear.y>42)&&(cv_p.linear.y<87.5))
    {
      number=38;
    }
  else if ((cv_p.linear.x>166)&&(cv_p.linear.x<209)&&(cv_p.linear.y>87.5)&&(cv_p.linear.y<131))
    {
      number=39;
    }
  else if ((cv_p.linear.x>166.5)&&(cv_p.linear.x<209.5)&&(cv_p.linear.y>131)&&(cv_p.linear.y<173))
    {
      number=40;
    }
  else if ((cv_p.linear.x>207)&&(cv_p.linear.x<251)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=33;
    }
  else if ((cv_p.linear.x>208.5)&&(cv_p.linear.x<251.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<86.5))
    {
      number=34;
    }
  else if ((cv_p.linear.x>209)&&(cv_p.linear.x<252)&&(cv_p.linear.y>86.5)&&(cv_p.linear.y<130.5))
    {
      number=35;
    }
else if ((cv_p.linear.x>209.5)&&(cv_p.linear.x<252.5)&&(cv_p.linear.y>130.5)&&(cv_p.linear.y<172.5))
    {
      number=36;
    }
  else if ((cv_p.linear.x>251)&&(cv_p.linear.x<294.5)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=29;
    }
  else if ((cv_p.linear.x>251.5)&&(cv_p.linear.x<294.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<86.5))
    {
      number=30;
    }
  else if ((cv_p.linear.x>252)&&(cv_p.linear.x<295)&&(cv_p.linear.y>86.5)&&(cv_p.linear.y<130.5))
    {
      number=31;
    }
  else if ((cv_p.linear.x>252.5)&&(cv_p.linear.x<294.5)&&(cv_p.linear.y>130.5)&&(cv_p.linear.y<173.5))
    {
      number=32;
    }
  else if ((cv_p.linear.x>294.5)&&(cv_p.linear.x<338)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=25;
    }
  else if ((cv_p.linear.x>294.5)&&(cv_p.linear.x<337.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<86))
    {
      number=26;
    }
  else if ((cv_p.linear.x>295)&&(cv_p.linear.x<337.5)&&(cv_p.linear.y>86)&&(cv_p.linear.y<129.5))
    {
      number=27;
    }
 else if ((cv_p.linear.x>294.5)&&(cv_p.linear.x<336.5)&&(cv_p.linear.y>129.5)&&(cv_p.linear.y<172.5))
    {
      number=28;
    }
  else if ((cv_p.linear.x>338)&&(cv_p.linear.x<381)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=21;
    }
  else if ((cv_p.linear.x>337.5)&&(cv_p.linear.x<380.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<85.5))
    {
      number=22;
    }
  else if ((cv_p.linear.x>337.5)&&(cv_p.linear.x<380.5)&&(cv_p.linear.y>85.5)&&(cv_p.linear.y<129.5))
    {
      number=23;
    }
else if ((cv_p.linear.x>336.5)&&(cv_p.linear.x<379.5)&&(cv_p.linear.y>129.5)&&(cv_p.linear.y<172.5))
    {
      number=24;
    }
  else if ((cv_p.linear.x>381)&&(cv_p.linear.x<424)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=17;
    }
  else if ((cv_p.linear.x>380.5)&&(cv_p.linear.x<423.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<85.5))
    {
      number=18;
    }
 else if ((cv_p.linear.x>380.5)&&(cv_p.linear.x<423.5)&&(cv_p.linear.y>85.5)&&(cv_p.linear.y<128.5))
    {
      number=19;
    }
  else if ((cv_p.linear.x>379.5)&&(cv_p.linear.x<422.5)&&(cv_p.linear.y>128.5)&&(cv_p.linear.y<173))
    {
      number=20;
    }
  else if ((cv_p.linear.x>424)&&(cv_p.linear.x<467)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=13;
    }
  else if ((cv_p.linear.x>423.5)&&(cv_p.linear.x<466.5)&&(cv_p.linear.y>42)&&(cv_p.linear.y<85.5))
    {
      number=14;
    }
else if ((cv_p.linear.x>423.5)&&(cv_p.linear.x<466.5)&&(cv_p.linear.y>85.5)&&(cv_p.linear.y<128.5))
    {
      number=15;
    }
else if ((cv_p.linear.x>422.5)&&(cv_p.linear.x<379.5)&&(cv_p.linear.y>129.5)&&(cv_p.linear.y<172.5))
    {
      number=16;
    }
  else if ((cv_p.linear.x>467)&&(cv_p.linear.x<511)&&(cv_p.linear.y>10)&&(cv_p.linear.y<41.5))
    {
      number=9;
    }
  else if ((cv_p.linear.x>466.5)&&(cv_p.linear.x<510)&&(cv_p.linear.y>41.5)&&(cv_p.linear.y<85))
    {
      number=10;
    }
 else if ((cv_p.linear.x>466.5)&&(cv_p.linear.x<510)&&(cv_p.linear.y>85)&&(cv_p.linear.y<128.5))
    {
      number=11;
    }
  else if ((cv_p.linear.x>465.5)&&(cv_p.linear.x<509)&&(cv_p.linear.y>128.5)&&(cv_p.linear.y<171.5))
    {
      number=12;
    }
  else if ((cv_p.linear.x>511)&&(cv_p.linear.x<555.5)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=5;
    }
  else if ((cv_p.linear.x>510)&&(cv_p.linear.x<554)&&(cv_p.linear.y>42)&&(cv_p.linear.y<85.5))
    {
      number=6;
    }
else if ((cv_p.linear.x>509)&&(cv_p.linear.x<553)&&(cv_p.linear.y>85.5)&&(cv_p.linear.y<128.5))
    {
      number=7;
    }
  else if ((cv_p.linear.x>509)&&(cv_p.linear.x<552.5)&&(cv_p.linear.y>128.5)&&(cv_p.linear.y<171.5))
    {
      number=8;
    }
 else if ((cv_p.linear.x>555.5)&&(cv_p.linear.x<598.5)&&(cv_p.linear.y>10)&&(cv_p.linear.y<42))
    {
      number=1;
    }
  else if ((cv_p.linear.x>554)&&(cv_p.linear.x<598)&&(cv_p.linear.y>42)&&(cv_p.linear.y<85.5))
    {
      number=2;
    }
  else if ((cv_p.linear.x>552.5)&&(cv_p.linear.x<597.5)&&(cv_p.linear.y>85.5)&&(cv_p.linear.y<128.5))
    {
      number=3;
    }
  else if ((cv_p.linear.x>552)&&(cv_p.linear.x<595)&&(cv_p.linear.y>128.5)&&(cv_p.linear.y<171.5))
    {
      number=4;
    }
else {
      number=49;
    }

    //////get the path to the target poistion
    getTargetPosition(number);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_grid");
  ros::NodeHandle n;
  GRIPCV grip;

  ros::spin();
  

  return 0;
}
