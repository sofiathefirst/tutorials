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
    hand_pub = nh_.advertise<std_msgs::UInt16>("servo", 1000);

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
    float fraction = 0.0;
    while(fraction != 1.0)
    {
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose;
    start_pose = group_ptr->getCurrentPose().pose;

    waypoints.push_back(start_pose);
 
    geometry_msgs::Pose target_pose1, target_pose2, temp;
//  waypoints.push_back(target_pose1);
//[0.281888,0.663603,-0.306005,0.621716,0.590112,-0.483928,0.195574]
    target_pose1.orientation.x = -0.239494;
    target_pose1.orientation.y = 0.682661;
    target_pose1.orientation.z = 0.220907;
    target_pose1.orientation.w = 0.654077;
    target_pose1.position.x = 0.590112;
    target_pose1.position.y = -0.483928;
    target_pose1.position.z = 0.195574;

    waypoints.push_back(target_pose1);

 /*   waypoints.push_back(target_pose1);
    target_pose2.orientation.x = -0.239494;
    target_pose2.orientation.y = 0.682661;
    target_pose2.orientation.z = 0.220907;
    target_pose2.orientation.w = 0.654077;
    target_pose2.position.x = 0.435729;
    target_pose2.position.y = 0.472243;
    target_pose2.position.z = 0.240597;*/

    waypoints.push_back(target_pose1);
    temp = start_pose;
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
   }
   ROS_INFO("finish robot_arm");
  }

  void getTargetPosition(int number)
  {
    ROS_INFO("number is %d)",number);
if (number==49)
{    ros::shutdown();}        //zliu7
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    geometry_msgs::Pose start_pose, target_pose0, target_pose1;
 /*   target_pose0.orientation.x = 0.727432;
    target_pose0.orientation.y = 0.000533;
    target_pose0.orientation.z = -0.686147;
    target_pose0.orientation.w = 0.006733;
    target_pose0.position.x = 0.052658;
    target_pose0.position.y = 0.624502;
    target_pose0.position.z = 0.359414;

    waypoints.push_back(target_pose0);*/
/////////////////////////pick_position_choices///////////////////////////////////////////////
   switch(number)
   {
    case 1:
    ROS_INFO("region number is 1");
     {
       target_pose0.orientation.x = 0.017606;
       target_pose0.orientation.y = 0.721895;
       target_pose0.orientation.z = -0.040219;
       target_pose0.orientation.w = 0.690608;
       target_pose0.position.x = 0.622394;
       target_pose0.position.y = 0.051074;
       target_pose0.position.z =0.138765;
     }
      break;
    case 2:
    ROS_INFO("region number is 2");
     {
        target_pose0.orientation.x = -0.000308;
        target_pose0.orientation.y = 0.689792;
        target_pose0.orientation.z = 0.006013;
        target_pose0.orientation.w = 0.723983;
        target_pose0.position.x = 0.683153;
        target_pose0.position.y = 0.110889;
        target_pose0.position.z = 0.146892;}
     break;
    case 3:
    ROS_INFO("region number is 3");
     {
       target_pose0.orientation.x = -0.062993;
       target_pose0.orientation.y = 0.719107;
       target_pose0.orientation.z = 0.036957;
       target_pose0.orientation.w = 0.691051;
       target_pose0.position.x = 0.756176;
       target_pose0.position.y = 0.209319;
       target_pose0.position.z = 0.141655;}
    break;
    case 4:
    ROS_INFO("region number is 4");
    {
      target_pose0.orientation.x = -0.041898;
      target_pose0.orientation.y = 0.716617;
      target_pose0.orientation.z = 0.078224;
      target_pose0.orientation.w = 0.691799;
      target_pose0.position.x = 0.811960;
      target_pose0.position.y = 0.249008;
      target_pose0.position.z = 0.147170;}
    break;
  case 5:
   ROS_INFO("region number is 5");
   {
     target_pose0.orientation.x = 0.008503;
     target_pose0.orientation.y = 0.698488;
     target_pose0.orientation.z = 0.033720;
     target_pose0.orientation.w = 0.714776;
     target_pose0.position.x = 0.530499;
     target_pose0.position.y = 0.128567;
     target_pose0.position.z = 0.164670;}
   break;
  case 6:
   ROS_INFO("region number is 6");
   {
     target_pose0.orientation.x = -0.028693;
     target_pose0.orientation.y = 0.719506;
     target_pose0.orientation.z = 0.061284;
     target_pose0.orientation.w = 0.691181;
     target_pose0.position.x = 0.622346;
     target_pose0.position.y = 0.190045;
     target_pose0.position.z = 0.146546;}
   break;
  case 7:
   ROS_INFO("region number is 7");
   {
     target_pose0.orientation.x = -0.051610;
     target_pose0.orientation.y = 0.726757;
     target_pose0.orientation.z =0.081701;
     target_pose0.orientation.w = 0.680063;
     target_pose0.position.x = 0.681262;
     target_pose0.position.y = 0.241718;
     target_pose0.position.z = 0.137118;}
    break;
  case 8:
   ROS_INFO("region number is 8");
   {
     target_pose0.orientation.x = -0.080201;
     target_pose0.orientation.y = 0.698413;
     target_pose0.orientation.z = 0.115162;
     target_pose0.orientation.w = 0.701801;
     target_pose0.position.x = 0.751599;
     target_pose0.position.y = 0.325651;
     target_pose0.position.z = 0.149686;}
   break;
  case 9:
   ROS_INFO("region number is 9");
   {
     target_pose0.orientation.x = -0.022938;
     target_pose0.orientation.y = 0.727782;
     target_pose0.orientation.z = 0.083244;
     target_pose0.orientation.w = 0.680351;
     target_pose0.position.x = 0.484826;
     target_pose0.position.y = 0.189406;
     target_pose0.position.z = 0.166061;}
   break;
   case 10:
   ROS_INFO("region number is 10");
   {
     target_pose0.orientation.x = -0.083571;
     target_pose0.orientation.y = 0.702377;
     target_pose0.orientation.z = 0.103018;
     target_pose0.orientation.w = 0.699336;
     target_pose0.position.x = 0.543319;
     target_pose0.position.y = 0.258628;
     target_pose0.position.z = 0.140065;}
   break;
   case 11:
  ROS_INFO("region number is 11");
   {
     target_pose0.orientation.x =-0.096969;
     target_pose0.orientation.y =0.703584;
     target_pose0.orientation.z = 0.115544;
     target_pose0.orientation.w = 0.694418;
     target_pose0.position.x = 0.633569;
     target_pose0.position.y = 0.309136;
     target_pose0.position.z = 0.165682;}
   break;

   case 12:
   ROS_INFO("region number is 12");
   {
     target_pose0.orientation.x = -0.135552;
     target_pose0.orientation.y = 0.676124;
     target_pose0.orientation.z = 0.147202;
     target_pose0.orientation.w = 0.709093;
     target_pose0.position.x = 0.681308;
     target_pose0.position.y = 0.403646;
     target_pose0.position.z = 0.176508;}
   break;

   case 13:
   ROS_INFO("region number is 13");
   {
     target_pose0.orientation.x = -0.090387;
     target_pose0.orientation.y = 0.668568;
     target_pose0.orientation.z = 0.134044;
     target_pose0.orientation.w = 0.725865;
     target_pose0.position.x = 0.396600;
     target_pose0.position.y = 0.245752;
     target_pose0.position.z = 0.160538;}
   break;

   case 14:
   ROS_INFO("region number is 14");
   {
     target_pose0.orientation.x = -0.134670;
     target_pose0.orientation.y = 0.701841;
     target_pose0.orientation.z = 0.164944;
     target_pose0.orientation.w = 0.679762;
     target_pose0.position.x = 0.478982;
     target_pose0.position.y = 0.338452;
     target_pose0.position.z = 0.170828;}
   break;
   case 15:
   ROS_INFO("region number is 15");
   {
     target_pose0.orientation.x = -0.135360;
     target_pose0.orientation.y =0.681858;
     target_pose0.orientation.z = 0.173761;
     target_pose0.orientation.w = 0.697535;
     target_pose0.position.x =0.548798;
     target_pose0.position.y = 0.378682;
     target_pose0.position.z = 0.169670;}
   break;
   case 16:
   ROS_INFO("region number is 16");
   {
     target_pose0.orientation.x = -0.183270;
     target_pose0.orientation.y = 0.683369;
     target_pose0.orientation.z = 0.171887;
     target_pose0.orientation.w = 0.685473;
     target_pose0.position.x = 0.628707;
     target_pose0.position.y = 0.466752;
     target_pose0.position.z = 0.164659;}
    break;
   case 17:
   ROS_INFO("region number is 17");
   {
     target_pose0.orientation.x = -0.203423;
     target_pose0.orientation.y = 0.660045;
     target_pose0.orientation.z = 0.188993;
     target_pose0.orientation.w = 0.698026;
     target_pose0.position.x = 0.329286;
     target_pose0.position.y = 0.330217;
     target_pose0.position.z = 0.154499;}
   break;
   case 18:
   ROS_INFO("region number is 18");
   {
     target_pose0.orientation.x = -0.217742;
     target_pose0.orientation.y = 0.664887;
     target_pose0.orientation.z = 0.199424;
     target_pose0.orientation.w = 0.686108;
     target_pose0.position.x = 0.404510;
     target_pose0.position.y = 0.400332;
     target_pose0.position.z = 0.156642;}
   break;
   case 19:
   ROS_INFO("region number is 19");
   {
     target_pose0.orientation.x = -0.225417;
     target_pose0.orientation.y = 0.657818;
     target_pose0.orientation.z = 0.209370;
     target_pose0.orientation.w = 0.687479;
     target_pose0.position.x = 0.466098;
     target_pose0.position.y = 0.462669;
     target_pose0.position.z = 0.175036;}
   break;
   case 20:
   ROS_INFO("region number is 20");
   {
     target_pose0.orientation.x = -0.237129;
     target_pose0.orientation.y = 0.655474;
     target_pose0.orientation.z = 0.224023;
     target_pose0.orientation.w = 0.681129;
     target_pose0.position.x = 0.530535;
     target_pose0.position.y = 0.544664;
     target_pose0.position.z = 0.175801;}
   break;
   case 21:
   ROS_INFO("region number is 21");
  {
     target_pose0.orientation.x = -0.288621;
     target_pose0.orientation.y = 0.651572;
     target_pose0.orientation.z = 0.262069;
     target_pose0.orientation.w = 0.650747;
     target_pose0.position.x = 0.250190;
     target_pose0.position.y =0.405635;
     target_pose0.position.z = 0.145628;}
   break;

   case 22:
   ROS_INFO("region number is 22");
   {
     target_pose0.orientation.x = -0.287163;
     target_pose0.orientation.y = 0.650286;
     target_pose0.orientation.z = 0.265466;
     target_pose0.orientation.w = 0.651301;
     target_pose0.position.x = 0.324561;
     target_pose0.position.y = 0.483913;
     target_pose0.position.z = 0.157352;}
   break;
   case 23:
   ROS_INFO("region number is 23");
  {target_pose0.orientation.x = -0.267627;
  target_pose0.orientation.y = 0.683796;
  target_pose0.orientation.z = 0.265323;
  target_pose0.orientation.w = 0.624822;
  target_pose0.position.x = 0.406132;
  target_pose0.position.y = 0.547112;
  target_pose0.position.z =0.148642;}
break;
   case 24:
ROS_INFO("region number is 24");
  {target_pose0.orientation.x = -0.262103;
  target_pose0.orientation.y = 0.669401;
  target_pose0.orientation.z = 0.271510;
  target_pose0.orientation.w = 0.639911;
  target_pose0.position.x = 0.478560;
  target_pose0.position.y = 0.616933;
  target_pose0.position.z = 0.141760;}
break;
 case 25:
ROS_INFO("region number is 25");
  {target_pose0.orientation.x = -0.719396;
  target_pose0.orientation.y = 0.072774;
  target_pose0.orientation.z = 0.686422;
  target_pose0.orientation.w = 0.077448;
  target_pose0.position.x = 0.192330;
  target_pose0.position.y = 0.476704;
  target_pose0.position.z = 0.097779;}
break;
 case 26:
ROS_INFO("region number is 26");
  {target_pose0.orientation.x = -0.695196;
  target_pose0.orientation.y = 0.130947;
  target_pose0.orientation.z = 0.696017;
  target_pose0.orientation.w = 0.122942;
  target_pose0.position.x = 0.293303;
  target_pose0.position.y = 0.555890;;
  target_pose0.position.z = 0.121671;}
break;
 case 27:
ROS_INFO("region number is 27");
  {target_pose0.orientation.x =-0.708267;
  target_pose0.orientation.y = 0.140587;
  target_pose0.orientation.z = 0.678053;
  target_pose0.orientation.w = 0.137252;
  target_pose0.position.x = 0.340604;
  target_pose0.position.y = 0.613533;
  target_pose0.position.z = 0.113691;}
break;
 case 28:
ROS_INFO("region number is 28");
  {target_pose0.orientation.x = -0.698795;
  target_pose0.orientation.y = 0.165534;
  target_pose0.orientation.z = 0.679060;
  target_pose0.orientation.w = 0.152192;
  target_pose0.position.x = 0.413686;
  target_pose0.position.y = 0.684245;
  target_pose0.position.z = 0.107767;}
break;
 case 29:
ROS_INFO("region number is 29");
  {target_pose0.orientation.x = -0.700975;
  target_pose0.orientation.y = 0.027664;
  target_pose0.orientation.z = 0.710571;
  target_pose0.orientation.w = 0.054375;
  target_pose0.position.x = 0.143463;
  target_pose0.position.y = 0.527480;
  target_pose0.position.z =0.109206;}
break;
  case 30:
ROS_INFO("region number is 30");
  {target_pose0.orientation.x = -0.699677;
  target_pose0.orientation.y = 0.061166;
  target_pose0.orientation.z = 0.706505;
  target_pose0.orientation.w = 0.086963;
  target_pose0.position.x = 0.206584;
  target_pose0.position.y =0.600496;
  target_pose0.position.z = 0.113070;}
break;
 case 31:
ROS_INFO("region number is 31");
  {target_pose0.orientation.x = -0.680342;
  target_pose0.orientation.y = 0.115092;
  target_pose0.orientation.z = 0.715887;
  target_pose0.orientation.w = 0.106748;
  target_pose0.position.x = 0.292841;
  target_pose0.position.y =0.678232;
  target_pose0.position.z = 0.103362;}
break;

     case 32:
ROS_INFO("region number is 32");
  {target_pose0.orientation.x = -0.705401;
  target_pose0.orientation.y = 0.124833;
  target_pose0.orientation.z = 0.687207;
  target_pose0.orientation.w = 0.120712;
  target_pose0.position.x = 0.347839;
  target_pose0.position.y =0.751361;
  target_pose0.position.z = 0.107018;}
break;
 case 33:
ROS_INFO("region number is 33");
  {target_pose0.orientation.x = -0.710903;
  target_pose0.orientation.y = -0.016698;
  target_pose0.orientation.z = 0.703079;
  target_pose0.orientation.w = 0.004257;
  target_pose0.position.x = 0.069197;
  target_pose0.position.y = 0.606241;
  target_pose0.position.z = 0.097372;}
break;
   case 34:
ROS_INFO("region number is 34");
  {target_pose0.orientation.x = -0.713868;
  target_pose0.orientation.y = 0.017645;
  target_pose0.orientation.z = 0.698605;
  target_pose0.orientation.w = 0.045088;
  target_pose0.position.x = 0.136843;
  target_pose0.position.y =0.669640;
  target_pose0.position.z = 0.097149;}
break;
 case 35:
ROS_INFO("region number is 35");
  {target_pose0.orientation.x = -0.698614;
  target_pose0.orientation.y = 0.062754;
  target_pose0.orientation.z = 0.709343;
  target_pose0.orientation.w = 0.069513;
  target_pose0.position.x =0.209669;
  target_pose0.position.y = 0.732384;
  target_pose0.position.z = 0.097321;}
break;
 case 36:
ROS_INFO("region number is 36");
  {target_pose0.orientation.x = -0.695533;
  target_pose0.orientation.y = 0.064708;
  target_pose0.orientation.z = 0.705778;
  target_pose0.orientation.w = 0.117999;
  target_pose0.position.x = 0.281249;
  target_pose0.position.y = 0.804326;
  target_pose0.position.z = 0.100363;}
break;
 case 37:
ROS_INFO("region number is 37");
  {target_pose0.orientation.x = 0.719109;
  target_pose0.orientation.y = 0.041243;
  target_pose0.orientation.z = -0.692045;
  target_pose0.orientation.w = 0.047493;
  target_pose0.position.x = -0.006127;
  target_pose0.position.y = 0.677538;
  target_pose0.position.z = 0.093670;}
break;
 case 38:
ROS_INFO("region number is 38");
  {target_pose0.orientation.x = 0.714851;
  target_pose0.orientation.y = 0.041243;
  target_pose0.orientation.z = -0.699250;
  target_pose0.orientation.w = 0.004077;
  target_pose0.position.x = 0.064760;
  target_pose0.position.y = 0.745914;
  target_pose0.position.z = 0.105250;}
break;
 case 39:
ROS_INFO("region number is 39");
//[0.616985,0.386698,-0.554745,0.402556,-0.137390,-0.714642,0.290678]
  {target_pose0.orientation.x = -0.712032;
  target_pose0.orientation.y = 0.016184;
  target_pose0.orientation.z = 0.700758;
  target_pose0.orientation.w = 0.041061;
  target_pose0.position.x = 0.135648;
  target_pose0.position.y = 0.816494;
  target_pose0.position.z = 0.107962;}
break;
  case 40:
ROS_INFO("region number is 40");
  {target_pose0.orientation.x = -0.691781;
  target_pose0.orientation.y = 0.026419;
  target_pose0.orientation.z = 0.715844;
  target_pose0.orientation.w = 0.091149;
  target_pose0.position.x = 0.210059;
  target_pose0.position.y = 0.869114;
  target_pose0.position.z =0.108533;}
break;
 case 41:
ROS_INFO("region number is 41");
  {target_pose0.orientation.x = 0.684155;
  target_pose0.orientation.y = 0.058006;
  target_pose0.orientation.z = -0.724181;
  target_pose0.orientation.w = 0.064254;
  target_pose0.position.x = -0.063476;
  target_pose0.position.y = 0.749891;
  target_pose0.position.z = 0.111125;}
break;

     case 42:
ROS_INFO("region number is 42");
  {target_pose0.orientation.x = 0.679160;
  target_pose0.orientation.y = 0.015696;
  target_pose0.orientation.z = -0.733235;
  target_pose0.orientation.w = 0.029358;
  target_pose0.position.x = 0.008754;
  target_pose0.position.y = 0.820741;
  target_pose0.position.z = 0.108781;}
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

 
 }
////////////////////////////////////////////////////////////////////////////////////////////////
  float fraction = 0.0;
  while(fraction != 1.0)
  { 
    std::vector<geometry_msgs::Pose> waypoints;

    start_pose = group_ptr->getCurrentPose().pose;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose0);
    target = target_pose0;
    ROS_INFO("START TO SEND TARGET");
    geometry_msgs::Pose temp = start_pose;
    std_msgs::Time squeeze_time;
    squeeze_time.data=ros::Time::now();
    while(ros::Time::now() - squeeze_time.data < ros::Duration(1))
    {
      tar_state_pub.publish(target);
    }
    ROS_INFO("START TO COMPUTING PATH");

    fraction = group_ptr->computeCartesianPath(waypoints,
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
//  while(TargetCheck(target_pose0) == false);
  robo_arm();

}

  void cv_position_Cb(const geometry_msgs::Twist& cv_p)
  { 
    int number=0;

    ROS_INFO("cv_p.linear.x is %.2f,cv_p.linear.y is %.2f)",cv_p.linear.x,cv_p.linear.y);

    if ((cv_p.linear.x>35.5)&&(cv_p.linear.x<98)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=1;
    }
  else if ((cv_p.linear.x>35.5)&&(cv_p.linear.x<98)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=2;
    } 
  else if ((cv_p.linear.x>35.5)&&(cv_p.linear.x<98)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=3;
    }
  else if ((cv_p.linear.x>35.5)&&(cv_p.linear.x<98)&&(cv_p.linear.y>186)&&(cv_p.linear.y<237))
    {
      number=4;
    }
  else if ((cv_p.linear.x>98)&&(cv_p.linear.x<152)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=5;
    }
  else if ((cv_p.linear.x>98)&&(cv_p.linear.x<152)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=6;
    }
  else if ((cv_p.linear.x>98)&&(cv_p.linear.x<152)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=7;
    }
  else if ((cv_p.linear.x>98)&&(cv_p.linear.x<152)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=8;
    }
  else if ((cv_p.linear.x>152)&&(cv_p.linear.x<202)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=9;
    }
  else if ((cv_p.linear.x>152)&&(cv_p.linear.x<202)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=10;
    }
else if ((cv_p.linear.x>152)&&(cv_p.linear.x<202)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=11;
    } 
  else if ((cv_p.linear.x>152)&&(cv_p.linear.x<202)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=12;
    }
  else if ((cv_p.linear.x>202)&&(cv_p.linear.x<254)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=13;
    }
  else if ((cv_p.linear.x>202)&&(cv_p.linear.x<254)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=14;
    }
  else if ((cv_p.linear.x>202)&&(cv_p.linear.x<254)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=15;
    }
  else if ((cv_p.linear.x>202)&&(cv_p.linear.x<254)&&(cv_p.linear.y>186)&&(cv_p.linear.y<254))
    {
      number=16;
    }
  else if ((cv_p.linear.x>254)&&(cv_p.linear.x<304)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=17;
    }
  else if ((cv_p.linear.x>254)&&(cv_p.linear.x<304)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=18;
    }
  else if ((cv_p.linear.x>254)&&(cv_p.linear.x<304)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=19;
    }
else if ((cv_p.linear.x>254)&&(cv_p.linear.x<304)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=20;
    }
  else if ((cv_p.linear.x>304)&&(cv_p.linear.x<355)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=21;
    }
  else if ((cv_p.linear.x>304)&&(cv_p.linear.x<355)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=22;
    }
  else if ((cv_p.linear.x>304)&&(cv_p.linear.x<355)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=23;
    }
  else if ((cv_p.linear.x>304)&&(cv_p.linear.x<355)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=24;
    }
  else if ((cv_p.linear.x>355)&&(cv_p.linear.x<405)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=25;
    }
  else if ((cv_p.linear.x>355)&&(cv_p.linear.x<405)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=26;
    }
  else if ((cv_p.linear.x>355)&&(cv_p.linear.x<405)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=27;
    }
 else if ((cv_p.linear.x>355)&&(cv_p.linear.x<405)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=28;
    }
  else if ((cv_p.linear.x>405)&&(cv_p.linear.x<454)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=29;
    }
  else if ((cv_p.linear.x>405)&&(cv_p.linear.x<454)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=30;
    }
  else if ((cv_p.linear.x>405)&&(cv_p.linear.x<454)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=31;
    }
else if ((cv_p.linear.x>405)&&(cv_p.linear.x<454)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=32;
    }
  else if ((cv_p.linear.x>454)&&(cv_p.linear.x<505)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=33;
    }
  else if ((cv_p.linear.x>454)&&(cv_p.linear.x<505)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=34;
    }
 else if ((cv_p.linear.x>454)&&(cv_p.linear.x<505)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=35;
    }
  else if ((cv_p.linear.x>454)&&(cv_p.linear.x<505)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=36;
    }
  else if ((cv_p.linear.x>505)&&(cv_p.linear.x<551)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=37;
    }
  else if ((cv_p.linear.x>505)&&(cv_p.linear.x<551)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=38;
    }
else if ((cv_p.linear.x>505)&&(cv_p.linear.x<551)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=39;
    }
else if ((cv_p.linear.x>505)&&(cv_p.linear.x<551)&&(cv_p.linear.y>186)&&(cv_p.linear.y<238))
    {
      number=40;
    }
  else if ((cv_p.linear.x>551)&&(cv_p.linear.x<600)&&(cv_p.linear.y>29)&&(cv_p.linear.y<83))
    {
      number=41;
    }
  else if ((cv_p.linear.x>551)&&(cv_p.linear.x<600)&&(cv_p.linear.y>83)&&(cv_p.linear.y<134))
    {
      number=42;
    }
 else if ((cv_p.linear.x>551)&&(cv_p.linear.x<600)&&(cv_p.linear.y>134)&&(cv_p.linear.y<186))
    {
      number=43;
    }
else {
      number=1;
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
