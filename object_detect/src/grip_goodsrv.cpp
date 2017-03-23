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
#include <object_detect/positionPose.h>

//author: Liu zhichao
//Usage: pick up "transformer" accurately.

//added by zhang yumei, for positions,181 positions.
//from switch codes to hash table find code, 
//0 point is the box position, 
//when the thing position is unachievble, we move to the box positon.
const float POINTS[][3]={
 0.588878, -0.484398, 0.095880,//0
 0.656776, -0.016534, 0.110638,//1
 0.676042, 0.039362, 0.093840,//2
 0.718261, 0.070278, 0.100132,//3
 0.743404, 0.106311, 0.090522,//4
 0.785288, 0.130807, 0.102340,//5
 0.825679, 0.176209, 0.113824,//6
 0.850681, 0.216154, 0.103387,//7
 0.868365, 0.267477, 0.095925,//8
 0.630194, 0.035903, 0.100658,//9
 0.622394, 0.051074,0.095324,//10
 0.679610, 0.123274, 0.095687,//11
 0.683153, 0.110889, 0.095324,//12
 0.774907, 0.158308, 0.102349,//13
 0.756176, 0.209319, 0.095324,//14
 0.813622, 0.277232, 0.096204,//15
 0.811960, 0.249008, 0.095324,//16
 0.586670, 0.079813, 0.093409,//17
 0.619266, 0.103402, 0.105437,//18
 0.636589, 0.141578, 0.095488,//19
 0.684276, 0.187466, 0.099027,//20
 0.709662, 0.221300, 0.088576,//21
 0.753408, 0.267930, 0.101802,//22
 0.787142, 0.296117, 0.085853,//23
 0.806699, 0.298644, 0.092277,//24
 0.547543, 0.096584, 0.097634,//25
 0.530499, 0.128567, 0.095324,//26
 0.618321, 0.186724, 0.096335,//27
 0.622346, 0.190045, 0.095324,//28
 0.676658, 0.260742, 0.090924,//29
 0.681262, 0.241718, 0.095324,//30
 0.745143, 0.337609, 0.117562,//31
 0.751599, 0.325651, 0.095324,//32
 0.510971, 0.135610, 0.111213,//33
 0.531349, 0.177002, 0.094448,//34
 0.582033, 0.213706, 0.099027,//35
 0.614309, 0.235583, 0.094612,//36
 0.648153, 0.275469, 0.104345,//37
 0.673921, 0.319432, 0.104194,//38
 0.712426, 0.351783, 0.138943,//39
 0.752049, 0.398500, 0.108082,//40
 0.472297, 0.160819, 0.107759,//41
 0.484826, 0.189406, 0.095324,//42
 0.548792, 0.259657, 0.117729,//43
 0.543319, 0.258628, 0.095324,//44
 0.592172, 0.325895, 0.093183,//45
 0.633569, 0.309136, 0.095324,//46
 0.681093, 0.408614, 0.113941,//47
 0.681308, 0.403646, 0.095324,//48
 0.442519, 0.207419, 0.101962,//49
 0.486515 , 0.232730, 0.104489,//50
 0.524884, 0.289815, 0.130716,//51
 0.541577,0.314560, 0.104423,//52
 0.578816, 0.349608, 0.093136,//53
 0.613627, 0.393022, 0.103637,//54
 0.642419, 0.426652, 0.100363,//55
 0.685012, 0.464838, 0.112814,//56
 0.398298, 0.251750, 0.097802,//57
 0.396600, 0.245752, 0.095324,//58
 0.473158, 0.324051, 0.107171,//59
 0.478982, 0.338452, 0.095324,//60
 0.539990, 0.395708, 0.109443,//61
0.548798, 0.378682, 0.095324,//62
 0.606155, 0.460532, 0.101206,//63
 0.628707, 0.466752, 0.095324,//64
 0.384883, 0.269441, 0.116869,//65
 0.405204, 0.314641, 0.120700,//66
 0.440418, 0.352902, 0.107078,//67
 0.467730, 0.390086, 0.102558,//68
 0.510590, 0.423858, 0.094530,//69
 0.538380, 0.460282, 0.095164,//70
 0.571244, 0.502761, 0.109293,//71
 0.598476, 0.532528, 0.120936,//72
 0.333838, 0.317362, 0.123075,//73
 0.329286, 0.330217, 0.095324,//74
 0.399398, 0.379880, 0.111782,//75
 0.404510, 0.400332, 0.095324,//76
 0.465872, 0.452621, 0.101593,//77
 0.466098, 0.462669, 0.095324,//78
 0.530675, 0.522973, 0.111618,//79
 0.530535, 0.544664, 0.095324,//80
 0.306145, 0.349984, 0.093180,//81
 0.340825, 0.375005, 0.107009,//82
 0.374182, 0.415389, 0.101704,//83
 0.402729, 0.455845, 0.109166,//84
 0.427571, 0.485528, 0.106693,//85
 0.474569, 0.532051, 0.115295,//86
 0.507748, 0.558386, 0.107127,//87
 0.533602, 0.603313, 0.099691,//88
 0.264770, 0.381431, 0.102866,//89
 0.250190,0.405635, 0.095324,//90
 0.331442, 0.451519, 0.111750,//91
 0.324561, 0.483913, 0.095324,//92
 0.401205, 0.511234, 0.100787,//93
 0.406132, 0.547112,0.095324,//94
 0.460245, 0.599157, 0.100205,//95
 0.478560, 0.616933, 0.095324,//96
 0.220741, 0.425203, 0.105743,//97
 0.246708, 0.446275, 0.102796,//98
 0.285277, 0.490628, 0.103791,//99
 0.333100, 0.545830, 0.097758,//100
 0.350020, 0.565388, 0.088006,//101
 0.394725 , 0.602981, 0.100699,//102
 0.423049, 0.634856, 0.101733,//103
 0.457562, 0.684716, 0.107170,//104
 0.168332, 0.462782, 0.100119,//105
 0.192330, 0.476704, 0.095324,//106
 0.228788, 0.527270, 0.102994,//107
 0.293303, 0.555890, 0.095324,//108
 0.308161, 0.609230, 0.105392,//109
 0.340604, 0.613533,0.095324,//110
 0.371340, 0.676070, 0.089197,//111
 0.413686, 0.684245, 0.095324,//112
 0.130318, 0.481532, 0.101879,//113
 0.171326, 0.516981, 0.088426,//114
 0.203557, 0.559041, 0.089172,//115
 0.252891, 0.610871, 0.099526,//116
 0.270015, 0.638702, 0.098221,//117
 0.308890, 0.683934, 0.099218,//118
 0.349277, 0.711288, 0.095142,//119
 0.381198, 0.740501, 0.087736,//120
 0.111684, 0.531506, 0.120215,//121
 0.143463, 0.527480,0.095324,//122
 0.167301, 0.604414, 0.098505,//123
 0.206584,0.600496, 0.095324,//124
 0.234433, 0.663241, 0.094418,//125
 0.292841,0.678232, 0.095324,//126
 0.301892, 0.727424, 0.087639,//127
 0.347839,0.751361, 0.095324,//128
 0.071354, 0.566152, 0.102213,//129
 0.110149, 0.606802, 0.099767,//130
 0.140143, 0.626988, 0.092928,//131
 0.186981, 0.671956, 0.099803,//132
 0.208988, 0.698777, 0.096054,//133
 0.251822, 0.736441, 0.100508,//134
 0.283623, 0.770833, 0.094795,//135
 0.311546, 0.812345, 0.102246,//136
 0.042825, 0.580516, 0.113910,//137
 0.069197, 0.606241,0.095324,//138
 0.108767, 0.651864, 0.090428,//139
 0.136843,0.669640, 0.095324,//140
 0.185048, 0.731954, 0.103047,//141
0.209669, 0.732384, 0.095324,//142
 0.247000, 0.799459, 0.102589,//143
 0.281249, 0.804326, 0.095324,//144
 0.004468, 0.624967, 0.096911,//145
 0.035523, 0.660484, 0.099441,//146
 0.069845, 0.690209, 0.107743,//147
 0.108574, 0.730046, 0.090543,//148
 0.148334, 0.760704, 0.106911,//149
 0.186186, 0.805578, 0.100930,//150
 0.211376, 0.827561, 0.087906,//151
 0.229261, 0.842757, 0.100774,//152
 -0.024980, 0.654166, 0.111949,//153
 -0.006127, 0.677538, 0.095324,//154
 0.024732, 0.716578, 0.087527,//155
 0.064760, 0.745914, 0.095324,//156
 0.103733, 0.794804, 0.113467,//157
 0.135648, 0.816494, 0.095324,//158
 0.171012, 0.839651, 0.096201,//159
 0.210059, 0.869114,0.095324,//160
 -0.069514, 0.688821, 0.100525,//161
 -0.032891, 0.726067, 0.109189,//162
 -0.002422, 0.750894, 0.109929,//163
 0.055612, 0.804277, 0.100768,//164
 0.073993, 0.842210, 0.102781,//165
 0.101386, 0.867036, 0.094504,//166
 0.130228, 0.891189, 0.098019,//167
 0.130228, 0.891189, 0.098019,//168
 -0.114268, 0.723997, 0.091173,//169
 -0.063476, 0.749891, 0.095324,//170
 -0.043618, 0.791653, 0.113267,//171
 0.008754, 0.820741, 0.095324,//172
 0.029891, 0.860954, 0.104201,//173
 0.078524, 0.762469, 0.159414,//174
  0.086338, 0.901270, 0.102573,//175
 0.148140, 0.831043, 0.159414,//176
 -0.161992, 0.757846, 0.104085,//177
 -0.095106, 0.780831, 0.101877,//178
 -0.053240, 0.824263, 0.104935,//179
 -0.035465, 0.874747, 0.090677,//180
// -0.176794, 0.802627, 0.087361//181
};
moveit::planning_interface::MoveGroup *group;
geometry_msgs::Pose target;
geometry_msgs::Pose transformer;
float error = 0.05;   //zliu7
class GRIPCV
{
  ros::NodeHandle nh_;
  ros::Publisher tar_state_pub;
  ros::Publisher display_publisher;
  ros::Publisher planning_scene_diff_publisher;
  ros::Subscriber cv_position_sub;
  ros::Subscriber pose_target_;
  ros::Publisher hand_pub;
  ros::Publisher hand_pub1;
  ros::ServiceClient client;
  object_detect::Check srv;
//the next two line is add by zhang yu mei 20160120, for service request , thus can solve the problem of data delay. 
  ros::ServiceClient pos_pose_client;
  object_detect::positionPose srvPositionPose;
  geometry_msgs::Pose thingpose, boxpose;
  bool thingposeflag;
 // boxpose
 

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
  //add by zhang yu mei 20160120,for service request 
    pos_pose_client = nh_.serviceClient<object_detect::positionPose>("/srvPositionPose");
 thingposeflag=false;
 // boxpose
   boxpose.orientation.x = -0.241013;
   boxpose.orientation.y = 0.682147;
   boxpose.orientation.z = 0.222108;
   boxpose.orientation.w = 0.653649;
   boxpose.position.x = 0.588878;
   boxpose.position.y = -0.484398;
   boxpose.position.z = 0.195880;

   hand_pub = nh_.advertise<std_msgs::UInt16>("servo", 1);
   hand_pub1 = nh_.advertise<std_msgs::UInt16>("servo4", 1);

    tar_state_pub = nh_.advertise<geometry_msgs::Pose>("/target_state", 1, true);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    //hand_pub = nh_.advertise<std_msgs::UInt16>("/servo", 1000);
    client = nh_.serviceClient<object_detect::Check>("/check_position");
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
//the next service related code written by zhang yumei 20160120, 
//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! return old place!");
	robo_arm();
	}
	    
	 ROS_INFO("new ***goog **srv2***POS_POSE_CALL IN GRIPCV");
	 movetopose(srvPositionPose.response.pose);
	 cv_position_Cb(srvPositionPose.response.cv_p);  
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

  void robo_arm(int number = 1)
  {
    ROS_INFO("*************8START TO PULL BACK**************88");
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    float fraction = 0.0;
    std::vector<geometry_msgs::Pose> waypoints;
    kinematic_state = group_ptr->getCurrentState();
    if (thingposeflag)
    {
	waypoints.push_back(thingpose);
	//thingposeflag=false;
    }
	
    if(number > 90)
	{
		geometry_msgs::Pose midpose;
		//midpose.position.x =  -0.385299;
		 //  midpose.position.y = 0.579176;
		 //  midpose.position.z = 0.420900;

		  midpose.position.x =  0.401205;
		  midpose.position.y = 0.511234;
		  midpose.position.z = 0.100787;

		  midpose.orientation =  boxpose.orientation;
		  
		 waypoints.push_back(midpose);
		 midpose.position.x =  0.633569;
		 midpose.position.y = 0.309136;
		 midpose.position.z = 0.095324;
		//0.401205, 0.511234, 0.100787
		
		waypoints.push_back(midpose);
		//0.633569, 0.309136, 0.095324,
	}
    //waypoints.push_back(thingpose);
    boxpose.position.z += 0.1;
    waypoints.push_back(boxpose);
    boxpose.position.z -= 0.1;
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
   
   gripper_release(0,0);
   sleep(2);
//the next service related code written by zhang yumei 20160120, 
//request position and pose data first time, if there is no object to garb now, then repeated request , until there is an object
	srvPositionPose.request.flag=true;
	while(!pos_pose_client.call(srvPositionPose))
	{
	sleep(1);
	ROS_INFO("no object ! waiting the target object position data");
	//robo_arm();
	}
	    
	 ROS_INFO("POS_POSE_CALL IN robot_arm");
	 movetopose(srvPositionPose.response.pose);
	 cv_position_Cb(srvPositionPose.response.cv_p);
   	 ROS_INFO("finish robot_arm");
  }
///// pose subscriber////////////////////////
  void movetopose(const geometry_msgs::Twist& pose)
  {
    ROS_INFO("I heard: [%f,%f,%f,%f]", pose.linear.x,pose.linear.y,pose.linear.z,pose.angular.x);

    transformer.orientation.x =   pose.linear.x;
    transformer.orientation.y =   pose.linear.y;
    transformer.orientation.z =   pose.linear.z;
    transformer.orientation.w =  pose.angular.x;
}
/////////////////////////////////////////////
  void getTargetPosition(int number)
  {
    ROS_INFO("number is %d)",number);
//if (number==181)
//{    ros::shutdown();}        //zliu7
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
    
    //start_pose = group_ptr->getCurrentPose().pose;
    
 
 
  //by zhang yu mei 20160120, other conditions , go to pose1, the old place

   thingpose.position.x = POINTS[number][0];
   thingpose.position.y =POINTS[number][1];
   thingpose.position.z =POINTS[number][2];

   thingpose.orientation.x = transformer.orientation.x;
   thingpose.orientation.y = transformer.orientation.y;
   thingpose.orientation.z = transformer.orientation.z;
   thingpose.orientation.w = transformer.orientation.w;
   thingpose.position.z+=0.1;
   waypoints.push_back(boxpose);
   thingposeflag = true;

   if(number > 90)
	{
		geometry_msgs::Pose midpose;
		//midpose.position.x =  -0.385299;
		 //  midpose.position.y = 0.579176;
		 //  midpose.position.z = 0.420900;

		   midpose.position.x =  0.633569;
		   midpose.position.y = 0.309136;
		   midpose.position.z = 0.095324;

		   midpose.orientation =  boxpose.orientation;
		  
		waypoints.push_back(midpose);
		//0.401205, 0.511234, 0.100787
		midpose.position.x =  0.401205;
		   midpose.position.y = 0.511234;
		   midpose.position.z = 0.100787;
		waypoints.push_back(midpose);
		//0.633569, 0.309136, 0.095324,
	}
   waypoints.push_back(thingpose);
   thingpose.position.z-=0.1;
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
  
//  while(TargetCheck(thingpose) == false);
   
   gripper_release(30,30);
   sleep(2);
   robo_arm(number);
}
void gripper_release(int x,int y)
{
    //ros::NodeHandle n;

  ros::Time start_time=ros::Time::now();
  //ros::Publisher hand_pub = n.advertise<std_msgs::UInt16>("servo", 1);
  //ros::Publisher hand_pub1 = n.advertise<std_msgs::UInt16>("servo4", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::Time::now()-start_time<ros::Duration(10))
  {

    std_msgs::UInt16 msg;  
    std_msgs::UInt16 msg1;
    msg.data=x;
    msg1.data=y;
    hand_pub.publish(msg);
    hand_pub1.publish(msg1);

//    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
 ROS_INFO("I release la~");
ROS_INFO("shut down");
//ros::shutdown();
}
  void cv_position_Cb(const geometry_msgs::Twist& cv_p)
{ 
    int number=0;

    ROS_INFO("cv_p.linear.x is %.2f,cv_p.linear.y is %.2f)",cv_p.linear.x,cv_p.linear.y);

    srv.request.input = cv_p;
    if (client.call(srv))
    {
      ROS_INFO("%f, %f",srv.response.output.linear.x,srv.response.output.linear.y);
    }
    else
    {
      ROS_ERROR("Failed to call check_position_service");
      return;
    }

   if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>14)&&(cv_p.linear.y<39.5))
    {
      number=1;
    }
     else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>40)&&(cv_p.linear.y<62))
    {
      number=2;
    } 
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>62)&&(cv_p.linear.y<85))
    {
      number=3;
    }
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>85)&&(cv_p.linear.y<109))
    {
      number=4;
    }
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>109)&&(cv_p.linear.y<132))
    {
      number=5;
    }
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>132)&&(cv_p.linear.y<156))
    {
      number=6;
    }
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>156)&&(cv_p.linear.y<180))
    {
      number=7;
    }
  else if ((cv_p.linear.x>19)&&(cv_p.linear.x<50)&&(cv_p.linear.y>180)&&(cv_p.linear.y<203))
    {
      number=8;
    }
  else if ((cv_p.linear.x>50)&&(cv_p.linear.x<76)&&(cv_p.linear.y>14)&&(cv_p.linear.y<40))
    {
      number=9;
    }
  else if ((cv_p.linear.x>50)&&(cv_p.linear.x<76)&&(cv_p.linear.y>40)&&(cv_p.linear.y<62))
    {
      number=10;
    }
else if ((cv_p.linear.x>51)&&(cv_p.linear.x<76)&&(cv_p.linear.y>62)&&(cv_p.linear.y<85))
    {
      number=11;
    } 
  else if ((cv_p.linear.x>49)&&(cv_p.linear.x<75)&&(cv_p.linear.y>85)&&(cv_p.linear.y<108))
    {
      number=12;
    }
  else if ((cv_p.linear.x>49)&&(cv_p.linear.x<74)&&(cv_p.linear.y>108)&&(cv_p.linear.y<132))
    {
      number=13;
    }
  else if ((cv_p.linear.x>49)&&(cv_p.linear.x<73)&&(cv_p.linear.y>132)&&(cv_p.linear.y<155))
    {
      number=14;
    }
  else if ((cv_p.linear.x>47)&&(cv_p.linear.x<73)&&(cv_p.linear.y>155)&&(cv_p.linear.y<180))
    {
      number=15;
    }
  else if ((cv_p.linear.x>47)&&(cv_p.linear.x<72)&&(cv_p.linear.y>180)&&(cv_p.linear.y<203))
    {
      number=16;
    }
  else if ((cv_p.linear.x>76)&&(cv_p.linear.x<101)&&(cv_p.linear.y>15)&&(cv_p.linear.y<40))
    {
      number=17;
    }
  else if ((cv_p.linear.x>76)&&(cv_p.linear.x<101)&&(cv_p.linear.y>40)&&(cv_p.linear.y<63))
    {
      number=18;
    }
  else if ((cv_p.linear.x>76)&&(cv_p.linear.x<100)&&(cv_p.linear.y>63)&&(cv_p.linear.y<86))
    {
      number=19;
    }
else if ((cv_p.linear.x>76)&&(cv_p.linear.x<100)&&(cv_p.linear.y>86)&&(cv_p.linear.y<108))
    {
      number=20;
    }
  else if ((cv_p.linear.x>74)&&(cv_p.linear.x<100)&&(cv_p.linear.y>108)&&(cv_p.linear.y<132))
    {
      number=21;
    }
  else if ((cv_p.linear.x>73)&&(cv_p.linear.x<98)&&(cv_p.linear.y>132)&&(cv_p.linear.y<155))
    {
      number=22;
    }
  else if ((cv_p.linear.x>73)&&(cv_p.linear.x<97)&&(cv_p.linear.y>155)&&(cv_p.linear.y<179))
    {
      number=23;
    }
  else if ((cv_p.linear.x>72)&&(cv_p.linear.x<97)&&(cv_p.linear.y>179)&&(cv_p.linear.y<203))
    {
      number=24;
    }
  else if ((cv_p.linear.x>101)&&(cv_p.linear.x<125)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=25;
    }
  else if ((cv_p.linear.x>101)&&(cv_p.linear.x<125)&&(cv_p.linear.y>41)&&(cv_p.linear.y<64))
    {
      number=26;
    }
  else if ((cv_p.linear.x>101)&&(cv_p.linear.x<125)&&(cv_p.linear.y>64)&&(cv_p.linear.y<87))
    {
      number=27;
    }
 else if ((cv_p.linear.x>100)&&(cv_p.linear.x<125)&&(cv_p.linear.y>87)&&(cv_p.linear.y<109))
    {
      number=28;
    }
  else if ((cv_p.linear.x>100)&&(cv_p.linear.x<124)&&(cv_p.linear.y>109)&&(cv_p.linear.y<132))
    {
      number=29;
    }
  else if ((cv_p.linear.x>98)&&(cv_p.linear.x<123)&&(cv_p.linear.y>132)&&(cv_p.linear.y<155))
    {
      number=30;
    }
  else if ((cv_p.linear.x>97)&&(cv_p.linear.x<122)&&(cv_p.linear.y>155)&&(cv_p.linear.y<179))
    {
      number=31;
    }
else if ((cv_p.linear.x>97)&&(cv_p.linear.x<122)&&(cv_p.linear.y>179)&&(cv_p.linear.y<203))
    {
      number=32;
    }
  else if ((cv_p.linear.x>125)&&(cv_p.linear.x<149)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=33;
    }
  else if ((cv_p.linear.x>125)&&(cv_p.linear.x<149)&&(cv_p.linear.y>41)&&(cv_p.linear.y<64))
    {
      number=34;
    }
 else if ((cv_p.linear.x>125)&&(cv_p.linear.x<149)&&(cv_p.linear.y>64)&&(cv_p.linear.y<86))
    {
      number=35;
    }
  else if ((cv_p.linear.x>125)&&(cv_p.linear.x<149)&&(cv_p.linear.y>86)&&(cv_p.linear.y<109))
    {
      number=36;
    }
  else if ((cv_p.linear.x>124)&&(cv_p.linear.x<148)&&(cv_p.linear.y>109)&&(cv_p.linear.y<132))
    {
      number=37;
    }
  else if ((cv_p.linear.x>123)&&(cv_p.linear.x<147)&&(cv_p.linear.y>132)&&(cv_p.linear.y<155))
    {
      number=38;
    }
else if ((cv_p.linear.x>122)&&(cv_p.linear.x<147)&&(cv_p.linear.y>155)&&(cv_p.linear.y<178))
    {
      number=39;
    }
else if ((cv_p.linear.x>122)&&(cv_p.linear.x<147)&&(cv_p.linear.y>178)&&(cv_p.linear.y<203))
    {
      number=40;
    }
  else if ((cv_p.linear.x>149)&&(cv_p.linear.x<173)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=41;
    }
  else if ((cv_p.linear.x>149)&&(cv_p.linear.x<172)&&(cv_p.linear.y>41)&&(cv_p.linear.y<64))
    {
      number=42;
    }
 else if ((cv_p.linear.x>149)&&(cv_p.linear.x<172)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=43;
    }
  else if ((cv_p.linear.x>149)&&(cv_p.linear.x<172)&&(cv_p.linear.y>86)&&(cv_p.linear.y<109))
    {
      number=44;
    }
  else if ((cv_p.linear.x>148)&&(cv_p.linear.x<172)&&(cv_p.linear.y>109)&&(cv_p.linear.y<131))
    {
      number=45;
    }
  else if ((cv_p.linear.x>147)&&(cv_p.linear.x<172)&&(cv_p.linear.y>131)&&(cv_p.linear.y<155))
    {
      number=46;
    }
else if ((cv_p.linear.x>147)&&(cv_p.linear.x<171)&&(cv_p.linear.y>155)&&(cv_p.linear.y<178))
    {
      number=47;
    }
  else if ((cv_p.linear.x>147)&&(cv_p.linear.x<171)&&(cv_p.linear.y>178)&&(cv_p.linear.y<203))
    {
      number=48;
    }
 else if ((cv_p.linear.x>173)&&(cv_p.linear.x<197)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {

      number=49;
    }
  else if ((cv_p.linear.x>172)&&(cv_p.linear.x<196)&&(cv_p.linear.y>41)&&(cv_p.linear.y<64))
    {
      number=50;
    }
else if ((cv_p.linear.x>172)&&(cv_p.linear.x<196)&&(cv_p.linear.y>63)&&(cv_p.linear.y<86))
    {
      number=51;
    } 
  else if ((cv_p.linear.x>172)&&(cv_p.linear.x<196)&&(cv_p.linear.y>86)&&(cv_p.linear.y<109))
    {
      number=52;
    }
  else if ((cv_p.linear.x>172)&&(cv_p.linear.x<195)&&(cv_p.linear.y>109)&&(cv_p.linear.y<131))
    {
      number=53;
    }
  else if ((cv_p.linear.x>171)&&(cv_p.linear.x<195)&&(cv_p.linear.y>131)&&(cv_p.linear.y<154))
    {
      number=54;
    }
  else if ((cv_p.linear.x>171)&&(cv_p.linear.x<195)&&(cv_p.linear.y>154)&&(cv_p.linear.y<178))
    {
      number=55;
    }
  else if ((cv_p.linear.x>171)&&(cv_p.linear.x<195)&&(cv_p.linear.y>178)&&(cv_p.linear.y<201))
    {
      number=56;
    }
  else if ((cv_p.linear.x>197)&&(cv_p.linear.x<221)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=57;
    }
  else if ((cv_p.linear.x>196)&&(cv_p.linear.x<221)&&(cv_p.linear.y>41)&&(cv_p.linear.y<64))
    {
      number=58;
    }
  else if ((cv_p.linear.x>196)&&(cv_p.linear.x<220)&&(cv_p.linear.y>63)&&(cv_p.linear.y<86))
    {
      number=59;
    }
else if ((cv_p.linear.x>196)&&(cv_p.linear.x<220)&&(cv_p.linear.y>86)&&(cv_p.linear.y<108))
    {
      number=60;
    } 
  else if ((cv_p.linear.x>196)&&(cv_p.linear.x<220)&&(cv_p.linear.y>108)&&(cv_p.linear.y<131))
    {
      number=61;
    }
  else if ((cv_p.linear.x>195)&&(cv_p.linear.x<220)&&(cv_p.linear.y>131)&&(cv_p.linear.y<155))
    {
      number=62;
    }
  else if ((cv_p.linear.x>195)&&(cv_p.linear.x<220)&&(cv_p.linear.y>155)&&(cv_p.linear.y<178))
    {
      number=63;
    }
  else if ((cv_p.linear.x>195)&&(cv_p.linear.x<220)&&(cv_p.linear.y>178)&&(cv_p.linear.y<201))
    {
      number=64;
    }
  else if ((cv_p.linear.x>221)&&(cv_p.linear.x<244)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=65;
    }
  else if ((cv_p.linear.x>221)&&(cv_p.linear.x<244)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=66;
    }
  else if ((cv_p.linear.x>220)&&(cv_p.linear.x<244)&&(cv_p.linear.y>63)&&(cv_p.linear.y<86))
    {
      number=67;
    }
  else if ((cv_p.linear.x>220)&&(cv_p.linear.x<244)&&(cv_p.linear.y>86)&&(cv_p.linear.y<108))
    {
      number=68;
    }
else if ((cv_p.linear.x>220)&&(cv_p.linear.x<244)&&(cv_p.linear.y>108)&&(cv_p.linear.y<131))
    {
      number=69;
    }
  else if ((cv_p.linear.x>220)&&(cv_p.linear.x<245)&&(cv_p.linear.y>131)&&(cv_p.linear.y<154))
    {
      number=70;
    }
  else if ((cv_p.linear.x>220)&&(cv_p.linear.x<245)&&(cv_p.linear.y>154)&&(cv_p.linear.y<178))
    {
      number=71;
    }
  else if ((cv_p.linear.x>220)&&(cv_p.linear.x<245)&&(cv_p.linear.y>178)&&(cv_p.linear.y<201))
    {
      number=72;
    }
  else if ((cv_p.linear.x>244)&&(cv_p.linear.x<266)&&(cv_p.linear.y>16)&&(cv_p.linear.y<41))
    {
      number=73;
    }
  else if ((cv_p.linear.x>244)&&(cv_p.linear.x<267)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=74;
    }
  else if ((cv_p.linear.x>244)&&(cv_p.linear.x<268)&&(cv_p.linear.y>63)&&(cv_p.linear.y<86))
    {
      number=75;
    }
  else if ((cv_p.linear.x>244)&&(cv_p.linear.x<268)&&(cv_p.linear.y>86)&&(cv_p.linear.y<108))
    {
      number=76;
    }
 else if ((cv_p.linear.x>244)&&(cv_p.linear.x<268)&&(cv_p.linear.y>108)&&(cv_p.linear.y<131))
    {
      number=77;
    }
  else if ((cv_p.linear.x>244)&&(cv_p.linear.x<268)&&(cv_p.linear.y>131)&&(cv_p.linear.y<154))
    {
      number=78;
    }
  else if ((cv_p.linear.x>245)&&(cv_p.linear.x<268)&&(cv_p.linear.y>154)&&(cv_p.linear.y<177))
    {
      number=79;
    }
  else if ((cv_p.linear.x>245)&&(cv_p.linear.x<269)&&(cv_p.linear.y>177)&&(cv_p.linear.y<201))
    {
      number=80;
    }
else if ((cv_p.linear.x>266)&&(cv_p.linear.x<290)&&(cv_p.linear.y>16)&&(cv_p.linear.y<42))
    {
      number=81;
    }
  else if ((cv_p.linear.x>267)&&(cv_p.linear.x<290)&&(cv_p.linear.y>42)&&(cv_p.linear.y<63))
    {
      number=82;
    }
  else if ((cv_p.linear.x>268)&&(cv_p.linear.x<290)&&(cv_p.linear.y>63)&&(cv_p.linear.y<85))
    {
      number=83;
    }
 else if ((cv_p.linear.x>268)&&(cv_p.linear.x<291)&&(cv_p.linear.y>85)&&(cv_p.linear.y<108))
    {
      number=84;
    }
  else if ((cv_p.linear.x>268)&&(cv_p.linear.x<291)&&(cv_p.linear.y>108)&&(cv_p.linear.y<131))
    {
      number=85;
    }
  else if ((cv_p.linear.x>268)&&(cv_p.linear.x<291)&&(cv_p.linear.y>131)&&(cv_p.linear.y<154))
    {
      number=86;
    }
  else if ((cv_p.linear.x>269)&&(cv_p.linear.x<292)&&(cv_p.linear.y>154)&&(cv_p.linear.y<177))
    {
      number=87;
    }
else if ((cv_p.linear.x>269)&&(cv_p.linear.x<293)&&(cv_p.linear.y>177)&&(cv_p.linear.y<200))
    {
      number=88;
    }
else if ((cv_p.linear.x>290)&&(cv_p.linear.x<314)&&(cv_p.linear.y>16)&&(cv_p.linear.y<42))
    {
      number=89;
    }
  else if ((cv_p.linear.x>290)&&(cv_p.linear.x<314)&&(cv_p.linear.y>42)&&(cv_p.linear.y<63))
    {
      number=90;
    }
  else if ((cv_p.linear.x>290)&&(cv_p.linear.x<314)&&(cv_p.linear.y>63)&&(cv_p.linear.y<85))
    {
      number=91;
    }
 else if ((cv_p.linear.x>291)&&(cv_p.linear.x<314)&&(cv_p.linear.y>85)&&(cv_p.linear.y<107))
    {
      number=92;
    }
  else if ((cv_p.linear.x>291)&&(cv_p.linear.x<316)&&(cv_p.linear.y>108)&&(cv_p.linear.y<131))
    {
      number=93;
    }
  else if ((cv_p.linear.x>292)&&(cv_p.linear.x<316)&&(cv_p.linear.y>131)&&(cv_p.linear.y<153))
    {
      number=94;
    }
  else if ((cv_p.linear.x>292)&&(cv_p.linear.x<316)&&(cv_p.linear.y>153)&&(cv_p.linear.y<176))
    {
      number=95;
    }
else if ((cv_p.linear.x>294)&&(cv_p.linear.x<318)&&(cv_p.linear.y>176)&&(cv_p.linear.y<200))
    {
      number=96;
    }
 else if ((cv_p.linear.x>314)&&(cv_p.linear.x<337)&&(cv_p.linear.y>15)&&(cv_p.linear.y<42))
    {
      number=97;
    } 
  else if ((cv_p.linear.x>314)&&(cv_p.linear.x<337)&&(cv_p.linear.y>42)&&(cv_p.linear.y<63))
    {
      number=98;
    }
  else if ((cv_p.linear.x>314)&&(cv_p.linear.x<338)&&(cv_p.linear.y>63)&&(cv_p.linear.y<85))
    {
      number=99;
    }
  else if ((cv_p.linear.x>315)&&(cv_p.linear.x<338)&&(cv_p.linear.y>85)&&(cv_p.linear.y<107))
    {
      number=100;
    }
else if ((cv_p.linear.x>315)&&(cv_p.linear.x<338)&&(cv_p.linear.y>107)&&(cv_p.linear.y<130))
    {
      number=101;
    }
  else if ((cv_p.linear.x>316)&&(cv_p.linear.x<340)&&(cv_p.linear.y>130)&&(cv_p.linear.y<153))
    {
      number=102;
    }
  else if ((cv_p.linear.x>316)&&(cv_p.linear.x<340)&&(cv_p.linear.y>153)&&(cv_p.linear.y<176))
    {
      number=103;
    }
  else if ((cv_p.linear.x>318)&&(cv_p.linear.x<342)&&(cv_p.linear.y>176)&&(cv_p.linear.y<199))
    {
      number=104;
    }
  else if ((cv_p.linear.x>336)&&(cv_p.linear.x<359)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=105;
    }
else if ((cv_p.linear.x>337)&&(cv_p.linear.x<361)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=106;
    } 
  else if ((cv_p.linear.x>337)&&(cv_p.linear.x<361)&&(cv_p.linear.y>64)&&(cv_p.linear.y<85))
    {
      number=107;
    }
  else if ((cv_p.linear.x>338)&&(cv_p.linear.x<362)&&(cv_p.linear.y>85)&&(cv_p.linear.y<107))
    {
      number=108;
    }
  else if ((cv_p.linear.x>338)&&(cv_p.linear.x<362)&&(cv_p.linear.y>107)&&(cv_p.linear.y<130))
    {
      number=109;
    }
  else if ((cv_p.linear.x>340)&&(cv_p.linear.x<364)&&(cv_p.linear.y>130)&&(cv_p.linear.y<153))
    {
      number=110;
    }
  else if ((cv_p.linear.x>340)&&(cv_p.linear.x<364)&&(cv_p.linear.y>153)&&(cv_p.linear.y<175))
    {
      number=111;
    }
  else if ((cv_p.linear.x>341)&&(cv_p.linear.x<366)&&(cv_p.linear.y>175)&&(cv_p.linear.y<199))
    {
      number=112;
    }
 else if ((cv_p.linear.x>359)&&(cv_p.linear.x<383)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=113;
    }
  else if ((cv_p.linear.x>359)&&(cv_p.linear.x<383)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=114;
    }
else if ((cv_p.linear.x>361)&&(cv_p.linear.x<384)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=115;
    }
  else if ((cv_p.linear.x>361)&&(cv_p.linear.x<385)&&(cv_p.linear.y>84)&&(cv_p.linear.y<107))
    {
      number=116;
    }
  else if ((cv_p.linear.x>362)&&(cv_p.linear.x<387)&&(cv_p.linear.y>107)&&(cv_p.linear.y<130))
    {
      number=117;
    }
  else if ((cv_p.linear.x>364)&&(cv_p.linear.x<388)&&(cv_p.linear.y>130)&&(cv_p.linear.y<153))
    {
      number=118;
    }
  else if ((cv_p.linear.x>364)&&(cv_p.linear.x<388)&&(cv_p.linear.y>153)&&(cv_p.linear.y<175))
    {
      number=119;
    }
  else if ((cv_p.linear.x>366)&&(cv_p.linear.x<389)&&(cv_p.linear.y>175)&&(cv_p.linear.y<199))
    {
      number=120;
    }
  
  else if ((cv_p.linear.x>383)&&(cv_p.linear.x<405)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=121;
    }
 else if ((cv_p.linear.x>383)&&(cv_p.linear.x<406)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=122;
    }
  else if ((cv_p.linear.x>383)&&(cv_p.linear.x<408)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=123;
    }
  else if ((cv_p.linear.x>386)&&(cv_p.linear.x<410)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=124;
    }
  else if ((cv_p.linear.x>387)&&(cv_p.linear.x<410)&&(cv_p.linear.y>106)&&(cv_p.linear.y<130))
    {
      number=125;
    }
 else if ((cv_p.linear.x>388)&&(cv_p.linear.x<411)&&(cv_p.linear.y>130)&&(cv_p.linear.y<153))
    {
      number=126;
    }
  else if ((cv_p.linear.x>389)&&(cv_p.linear.x<413)&&(cv_p.linear.y>152)&&(cv_p.linear.y<175))
    {
      number=127;
    }
  else if ((cv_p.linear.x>389)&&(cv_p.linear.x<414)&&(cv_p.linear.y>175)&&(cv_p.linear.y<198))
    {
      number=128;
  }
  else if ((cv_p.linear.x>406)&&(cv_p.linear.x<428)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=129;
    }
 else if ((cv_p.linear.x>406)&&(cv_p.linear.x<429)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=130;
    }
  else if ((cv_p.linear.x>407)&&(cv_p.linear.x<430)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=131;
    }
  else if ((cv_p.linear.x>408)&&(cv_p.linear.x<431)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=132;
    }
  else if ((cv_p.linear.x>410)&&(cv_p.linear.x<434)&&(cv_p.linear.y>106)&&(cv_p.linear.y<129))
    {
      number=133;
    }
else if ((cv_p.linear.x>411)&&(cv_p.linear.x<435)&&(cv_p.linear.y>129)&&(cv_p.linear.y<152))
    {
      number=134;
    }
else if ((cv_p.linear.x>413)&&(cv_p.linear.x<436)&&(cv_p.linear.y>152)&&(cv_p.linear.y<175))
    {
      number=135;
    }
  else if ((cv_p.linear.x>414)&&(cv_p.linear.x<436)&&(cv_p.linear.y>175)&&(cv_p.linear.y<197))
    {
      number=136;
    }
  else if ((cv_p.linear.x>428)&&(cv_p.linear.x<450)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=137;
    }
 else if ((cv_p.linear.x>429)&&(cv_p.linear.x<451)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=138;
    }
  else if ((cv_p.linear.x>430)&&(cv_p.linear.x<453)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=139;
    }
  else if ((cv_p.linear.x>432)&&(cv_p.linear.x<455)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=140;
    }
  else if ((cv_p.linear.x>434)&&(cv_p.linear.x<457)&&(cv_p.linear.y>106)&&(cv_p.linear.y<129))
    {
      number=141;
    }
else if ((cv_p.linear.x>435)&&(cv_p.linear.x<459)&&(cv_p.linear.y>129)&&(cv_p.linear.y<151))
    {
      number=142;
    }
  else if ((cv_p.linear.x>435)&&(cv_p.linear.x<459)&&(cv_p.linear.y>151)&&(cv_p.linear.y<174))
    {
      number=143;
    }
 else if ((cv_p.linear.x>436)&&(cv_p.linear.x<461)&&(cv_p.linear.y>174)&&(cv_p.linear.y<197))
    {
      number=144;
    }
  else if ((cv_p.linear.x>450)&&(cv_p.linear.x<473)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=145;
    }
else if ((cv_p.linear.x>453)&&(cv_p.linear.x<475)&&(cv_p.linear.y>41)&&(cv_p.linear.y<62))
    {
      number=146;
    } 
  else if ((cv_p.linear.x>454)&&(cv_p.linear.x<477)&&(cv_p.linear.y>62)&&(cv_p.linear.y<84))
    {
      number=147;
    }
  else if ((cv_p.linear.x>455)&&(cv_p.linear.x<478)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=148;
    }
  else if ((cv_p.linear.x>457)&&(cv_p.linear.x<480)&&(cv_p.linear.y>106)&&(cv_p.linear.y<129))
    {
      number=149;
    }
  else if ((cv_p.linear.x>457)&&(cv_p.linear.x<480)&&(cv_p.linear.y>129)&&(cv_p.linear.y<151))
    {
      number=150;
    }
  else if ((cv_p.linear.x>461)&&(cv_p.linear.x<484)&&(cv_p.linear.y>152)&&(cv_p.linear.y<174))
    {
      number=151;
    }
  else if ((cv_p.linear.x>462)&&(cv_p.linear.x<486)&&(cv_p.linear.y>174)&&(cv_p.linear.y<196))
    {
      number=152;
    }
  else if ((cv_p.linear.x>473)&&(cv_p.linear.x<495)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=153;
    }
  else if ((cv_p.linear.x>473)&&(cv_p.linear.x<497)&&(cv_p.linear.y>41)&&(cv_p.linear.y<62))
    {
      number=154;
    }
else if ((cv_p.linear.x>475)&&(cv_p.linear.x<498)&&(cv_p.linear.y>62)&&(cv_p.linear.y<84))
    {
      number=155;
    } 
  else if ((cv_p.linear.x>477)&&(cv_p.linear.x<499)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=156;
    }
  else if ((cv_p.linear.x>480)&&(cv_p.linear.x<502)&&(cv_p.linear.y>106)&&(cv_p.linear.y<129))
    {
      number=157;
    }
  else if ((cv_p.linear.x>482)&&(cv_p.linear.x<504)&&(cv_p.linear.y>129)&&(cv_p.linear.y<152))
    {
      number=158;
    }
  else if ((cv_p.linear.x>484)&&(cv_p.linear.x<507)&&(cv_p.linear.y>151)&&(cv_p.linear.y<174))
    {
      number=159;
    }
  else if ((cv_p.linear.x>486)&&(cv_p.linear.x<509)&&(cv_p.linear.y>174)&&(cv_p.linear.y<196))
    {
      number=160;
    }
  else if ((cv_p.linear.x>495)&&(cv_p.linear.x<518)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=161;
    }
  else if ((cv_p.linear.x>497)&&(cv_p.linear.x<519)&&(cv_p.linear.y>41)&&(cv_p.linear.y<62))
    {
      number=162;
    }
  else if ((cv_p.linear.x>499)&&(cv_p.linear.x<523)&&(cv_p.linear.y>62)&&(cv_p.linear.y<84))
    {
      number=163;
    }
else if ((cv_p.linear.x>501)&&(cv_p.linear.x<524)&&(cv_p.linear.y>84)&&(cv_p.linear.y<106))
    {
      number=164;
    }
  else if ((cv_p.linear.x>502)&&(cv_p.linear.x<526)&&(cv_p.linear.y>106)&&(cv_p.linear.y<128))
    {
      number=165;
    }
  else if ((cv_p.linear.x>504)&&(cv_p.linear.x<528)&&(cv_p.linear.y>128)&&(cv_p.linear.y<151))
    {
      number=166;
    }
  else if ((cv_p.linear.x>507)&&(cv_p.linear.x<529)&&(cv_p.linear.y>151)&&(cv_p.linear.y<174))
    {
      number=167;
    }
  else if ((cv_p.linear.x>509)&&(cv_p.linear.x<532)&&(cv_p.linear.y>173)&&(cv_p.linear.y<196))
    {
      number=168;
    }
  else if ((cv_p.linear.x>518)&&(cv_p.linear.x<540)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=169;
    }
  else if ((cv_p.linear.x>519)&&(cv_p.linear.x<541)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=170;
    }
  else if ((cv_p.linear.x>523)&&(cv_p.linear.x<545)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=171;
    }
 else if ((cv_p.linear.x>524)&&(cv_p.linear.x<548)&&(cv_p.linear.y>84)&&(cv_p.linear.y<105))
    {
      number=172;
    }
  else if ((cv_p.linear.x>526)&&(cv_p.linear.x<549)&&(cv_p.linear.y>105)&&(cv_p.linear.y<128))
    {
      number=173;
    }
  else if ((cv_p.linear.x>528)&&(cv_p.linear.x<552)&&(cv_p.linear.y>128)&&(cv_p.linear.y<150))
    {
      number=174;
    }
  else if ((cv_p.linear.x>529)&&(cv_p.linear.x<554)&&(cv_p.linear.y>150)&&(cv_p.linear.y<173))
    {
      number=175;
    }
else if ((cv_p.linear.x>532)&&(cv_p.linear.x<554)&&(cv_p.linear.y>173)&&(cv_p.linear.y<195))
    {
      number=176;
    }
  else if ((cv_p.linear.x>541)&&(cv_p.linear.x<564)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
    {
      number=177;
    }
  else if ((cv_p.linear.x>543)&&(cv_p.linear.x<566)&&(cv_p.linear.y>41)&&(cv_p.linear.y<63))
    {
      number=178;

    }
 else if ((cv_p.linear.x>543)&&(cv_p.linear.x<568)&&(cv_p.linear.y>63)&&(cv_p.linear.y<84))
    {
      number=179;
    }
  else if ((cv_p.linear.x>548)&&(cv_p.linear.x<570)&&(cv_p.linear.y>84)&&(cv_p.linear.y<105))
    {
      number=180;
    }
  //else if ((cv_p.linear.x>564)&&(cv_p.linear.x<586)&&(cv_p.linear.y>15)&&(cv_p.linear.y<41))
   // {
   //   number=181;
   // }
    else number =0;

    //////get the path to the target poistion
 getTargetPosition(number);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grip_rect");
  ros::NodeHandle n;
  GRIPCV grip;

  ros::spin();
  

  return 0;
}
