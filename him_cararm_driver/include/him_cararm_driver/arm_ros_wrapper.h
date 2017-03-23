#ifndef _ARM_ROS_WRAPPER_H_
#define _ARM_ROS_WRAPPER_H_
#include <string.h>
#include <vector>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>
#include <vector>
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"	
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <actionlib/server/simple_action_server.h>

#include <him_cararm_driver/WayPointPoseAction.h>
#include <him_cararm_driver/CarPoseAction.h>

#include "him_cararm_driver/Communicate.h"
#include "him_cararm_driver/motion_planning.h"
#include "him_cararm_driver/fk_ik.h"
  

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
 #include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <fstream>

#include<arpa/inet.h>
#include<sys/socket.h>
#include <pthread.h> 
#include <errno.h> 
#include <sys/stat.h>  
#include <fcntl.h>  
#include <sys/types.h>  
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <wait.h>
#include <netinet/in.h>       
#include <stdio.h>              
#include <string.h>        

#define UDP_PORT 6666

#define TCP_PORT 12345
#define LENGTH_OF_LISTEN_QUEUE 20
#define BUFFER_SIZE 200

struct dataPacket
{
	int flag;
	float X;
	float Y;
	float angle;
};


typedef struct OrderTable{
	int x;
	int y;
	char *s;
}OrderTable;
#endif

