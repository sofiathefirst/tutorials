#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include <tf/tf.h>
#include <fstream>
#include "sensor_msgs/JointState.h"
#include "common.h"
#define TT 0.001
#define min(x,y)  ((x)<(y)?(x):(y))
std::vector<float> interpolation(const std::vector<float> a, const std::vector<float> b, const float t);
void poseToMatrix(float matrix[][16], const geometry_msgs::PoseArray &keyPoses);
void key_points_with_fixed_num(geometry_msgs::PoseArray waypointPoses, geometry_msgs::PoseArray &keyPoses);
void key_points_with_fixed_distance(geometry_msgs::PoseArray waypointPoses, geometry_msgs::PoseArray &keyPoses);
void cubic(double joint[][7],int flag, int points_num, ros::Publisher &joint_pub, sensor_msgs::JointState &joint_msg);
void changeEndffectorPoseToLink6Pose(geometry_msgs::PoseArray &waypointPoses);
void FK_add_end_effector_length(float &x, float &y, float &z, const float ax, const float ay, const float az);
#endif
