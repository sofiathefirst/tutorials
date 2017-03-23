#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>                       //zliu7
#include <opencv2/video/background_segm.hpp>           //zliu7
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <object_detect/CheckSide.h>
#include <object_detect/PointVector.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>

using namespace cv;	
using namespace std;
bool receive = false;
pcl::PointCloud<pcl::PointXYZ> cloud;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud); 
  receive = true;
//  cloud.points.at<float>(256,64);
}
int findRightSide(geometry_msgs::Point point)
{
  ROS_INFO("RIGHT SIDE POINT: %f, %f", point.x,point.y);
  bool change;
  int index = point.y*cloud.width+point.x;
  float depth = cloud.points[index].z;
  int x = 0, j = 0;
  for(int i = 1;(i+point.x)<=640;i++,index++)
  {  
/*     if(fabs(cloud.points[index].z - depth) <= 0.1)
     { 
       for(j = index; j < index+6; j++)
       {
         if(!((fabs(cloud.points[j].z) - cloud.points[j+1].z) <= 0.007))
           continue;
       }
       if(j == index+6 )
       {
         x = i;
         ROS_INFO("FIND RIGHT SIDE ONE DATA2");
       }
       break;
   }*/
    if(!change && (fabs(cloud.points[index].z) - depth) >= 0.15)
    {
      change = true;
    }
    else if(change && (fabs(cloud.points[index].z) - depth) < 0.15)
    {
      
      x = i;
    } 
  }
  if(x == 0)
    x = 640;
  return x;
}
int findLeftSide(geometry_msgs::Point point)
{
  int index = point.y*cloud.width+point.x;
  int x = 0;
  bool change = false;
  float depth = cloud.points[index].z;
  for(int i = 1;(i+point.x)>0;i--,index--)
  {  
    if(!change && (fabs(cloud.points[index].z-depth) >= 0.15))
    { ROS_INFO("LEFT SIDE CHANGE");
      change = true;
    }
    else if(change && (fabs(cloud.points[index].z-depth) < 0.15))
    {
      x = i;
      break;
    }
  }
  if(x == 0)
    x = -640;
  return -x;
}
bool checkRightArea(float length, float width)
{
  bool check = true;
  if(width <= 0 && length <= 0.38 && length > -0.11)
    check = false;
  if(width > 0 && length <= -0.15 && length >= -0.4)
    check = false;
  return check;
}
bool checkRight(object_detect::CheckSide::Request &req,object_detect::CheckSide::Response &res)
{
  
  if(receive)
  {
    geometry_msgs::Point point = req.input;
    int index = 0,right = 0;
    float depth = 0, length = 0, width = 0;
    index = point.y*cloud.width + point.x;
    length = cloud.points[index].x;
    width = -cloud.points[index].y;
    depth = cloud.points[index].z;
    if(!checkRightArea(length,width))
    {
      res.output.data = -1;
    }
    else
    {
      int index = 0,right = 0;
      float depth = 0;
      index = point.y*cloud.width + point.x;
      depth = cloud.points[index].z;
      right = findRightSide(point);
      res.output.data = right;
    }
    return true;
  }
  return false;
}
bool checkLeftArea(float length, float width)
{
  bool check = true;
  ROS_INFO("LENGTH WIDTH: %f, %f", length,width);
  if(width <= 0 &&  length >= -0.75 && length <= -0.11)
    check = false;
  if(width > 0 && length > -0.1)
    check = false;
  return check;
}
bool checkLeft(object_detect::CheckSide::Request &req,object_detect::CheckSide::Response &res)
{
  
  if(receive)
  {ROS_INFO("CALLING LEFT CHECK");
    geometry_msgs::Point point = req.input;
    int index = 0,left = 0;
    float depth = 0, length = 0, width = 0;
    index = point.y*cloud.width + point.x;
    length = cloud.points[index].x;
    width = -cloud.points[index].y;
    depth = cloud.points[index].z;
    if(!checkLeftArea(length,width))
    {
      res.output.data = -1;
    }
    else
    {
      left = findLeftSide(point);
      res.output.data = left;
    }
    return true;
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "box_pos_check");
  ros::NodeHandle nh_;

  ros::Rate loop_rate(100);
//  ros::Subscriber center_sub = nh_.subscribe("/object_center_2d_pub", 1, &position);
  ros::Subscriber pcl_sub = nh_.subscribe("/camera/depth_registered/points", 1, &cloud_cb);
  ros::ServiceServer service1 = nh_.advertiseService("/box_check_rightside", checkRight);	
  ros::ServiceServer service2 = nh_.advertiseService("/box_check_leftside", checkLeft);                                    
  ros::spin();
  
  return 0;
}



