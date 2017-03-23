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
#include <object_detect/CheckBox.h>
#include <object_detect/PointVector.h>
#include <math.h>
using namespace cv;	
using namespace std;
object_detect::PointVector box_pos;
int flow = 0.04;

bool isSame(geometry_msgs::Point t1, geometry_msgs::Point t2)
{
  float error[3] = {0};
  error[0] = fabs(t1.x - t2.x);
  error[1] = fabs(t1.y - t2.y);
  if(error[0] <= flow)
    return false;
  else if(error[1] <= flow)
    return false;
  else return true;
}

void position(const object_detect::PointVector p)
{
  box_pos = p;
}

bool check(object_detect::CheckBox::Request &req,object_detect::CheckBox::Response &res)
{
  int num = box_pos.points.size();
  bool find = false;
  geometry_msgs::Point temp;
  for(int i = 0; i < num; i++)
  {
    temp = box_pos.points.at(i);
    if(isSame(req.input, temp))
    {
      res.output = req.input;
      find = true;
    }
  }
  if(!find)
  {
    temp.x = 0;
    temp.y = 0;
    res.output = temp;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "box_pos_check");
  ros::NodeHandle nh_;

  ros::Rate loop_rate(100);
  ros::Subscriber box_p_sub = nh_.subscribe("/object_center_pub", 1, &position);
  ros::ServiceServer service = nh_.advertiseService("/box_check_position", check);	                                    
  ros::spin();
  
  return 0;
}



