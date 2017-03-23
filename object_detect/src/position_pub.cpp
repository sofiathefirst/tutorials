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
#include <geometry_msgs/Twist.h>
#include <vector>

using namespace cv;	
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detect");
  ros::NodeHandle nh_;
  geometry_msgs::Twist cv_p;
  ros::Rate loop_rate(100);
  ros::Publisher cv_p_pub = nh_.advertise<geometry_msgs::Twist>("cv_position",1);
  cv_p.linear.x=180;
  cv_p.linear.y=120;
  cv_p.linear.z=0;
  cv_p.angular.x=0;
  cv_p.angular.y=0;
  cv_p.angular.z=0;			                                    
  while (ros::ok())
  {
    cv_p_pub.publish(cv_p);
    loop_rate.sleep();

  }
  
  return 0;
}



