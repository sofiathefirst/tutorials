#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>                       //zliu7
#include <opencv2/video/background_segm.hpp>           //zliu7
#include "geometry_msgs/Pose2D.h"
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"          
#include <vector>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include "mobile_shelf.h"
#include<object_detect/shelfData.h>
int dex;

bool getsrvPose(object_detect::shelfData::Request &req,object_detect::shelfData::Response &res)
{
	if(req.flag && srvsend)
	{
		
		res.pose = srvpose;
		srvsend = false;
		return true;
	}
	return false;
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bottle");
  ros::NodeHandle nh_;
  mat_kinect<<-0.210849609870077,-0.417180093222539,0.884026703124103,-807.988749607123,
0.979858609935159,0.0120956966309015,0.199325860487169,152.480252653798,
0.0612807582110184,-0.914757337523637,-0.399329039915393,1148.60152158871,
0,0,0,1;
  mat3.setValue(0.655277590799482,-0.275409849707017,0.703424248354783,
0.736124098193809,0.285287908906006,-0.61371929937614,
0.0461805549779639,-0.919349901056736,-0.390793502296213
);

  ros::Rate loop_rate(100);
  ros::ServiceServer service = nh_.advertiseService("/srvshelfData",getsrvPose);
  // Create a ROS subscriber for the input point cloud

kinect_init();
  ros::spinOnce();
  int count = 0;
  while (ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
 test2();

    //++count;
  }
  
kinect_close();
  return 0;
}
