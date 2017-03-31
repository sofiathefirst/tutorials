#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
using namespace pcl;
using namespace pcl::io;

  using namespace std;


ros::Subscriber object_pcl;
pcl::PointCloud<pcl::PointXYZ> cloud;

//zxx
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud);
  
 // ROS_INFO("%d,%d,%d",(int)cloud.width,(int)cloud.height,(int) cloud.points.size());
  //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr = &cloud;

pcl::io::savePCDFileASCII ("mbox_subs.pcd", cloud);
  bool receive = true;
//  cloud.points.at<float>(256,64);
}

 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "boxpclSub");

  ros::NodeHandle nh_;

  ros::Subscriber object_pcl;
  
  ros::Rate loop_rate(100);

  object_pcl = nh_.subscribe ("/boxpcl_pub", 1, cloud_cb);

  ros::spin ();
  return 0;
}
