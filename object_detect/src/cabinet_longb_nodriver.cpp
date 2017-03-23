#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>                       //zliu7
#include <opencv2/video/background_segm.hpp>           //zliu7
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"          //bzj
#include <vector>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include "object_detect/cabinetPose.h"
#include <XnCppWrapper.h>  
#include <iostream>  
#include <fstream>
#include <iomanip>  
#include <vector>  
#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include "BI_main.cpp"
  using namespace pcl;
  using namespace pcl::io;
  using namespace cv;
  using namespace std;
 using namespace Eigen;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cabinet_center");
  ros::NodeHandle nh_;


ROS_INFO("1");

test2();
  ros::Rate loop_rate(100);
		//¶ÁÈ¡±³Ÿ°Éî¶ÈÍŒ£¬Ò»¶šÒªÎª16Î»Éî
  //object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  //image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, imageCb);      
  //image_subc_ = it_.subscribe("/camera/rgb/image_raw", 1, imageCbc); 
 // ros::ServiceServer service = nh_.advertiseService("/srvcabinetPose",getsrvPose);
  ros::spin();
  
  return 0;
}
