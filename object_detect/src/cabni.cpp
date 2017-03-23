#include <ros/ros.h>
#include"cabni.h"
//#include "boxbox.h"  //lzc
#include<object_detect/cabinetPose.h>
bool getsrvPose(object_detect::cabinetPose::Request &req,object_detect::cabinetPose::Response &res)
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
  ros::init(argc, argv, "bottle1");
  ros::NodeHandle nh_;
  mat_kinect<<0.710404172113503,0.695085958501501,0.108689985780883,-741.995840641209,
-0.678702878574137,0.731763022688003,-0.0581245860690643,-599.539749081723,
0.106355332233201,-0.0647773362770584,-0.992215924086292,1292.36267890099,
0,0,0,1;

  ros::Rate loop_rate(50);
  ros::ServiceServer service = nh_.advertiseService("/srvcabinetPose",getsrvPose);
kinect_start();
  ros::spinOnce();
  int count = 0;
  while (ros::ok())
  {
   // ROS_INFO("AF");
    ros::spinOnce();
    loop_rate.sleep();
    test2();
  }
  
kinect_close();
  return 0;
}
