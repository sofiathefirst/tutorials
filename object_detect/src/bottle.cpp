#include <ros/ros.h>
#include"bottle_cv.h"
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
  /*mat_kinect<<-0.178006119340335,0.984533051229324,-0.00681022255602800,-694.947975058230,
-0.981916323476816,-0.149140134744755,-0.113464024843861,125.324281952675,
0.124150927187861,0.0458259432940302,-0.990563792361708,1070.05989618072,
0,0,0,1;*/

mat_kinect<<-0.0937430868841260,0.989387547044136,-0.0279459239529103,-630.791774341251,
-0.989180093290474,-0.100658786393352,-0.105744007148193,136.867990877544,
0.0477462282235069,0.0565595763265067,-0.998827236961359,1082.14621224333,
0,0,0,1;

  ros::Rate loop_rate(50);
  ros::ServiceServer service = nh_.advertiseService("/srvbottlePose",getsrvPose);
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
