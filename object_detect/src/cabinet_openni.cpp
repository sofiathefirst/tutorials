#include <ros/ros.h>
#include"cab_ni.cpp"

bool getsrvPose(object_detect::cabinetPose::Request &req,object_detect::cabinetPose::Response &res)
{ROS_INFO("IN SRV 1");
	if(req.flag && srvsend)
	{
		ROS_INFO("IN SRV");
		res.pose = srvpose;
		srvsend = false;
		return true;
	}
	return false;
} 


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cabinet_center");
  ros::NodeHandle nh_;

mat_kinect<<0.625408802070069,-0.254504679641369,0.737799605995627,-789.765926075889,
0.732997492837814,0.263645161223146,-0.627303499018915,330.888975931973,
0.0236277436895945,-0.896255895305970,-0.442589731328521,1298.28225768603,
0,0,0,1;
  mat3.setValue(0.725612823314762,0.261596096585221,-0.637452131842839,
-0.624635823365322,0.266859640484080,-0.734257061790433,
0.00263389879853513,-0.894614159049920,-0.446053560056995);



  ros::Rate loop_rate(100);
  positionpub = nh_.advertise<geometry_msgs::Pose> ("/cab_pose_pubni", 1);
  ros::ServiceServer service = nh_.advertiseService("/srvcabinetPose",getsrvPose);
  test2();
  ros::spin();
  
  return 0;
}
