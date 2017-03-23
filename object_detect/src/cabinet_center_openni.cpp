#include <ros/ros.h>
#include <iostream>
#include "object_detect/cabinetPose.h"

 using namespace Eigen;
Matrix<float,4,4> mat_kinect;


geometry_msgs::Pose srvpose;

bool srvsend;

bool setsrvPose(tf2::Vector3 v)
{
	srvpose.position.z = v.z;
	srvpose.position.x = v.x;
	srvpose.position.y = -v.y;
        
	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;
	//if(srvpose.position.z>2 ||srvpose.position.z < 1.2 ) return false;
	//ROS_INFO("DEPTH : %f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
	
	Matrix<float,4,1> Pr;
	Matrix<float,4,1> Pc;
	Pc << 	srvpose.position.x*1000,
	  	srvpose.position.y*1000,
	  	srvpose.position.z*1000,
		    1;

	Pr = mat_kinect *Pc;

	srvpose.position.x = Pr(0,0)/1000;
	srvpose.position.y = Pr(1,0)/1000;
	srvpose.position.z = Pr(2,0)/1000;

	ROS_ERROR("rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
        
	return true;
  
}

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
  ros::init(argc, argv, "cabinet_center");
  ros::NodeHandle nh_;

    mat_kinect<<0.725612823314762,0.261596096585221,-0.637452131842839,391.656631936501,
-0.624635823365322,0.266859640484080,-0.734257061790433,838.964672540469,
0.00263389879853513,-0.894614159049920,-0.446053560056995,1291.50279285307,
0,0,0,0;

  ros::Rate loop_rate(100);

  ros::ServiceServer service = nh_.advertiseService("/srvcabinetPose",getsrvPose);
  ros::spin();
  
  return 0;
}
