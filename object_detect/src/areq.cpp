#include <ros/ros.h>

#include<object_detect/desktopData.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bottle1");
  ros::NodeHandle nh_;
  
  ros::Rate loop_rate(50);
  object_detect::desktopData srvPositionPose;
  srvPositionPose.request.flag=true;
  ros::ServiceClient  	pose_client = nh_.serviceClient<object_detect::desktopData>("/srvDesktopData");
  ros::spinOnce();
  int count = 0;
  while (ros::ok())
  {
 	ros::Time start_time=ros::Time::now();
		if(pose_client.call(srvPositionPose) )
		{
		
			geometry_msgs::Pose target_pose_world;
			target_pose_world.position =srvPositionPose.response.position;
	ROS_ERROR("i heard world target_POSE: [,%f,%f,%f,",target_pose_world.position.x,
target_pose_world.position.y,target_pose_world.position.z);
		} 
	//	usleep(100000);
		else ROS_INFO("no object !");		

	ros::Duration dur2=ros::Time::now() - start_time;
	ROS_INFO("The time is:%f",dur2.toSec());

	    loop_rate.sleep();
	sleep(4);
  }

  
  return 0;
}
