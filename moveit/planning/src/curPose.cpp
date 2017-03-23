#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>
moveit::planning_interface::MoveGroup *group;// = new moveit::planning_interface::MoveGroup("manipulator");

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorialpub");
  ros::NodeHandle nodeHandle;

  group = new moveit::planning_interface::MoveGroup("manipulator");
  ros::Publisher curposepub = nodeHandle.advertise<geometry_msgs::Pose>("/eef_pose", 1);
  ros::Rate loop_rate(10);
   ros::AsyncSpinner spinner(1);
  spinner.start();
 
  while (ros::ok())
  {

	geometry_msgs::Pose curPose;

	curPose = group->getCurrentPose().pose;

	
	std::cout<<"mypose.orientation.x="<<curPose.orientation.x <<";\n" ;
 	std::cout<<"mypose.orientation.y="<<curPose.orientation.y <<";\n" ;
 	std::cout<<"mypose.orientation.z="<<curPose.orientation.z <<";\n" ;
 	std::cout<<"mypose.orientation.w="<<curPose.orientation.w <<";\n" ;
	std::cout<<"mypose.position.x="<<curPose.position.x <<";\n" ;
 	std::cout<<"mypose.position.y="<<curPose.position.y <<";\n" ;
 	std::cout<<"mypose.position.z="<<curPose.position.z <<";\n" ;

	curposepub.publish(curPose);
	ROS_WARN("xyz CUR POSE : %f , %f,%f, %f , %f,%f,%f,dist = %f",
curPose.orientation.x,curPose.orientation.y,curPose.orientation.z,curPose.orientation.w,
curPose.position.x,curPose.position.y,curPose.position.z,tf2::Vector3(curPose.position.x,curPose.position.y,curPose.position.z).length());
	ros::spinOnce();

	loop_rate.sleep();
   // ++count;
  }
  //vsMove( targetPose);
  return 0;
}
