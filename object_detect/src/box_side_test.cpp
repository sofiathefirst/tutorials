#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <object_detect/PointVector.h>
#include <object_detect/CheckSide.h>
#include <geometry_msgs/Quaternion.h>	
using namespace std;

  object_detect::CheckSide srv_rightside;       //zxx
  object_detect::CheckSide srv_leftside;       //zxx
  ros::ServiceClient client_rightside;      //zxx
  ros::ServiceClient client_leftside;      //zxx
  object_detect::PointVector pointsVector, leftCorner, rightCorner, right2d, left2d;   //zxx
  ros::Subscriber object_corner_left_target;   //subscriber for receiving object's left corner 3d data  author zxx
  ros::Subscriber object_corner_right_target;   //subscriber for receiving object's right corner 3d data  author zxx
  ros::Subscriber object_corner_left_2d_target;   //subscriber for receiving object's left corner 2d data  author zxx
  ros::Subscriber object_corner_right_2d_target;   //subscriber for receiving object's right corner 2d data  author zxx
  geometry_msgs::Quaternion orientation_left;          //zxx
  geometry_msgs::Quaternion orientation_right;          //zxx
  bool left_side;

  bool checkSide(int index, geometry_msgs::Pose object)
  {
    //call the service to check
    srv_rightside.request.input = right2d.points[index];
    srv_leftside.request.input = left2d.points[index];
    client_leftside.call(srv_leftside);
    client_rightside.call(srv_rightside);
    int right = srv_rightside.response.output.data;
    int left = srv_leftside.response.output.data;
    ROS_INFO("LEFT :%d, RIGHT: %d",left,right);
    if(right && left)
    {
      if(right >= left)
      {
        object.orientation.x = orientation_right.x;
        object.orientation.y = orientation_right.y;
        object.orientation.z = orientation_right.z;
        object.orientation.w = orientation_right.w;
        object.position.x = rightCorner.points[index].x;
        object.position.y = rightCorner.points[index].y;
        object.position.z = rightCorner.points[index].z;
        left_side = false;
      }
      else
      {
        object.orientation.x = orientation_left.x;
        object.orientation.y = orientation_left.y;
        object.orientation.z = orientation_left.z;
        object.orientation.w = orientation_left.w;
        object.position.x = leftCorner.points[index].x;
        object.position.y = leftCorner.points[index].y;
        object.position.z = leftCorner.points[index].z;
        left_side = true;
      } 
    }
    else if(!left && right)
    {
      object.orientation.x = orientation_left.x;
      object.orientation.y = orientation_left.y;
      object.orientation.z = orientation_left.z;
      object.orientation.w = orientation_left.w;
      object.position.x = leftCorner.points[index].x;
      object.position.y = leftCorner.points[index].y;
      object.position.z = leftCorner.points[index].z;
      left_side = true;
    }
    else if(!right)
    {
      object.orientation.x = orientation_right.x;
      object.orientation.y = orientation_right.y;
      object.orientation.z = orientation_right.z;
      object.orientation.w = orientation_right.w;
      object.position.x = rightCorner.points[index].x;
      object.position.y = rightCorner.points[index].y;
      object.position.z = rightCorner.points[index].z;
      left_side = false;
    }
  }
//zxx
  void GetCornerLeft2dData(const object_detect::PointVector leftVector)
  {
    left2d = leftVector;
  }
//zxx
  void GetCornerRight2dData(const object_detect::PointVector rightVector)
  {
    right2d = rightVector;
  }
  void GetCornerLeftData(const object_detect::PointVector& pVector)
  {
    leftCorner = pVector;
  }

  void ObjectTransform(const object_detect::PointVector& pVector)
  {
    rightCorner = pVector;
    if(!right2d.points.size() || !leftCorner.points.size())
      return;
    else
    {
      for(int i = 0; i < right2d.points.size(); i++)
      {
        
      }
    }
  }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detect");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(100);	
  left_side = false;
//I heard: [-0.586118,-0.302051,0.633121,0.405448,-0.700445,0.228948,0.448364]
    orientation_left.x = -0.586118;
    orientation_left.y = -0.302051;
    orientation_left.z = 0.633121;
    orientation_left.w = 0.405448;
//I heard: [-0.294583,0.302412,0.881365,0.212046,-0.556905,0.432729,0.454853]
    orientation_right.x = -0.294583;
    orientation_right.y = 0.302412;
    orientation_right.z = 0.881365;
    orientation_right.w = 0.212046;
    object_corner_left_target = nh_.subscribe("/object_corner_left_pub", 1, &GetCornerLeftData);  //zxx
    object_corner_right_target = nh_.subscribe("/object_corner_right_pub", 1, &ObjectTransform);  //zxx
    object_corner_left_2d_target = nh_.subscribe("/object_corner_left_2d_pub", 1, &GetCornerLeft2dData);  //zxx
    object_corner_right_2d_target = nh_.subscribe("/object_corner_right_2d_pub", 1, &GetCornerRight2dData);  //zxx
		                                    
    ros::spin();
  return 0;
}

