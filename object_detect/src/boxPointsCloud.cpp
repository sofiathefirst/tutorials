#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_detect/PointVector.h>

#include <geometry_msgs/Quaternion.h>
const float PI=3.1415926;

object_detect::PointVector centerPoints;
object_detect::PointVector leftPoints;
ros::Publisher boxpclpub;
bool receive;


void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud; 
  pcl::PointCloud<pcl::PointXYZ> boxcloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud);
  for(int i = 0; i < leftPoints.points.size();i++)
  {
	int left[2]= {leftPoints.points.at(i).x,leftPoints.points.at(i).y};
	int center[2] = {centerPoints.points.at(i).x,centerPoints.points.at(i).y};
	int w = int(1.2*(center[0]-left[0]));
	int h = int(1.2*(center[1]-left[1]));
     	
	for (int m = left[1]; m < left[1]+2*h; m++)
		for ( int n =left[0];n < left[0]+2*w; n++)
		{
			int index = m*cloud.width + n;
			if (index < cloud.points.size())
				boxcloud.points.push_back(cloud.points[index]);		
		}
	boxcloud.width = 2*w;
	boxcloud.height = 2*h;
	
	pcl::io::savePCDFileASCII ("box.pcd", boxcloud);
  }
  receive = true;
//  cloud.points.at<float>(256,64);
}
 void cornerCb(const object_detect::PointVector& pVector)
  {
     leftPoints = pVector;
     for(int i = 0; i < leftPoints.points.size();i++)
     ROS_INFO("Hearing the data%d,%d",(int)leftPoints.points.at(i).x,(int)leftPoints.points.at(i).y);
     
  }

 void centerCb(const object_detect::PointVector& pVector)
  {
     centerPoints = pVector;
     for(int i = 0; i < centerPoints.points.size();i++)
     ROS_INFO("Hearing the data%d,%d",(int)centerPoints.points.at(i).x,(int)centerPoints.points.at(i).y);
     
  }
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_pose");
  ros::NodeHandle nh_;
  ros::Subscriber object_corner_target = nh_.subscribe("/object_corner_pub", 1,cornerCb);
  ros::Subscriber object_center_target = nh_.subscribe("/object_center_pub", 1, centerCb);
  ros::Subscriber pclsub = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  boxpclpub = nh_.advertise<sensor_msgs::PointCloud2> ("/pointCloud2_bgoutput", 10);

  ros::spin();

  return 0;
}
