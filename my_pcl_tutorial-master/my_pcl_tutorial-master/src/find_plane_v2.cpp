#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// Source https://github.com/OctoMap/octomap_mapping/blob/indigo-devel/octomap_server/src/OctomapServer.cpp
//

ros::Publisher cloud_filtered_pub;
ros::Publisher cloud_nonground_pub;
ros::Publisher cloud_output_pub;

namespace find_plane{
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
}

using namespace find_plane;

void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground); 

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){

  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud_msg, pc);

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.6, 2.);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else
  pass.setInputCloud(pc.makeShared());
  pass.filter(pc);
  filterGroundPlane(pc, pc_ground, pc_nonground);

  //Reconstruct without ground by taking the negative of passThrough filter
  pass.setNegative(true);
  pass.filter(pc);
  pc+=pc_nonground;

  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_output_msg;
  pcl::toROSMsg(pc, cloud_output_msg);
  cloud_output_pub.publish(cloud_output_msg);
}

void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground)
{
    ground.header = pc.header;
    nonground.header = pc.header;

    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.05);
    //seg.setAxis(Eigen::Vector3f(0,0,1));  Can be added to accurately find the ground
    //seg.setEpsAngle(m_groundFilterAngle);

    PCLPointCloud cloud_filtered(pc);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(cloud_filtered, cloud_filtered_msg);
    cloud_filtered_pub.publish(cloud_filtered_msg);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setInputCloud(cloud_filtered.makeShared());
    seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");
      }

    extract.setInputCloud(cloud_filtered.makeShared());
    extract.setIndices(inliers);

    float m_groundFilterPlaneDistance = 0.0;

    extract.setNegative (false);
    extract.filter(ground);

    // remove ground points from full pointcloud:
    // workaround for PCL bug:
    if(inliers->indices.size() != cloud_filtered.size()){
      ROS_INFO("Different indices");

      extract.setNegative(true);
      PCLPointCloud cloud_out;
      extract.filter(cloud_out);
      nonground += cloud_out;
      cloud_filtered = cloud_out;
    }

    //Reconstruct nonground with passTrough output setNegative(true)
    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_nonground_msg;
    pcl::toROSMsg(nonground, cloud_nonground_msg);
    cloud_nonground_pub.publish(cloud_nonground_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cluster_kinect");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloudCallback);

  cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);
  cloud_nonground_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_nonground", 1);
  cloud_output_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_output", 1);


  // Spin
  ros::spin ();
}