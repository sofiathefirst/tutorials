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
#include "object_detect/PoseDepth.h"
  using namespace pcl;
  using namespace pcl::io;
  using namespace cv;
  using namespace std;
 using namespace Eigen;
Matrix<float,4,4> mat_kinect;

 
 
  static const std::string OPENCV_WINDOW_1 = "Image window1";
  static const std::string OPENCV_WINDOW_2 = "Image window2";
  static const std::string OPENCV_WINDOW_3 = "Image window3";
  static const std::string OPENCV_WINDOW_4 = "Image window4";


std::vector<geometry_msgs::Point> right_corner, left_corner;


geometry_msgs::Point point;           //bzj

ros::Publisher object_center_pub;   //zxx
ros::Publisher object_center_2d_pub;   //zxx

ros::Subscriber object_pcl;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool receive = false;
int count_frame = 0;
    Mat cameraFeed;

geometry_msgs::Pose srvpose;

bool srvsend;

bool setsrvPose(int index)
{
	srvpose.position.z = cloud.points[index].z;
	srvpose.position.x = cloud.points[index].x;
	srvpose.position.y = -cloud.points[index].y;
        
	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;
	if(srvpose.position.z>2 ||srvpose.position.z < 1.2 ) return false;
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

bool getsrvPose(object_detect::PoseDepth::Request &req,object_detect::PoseDepth::Response &res)
{
	if(req.flag && srvsend)
	{
		res.pose = srvpose;
		srvsend = false;
		return true;
	}
	return false;
} 


//zxx
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud);
  receive = true;
//  cloud.points.at<float>(256,64);
}
  string intToString(int number)
  {
	std::stringstream ss;
	ss << number;
	return ss.str();
  }

//zxx
  bool isSame(float a, float b)
  {
    float diff = a - b;
    if(diff <= 0.2 && diff >= -0.2)
      return true;
    else return false;
  }

bool findCenter(vector<Point> contour,geometry_msgs::Point &leftcorner , geometry_msgs::Point &center  )
{

	RotatedRect rRect = minAreaRect( contour );
	int x=(int)rRect.center.x, y=(int)rRect.center.y;
 	ROS_WARN("%d,%d,%d,%d",(int)rRect.size.width,(int)rRect.size.height,(int)rRect.center.x,(int)rRect.center.y);
	
	cv::Rect r0 = rRect.boundingRect();
	ROS_WARN("%d,%d,%d,%d",(int)r0.width,(int)r0.height,(int)r0.x,(int)r0.y);
	pcl::PointCloud<pcl::PointXYZ> boxcloud; 
	
	
	center.x = r0.x + 3*r0.width/4;
	center.y = r0.y+r0.height/4;
	int index = center.y*cloud.width + center.x;
	//double dist = cv::pointPolygonTest(contour, cv::Point2f(center.x,center.y), true);
	//ROS_INFO("center dist %f",dist);	
		
	ROS_ERROR("center find 2d,%d,%d ",(int)center.x,(int)center.y); 
	//center.x = cloud.points[index].x; center.y = -cloud.points[index].x; center.z = cloud.points[index].z;
	srvsend = setsrvPose(index);
	if(!srvsend)
	{
		index =(int) (y-rRect.size.height/4)*cloud.width + x;
		srvsend = setsrvPose(index);
	}
	return srvsend;
			
}
//no use findLeftCenter
bool findLeftCenter(vector<Point> contour,geometry_msgs::Point &leftcorner , geometry_msgs::Point &center  )
{

	RotatedRect rRect = minAreaRect( contour );
	int x=(int)rRect.center.x, y=(int)rRect.center.y;
	
	int tz,tzmin;
 	ROS_WARN("%d,%d,%d,%d",(int)rRect.size.width,(int)rRect.size.height,(int)rRect.center.x,(int)rRect.center.y);
	
	cv::Rect r0 = rRect.boundingRect();

	//10> r0.width/5? x=x+r0.width/5: x +=10;
	x=x+r0.width/4;
	ROS_WARN("%d,%d,%d,%d",(int)r0.width,(int)r0.height,(int)r0.x,(int)r0.y);
	pcl::PointCloud<pcl::PointXYZ> boxcloud; 
	int index,miny=r0.y,minindex=r0.y*cloud.width + x,mincnt=0;
	tzmin = 500000;
	ROS_INFO("start");
	for(int iy = r0.y-5;iy<r0.y+r0.height;iy++)
	{	
		index = iy*cloud.width + x;
		if(isnan(cloud.points[index].x) ||isnan(cloud.points[index].y) ||isnan(cloud.points[index].z))
			continue;
		//ROS_INFO("X,Y=%d,%d,x,y,z%f,%f,%f",x,iy,cloud.points[index].x,cloud.points[index].y,cloud.points[index].z);
		tz =(int) (cloud.points[index].z*10000);
		
		if(  tz < tzmin )
		{
			ROS_INFO("TZ < TZMIN");
			minindex = index; miny = iy; mincnt=0;
			tzmin = tz;
		}
		else if ( tz == tzmin )
		{mincnt+=1;}
	}
	miny = miny+mincnt/2;
	minindex = miny*cloud.width + x;
ROS_ERROR("left_center find 2d,%d,%d ",(int)x,miny); 

	srvsend = setsrvPose(minindex);
	if(srvsend)
		{ROS_INFO("end");return true;}

//ros::shutdown();
	/*
	center.x = r0.x + 3*r0.width/4;
	center.y = r0.y+r0.height/4;
	int index = center.y*cloud.width + center.x;
	//double dist = cv::pointPolygonTest(contour, cv::Point2f(center.x,center.y), true);
	//ROS_INFO("center dist %f",dist);	
		
	ROS_ERROR("center find 2d,%d,%d ",(int)center.x,(int)center.y); 
	//center.x = cloud.points[index].x; center.y = -cloud.points[index].x; center.z = cloud.points[index].z;
ROS_INFO("center start");
	srvsend = setsrvPose(index);
	if(!srvsend)
	{
		ROS_INFO("USING MIN AREA RECT");
		index =(int) (y-rRect.size.height/4)*cloud.width + x;
		srvsend = setsrvPose(index);

	}
	//return srvsend;
	int centerindex = index;
ROS_INFO("center end\n\nleft ->edge center start");
	for( x = r0.x+r0.width+2 ; x > r0.x+3.0/4.0*r0.width; x--)
	{
		for(y = r0.y+r0.height/2 ; y > r0.y+r0.height/4; y--)
		{
			//dist = cv::pointPolygonTest(contour, cv::Point2f(x,y), true);
			//ROS_INFO("dist %f",dist);
			//if( dist > 0)
			
			index = y*cloud.width + x;
			if(isnan(cloud.points[index].x) ||isnan(cloud.points[index].y) ||isnan(cloud.points[index].z))
				continue;
			else if(fabs(cloud.points[index].z-cloud.points[centerindex].z)<0.05)
			{
				leftcorner.x = x; leftcorner.y = y;  
				ROS_ERROR("left find 2d,%d,%d ",(int)leftcorner.x,(int)leftcorner.y); 
				//leftcorner.x = cloud.points[index].x; leftcorner.y = -cloud.points[index].x; leftcorner.z = cloud.points[index].z; return true;
				x = x-0.333*r0.width;//edge of center
				index = y*cloud.width + x;
				if(isnan(cloud.points[index].x) ||isnan(cloud.points[index].y) ||isnan(cloud.points[index].z))
				continue;
				srvsend = setsrvPose(index);
				if(srvsend)
					return true;
			}
				
						
		}
	}
	ROS_INFO("left ->edge center end\n");
	*/
	return false;
}
  


  bool contoursDetect(cv::Mat &im)
  {
    int x,y,ind;
    x = y = ind = 0;
    cv::Mat image, image1, kernel;
    image = im;
   
    
    double minval, maxval;
    minMaxIdx(image, &minval, &maxval);
    cv::threshold(image, image, 0.05, 255, CV_THRESH_BINARY);

    image.convertTo(image, CV_8U);
    cameraFeed = image;

    cv::namedWindow(OPENCV_WINDOW_4);
    cv::imshow(OPENCV_WINDOW_4, image);

    cv::medianBlur(image, image, 19);

    kernel = cv::getStructuringElement(0, cv::Size(19,19));

    cv::dilate(image, image, kernel);
    cv::morphologyEx(image, image, CV_MOP_CLOSE, kernel, cv::Point(-1, -1), 2);
    cv::erode(image, image, kernel);
    
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Canny(image,image, 235, 255, 3);
    cv::findContours(image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    cv::Mat result(image.size(), CV_8U, cv::Scalar(0));
    cv::drawContours(result, contours, -1, cv::Scalar(255), 2);
    cv::namedWindow("contours");
    cv::imshow("contours", result);


    	double refArea = 0;
	bool objectFound = false;

	int cmin = 30;  // minimum contour length  //Change these two parameters
	int cmax = 500; // maximum contour length  
	vector<vector<Point> >::iterator itc = contours.begin();
	while (itc != contours.end())
	{

		if (itc->size() >= cmin && itc->size() <= cmax)
			++itc;
		else itc = contours.erase(itc);
		
	}

	vector<vector<Point> >::const_iterator itcon = contours.begin();
	
	for (; itcon != contours.end(); ++itcon)
	{
		geometry_msgs::Point leftcorner, center;
		if(findLeftCenter(*itcon, leftcorner ,  center))
			{ROS_INFO("end");return true;}//find one , then return;
	}

	return false;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {	if(!receive) return;
    cv_bridge::CvImagePtr cv_ptr;
												   
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  // cv::FileStorage fs("desktop_ref.xml", cv::FileStorage::WRITE);
  // fs<<"desktop_ref"<<cv_ptr->image;
   
    cv::namedWindow("original img");
    cv::imshow("original img", cv_ptr->image);
    cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("desktop_ref.xml", cv::FileStorage::READ); 
    fs["desktop_ref"]>>ref;

    cv::Mat trans = ref - cv_ptr->image ;
    trans.convertTo(ref, CV_32FC1);

    contoursDetect(ref);
/*
for (int i = 0;i < pointVector.points.size();i++)
{
geometry_msgs::Pose pose;
int index = pointVector[i].y*cloud.width + pointVector[i].x;
pose.position.x = cloud.points[index].x;
pose.position.y  = -cloud.points[index].y;
pose.position.z = cloud.points[index].z;

matrix_trans(pose);
}
*/

}
 void imageCbc(const sensor_msgs::ImageConstPtr& msg)								  //callback function defination
  {
     
    cv_bridge::CvImagePtr cv_ptr;													   
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    } imwrite("cabimg.jpg",cv_ptr->image);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cabinet_center");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_,image_subc_;
 mat_kinect<<-0.715875023405966,0.701298896684548,-0.00740957341752352,-459.536164568441,
-0.698172278084700,-0.718013377607625,-0.00546549008303445,798.309923384899,
-0.0262345080443927,0.0432323709600252,-1.03615505642471,1317.01144610970,
0,0,0,0;

  ros::Rate loop_rate(100);
  object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, imageCb);      
  image_subc_ = it_.subscribe("/camera/rgb/image_raw", 1, imageCbc); 
  ros::ServiceServer service = nh_.advertiseService("/srvPoseDepth",getsrvPose);
  ros::spin();
  
  return 0;
}
