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
#include "object_detect/cabinetPose.h"
  using namespace pcl;
  using namespace pcl::io;
  using namespace cv;
  using namespace std;
 using namespace Eigen;
Matrix<float,4,4> mat_kinect;

 
 
  static const  Rect rOI(180, 0, 435,230);
//static const  Rect rOI(0, 0, 640,480);
static const int PIXL_OFFSET = -24;//big box -30,small box 20

std::vector<geometry_msgs::Point> right_corner, left_corner;


geometry_msgs::Point point;           //bzj

ros::Publisher object_center_pub;   //zxx
ros::Publisher object_center_2d_pub;   //zxx

ros::Subscriber object_pcl;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool receive = false;
int count_frame = 0;
    Mat cameraFeed;
Mat colorimg;
geometry_msgs::Pose srvpose;

bool srvsend;
int resx = 0;


float T_data[4][4] = {		0.241919591653694,		0.844294602673505,		0.592067560553167,		-610.386467528557,
						1.04930561312756,		-0.131409689525925,		-0.200188098893888,		607.274109082033,
						0.104642093167853,		-0.650161998230167,		0.764883731680464,		-2176.08013959110,
						0,						0,						0,						1	};

Mat	T4x4_32F(4, 4, CV_32FC1, T_data);

bool ConverKinectToWorld(cv::Point3f &PtInKinect, cv::Mat &T4x4_32F)
{
	int const	TSize = 4; 
	if(T4x4_32F.rows != T4x4_32F.cols && T4x4_32F.rows != TSize)
		return false;
	float	m_pt_k_data[3];
	m_pt_k_data[0] = PtInKinect.x;
	m_pt_k_data[1] = PtInKinect.y;
	m_pt_k_data[2] = PtInKinect.z;
	m_pt_k_data[3] = 1;

	cv::Mat	m_pt_k(4, 1, CV_32FC1, m_pt_k_data);
	cv::Mat	m_pt_w = cv::Mat::zeros(4, 1, CV_32FC1);
	
	m_pt_w = T4x4_32F * m_pt_k;
	srvpose.position.x = -m_pt_w.ptr<float>(1)[0]/1000; 
	srvpose.position.y = -m_pt_w.ptr<float>(0)[0]/1000;
	srvpose.position.z = -m_pt_w.ptr<float>(2)[0]/1000;

	return true;
}



bool setsrvPose(int index)
{
	srvpose.position.z = cloud.points[index].z*1000;
	srvpose.position.x = cloud.points[index].x*1000;
	srvpose.position.y = -cloud.points[index].y*1000;

	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;

	Point3f ptKinect, ptWorld;
        ptKinect = Point3f(srvpose.position.x, srvpose.position.y, srvpose.position.z);
	ptKinect = Point3f(srvpose.position.x, srvpose.position.y, srvpose.position.z);
	ROS_ERROR("before rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
	ConverKinectToWorld(ptKinect, T4x4_32F);

	ROS_ERROR("after rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);

/*
	if(srvpose.position.z>2 ||srvpose.position.z < 1.6 ) return false;
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
	double dist = sqrt(srvpose.position.x * srvpose.position.x +srvpose.position.y*srvpose.position.y);
	ROS_ERROR("DIST:%f",dist);
	if(dist<0.6)
		return false;*/
	ROS_ERROR("rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
        
	return true;
  
}

bool getsrvPose(object_detect::cabinetPose::Request &req,object_detect::cabinetPose::Response &res)
{
	if(req.flag && srvsend)
	{
		res.pose = srvpose;
		res.x = resx;
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

//no use findLeftCenter
bool findLeftCenter(vector<Point> contour,geometry_msgs::Point &leftcorner , geometry_msgs::Point &center  )
{

	RotatedRect rRect = minAreaRect( contour );
	Point2f vertices[4];
	rRect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(colorimg, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));

	int x=(int)rRect.center.x, y=(int)rRect.center.y;
	
	int tz,tzmin;
 	ROS_WARN("RotatedRect width, height ,centerx ,centery%d,%d,%d,%d",(int)rRect.size.width,(int)rRect.size.height,(int)rRect.center.x,(int)rRect.center.y);
	
	cv::Rect r0 = rRect.boundingRect();
	rectangle(colorimg, r0, Scalar(255,0,0));

	//10> r0.width/5? x=x+r0.width/5: x +=10;
	x=r0.x+r0.width + PIXL_OFFSET;

	ROS_WARN("rect width,height x,y%d,%d,%d,%d",(int)r0.width,(int)r0.height,(int)r0.x,(int)r0.y);
	pcl::PointCloud<pcl::PointXYZ> boxcloud; 
	int index,miny=r0.y,minindex,mincnt=0;
	tzmin = 500000;
	ROS_INFO("start");
	for(int iy = r0.y;iy<r0.y+r0.height/2.0;iy++)
	{			index = (rOI.y+iy)*cloud.width + rOI.x+x;
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
	minindex = (rOI.y+miny)*cloud.width + rOI.x +x;
	ROS_ERROR("left_center find 2d,%d,%d ",(int)x,miny); 
	resx =  rOI.x +x;
	line(colorimg, Point2f(x-10,miny) ,Point2f(x+10,miny) , Scalar(0,255,0));
	line(colorimg, Point2f(x,miny-10) ,Point2f(x,miny+10) , Scalar(0,255,0));

	imshow("oriig",colorimg);
	
	srvsend = setsrvPose(minindex);
	if(srvsend)
		{//ROS_INFO("end");
return true;}

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

    cv::medianBlur(image, image, 19);

    kernel = cv::getStructuringElement(0, cv::Size(9,9));

    cv::dilate(image, image, kernel);
    cv::morphologyEx(image, image, CV_MOP_CLOSE, kernel, cv::Point(-1, -1), 2);
    cv::erode(image, image, kernel);
    
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;

    cv::Canny(image,image, 235, 255, 3);
    cv::findContours(image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    cv::Mat result(image.size(), CV_8U, cv::Scalar(0));
   

    	double refArea = 0;
	bool objectFound = false;

	int cmin = 30;  // minimum contour length  //Change these two parameters
	int cmax = 50; // maximum contour length  
	vector<vector<Point> >::iterator itc = contours.begin();
	while (itc != contours.end())
	{
		//ROS_INFO("%d",(int)itc->size());
		if (itc->size() >= cmin && itc->size() <= cmax)
			++itc;
		else itc = contours.erase(itc);
		
	}
    cv::drawContours(result, contours, -1, cv::Scalar(255), 2);
    cv::namedWindow("contours");
    cv::imshow("contours", result);
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
	srvsend = false;
	Mat imgroi = cv_ptr->image;
	imgroi = imgroi(rOI);
	//cv::FileStorage fs("cabnet_refROI380.xml", cv::FileStorage::WRITE);
	//fs<<"cabnet_refROI"<<imgroi;
	
    cv::namedWindow("original img");
    cv::imshow("original img", cv_ptr->image);
	cv::imshow("roi img", imgroi);
    cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("cabnet_refROI380.xml", cv::FileStorage::READ); 
    fs["cabnet_refROI"]>>ref;

    cv::Mat trans = ref - imgroi ;
    trans.convertTo(ref, CV_32FC1);

    contoursDetect(ref);
	//ROS_INFO("srvsend=%d",srvsend);
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
	colorimg = cv_ptr -> image;
	colorimg = colorimg(rOI);
imwrite("cabimgroi.jpg",colorimg);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cabinet_center");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_,image_subc_;
   mat_kinect<<0.625408802070069,-0.254504679641369,0.737799605995627,-789.765926075889,
0.732997492837814,0.263645161223146,-0.627303499018915,330.888975931973,
0.0236277436895945,-0.896255895305970,-0.442589731328521,1298.28225768603,
0,0,0,1;

  ros::Rate loop_rate(100);
  object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  image_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, imageCb);      
  image_subc_ = it_.subscribe("/camera/rgb/image_raw", 1, imageCbc); 
  ros::ServiceServer service = nh_.advertiseService("/srvcabinetPose",getsrvPose);
  ros::spin();
  
  return 0;
}
