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
#include "object_detect/PointVector.h"	  //bzj
#include <vector>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>

  using namespace pcl;
  using namespace pcl::io;
  using namespace cv;
  using namespace std;
 using namespace Eigen;
  using namespace std;
Matrix<float,4,4> mat_kinect;

  bool matrix_trans(geometry_msgs::Pose& pose)
  {
    Matrix<float,4,1> Pr;
    Matrix<float,4,1> Pc;
    Pc << pose.position.x*1000,
          pose.position.y*1000,
          pose.position.z*1000,
                    1;

   Pr = mat_kinect *Pc;

   pose.position.x = Pr(0,0)/1000;
   pose.position.y = Pr(1,0)/1000;
   pose.position.z = Pr(2,0)/1000;
   if(isnan(pose.position.x) || isnan(pose.position.y) || isnan(pose.position.z))
     return false;
   else 
     ROS_WARN("rotation:%f,%f,%f",pose.position.x,pose.position.y,pose.position.z);
   return true;
  }  
  static const std::string OPENCV_WINDOW_1 = "Image window1";
  static const std::string OPENCV_WINDOW_2 = "Image window2";
  static const std::string OPENCV_WINDOW_3 = "Image window3";
  static const std::string OPENCV_WINDOW_4 = "Image window4";
  const int FRAME_WIDTH = 1288;
  const int FRAME_HEIGHT = 964;
  const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
  const int MIN_OBJECT_AREA = 10*10;
  const int MAX_OBJECT_AREA =  100000;//FRAME_HEIGHT*FRAME_WIDTH/1.5;

int xa,ya;
int xa_old, ya_old;
int x_offset=0;                 //upper left position of region of intest
int y_offset=0;                 //upper left position of region of intest
int n = 0;
std::vector<geometry_msgs::Point> right_corner, left_corner;
int object_x[MAX_NUM_OBJECTS],object_y[MAX_NUM_OBJECTS];
//float dist_val[MAX_NUM_OBJECTS];
float f_x=585, f_y=585;
float X_dis,Y_dis,Z_dis;
geometry_msgs::Point point;           //bzj
object_detect::PointVector pointVector;	      //bzj
object_detect::PointVector rightCorner, leftCorner,right2d,left2d, right_pre, right_next;   //zxx
ros::Publisher object_center_pub;   //zxx
ros::Publisher object_center_2d_pub;   //zxx
ros::Publisher object_corner_left_pub;   //zxx
ros::Publisher object_corner_right_pub;   //zxx
ros::Publisher object_corner_left_2d_pub;   //zxx
ros::Publisher object_corner_right_2d_pub;   //zxx
ros::Subscriber object_pcl;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool receive = false;
int count_frame = 0;
    Mat cameraFeed;
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

void drawObject(int x, int y,Mat &frame){  //RED TARGET

	//use some of the openCV drawing functions to draw crosshairs
	//on the tracked image!

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);

    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

  bool findContourArea(vector<Point> contour, geometry_msgs::Point& min, geometry_msgs::Point& max)
  {
	  min.x = min.y = max.x = max.y = 0;
	  if(contour.size() <= 0)
	  {
		  return false;
	  }
	  min.x = max.x = contour.at(0).x;
	  min.y = max.y = contour.at(0).y;
	  for(int i = 0; i < contour.size(); i++)
	  {
		  if(min.x > contour.at(i).x)
		  {
			  min.x = contour.at(i).x;
		  }
		  if(min.y > contour.at(i).y)
		  {
			  min.y = contour.at(i).y;
		  }
		  if(max.x < contour.at(i).x)
		  {
			  max.x = contour.at(i).x;
		  }
		  if(max.y < contour.at(i).y)
		  {
			  max.y = contour.at(i).y;
		  }
	  }
          ROS_INFO("MIN MAX %f,%f,%f,%f",min.x,min.y,max.x,max.y);
	  return true;
  }


//zxx
  bool isSame(float a, float b)
  {
    float diff = a - b;
    if(diff <= 0.2 && diff >= -0.2)
      return true;
    else return false;
  }

  void drawCorner(object_detect::PointVector pVector, Mat &frame){  //RED TARGET

	//use some of the openCV drawing functions to draw crosshairs
	//on the tracked image!
    int i = 0;
    int x,y; x = y =0;
    for(int i = 0; i<pVector.points.size();i++)
   {
       x = pVector.points[i].x;
       y = pVector.points[i].y;
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);

    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
    }
}
  void drawObject(int xa[MAX_NUM_OBJECTS], int ya[MAX_NUM_OBJECTS],Mat &frame){  //RED TARGET

	//use some of the openCV drawing functions to draw crosshairs
	//on the tracked image!
    int i = 0;
    int x,y; x = y =0;
    while(xa[i] && ya[i])
   {
       x = xa[i];
       y = ya[i];
       i++;
	circle(frame,Point(x,y),20,Scalar(0,255,0),2);

    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);
    }
}

  void contoursDetect(cv::Mat &im)
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
    cv::namedWindow(OPENCV_WINDOW_3);
    cv::imshow(OPENCV_WINDOW_3, result);

        memset(object_x,0,MAX_NUM_OBJECTS);
        memset(object_y,0,MAX_NUM_OBJECTS);
    	double refArea = 0;
	bool objectFound = false;
	
	if (hierarchy.size() > 0) 			//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
	{
                
		int numObjects = hierarchy.size();
		
        if(numObjects<MAX_NUM_OBJECTS)
        {
	  for (int index = 0; index < numObjects; index++) 
	    {
		Moments moment = moments((cv::Mat)contours[index]);	//use moments method to find our filtered object
		double area = moment.m00;

	//if the area is less than 20 px by 20px then it is probably just noise
	//if the area is the same as the 3/2 of the image size, probably just a bad filter
	//we only want the object with the largest area so we safe a reference area each
	//iteration and compare it to the area in the next iteration.
        //if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                if(area>MIN_OBJECT_AREA && area <MAX_OBJECT_AREA)
                {
		  x = moment.m10/area;
		  y = moment.m01/area;
                    object_x[index] = x;
                    object_y[index] = y;
		//objectFound = true;
					
		}
				//else if( area  refArea )objectFound = false;
	     }
			//let user know you found an object
			//if(objectFound == true)
			//{
          }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
        cameraFeed = 255 - cameraFeed;
	putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);

        //delete the same data and get the corner of the object
        checkPosition(object_x,object_y,contours);

	 //draw object location on screen
	   drawObject(object_x,object_y,cameraFeed);
         
        cv::namedWindow(OPENCV_WINDOW_2);
        cv::imshow(OPENCV_WINDOW_2, cameraFeed);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
 //   cv::FileStorage fs("box_ref.xml", cv::FileStorage::WRITE);
 //   fs<<"box_ref"<<cv_ptr->image;

    cv::namedWindow(OPENCV_WINDOW_1);
    cv::imshow(OPENCV_WINDOW_1, cv_ptr->image);
    cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("box_ref.xml", cv::FileStorage::READ); 
    fs["box_ref"]>>ref;


    cv::Mat trans = ref - cv_ptr->image ;
    trans.convertTo(ref, CV_32FC1);
//    cv::FileStorage f("box.xml", cv::FileStorage::WRITE);
 //   f<<"box"<<ref;
//    Mat ROI(ref,Rect(140, 100, 500, 300));
    contoursDetect(ref);

for (int i = 0;i < pointVector.points.size();i++)
{
geometry_msgs::Pose pose;
int index = pointVector[i].y*cloud.width + pointVector[i].x;
pose.position.x = cloud.points[index].x;
pose.position.y  = -cloud.points[index].y;
pose.position.z = cloud.points[index].z;

matrix_trans(pose);
}


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiple_detect");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  ros::Rate loop_rate(100);
  image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, imageCb);      
  object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  object_center_2d_pub = nh_.advertise<object_detect::PointVector>("/object_center_2d_pub",1);        //zxx
  object_center_pub = nh_.advertise<object_detect::PointVector>("/object_center_pub",1);        //zxx
  ros::spin();
  
  return 0;
}
