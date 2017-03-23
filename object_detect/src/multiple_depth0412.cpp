#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>                       //zliu7
#include <opencv2/video/background_segm.hpp>           //zliu7
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <stdio.h>
#include<geometry_msgs/Pose.h>
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
  static const std::string OPENCV_WINDOW_5 = "Image window5";
const int FRAME_WIDTH = 1288;
const int FRAME_HEIGHT = 964;
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

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
object_detect::PointVector pointVector,corners;	      //bzj
ros::Publisher object_center_pub;   //zxx
ros::Publisher object_corner_pub;   //zxx
ros::Subscriber object_pcl;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool receive = false;
bool receive2 = false;
cv::Mat color;
//zxx
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud);
  receive = true;
//  cloud.points.at<float>(256,64);
}
//zxx
  geometry_msgs::Point findRightcorner(vector<Point> contour)
  {
    geometry_msgs::Point result;
    result.x = 0;
    result.y = 480;
    if(contour.size() <= 0)
    {
      return result;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(result.x  <= contour.at(i).x)          
        {
          result.x = contour.at(i).x;
        }
      }
      for(int j = 0; j < contour.size(); j++)
      {
        if(fabs(contour.at(j).x - result.x) <= 1)
        {
          if(result.y  <= contour.at(j).y)          
          {
            result.y = contour.at(j).y;
            result.x = contour.at(j).x;
          }
        }
      }
    }
    ROS_INFO("right CORNER %f,%f",result.x,result.y);
    return result;
  }
//zxx
  geometry_msgs::Point findLeftcorner(vector<Point> contour)
  {
    geometry_msgs::Point result;
    result.x = 0;
    result.y = 0;
//        if(result.x + result.y >= contour.at(i).x + contour.at(i).y)            //lzc
    if(contour.size() <= 0)
    {
      return result;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(result.x  <= contour.at(i).x)          
        {
          result.x = contour.at(i).x;
        }
      }
      for(int j = 0; j < contour.size(); j++)
      {
        if(fabs(contour.at(j).x - result.x) <= 1)
        {
          if(result.y  <= contour.at(j).y)          
          {
            result.y = contour.at(j).y;
            result.x = contour.at(j).x;
          }
        }
      }
    }
    ROS_INFO("RIGHT CORNER %f,%f",result.x,result.y);
    return result;
  }
//zxx
  bool getData(geometry_msgs::Point &data)
  {
    if(receive == false)
      return false;
    int i = 0, index =0, num = 0;
    geometry_msgs::Point point;

    index = data.y*cloud.width + data.x;
    point.x = cloud.points[index].x;
    point.y = -cloud.points[index].y;
    point.z = cloud.points[index].z;      
    if(point.x == 0 && point.y == 0 && point.z == 0)
      return false;
    else 
    {
      data = point;
      return true;
    }
  }

//zxx
  bool isSame(float a, float b)
  {
    float diff = fabs(a - b);
    if(diff <= 0.08)
      return true;
    else return false;
  }


//zxx
  bool get3dData(Mat &image)
  {
    int length = 0, width = 0;
    geometry_msgs::Point center,corner,temp;
    float diff[2] = {0};
    if(!pointVector.points.size())
      return false;
    for(int i = 0; i<pointVector.points.size();i++)
    {
      center = pointVector.points.at(i);
      corner = corners.points.at(i);
      temp = corner;
      getData(center);
      getData(corner);
      
ROS_INFO("CENTER POINT %f,%f,%f",center.x,center.y,center.z);
ROS_INFO("CORNER POINT %f,%f,%f",corner.x,corner.y,corner.z);
/*      while(!isSame(center.z,corner.z))
      {
        if(corners.flags.at(i).data==true)
        {
          temp.x += 2;
          temp.y += 2;
          corner = temp;
          getData(corner);
        }
        else 
        {
          temp.x -= 2;
          temp.y += 2;
          corner = temp;
          getData(corner);
        }
      }*/
      corners.points.at(i) = corner;
    }
    return true;
  }
  float findObjectHeigth(vector<Point> contour)
  {
    float min = 480,max = 0;
    if(contour.size() <= 0)
    {
      return 0;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(min  >= contour.at(i).y)          
        {
          min = contour.at(i).y;
        }
        if(max  <= contour.at(i).y)          
        {
          max = contour.at(i).y;
        }
      }
    }
    return (max-min);
  }
//zxx
  bool findEdgeLength(vector<Point> contour,geometry_msgs::Point& min, geometry_msgs::Point& max)
  {
    geometry_msgs::Point result;
    result.x = result.y = 0;
    min.x = 640;max.x = 0;
    min.y = 480,max.y=0;
    float ymin = 480;
    if(contour.size() <= 0)
    {
      return false;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(ymin  >= contour.at(i).y)          
        {
          ymin = contour.at(i).y;
        }
      }
      for(int i = 0; i < contour.size(); i++)
      {
        if( fabs(ymin - contour.at(i).y) <= 2)
        {
          if(max.x  <= contour.at(i).x)          
          {
            max.x = contour.at(i).x;
            max.y = contour.at(i).y;
          }
          if(min.x  >= contour.at(i).x)          
          {
            min.x = contour.at(i).x;
            min.y = contour.at(i).y; 
          }
        }
      }
    }
    ROS_INFO("LENGTH %f,%f,%f,%f",min.x,max.x,min.y,max.y);
    
    return true;
  }
//zxx
  geometry_msgs::Point findContourPoint(vector<Point> contour,geometry_msgs::Point point)
  {
    geometry_msgs::Point result,temp1,temp2;
    if(contour.size() > 0)
    {
      temp1 = point;
      getData(temp1);
      for(int i = 0; i<contour.size();i++)
      {
        if(fabs(point.y - contour.at(i).y) <= 3 && (point.x - contour.at(i).x) <= 0)
        {     ROS_INFO("TEMP %d,%d",contour.at(i).x,contour.at(i).y);
          temp2.x = contour.at(i).x;
          temp2.y = contour.at(i).y;
          getData(temp2);
          if(fabs(temp1.z - temp2.z) <= 0.07)
          {
            result.x = contour.at(i).x;
            result.y = contour.at(i).y;
          }
        }
      }
    }

    return result;
  }
//zxx
  geometry_msgs::Point findObjectEdge(vector<Point> contour)
  {
    geometry_msgs::Point result,lbottom,rbottom,temp;
    float height=0,ycounter = 0,zmin = 3;
    findEdgeLength(contour,lbottom,rbottom);
    height = findObjectHeigth(contour);
    if(lbottom.x == 0 && lbottom.y == 0 && rbottom.x == 0 && rbottom.y == 0 && height == 0)
    {
      return result;
    }
    else
    {
      result.x = (int)(lbottom.x+rbottom.x)/2;
      result.y = lbottom.y;
      for(int i = 0; i < height-5;i++)
      {
        temp = result;
        if(getData(temp))
        {
          if(zmin > temp.z)
          {
            zmin = temp.z;
            ycounter = result.y;
          }
        }
        result.y++;
      }
      result.y = ycounter;
    }
    temp = findContourPoint(contour,result);
    ROS_INFO("RESULT1 %f,%f",result.x,result.y);
    ROS_INFO("TEMP %f,%f",temp.x,temp.y);
    if(temp.x != 0 && temp.y != 0)
    {
      float delta;
      if(fabs(temp.x-rbottom.x) > fabs(temp.x - lbottom.x))
      {
        delta = fabs(temp.x - lbottom.x);
        result.x -= delta;
      }
      else
      {
        delta = fabs(temp.x - rbottom.x);
        result.x += delta;
      }
    }
    
    ROS_INFO("RESULT2 %f,%f",result.x,result.y);
    return result;
  }
//zxx
bool checkPosition(const int x[MAX_NUM_OBJECTS], const int y[MAX_NUM_OBJECTS], std::vector<std::vector<cv::Point> > contours)
{
  int num = 0;
  int pre_x = -1;
  int pre_y = -1;
  while(x[num]>150 && y[num]>50 && x[num] <=600 && y[num]<=450)
  {
    if(x[num] == pre_x && y[num] == pre_y )
    {
      num++;
      continue;
    }
    else
    {
      pre_x = x[num];
      pre_y = y[num];
      geometry_msgs::Point point1, point2;
      point1.x = x[num];
      point1.y = y[num];
      pointVector.points.push_back(point1);
      point2 = findObjectEdge(contours.at(num));
      if(point2.x!=0&&point2.y!=0)
        corners.points.push_back(point2);
      num++;
    }
    
  }
  if(num)
    return true;
  else return false;
}
  string intToString(int number)
  {
	std::stringstream ss;
	ss << number;
	return ss.str();
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
    Mat cameraFeed = image;

    cv::namedWindow(OPENCV_WINDOW_4);
    cv::imshow(OPENCV_WINDOW_4, image);

    cv::medianBlur(image, image, 11);

    kernel = cv::getStructuringElement(0, cv::Size(11,11));

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
                if(area>MIN_OBJECT_AREA)
                {
		  x = moment.m10/area;
		  y = moment.m01/area;
                  //if(x)
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
           drawCorner(corners,cameraFeed);
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

//    cv::FileStorage fs("box_ref.xml", cv::FileStorage::WRITE);
//    fs<<"box_ref"<<cv_ptr->image;

    cv::namedWindow(OPENCV_WINDOW_1);
    cv::imshow(OPENCV_WINDOW_1, cv_ptr->image);
    cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("/home/lzc/moveit/box_ref.xml", cv::FileStorage::READ); 
    fs["box_ref"]>>ref;


    cv::Mat trans = ref - cv_ptr->image ;
    trans.convertTo(ref, CV_32FC1);

    contoursDetect(ref);
    if(receive2 == true)
    {
      drawCorner(corners,color);
      cv::namedWindow(OPENCV_WINDOW_5);
      cv::imshow(OPENCV_WINDOW_5, color);
      cv::waitKey(3);      
    }
   if(receive && get3dData(cv_ptr->image))
    {
      ROS_INFO("get the center and corner of the objects");

      object_center_pub.publish(pointVector);
      object_corner_pub.publish(corners);

    }
    pointVector.points.clear();
    corners.points.clear();	
  }

  void imageCbRGB(const sensor_msgs::ImageConstPtr& msg)
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
    }
    color = cv_ptr->image;
    receive2 = true;
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiple_detect");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub_rgb;
  ros::Rate loop_rate(100);
  image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, imageCb);    
  image_sub_rgb = it_.subscribe("/camera/rgb/image_color", 1, imageCbRGB);   
   object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

 object_center_pub = nh_.advertise<object_detect::PointVector>("/object_center_pub",1);        //zxx
 object_corner_pub = nh_.advertise<object_detect::PointVector>("/object_corner_pub",1);        //zxx          
  ros::spin();
  
  return 0;
}
