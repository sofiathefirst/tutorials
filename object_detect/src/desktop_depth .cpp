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
   
  static const std::string OPENCV_WINDOW_1 = "Image window1";
  static const std::string OPENCV_WINDOW_2 = "Image window2";
  static const std::string OPENCV_WINDOW_3 = "Image window3";
  static const std::string OPENCV_WINDOW_4 = "Image window4";
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
//zxx
  bool checkLeftCorner()
  {
     if(!leftCorner.points.size())
       return false;
     int index = 0;
     for(int i =0;i<leftCorner.points.size();i++)
     {
       if(fabs(leftCorner.points[i].z - pointVector.points[i].z) >= 0.1)
       {
         do
         {
           left2d.points[i].x += 2;
           left2d.points[i].y += 2;
           index = left2d.points[i].y*cloud.width+left2d.points[i].x;
           leftCorner.points[i].x = cloud.points[index].x;
           leftCorner.points[i].y = -cloud.points[index].y;
           leftCorner.points[i].z = cloud.points[index].z;
         }
         while(fabs(leftCorner.points[i].z - pointVector.points[i].z) >= 0.1);
       }
     }
     return true;
  }
//zxx
  bool checkRightCorner()
  {
    bool check = false;
    if(!right2d.points.size())
      return false;
    if(count_frame == 0)
    {
      right_pre = right2d;
      count_frame++;
//      ROS_INFO("COUNT 1: %f,%f",right_pre.points[0].x,right_pre.points[0].y);
      return false;
    }
    else if(count_frame == 1)
    {
      right_next = right2d;
      count_frame++;
//      ROS_INFO("COUNT 2: %f,%f",right_next.points[0].x,right_next.points[0].y);
      return false;
    }
    else if(count_frame == 2)
    {
/*      if(right2d.points.size() != right_pre.points.size() ||right2d.points.size() != right_next.points.size())
      {
        count_frame = 0;
        return false;
      }*/
      for(int i = 0; i<right2d.points.size();i++)
      {     
        geometry_msgs::Point p = right2d.points.at(i);
        if(fabs(right_pre.points[i].x - p.x) <= 2 && fabs(right_pre.points[i].y - p.y) <= 2 || fabs(right_next.points[i].x - p.x) <= 2 || fabs(right_next.points[i].y - p.y) <= 2)
        {
          check = true;
        }
      }
      right_pre = right_next;
      right_next = right2d;
    }
    return check;
  }
//zxx
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
  geometry_msgs::Point findRightcorner1(vector<Point> contour)
  {
    geometry_msgs::Point result;
    result.x = 0;
    result.y = 500;
    if(contour.size() <= 0)
    {
      return result;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(result.y  >= contour.at(i).y)          
        {
          result.y = contour.at(i).y;
        }
      }
      for(int j = 0; j < contour.size(); j++)
      {
        if((contour.at(j).y - result.y) <= 1)
        {
          if(result.x  <= contour.at(j).x)          
          {
            result.y = contour.at(j).y;
            result.x = contour.at(j).x;
          }
        }
      }
    }
    return result;
  }
//zxx
  geometry_msgs::Point findLeftcorner(vector<Point> contour)
  {
    geometry_msgs::Point result;
    result.x = 2000;
    result.y = 2000;
//        if(result.x + result.y >= contour.at(i).x + contour.at(i).y)            //lzc
    if(contour.size() <= 0)
    {
      return result;
    }
    else
    { 
      for(int i = 0; i < contour.size(); i++)
      {
        if(result.x  >= contour.at(i).x)          
        {
          result.x = contour.at(i).x;
        }
      }
      for(int j = 0; j < contour.size(); j++)
      {
        if((contour.at(j).x - result.x) <= 1)
        {
          if(result.y  >= contour.at(j).y)          
          {
            result.y = contour.at(j).y;
            result.x = contour.at(j).x;
          }
        }
      }
    }
    return result;
  }
//zxx
  geometry_msgs::Point findRightcorner2(geometry_msgs::Point center2d,geometry_msgs::Point min, geometry_msgs::Point max)
  {
    if(receive)
{
    ROS_INFO("RIGHT CORNER");
    geometry_msgs::Point center,dimension2,result;
    int index = 0, x = 0, y = 0;
    index = center2d.y*cloud.width+center2d.x;ROS_INFO("%d,%f,%f",index,center2d.x,center2d.y);
    center.x = cloud.points[index].x;
    center.y = -cloud.points[index].y;
    center.z = cloud.points[index].z;

    result.x = 0;
    result.y = 0;
    result.z = center.z;
    min.x -= 5;
    min.y -= 5;
    max.x += 5;
    max.y += 5;
    for(int i = min.y; i < max.y; i++)
    {
      for(int j = min.x; j < max.x; j++)
      {
        if(j >= 480)
          break;
        if(i >= 640)
          break;
        index  = i*cloud.width + j;
        if(isnan(cloud.points[index].x) ||isnan(cloud.points[index].y) ||isnan(cloud.points[index].z))
          continue;
//ROS_INFO("DIFF %f,%f,%f,%f",(cloud.points[index].x - center.x),diff[0],(-cloud.points[index].y - center.y),diff[1]);  
        if((cloud.points[index].x - center.x)<= 0.15 && (-cloud.points[index].y - center.y) >0)
        { 
          if(cloud.points[index].z < result.z)
          {
            dimension2.x = j; dimension2.y = i;
            result.x = cloud.points[index].x;
            result.y = -cloud.points[index].y;
            result.z = cloud.points[index].z;
          }
        }
      }
    }
ROS_INFO("DIFF %f,%f,%f",result.x,result.y, result.z);
    if(dimension2.x != 0 && dimension2.y != 0)
      right2d.points.push_back(dimension2);
    drawObject(dimension2.x,dimension2.y,cameraFeed);
    return result;
}
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
  bool getDataCenter(Mat &image)
  {
    if(receive == false)
      return false;
    int i = 0, index =0, num = 0;
//    float x = 0, y = 0, z = 0, dist_val = 0.0;
    geometry_msgs::Point point;
//    memset(dist_val,0,MAX_NUM_OBJECTS);
    for(i = 0; i<pointVector.points.size(); i++)
    {
      index = pointVector.points[i].y*cloud.width + pointVector.points[i].x;

      point.x = cloud.points[index].x;
      point.y = -cloud.points[index].y;
      point.z = cloud.points[index].z;      
      pointVector.points[i] = point;
/*      dist_val = image.at<float>( pointVector.points[i].x, pointVector.points[i].y );
      pointVector.points[i].x = (pointVector.points[i].x-320)*dist_val/f_x;
      pointVector.points[i].y = (pointVector.points[i].y-240)*dist_val/f_x;
      pointVector.points[i].z = dist_val;*/

    }
    
    if(i == 0)
      return false;
    else return true;
  }
//zxx
  bool isSame(float a, float b)
  {
    float diff = a - b;
    if(diff <= 0.2 && diff >= -0.2)
      return true;
    else return false;
  }
//zxx
  bool getDataCorner(Mat &image)
  {
    int i = 0, index = 0, num1 = 0, num2 = 0;
    float dist_val[2] = {0};
    geometry_msgs::Point right, left;
    object_detect::PointVector rightPoints, leftPoints;
//    memset(dist_val,0,MAX_NUM_OBJECTS);
    for( i = 0; i < leftCorner.points.size(); i++)
    {
      index = leftCorner.points[i].y*cloud.width + leftCorner.points[i].x;
      if(!isnan(cloud.points[index].z))
      {
        if(isSame(pointVector.points[i].z, cloud.points[index].z))
        {
          left.x = cloud.points[index].x;
          left.y = -cloud.points[index].y;
          left.z = cloud.points[index].z;
          leftPoints.points.push_back(left);
          num2++;
        }  
        leftCorner = leftPoints;
      }
/*      leftCorner.points[i] = point;
      dist_val[0] = image.at<float>( rightCorner.points[i].x, rightCorner.points[i].y );
      dist_val[1] = image.at<float>( leftCorner.points[i].x, leftCorner.points[i].y );
      rightCorner.points[i].x = (rightCorner.points[i].x-320)*dist_val[0]/f_x;
      rightCorner.points[i].y = (rightCorner.points[i].y-240)*dist_val[0]/f_x;
      rightCorner.points[i].z = dist_val[0];
 //     pointVector.points.push_back(rightPoint);

      leftCorner.points[i].x = (leftCorner.points[i].x-320)*dist_val[1]/f_x;
      leftCorner.points[i].y = (leftCorner.points[i].y-240)*dist_val[1]/f_x;
      leftCorner.points[i].z = dist_val[1];ROS_INFO("POINTS %f, %f, %f",leftPoint.x,leftPoint.y,leftPoint.z);*/
//      pointVector.points.push_back(leftPoint);

    }
    
    if(i == 0)
      return false;
    else return true;
  }
  int findArea(geometry_msgs::Point center)
  {
    
    if(center.z >= -0.05)
    {
      return 1;
    }
    else
    {
      return 2;
    }
  }
//zxx
  bool get3dData(Mat &image)
  {
    int length = 0, width = 0;
    geometry_msgs::Point center,lcorner;
    float diff[2] = {0};
    if(!pointVector.points.size())
      return false;
    for(int i = 0; i<pointVector.points.size();i++)
    {
      length = pointVector.points.at(i).x - leftCorner.points.at(i).x;
      width = pointVector.points.at(i).y - leftCorner.points.at(i).y;
      center = pointVector.points.at(i);
      getData(pointVector.points.at(i));
      getData(leftCorner.points.at(i));
      //determine which side
      //center : 2d positon of the object
      //pointVector.points.at(i) : 3d position of the object
    }
    return true;
  }
//zxx
bool checkPosition(const int x[MAX_NUM_OBJECTS], const int y[MAX_NUM_OBJECTS], std::vector<std::vector<cv::Point> > contours)
{
  int num = 0;
  int pre_x = -1;
  int pre_y = -1;
  geometry_msgs::Point min,max;
  while(x[num]>40 && y[num]>100 && x[num] <=500 && y[num]<=400)
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
      point2 = findLeftcorner(contours.at(num));
      findContourArea(contours.at(num),min,max);
      leftCorner.points.push_back(point2);
      left2d.points.push_back(point2);
      point1 = findRightcorner2(point1,min,max);
      rightCorner.points.push_back(point1);
      num++;
    }
    
  }
  if(num)
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
           drawCorner(leftCorner,cameraFeed);
           drawCorner(rightCorner,cameraFeed);
        cv::namedWindow(OPENCV_WINDOW_2);
        cv::imshow(OPENCV_WINDOW_2, cameraFeed);
	waitKey(3);
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
    object_center_2d_pub.publish(pointVector);
    //object_corner_left_2d_pub.publish(leftCorner);
    if(receive && get3dData(cv_ptr->image))
//   if(receive && getDataCenter(cv_ptr->image) && getDataCorner(cv_ptr->image))
    {
      ROS_INFO("get the center and corner of the objects");
      if(checkRightCorner() && checkLeftCorner())
{
      object_center_pub.publish(pointVector);
      //object_corner_left_pub.publish(leftCorner);
     // object_corner_right_pub.publish(rightCorner);
     // object_corner_right_2d_pub.publish(right2d);
     // object_corner_left_2d_pub.publish(left2d); }
      pointVector.points.clear();
     // rightCorner.points.clear();
     // leftCorner.points.clear();
     // left2d.points.clear();
     // right2d.points.clear();
        cv::namedWindow(OPENCV_WINDOW_2);
        cv::imshow(OPENCV_WINDOW_2, cameraFeed);
    }
//    trans2dTo3d(cv_ptr->image);
	
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
  object_corner_left_pub = nh_.advertise<object_detect::PointVector>("/object_corner_left_pub",1);        //zxx 
  object_corner_right_pub = nh_.advertise<object_detect::PointVector>("/object_corner_right_pub",1);        //zxx       
  object_corner_left_2d_pub = nh_.advertise<object_detect::PointVector>("/object_corner_left_2d_pub",1);        //zxx 
  object_corner_right_2d_pub = nh_.advertise<object_detect::PointVector>("/object_corner_right_2d_pub",1);        //zxx         
  ros::spin();
  
  return 0;
}
