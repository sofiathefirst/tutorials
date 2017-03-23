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
#include <vector>

  using namespace cv;
  using namespace std;

  static const std::string OPENCV_WINDOW_1 = "Image window1";
  static const std::string OPENCV_WINDOW_2 = "Image window2";
  static const std::string OPENCV_WINDOW_3 = "Image window3";
  static const std::string OPENCV_WINDOW_4 = "Image window4";

ros::Publisher cv_p_pub;

const string windowName3 = "OPENCV_WINDOW_depth";

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 2*2;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

double cameraM[3][3] = {{ 521.906925, 0, 330.571743}, {0, 522.912033, 267.759287}, {0, 0, 1}} ;                            
Mat cameraMatrix = Mat(3, 3, CV_64FC1, cameraM);//.inv();

double distortionC[5] = {0.160792,-0.279956,-0.000324,-0.000310, 0};				//distortioncoefficient to be edited
Mat distCoeffs = Mat(1, 5, CV_64FC1, distortionC);							

double rArray[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
Mat RArray = Mat(3, 3, CV_64FC1, rArray);					//originally CV_64F

double newCameraM[3][3] = {{532.206543, 0, 330.202770}, {0, 533.930420, 267.077297}, {0, 0, 1}};
Mat NewCameraMatrix = Mat(3, 3, CV_64FC1, newCameraM);
Size UndistortedSize(640, 360);

Mat map1;
Mat map2;	

int xa,ya;
int xa_old, ya_old;
int x_offset=0;                 //upper left position of region of intest
int y_offset=0;                 //upper left position of region of intest
int n = 0;
int object_x[MAX_NUM_OBJECTS],object_y[MAX_NUM_OBJECTS];
float dist_val[MAX_NUM_OBJECTS];
float X_dist[MAX_NUM_OBJECTS],Y_dist[MAX_NUM_OBJECTS],Z_dist[MAX_NUM_OBJECTS];;
float fx_depth=597.5;
float fy_depth=597.5;
float offset = 5;
geometry_msgs::Twist cv_p;
////////////////definition of background substraction//////////////

 Mat fgMaskMOG; //fg mask generated by MOG method
 Ptr <BackgroundSubtractor> pMOG; //MOG Background subtractor

////////////////////////////////////////////////////////////////////

  bool checkExist(int xa[MAX_NUM_OBJECTS], int ya[MAX_NUM_OBJECTS], geometry_msgs::Twist cv_p)
  {
    if (cv_p.linear.x == 0 && cv_p.linear.y == 0)
      return false;
    
    int i = 0;
    while(xa[i] && ya[i])
    {
      float xe = cv_p.linear.x - xa[i];
      float ye = cv_p.linear.y - ya[i];

      if(xe > offset || xe < -offset || ye > offset || ye < -offset)
        return true;
      i++;
    }

    return false;
  }
  void insertData(int xa[MAX_NUM_OBJECTS], int ya[MAX_NUM_OBJECTS], geometry_msgs::Twist cv_p)
  {
    int i = 0;
    while(xa[i] && ya[i]) i++;
    cv_p.linear.x = xa[i-1];
    cv_p.linear.y = ya[i-1];
  }
  bool get3dData(Mat &image)
  {
    int i = 0;
 
    memset(dist_val,0,MAX_NUM_OBJECTS);
    while(object_x[i] && object_y[i])
    {
      dist_val[i] = image.at<float>( object_y[i], object_x[i] );
      X_dist[i]=dist_val[i]*(object_x[i]-320)/fx_depth;     
      Y_dist[i]=dist_val[i]*(object_y[i]-240)/fy_depth;
      Z_dist[i]=dist_val[i];
      i++;
    }

    if(i == 0)
      return false;
    else return true;
  }
  string intToString(int number)
  {
	std::stringstream ss;
	ss << number;
	return ss.str();
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
    int x,y;
    x = y = 0;
    cv::Mat image, image1, kernel;
    image = im;
   
    
    double minval, maxval;
    minMaxIdx(image, &minval, &maxval);
    cv::threshold(image, image, 0.05, 255, CV_THRESH_BINARY);

    image.convertTo(image, CV_8U);
    Mat cameraFeed = image;

    cv::namedWindow(OPENCV_WINDOW_4);
    cv::imshow(OPENCV_WINDOW_4, image);

    cv::medianBlur(image, image, 19);

    kernel = cv::getStructuringElement(2, cv::Size(3,3));
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
                if(area>MIN_OBJECT_AREA && area>refArea)
                {
		  x = moment.m10/area;
		  y = moment.m01/area;
		//objectFound = true;
					
	//	  refArea = area;
				
		  if (n == 0)
		  {
		    xa_old = x;
		    ya_old = y;
                    object_x[index] = x+x_offset;
                    object_y[index] = y+y_offset;
		    n = 1;
		  }
					
		  if ((abs(xa_old-x)<= 2) && (abs(ya_old-y) <= 2))
		  {
		    xa = xa_old;
		    ya = ya_old;
                    object_x[index] = x+x_offset;
                    object_y[index] = y+y_offset;
		  }
		  else 
		  {	
                    xa = x+x_offset;
                    ya = y+y_offset;
                    object_x[index] = x+x_offset;
                    object_y[index] = y+y_offset;
                    xa_old = x+x_offset;
                    ya_old = y+y_offset;    
                  }
		}
				//else if( area  refArea )objectFound = false;
	     }
			//let user know you found an object
			//if(objectFound == true)
			//{
          }
          else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}

/*         cameraFeed = 255 - cameraFeed;
	 putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
	 //draw object location on screen
	   drawObject(object_x,object_y,cameraFeed);
        cv::namedWindow(OPENCV_WINDOW_2);
        cv::imshow(OPENCV_WINDOW_2, cameraFeed);*/
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

	/*	image working procedure starting from here inside the main function.
	 *  The purpose of the image processing is to use the existing video to working out the 
	 *  cordinate of the detected object, using color extraction technique.
	 */
	
    bool trackObjects = true;
    bool useMorphOps = true;

	Mat cameraFeed;
        Mat DistortedImg;
	Mat HSV_1;
    
	Mat threshold_1;
	//x and y values for the location of the object
	int x=0, y=0;

		//store image to matrix
		cv_ptr->image.copyTo(DistortedImg);											//=Tan= copy the image from ardrone to DistortedImg for processing
		initUndistortRectifyMap(cameraMatrix, distCoeffs, RArray, NewCameraMatrix, UndistortedSize, CV_32FC1, map1, map2);
		remap(DistortedImg, cameraFeed, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));		// maybe try this one instead of the above two instead: cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
//////////////////////////////////////////background substraction 1////////////////////////////////////////////////////////////////

  pMOG->operator()(cameraFeed, fgMaskMOG);
//  imwrite( "fgMaskMOG.jpg", fgMaskMOG);
////////////////////////////////////////HSV/////////////////////////////////////////////////////////////////////////////////////

              // Mat fgMaskMOG_ROI(fgMaskMOG,Rect(100, 170, 340, 160));//设置ROI 
    contoursDetect(fgMaskMOG);
	
     putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
	   drawObject(object_x,object_y,cameraFeed);
        cv::namedWindow(OPENCV_WINDOW_2);
        cv::imshow(OPENCV_WINDOW_2, cameraFeed);
     if(!checkExist(object_x,object_y,cv_p))
     {
       cv_p.linear.x = object_x[0];
       cv_p.linear.y = object_y[0];
     }
     cv_p_pub.publish(cv_p);
     ROS_INFO("target.x is %.2f, target.y is %.2f)",cv_p.linear.x,cv_p.linear.y);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv::imshow(windowName3, cv_ptr->image);
    cv::waitKey(3);

    cv::Mat ref;

    cv::Mat trans = cv_ptr->image ;
    trans.convertTo(ref, CV_32FC1);


    if(get3dData(cv_ptr->image))
    {
      int dist = 0;
/*      while(dist_val[dist])
      {
//        ROS_INFO("%d, %d, %d, %f", dist,object_x[dist], object_y[dist], dist_val[dist]);
        ROS_INFO("%d, %f, %f, %f", dist,X_dist[dist], Y_dist[dist], Z_dist[dist]);
        dist++;
      }*/
    }

  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiple_rgb");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_depth;
  
  
  cv_p.linear.x=0;
  cv_p.linear.y=0;
  cv_p.linear.z=0;
  cv_p.angular.x=0;
  cv_p.angular.y=0;
  cv_p.angular.z=0;

  pMOG = new BackgroundSubtractorMOG();
  ros::Rate loop_rate(100);
  cv_p_pub = nh_.advertise<geometry_msgs::Twist>("cv_position",1);
  image_sub_rgb = it_.subscribe("/camera/rgb/image_raw", 1, imageCb);	
  image_sub_depth = it_.subscribe("/camera/depth/image", 1, depthCb);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }
  
  return 0;
}