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
#include <std_msgs/Time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <vector>

using namespace cv;	
using namespace std;
																	//=T
static const std::string OPENCV_WINDOW = "Image window";

//======================================================================================================================================================
//======================================================================================================================================================
geometry_msgs::Pose2D position;				// create a struct to publish cordinate info later on
ros::Publisher pub;
////////////////definition of background substraction//////////////

 Mat fgMaskMOG; //fg mask generated by MOG method
 Ptr <BackgroundSubtractor> pMOG; //MOG Background subtractor

////////////////////////////////////////////////////////////////////
int iLowH_1 = 0;
int iHighH_1 = 69;
int iLowS_1 = 43;
int iHighS_1 = 140;
int iLowV_1 = 126;
int iHighV_1 = 237;


//default capture width and height
const int FRAME_WIDTH = 1288;
const int FRAME_HEIGHT = 964;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 2*2;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName2 = "Thresholded Image 1";

const string windowName_g = "Gaussian Image 1";

const string trackbarWindowName = "Trackbars";

Mat DistortedImg;											//storage for copy of the image raw
Mat	UndistortedImg;											//



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
int x_offset=20;               
int y_offset=0;                 


double x_init, z_init, x_final, z_final;
double alpha, beta, angle_dif;

double distance_x, distance_z, rotation_y, asymptote;
int n = 0;
int t= 0;

void createTrackbars(){
	//create window for trackbars

    namedWindow("Control_1", CV_WINDOW_AUTOSIZE);
	//create memory to store trackbar name on window
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
 //Create trackbars in "Control" window
	cvCreateTrackbar("LowH_1", "Control_1", &iLowH_1, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH_1", "Control_1", &iHighH_1, 179);

	cvCreateTrackbar("LowS_1", "Control_1", &iLowS_1, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS_1", "Control_1", &iHighS_1, 255);

	cvCreateTrackbar("LowV_1", "Control_1", &iLowV_1, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV_1", "Control_1", &iHighV_1, 255);
	
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

void morphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
    //dilate with larger element so make sure object is nicely visible

	erode(thresh,thresh,getStructuringElement( MORPH_RECT,Size(2,2)));
	dilate(thresh,thresh,getStructuringElement( MORPH_RECT,Size(2,2)));	
}

void trackFilteredObject1(int &x, int &y, Mat threshold, Mat &cameraFeed, int &x_offset, int &y_offset){

	Mat temp;
	threshold.copyTo(temp);				//these two vectors needed for output of findContours
	
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;			//find contours of filtered image using openCV findContours function
	
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
	
	double refArea = 0;
	bool objectFound = false;

	if (hierarchy.size() > 0) 			//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
	{
		int numObjects = hierarchy.size();
		
        if(numObjects<MAX_NUM_OBJECTS)
        {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
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
					
					refArea = area;
					
					if (n == 0)
					{
					xa_old = x;
					ya_old = y;
					n = 1;
					}
					
					if ((abs(xa_old-x)<= 2) && (abs(ya_old-y) <= 2))
					{
						xa = xa_old;
						ya = ya_old;
					}
					else 
					{	
                    xa = x+x_offset;
                    ya = y+y_offset;
                    xa_old = x+x_offset;
                    ya_old = y+y_offset;
                    
                    }
				}
				//else if( area  refArea )objectFound = false;

			}
			//let user know you found an object
			//if(objectFound == true)
			//{
			putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(xa,ya,cameraFeed);

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}


//======================================================================================================================================================
    
  void imageCb(const sensor_msgs::ImageConstPtr& msg)								  //callback function defination
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

	Mat HSV_1;
    
	Mat threshold_1;
	//x and y values for the location of the object
	int x=0, y=0;
	createTrackbars();

		//store image to matrix
		cv_ptr->image.copyTo(DistortedImg);			fgMaskMOG								//=Tan= copy the image from ardrone to DistortedImg for processing
		initUndistortRectifyMap(cameraMatrix, distCoeffs, RArray, NewCameraMatrix, UndistortedSize, CV_32FC1, map1, map2);
		remap(DistortedImg, cameraFeed, map1, map2, INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));		// maybe try this one instead of the above two instead: cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
//////////////////////////////////////////background substraction 1////////////////////////////////////////////////////////////////
    cv::Mat trans = cv_ptr->image ;
//    trans.convertTo(trans, CV_32FC1);
//    cv::FileStorage f("table.xml", cv::FileStorage::WRITE);
//    f<<"table"<<trans;

    cv::FileStorage fs("table.xml", cv::FileStorage::READ); 
    fs["table"]>>trans;

    Mat &pBkImg=trans;


	namedWindow("Gray Background", 1);
	imshow("Gray Background", trans);
       GaussianBlur(pBkImg, pBkImg, Size(3, 3), 0, 0);
       GaussianBlur(cameraFeed, cameraFeed,Size(3,3),0,0);
      Mat pFrMat;
      absdiff(cameraFeed, pBkImg, pFrMat);
      Mat fgMaskMOG=pFrMat;
  pMOG->operator()(cameraFeed, fgMaskMOG);
 imwrite( "fgMaskMOG.jpg", fgMaskMOG);
////////////////////////////////////////HSV/////////////////////////////////////////////////////////////////////////////////////
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV_1,COLOR_BGR2HSV);

		//fiter HSV image between values and store filtered image to
		//threshold matrix
				
		//output the after-threshold matrix to Mat threshold
		inRange(HSV_1,Scalar(iLowH_1, iLowS_1, iLowV_1),Scalar(iHighH_1, iHighS_1, iHighV_1),threshold_1); 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               Mat fgMaskMOG_ROI(fgMaskMOG,Rect(0, 0, 640, 480));
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
		{
			//morphOps(threshold_1);
                        //morphOps(fgMaskMOG);
                        morphOps(fgMaskMOG_ROI);
		}
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects)
		{
		//	trackFilteredObject1(x,y,threshold_1,cameraFeed);
                //        trackFilteredObject1(x,y,fgMaskMOG,cameraFeed);
                        trackFilteredObject1(x,y,fgMaskMOG_ROI,cameraFeed,x_offset,y_offset);
		}


//		imshow(windowName2,threshold_1);
		imshow(windowName,cameraFeed);		
                imshow(windowName_g,fgMaskMOG);
                imwrite("table.jpg", cameraFeed);
//		imshow("HSV",HSV_1);
  cv::waitKey(30);  //to show video steady.
////////////////////////////////////publish object information to path planner///////////////////////////////////////////////
  ros::NodeHandle nh;
  geometry_msgs::Twist cv_p;
  cv_p.linear.x=xa;
  cv_p.linear.y=ya;
  cv_p.linear.z=0;
  cv_p.angular.x=0;
  cv_p.angular.y=0;
  cv_p.angular.z=0;
  ros::Publisher cv_p_pub = nh.advertise<geometry_msgs::Twist>("cv_position",1);
  ROS_INFO("target.x is %.2f, target.y is %.2f)",cv_p.linear.x,cv_p.linear.y);
  std_msgs::Time time;
  time.data=ros::Time::now();
  while(ros::Time::now() - time.data < ros::Duration(0.05))
  {
    cv_p_pub.publish(cv_p);
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "background");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  
  pMOG = new BackgroundSubtractorMOG();
  ros::Rate loop_rate(100);
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &imageCb); 			                                    

  ros::spinOnce();
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }
  
  return 0;
}



