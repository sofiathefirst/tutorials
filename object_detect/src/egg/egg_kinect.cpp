#include <stdlib.h>
#include <iostream>
#include <string>
//【1】
#include <XnCppWrapper.h>
//#include "opencv/cv.h"
//#include "opencv/highgui.h"
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <object_detect/penPoint3D.h>

#include<object_detect/desktopData.h>

int gwidth;
double gangel;
geometry_msgs::Point srvposition;
bool srvsend;
void setSrvPosition(cv::Point3f p, double angle=0)
{
srvposition.x = p.x/1000.;
srvposition.y = p.y/1000.;
srvposition.z = 0.09;//p.z/1000. +0.1; 0.83
ROS_WARN("%f,%f,%f,%f,degree=%f",srvposition.x,srvposition.y,srvposition.z,angle, angle *180./3.1415926);
gangel = angle;
srvsend = true;
}


bool getsrvPosition(object_detect::desktopData::Request &req,object_detect::desktopData::Response &res)
{
	ros::Time start_time=ros::Time::now(); 
	//test2();
	ros::Duration dur1=ros::Time::now() - start_time;
	ROS_WARN("The CALtime is:%f",dur1.toSec());

	if(req.flag && srvsend)
	{
		res.position = srvposition;
		res.angle = gangel;
		res.width = gwidth;
		srvsend = false;
		ros::Duration dur2=ros::Time::now() - start_time;
		ROS_WARN("The CAL + RES time is:%f",dur2.toSec());

		return true;
	}
	return false;
} 


using namespace std;
using namespace cv;

void CheckOpenNIError( XnStatus result, string status )
{ 
	if( result != XN_STATUS_OK ) 
		cerr << status << " Error: " << xnGetStatusString( result ) << endl;
}
void LabColorDetector(cv::Mat&colorbg, cv::Mat& image,cv::Mat&map,vector< vector<Point> >& contours);

int rows=480;
int cols=640;
//Mat map1;
//vector< vector<Point> > contours;

cv::Point star(130,130),end(465,320);

cv::Point3f BasePoint(float x,float y,float z)
{
	/*double _Tt[16] = {-0.71427709, 0.69756144, 0.13570389, -800.55682,
                -0.71068698, -0.70699161, -0.28042895, 711.62848,
                 0.11088955, 0.28929594, -0.91817391, 905.74896,
                  0,           0,           0,          1};*/

double _Tt[16]={-0.70926559, 0.7130816, 0.13123523, -821.27478,
  -0.71648568, -0.66735822, -0.2229622, 666.95911,
  0.072900169, 0.26363891, -0.93118078, 912.82019,
  0, 0, 0, 1};





	cv::Mat Tk_r = cv::Mat(4, 4, CV_64F, _Tt);
	cv::Mat pk=cv::Mat(4, 1, CV_64F,cv::Scalar(0));
	pk.at<double>(0) = x;
	pk.at<double>(1) = y;
	pk.at<double>(2) = z;
	pk.at<double>(3) = 1;
	cv::Mat pr(4, 1, CV_64F, cv::Scalar(0));
	pr = Tk_r*pk;
    cv::Point3f Pk_r(0,0,0);
    Pk_r.x=pr.at<double>(0);
	Pk_r.y=pr.at<double>(1);
	Pk_r.z=pr.at<double>(2);

	return Pk_r;

}

struct SColorPoint3D  
{  
	float  X;  
	float  Y;  
	float  Z;  
	float  R;  
	float  G;  
	float  B;  

	SColorPoint3D( XnPoint3D pos, XnRGB24Pixel color )  
	{  
		X = pos.X;  
		Y = pos.Y;  
		Z = pos.Z;  
		R = (float)color.nRed / 255;  
		G = (float)color.nGreen / 255;  
		B = (float)color.nBlue / 255;  
	}  
}; 
struct Colorflag
{
	float x;
	float y;
	float z;
	int N;
	int flag;

};


int main( int argc, char** argv )
{
	XnStatus result = XN_STATUS_OK;  
	xn::DepthMetaData depthMD;
	xn::ImageMetaData imageMD;

	char key=0;

	//【2】
	// context 
	xn::Context context; 
	result = context.Init(); 
	CheckOpenNIError( result, "initialize context" );  

	// creategenerator  
	xn::DepthGenerator depthGenerator;  
	result = depthGenerator.Create( context ); 
	CheckOpenNIError( result, "Create depth generator" );  
	xn::ImageGenerator imageGenerator;
	result = imageGenerator.Create( context ); 
	CheckOpenNIError( result, "Create image generator" );

	//【3】
	//map mode  
	XnMapOutputMode mapMode; 
	mapMode.nXRes = 640;  
	mapMode.nYRes = 480; 
	mapMode.nFPS = 30; 
	result = depthGenerator.SetMapOutputMode( mapMode );  
	result = imageGenerator.SetMapOutputMode( mapMode );  

	//【4】
	// correct view port  
	depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator ); 

	//【5】
	//read data
	result = context.StartGeneratingAll();  
	//【6】
	result = context.WaitNoneUpdateAll();  

	//OpenCV
	IplImage*  imgDepth16u=cvCreateImage(cvSize(640,480),IPL_DEPTH_16U,1);
	//IplImage* imgRGB8u=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	IplImage*  depthShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	//IplImage* imageShow=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

    cv::Mat imgDepth16u_Mat=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));


	//Mat depthmap1=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));
	//Mat depthmap2=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));
	//Mat depthmap3=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));
	//Mat depthmap4=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));
	//Mat depthmap5=cv::Mat(rows,cols,CV_16UC1,cv::Scalar(0,0,0));

	cv::Mat imgRGB8u=cv::Mat(rows,cols,CV_8UC3,cv::Scalar(0,0,0));




	cvNamedWindow("depth",1);
	//cvNamedWindow("image",1);

	ros::init(argc, argv, "penDetecter");
	ros::NodeHandle n;
	//ros::ServiceServer service = n.advertiseService("/srvDesktopData",getsrvPosition);
	//ros::Publisher //chatter_pub = n.advertise<object_detect::penPoint3D>("penDetecter", 1000);
	ros::Rate loop_rate(10);
	ros::ServiceServer service = n.advertiseService("/srvDesktopData",getsrvPosition);

	while(ros::ok()&&(key!=27) && !(result = context.WaitNoneUpdateAll( ))  ) 
	{  
		srvsend = false;
		//get meta data
		depthGenerator.GetMetaData(depthMD); 
		imageGenerator.GetMetaData(imageMD);
		

		//【7】
		//OpenCV output
		memcpy(imgDepth16u->imageData,depthMD.Data(),640*480*2);
		cvConvertScale(imgDepth16u,depthShow,255/4096.0,0);
		//memcpy(imgRGB8u->imageData,imageMD.Data(),640*480*3);
		//cvCvtColor(imgRGB8u,imageShow,CV_RGB2BGR);
		cvShowImage("depth", depthShow);
		//cvShowImage("image",imageShow);

		memcpy(imgDepth16u_Mat.data,depthMD.Data(),640*480*2);

		//memcpy(depthmap1.data,depthMD.Data(),640*480*2);
		//memcpy(depthmap2.data,depthMD.Data(),640*480*2);
		//memcpy(depthmap3.data,depthMD.Data(),640*480*2);
		//memcpy(depthmap4.data,depthMD.Data(),640*480*2);
		//memcpy(depthmap5.data,depthMD.Data(),640*480*2);
		
		memcpy(imgRGB8u.data,imageMD.Data(),640*480*3);
		cvtColor(imgRGB8u, imgRGB8u, CV_RGB2BGR);
		Mat map;
		vector< vector<Point> > contours;
		//rectangle(imgRGB8u,star,end,Scalar(255,0,0),2);  
		LabColorDetector(imgRGB8u, imgRGB8u,map,contours);

		const XnDepthPixel* pDepth=(XnDepthPixel *)imgDepth16u_Mat.data;
		vector<SColorPoint3D> vPointCloud; 
		unsigned int uPointNum = depthMD.FullXRes() * depthMD.FullYRes();  
		XnPoint3D* pDepthPointSet = new XnPoint3D[ uPointNum ];  
		unsigned int i, j, idxShift, idx;  
		for( j = 0; j < depthMD.FullYRes(); ++j )  
		{  
			idxShift = j * depthMD.FullXRes();  
			for( i = 0; i < depthMD.FullXRes(); ++i )  
			{  
				idx = idxShift + i;  
				pDepthPointSet[idx].X = i;  
				pDepthPointSet[idx].Y = j;  
				pDepthPointSet[idx].Z = pDepth[idx];  
			}  
		}
	  	// un-project points to real world  
		XnPoint3D* p3DPointSet = new XnPoint3D[ uPointNum ];  
		depthGenerator.ConvertProjectiveToRealWorld( uPointNum, pDepthPointSet, p3DPointSet ); 
		delete[] pDepthPointSet;

		unsigned int eggNum=contours.size();
		unsigned int* eggPointNum=new unsigned int[eggNum];
		cv::Point3f* eggPoint3d=new cv::Point3f[eggNum];

		for(int i=0;i<eggNum;++i)
		{
			eggPointNum[i]=0;
			eggPoint3d[i].x=0;
			eggPoint3d[i].y=0;
			eggPoint3d[i].z=0;
		}



		if(eggNum>0)
		{
			for(int v = 0; v < rows; v++)
			{
				for(int u = 0; u <cols; u++)
				{
					unsigned int indx=u+v*cols;
					if(p3DPointSet[indx].Z!=0)
					{

						for(int i=0;i<eggNum;++i)
						{
							if(map.ptr<uchar>(v)[u]>0)
							{
								double dist = cv::pointPolygonTest(contours[i], cv::Point2f(u, v), 0);
								if (dist >= 0)
								{

									eggPointNum[i]++;
									eggPoint3d[i].x+=p3DPointSet[indx].X;
									eggPoint3d[i].y+=p3DPointSet[indx].Y;
									eggPoint3d[i].z+=p3DPointSet[indx].Z;
								}

							}
						}

					}
				}
			}	


			for(int i=0;i<eggNum;++i)
			{
				if(eggPointNum[i]>0)
				{
					eggPoint3d[i].x=eggPoint3d[i].x/eggPointNum[i];
					eggPoint3d[i].y=eggPoint3d[i].y/eggPointNum[i];
					eggPoint3d[i].z=eggPoint3d[i].z/eggPointNum[i];
					cv::Point3f pegg=BasePoint(eggPoint3d[i].x,eggPoint3d[i].y,eggPoint3d[i].z);
					eggPoint3d[i].x=pegg.x;
					eggPoint3d[i].y=pegg.y;
					eggPoint3d[i].z=pegg.z;
					setSrvPosition(pegg);

					cout<<"egg: "<<i<<"  "<<pegg.x<<"\t"<<pegg.y<<"\t"<<pegg.z<<endl;

				}
			}
		


		}
		delete [] p3DPointSet;
		delete [] eggPointNum;
		delete [] eggPoint3d;
               
		cv::imshow("image",imgRGB8u);
		
		key=cvWaitKey(1);

		ros::spinOnce();
		loop_rate.sleep();

	}
        
//srvposition.x

//res.position 
	//destroy
	//cvDestroyWindow("depth");
	//cvDestroyWindow("image");
	//cvReleaseImage(&imgDepth16u);
	//cvReleaseImage(&imgRGB8u);
	//cvReleaseImage(&depthShow);
	//cvReleaseImage(&imageShow);

	context.StopGeneratingAll();
	context.Shutdown();
	return 0;
}
void LabColorDetector(cv::Mat&colorbg, cv::Mat& image,cv::Mat&map,vector< vector<Point> >& contours)
{
	Mat LAB;
	cvtColor(colorbg, LAB, CV_BGR2Lab);
	vector<Mat> lab;
	split(LAB, lab);
	Mat L, A, B;
	lab[0].copyTo(L);
	lab[1].copyTo(A);
	lab[2].copyTo(B);

	imshow("A", A);
	imshow("L", L);
	imshow("B", B);


	Mat YUV;
	cvtColor(colorbg, YUV, CV_BGR2YUV);
	//imshow("hsv", HSV);
	vector<Mat> yuv;
	split(YUV, yuv);
	cv::Mat Y = yuv[0];
	cv::Mat U = yuv[1];
	cv::Mat yV = yuv[2];

	imshow("Y",Y);
	imshow("U", U);
	imshow("yV", yV);


	Mat HSV;
	cvtColor(colorbg, HSV, CV_BGR2HSV);
	//imshow("hsv", HSV);
	vector<Mat> hsv;
	split(HSV, hsv);
	cv::Mat H, S, V;
	hsv[0].copyTo(H);
	hsv[1].copyTo(S);
	hsv[2].copyTo(V);

	imshow("H2", H);
	imshow("S2", S);
	imshow("V2", V);

	Mat mapA;
	A.copyTo(mapA);
	float anlge=380;
	float Ang = 380;
	float a = 258, b = 258, l = 258;
	Mat imgdist = cv::Mat(colorbg.rows, colorbg.cols, CV_8UC3, cv::Scalar(0));
	Mat imgdist1 = cv::Mat(colorbg.rows, colorbg.cols, CV_8UC1, cv::Scalar(0));
	int flag = 0;
	imgdist1.copyTo(map);


	for (int j = star.y; j < end.y; j++)
	{
	
		for (int i = star.x; i < end.x; i++)
		{
		
			float h = float(H.at<uchar>(j, i)) * 360 / 255.0;
			float s = float(S.at<uchar>(j, i))/255.0;
			float v = float(V.at<uchar>(j, i)) / 255.0;
			float u = float(U.at<uchar>(j, i));

		
			if ( s > 0.25&&u>135&&h <50)
			{
				
					map.at<uchar>(j, i) = 255; 
			}
		
			
		}//end for
	}//end for

	dilate(map, map, cv::Mat(3, 3, CV_64F, Scalar(1)));
	medianBlur(map, map,5);
	morphologyEx(map, map, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));
	medianBlur(map, map, 5);
	imshow("egg", map);

	contours.clear();
	findContours(map, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int length=0;
	if(contours.size()>0)
	{
		vector< vector<Point> >::iterator it = contours.begin();
		while (it != contours.end())
		{
			RotatedRect mr = cv::minAreaRect(cv::Mat(*it));

			length=mr.size.height;

			if (mr.size.width*mr.size.height < 200||length<20||length>100)
				it = contours.erase(it);
			else
			{
				it++;
				
			}
		
		}
		drawContours(image, contours, -1, cv::Scalar(0, 0, 255), 1);
	
	}
	
	//huang
	//Findcenter(map, image, 3, 5, 7, 15, "egg", contours);


	waitKey(1);
}

