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
void setSrvPosition(object_detect::penPoint3D p, double angle)
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
void Findcenter(cv::Mat&map, cv::Mat& image, int n1, int n2, int n3, int n4 , char*ch, vector< vector<Point> >& outContours);
void LabColorDetector(cv::Mat&colorbg, cv::Mat& image);

int rows=480;
int cols=640;
Mat map1, map2, map3, map4, map5, map6, map7,map8,map9;
vector< vector<Point> > contours1,contours2,contours3,contours4,contours5,contours6,contours7;

cv::Point star(80,130),end(425,320);



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
	ros::ServiceServer service = n.advertiseService("/srvDesktopData",getsrvPosition);
	//ros::Publisher //chatter_pub = n.advertise<object_detect::penPoint3D>("penDetecter", 1000);
	ros::Rate loop_rate(10);


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

		//rectangle(imgRGB8u,star,end,Scalar(255,0,0),2);  
		LabColorDetector(imgRGB8u, imgRGB8u);

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
        Colorflag color1={0,0,0,0,1},color2={0,0,0,0,2},color3={0,0,0,0,3},color4={0,0,0,0,4},color42={0,0,0,0,4};
		Colorflag color5={0,0,0,0,5},color6={0,0,0,0,6},color7={0,0,0,0,7};


		for(int v = 0; v < rows; v++)
		{
			for(int u = 0; u <cols; u++)
			{
				unsigned int indx=u+v*cols;
				if(p3DPointSet[indx].Z!=0)
				{
					if(contours1.size()==1)
					{
						if(map1.ptr<uchar>(v)[u]>0)
						{
						 	double dist = cv::pointPolygonTest(contours1[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color1.x+=p3DPointSet[indx].X;
								color1.y+=p3DPointSet[indx].Y;
								color1.z+=p3DPointSet[indx].Z;
								color1.N++;
							}
						}
					}
					
					if(contours2.size()==1)
					{
						if(map2.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours2[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color2.x+=p3DPointSet[indx].X;
								color2.y+=p3DPointSet[indx].Y;
								color2.z+=p3DPointSet[indx].Z;
								color2.N++;
							}
	    						
						}
					}

					if(contours3.size()==1)
					{
						if(map3.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours3[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color3.x+=p3DPointSet[indx].X;
								color3.y+=p3DPointSet[indx].Y;
								color3.z+=p3DPointSet[indx].Z;
								color3.N++;
							}
	    						
						}
					}


					if(contours4.size()==2)
					{
						if(map4.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours4[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color4.x+=p3DPointSet[indx].X;
								color4.y+=p3DPointSet[indx].Y;
								color4.z+=p3DPointSet[indx].Z;
								color4.N++;
							}
							dist = cv::pointPolygonTest(contours4[1], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color42.x+=p3DPointSet[indx].X;
								color42.y+=p3DPointSet[indx].Y;
								color42.z+=p3DPointSet[indx].Z;
								color42.N++;
							}
	    						
						}
					}
					else
						if(contours4.size()==1)
						{
							if(map4.ptr<uchar>(v)[u]>0)
							{
								double dist = cv::pointPolygonTest(contours4[0], cv::Point2f(u, v), 0);
								if (dist >= 0)
								{
									color4.x+=p3DPointSet[indx].X;
									color4.y+=p3DPointSet[indx].Y;
									color4.z+=p3DPointSet[indx].Z;
									color4.N++;
								}
		    						
							}
						}


					
					if(contours5.size()==1)
					{
						if(map5.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours5[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color5.x+=p3DPointSet[indx].X;
								color5.y+=p3DPointSet[indx].Y;
								color5.z+=p3DPointSet[indx].Z;
								color5.N++;
							}
						}
					}

					if(contours6.size()==1)
					{
						if(map6.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours6[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color6.x+=p3DPointSet[indx].X;
								color6.y+=p3DPointSet[indx].Y;
								color6.z+=p3DPointSet[indx].Z;
								color6.N++;
							}
						}
					}

					if(contours7.size()==1)
					{
						if(map7.ptr<uchar>(v)[u]>0)
						{
							double dist = cv::pointPolygonTest(contours7[0], cv::Point2f(u, v), 0);
							if (dist >= 0)
							{
								color7.x+=p3DPointSet[indx].X;
								color7.y+=p3DPointSet[indx].Y;
								color7.z+=p3DPointSet[indx].Z;
								color7.N++;
							}
						}
					}

				}
			}
		}
 		delete [] p3DPointSet;

                object_detect::penPoint3D p1,p2,p3,p4,p42,p5,p6,p7;
		p1.flag=1;
		p2.flag=2;
		p3.flag=3;
		p4.flag=4;
		p42.flag=4;
		p5.flag=5;
		p6.flag=6;
		p7.flag=7;


		cv::Point3f pointLH(0,0,0), pointSL(0,0,0), pointQL(0,0,0),pointSL1(0,0,0);
		cv::Point3f pointSL2(0,0,0),pointZH(0,0,0),pointFH(0,0,0),pointQH(0,0,0);

		float angle1=0,angle2=0,angle3=0,angle4=0,angle42=0;
		float angle5=0,angle6=0,angle7=0;

		if(color1.N>0)
		{
			color1.x=color1.x/color1.N;
			color1.y=color1.y/color1.N;
			color1.z=color1.z/color1.N;
			pointLH=BasePoint(color1.x,color1.y,color1.z);


			RotatedRect mr = cv::minAreaRect(cv::Mat(contours1[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle1 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle1 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p1.x=pointLH.x;
			p1.y=pointLH.y;
			p1.z=pointLH.z;
			setSrvPosition(p1 , angle1);
			
			//chatter_pub.publish(p1);
			cout<<"liang huang: "<<pointLH.x<<"\t"<<pointLH.y<<"\t"<<pointLH.z<<endl;
		}
                
		if(color2.N>0)
		{
			color2.x=color2.x/color2.N;
			color2.y=color2.y/color2.N;
			color2.z=color2.z/color2.N;
			pointSL=BasePoint(color2.x,color2.y,color2.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours2[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle2 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle2 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}
			

			p2.x=pointSL.x;
			p2.y=pointSL.y;
			p2.z=pointSL.z;
			setSrvPosition(p2 ,angle2);
			//chatter_pub.publish(p2);
			cout<<"shen lv: "<<pointSL.x<<"\t"<<pointSL.y<<"\t"<<pointSL.z<<endl;
		
		}

		if(color3.N>0)
		{
			color3.x=color3.x/color3.N;
			color3.y=color3.y/color3.N;
			color3.z=color3.z/color3.N;
			pointQL=BasePoint(color3.x,color3.y,color3.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours3[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle3 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle3 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}


			
			p3.x=pointQL.x;
			p3.y=pointQL.y;
			p3.z=pointQL.z;
			setSrvPosition(p3 , angle3);
			//chatter_pub.publish(p3);
			cout<<"qian lan: "<<pointQL.x<<"\t"<<pointQL.y<<"\t"<<pointQL.z<<endl;
			
		}

		if(color4.N>0)
		{
			color4.x=color4.x/color4.N;
			color4.y=color4.y/color4.N;
			color4.z=color4.z/color4.N;
			pointSL1=BasePoint(color4.x,color4.y,color4.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours4[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle4 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle4 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p4.x=pointSL1.x;
			p4.y=pointSL1.y;
			p4.z=pointSL1.z;
			setSrvPosition(p4 , angle4);
			//chatter_pub.publish(p4);
			cout<<"shen lan 1: "<<pointSL1.x<<"\t"<<pointSL1.y<<"\t"<<pointSL1.z<<endl;
			
		}

		if(color42.N>0)
		{
			color42.x=color42.x/color42.N;
			color42.y=color42.y/color42.N;
			color42.z=color42.z/color42.N;
			pointSL2=BasePoint(color42.x,color42.y,color42.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours4[1]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle42 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle42 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p42.x=pointSL2.x;
			p42.y=pointSL2.y;
			p42.z=pointSL2.z;
			setSrvPosition(p42 , angle42);
			//chatter_pub.publish(p42);
			cout<<"shen lan 2: "<<pointSL2.x<<"\t"<<pointSL2.y<<"\t"<<pointSL2.z<<endl;
			
		}

		if(color5.N>0)
		{
			color5.x=color5.x/color5.N;
			color5.y=color5.y/color5.N;
			color5.z=color5.z/color5.N;
           	pointZH=BasePoint(color5.x,color5.y,color5.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours5[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle5 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle5 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p5.x=pointZH.x;
			p5.y=pointZH.y;
			p5.z=pointZH.z;
			setSrvPosition(p5 ,angle5);
			//chatter_pub.publish(p5);
			cout<<"zihong: "<<pointZH.x<<"\t"<<pointZH.y<<"\t"<<pointZH.z<<endl;
			
		}

		if(color6.N>0)
		{
			color6.x=color6.x/color6.N;
			color6.y=color6.y/color6.N;
			color6.z=color6.z/color6.N;
			pointFH=BasePoint(color6.x,color6.y,color6.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours6[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle6 = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle6 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p6.x=pointFH.x;
			p6.y=pointFH.y;
			p6.z=pointFH.z;
			setSrvPosition(p6 , angle6);
			//chatter_pub.publish(p6);
			cout<<"fenhong: "<<pointFH.x<<"\t"<<pointFH.y<<"\t"<<pointFH.z<<endl;
			
		}

		if(color7.N>0)
		{
			color7.x=color7.x/color7.N;
			color7.y=color7.y/color7.N;
			color7.z=color7.z/color7.N;
			pointQH=BasePoint(color7.x,color7.y,color7.z);

			RotatedRect mr = cv::minAreaRect(cv::Mat(contours7[0]));
			Point2f vertices[4];
			mr.points(vertices);
			if(mr.size.width> mr.size.height)
			{
				Point2f pttA[2];
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
				angle7= atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));
			}
			else
			{
						Point2f ptt[2];
						ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
						ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
						ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
						ptt[1].y = (vertices[0].y + vertices[3].y) / 2; 
						angle7 = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

			}

			p7.x=pointQH.x;
			p7.y=pointQH.y;
			p7.z=pointQH.z;
			setSrvPosition(p7 , angle7);
			//chatter_pub.publish(p7);
			cout<<"qianhuang: "<<pointQH.x<<"\t"<<pointQH.y<<"\t"<<pointQH.z<<endl;
			
		}

		cv::imshow("image",imgRGB8u);
		
		key=cvWaitKey(1);

		ros::spinOnce();
		loop_rate.sleep();

	}
        

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
void LabColorDetector(cv::Mat&colorbg, cv::Mat& image)
{
	Mat LAB;
	cvtColor(colorbg, LAB, CV_BGR2Lab);
	vector<Mat> lab;
	split(LAB, lab);
	Mat L, A, B;
	lab[0].copyTo(L);
	lab[1].copyTo(A);
	lab[2].copyTo(B);

	//imshow("A", A);
	//imshow("L", L);
	//imshow("B", B);


	Mat YUV;
	cvtColor(colorbg, YUV, CV_BGR2YUV);
	//imshow("hsv", HSV);
	vector<Mat> yuv;
	split(YUV, yuv);
	cv::Mat Y = yuv[0];
	cv::Mat U = yuv[1];
	cv::Mat yV = yuv[2];

	//imshow("Y",Y);
	//imshow("U", U);
	//imshow("yV", yV);


	Mat HSV;
	cvtColor(colorbg, HSV, CV_BGR2HSV);
	//imshow("hsv", HSV);
	vector<Mat> hsv;
	split(HSV, hsv);
	cv::Mat H, S, V;
	hsv[0].copyTo(H);
	hsv[1].copyTo(S);
	hsv[2].copyTo(V);

	//imshow("H2", H);
	//imshow("S2", S);
	//imshow("V2", V);

	Mat mapA;
	A.copyTo(mapA);
	float anlge=380;
	float Ang = 380;
	float a = 258, b = 258, l = 258;
	//Mat imgdist = cv::Mat(colorbg.rows, colorbg.cols, CV_8UC3, cv::Scalar(0));
	Mat imgdist1 = cv::Mat(colorbg.rows, colorbg.cols, CV_8UC1, cv::Scalar(0));
	int flag = 0;
	
	imgdist1.copyTo(map1);
	imgdist1.copyTo(map2);
	imgdist1.copyTo(map3);
	imgdist1.copyTo(map4);
	imgdist1.copyTo(map5);
	imgdist1.copyTo(map6);
	imgdist1.copyTo(map7);
	imgdist1.copyTo(map8);
	imgdist1.copyTo(map9);



	for (int j = star.y; j < end.y; j++)
	{
	
		for (int i = star.x; i < end.x; i++)
		{
		
			float h = float(H.at<uchar>(j, i)) * 360 / 255.0;
			float s = float(S.at<uchar>(j, i))/255.0;
			float v = float(V.at<uchar>(j, i)) / 255.0;

		
			if (h >= 10 && h <50 /*&& v>0 && s > 0.35*/)
			{
				float b = float(B.at<uchar>(j, i));
				float y = float(yV.at<uchar>(j, i));
				if (b > 170/*&&y<90*/) //170
				{
					//imgdist.at<Vec3b>(j, i)[0] = 0;
					//imgdist.at<Vec3b>(j, i)[1] = 247;
					//imgdist.at<Vec3b>(j, i)[2] = 255;
					map1.at<uchar>(j, i) = 255;  //liang huang
				}
			}

			if (h >= 70 && h<160  /* && v>0.1*/ /* &&v<0.7 */ )
			{
				float a = float(A.at<uchar>(j, i));
				float y = float(Y.at<uchar>(j, i));
				if (a < 135 && y<140)
				{
					//imgdist.at<Vec3b>(j, i)[0] = 51;
					//imgdist.at<Vec3b>(j, i)[1] = 153; //shen lv
					//imgdist.at<Vec3b>(j, i)[2] = 0;
					map2.at<uchar>(j, i) = 255;
				}
			}

			if (h >= 130 && h < 180 /*&& v>0 && v<0.8*/ /*&& s>0.6*/)
			{
				
				float b = float(B.at<uchar>(j, i));
				float l = float(L.at<uchar>(j, i));
				if (b < 118 && l>110)
				{
					//imgdist.at<Vec3b>(j, i)[0] = 153;
					//imgdist.at<Vec3b>(j, i)[1] = 51;//qian lan
					//imgdist.at<Vec3b>(j, i)[2] = 0;
					map3.at<uchar>(j, i) = 255;
				}

				}

			if (h >= 105 && h < 190 /*&& v>0 && v<0.8*/ /*&& s>0.6*/)
			{
				
				float b = float(B.at<uchar>(j, i));
				float l = float(L.at<uchar>(j, i));
				float a = float(A.at<uchar>(j, i));
				if (b < 125 && l<110 && a>140)
				{
				
					//imgdist.at<Vec3b>(j, i)[0] = 255;
					//imgdist.at<Vec3b>(j, i)[1] = 255;//shen lan
					//imgdist.at<Vec3b>(j, i)[2] = 0;
					map4.at<uchar>(j, i) = 255;
		
				}

			}

			

			if (s>0.35)
			{
				float y = float(yV.at<uchar>(j, i));
				float a = float(A.at<uchar>(j, i));

				if (y < 150)
				{
					if (a > 165&&v>0.7)
					{
						//imgdist.at<Vec3b>(j, i)[0] = 179; //fenhong
						//imgdist.at<Vec3b>(j, i)[1] = 139;
						//imgdist.at<Vec3b>(j, i)[2] = 255;
						map6.at<uchar>(j, i) = 255;
					}

					
				}
				
			}

			if (s<0.45&&s>0.10&&h<50)
			{
				
				float u = float(U.at<uchar>(j, i));
				float b = float(B.at<uchar>(j, i));
				float a = float(A.at<uchar>(j, i));
				
				float v = float(V.at<uchar>(j, i));
				float yv = float(yV.at<uchar>(j, i));
				if (b > 135&&yv>80&&v>120)//&&u>135&&a<140&&a<150&&v>120)
				{
					//imgdist.at<Vec3b>(j, i)[0] = 0; //qianhuang
					//imgdist.at<Vec3b>(j, i)[1] = 250;
					//imgdist.at<Vec3b>(j, i)[2] = 225;
					map7.at<uchar>(j, i) = 255;
				}
				//qianlv  qianzi  qianhuang
			}

			if (h >= 160 && h<255&&s>0.30)//&& v>0.45&&v<0.75 )//&& v>0.3&&v<0.75&&s>0.50)
			{
				float a = float(A.at<uchar>(j, i));
				float y = float(yV.at<uchar>(j, i));
				float u = float(U.at<uchar>(j, i));
				if (y>140&&a>160&&u>130)//
				{
					//imgdist.at<Vec3b>(j, i)[0] = 251; //zi hong
					//imgdist.at<Vec3b>(j, i)[1] = 0;
					//imgdist.at<Vec3b>(j, i)[2] = 255;
					map5.at<uchar>(j, i) = 255;
				}
				
			}


			
			
		}//end for
	}//end for

	//imshow(" imgdist ", imgdist);
       
	//huang
	Findcenter(map1, image, 3, 5, 7, 15, "lianghuang", contours1);


	//shen lv
	Findcenter(map2, image, 3, 5, 5, 5, "shenlv",contours2);

	//ÉîÀ¶
	Findcenter(map3, image, 3, 3, 7, 5, "qianlan",contours3);

	//shen lan
	Findcenter(map4, image, 3, 5, 7, 5, "shenlan",contours4);


	//zihong
	 Findcenter(map5, image, 3, 3, 3, 5, "zihong",contours5);
	
	//fenhong
	Findcenter(map6, image, 5, 3, 7, 5, "fenhong",contours6);

	//qianhuang
	Findcenter(map7, image, 7, 3, 5, 15, "qianhuang",contours7);

	//qianlv
	//Findcenter(map8, image, 15, 3, 7, 15, "qianlv");

      // imshow("colorbg ", image);

	
	

	waitKey(1);
}
void Findcenter(cv::Mat&map, cv::Mat& image, int n1, int n2, int n3, int n4 , char*ch, vector< vector<Point> >& contours)
{
	medianBlur(map, map, n1);
	dilate(map, map, cv::Mat(n2, n2, CV_64F, Scalar(1)));
	morphologyEx(map, map, MORPH_CLOSE, Mat(n3, n3, CV_8U, Scalar(1)));
	//medianBlur(map, map, n4);
	imshow(ch, map);

	contours.clear();
	/////vector< vector<Point> > outcontours;
	findContours(map, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int length=0;
	if(contours.size()>0)
	{
		vector< vector<Point> >::iterator it = contours.begin();
		//int idx=0;
		//double maxArea=cv::contourArea(contours[0]);
		//int i=0;
		while (it != contours.end())
		{
			RotatedRect mr = cv::minAreaRect(cv::Mat(*it));
			length=(mr.size.width>mr.size.height)? mr.size.width : mr.size.height;

			if (mr.size.width*mr.size.height < 200||length<70)
				it = contours.erase(it);
			else
			{

				//double area=cv::contourArea((*it));
				//if(area>maxArea)
				//{
				//	idx=i;
				//	maxArea=area;
				//}
					
				//outContours.push_back((*it));
				circle(image, cv::Point(mr.center.x, mr.center.y), 3, cv::Scalar(0, 0, 255), 2);
				//cout<<"center: "<<mr.center.x<<"\t"<<mr.center.y<<endl;
				it++;
				//cout<<"lenth: "<<length<<endl;
			}
			//i++;
		}
		
      //  cout<<"idx: "<<outContours.size()<<endl;
		//if(idx>=0 && idx<contours.size())
		//{
			//outcontours.push_back(contours[idx]);
			drawContours(image, contours, -1, cv::Scalar(0, 0, 255), 1);
		//}
			
	}
	

	//outContours.push_back(contours[idx]);

	//if(outContours.size()>0)
		
}

