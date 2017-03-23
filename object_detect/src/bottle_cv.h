

#include <XnCppWrapper.h>  
#include <iostream>  
#include <fstream>
#include <iomanip>  
#include <vector>  
#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <vector>   //lzc
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include<geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>  //lzc
#include <pcl/point_types.h>
#include <ros/ros.h>

#define IMGWIDTH	640
#define IMGHEIGHT	480
#define BG	0
using namespace std;
using namespace cv;
using namespace xn;
using namespace pcl;

using namespace Eigen;
int gwidth;
double gangel;
Matrix<float,4,4> mat_kinect;
tf2::Matrix3x3 mat3;
geometry_msgs::Pose srvpose;
bool srvsend;
const static double PI = 3.1415926;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
}



bool setsrvPose()
{
	//srvpose.position.z = cloud.points[index].z;
	//srvpose.position.x = cloud.points[index].x;
	//srvpose.position.y = -cloud.points[index].y;
        
	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;
	//if(srvpose.position.z>2 ||srvpose.position.z < 1.2 ) return false;
	//ROS_INFO("DEPTH : %f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
	
	Matrix<float,4,1> Pr;
	Matrix<float,4,1> Pc;
	Pc << 	srvpose.position.x*1000,
	  	srvpose.position.y*1000,
	  	srvpose.position.z*1000,
		    1;

	Pr = mat_kinect *Pc;
/*
	srvpose.position.x = Pr(0,0)/1000;
	srvpose.position.y = Pr(1,0)/1000;
	srvpose.position.z = Pr(2,0)/1000;*/

	srvpose.position.x = Pr(0,0)/1000;
	srvpose.position.y = Pr(1,0)/1000;
	srvpose.position.z = Pr(2,0)/1000;

	ROS_ERROR("rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
        
	return true;
  
}

bool setsrvOrientation(tf2::Vector3 v1, tf2::Vector3 v2)
{
	v1 = mat3 * v1;
	ROS_ERROR("ROS VECTOR1: UP");
	seeV3(v1);
	v2 = mat3 * v2;
	ROS_ERROR("ROS VECTOR2: UP?");
	seeV3(v2);
	v1.normalize();
	v2.normalize();
	v1 = -v1 - v2;
	v1.normalize();
	ROS_ERROR("ROS VECTOR3 45:");
	seeV3(v1);
	
	ROS_ERROR("rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);      
	return true;
  
}

void calFixQright( tf2::Vector3 position2, geometry_msgs::Quaternion &gq)
{
	ROS_INFO("\n\n*****calFixAngleQ****THING PISTION , ROBOT POSITION*********start******");
	
	tf2::Quaternion q;
	tf2::Vector3 OA(0,0,1);
	double mangle;
	tf2::Vector3 maxis;
	position2.normalize();
	mangle= position2.angle(OA);
	
	maxis  =  OA.cross(position2);

	tf2::Quaternion fixq,oaq(0.500000,0.500000,0.500000,-0.50000);
	ROS_INFO("MANGEL-%f",mangle);
	q.setRotation(maxis,mangle);//FOR FINALE LEFT
	q.normalize();
	q = q*oaq;
	ROS_INFO( "  \n\n\ni calculate %.8f,%.8f,%.8f,%.8f,]'\n\n\n ",q.x(),q.y(),q.z(),q.w());
	gq.x = q.x();
	gq.y = q.y();
	gq.z = q.z();
	gq.w = q.w();
	return ;
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



}

/*
	函数功能：检查错误，并翻译错误提示
	输入参数：
	result		in	运行结果
	status		in	运行的函数
*/
void CheckOpenNIError( XnStatus result, string status );

/*
	函数功能：计算三维点云数据
	输入参数：
	rDepthGen		in	深度数据生成节点
	pDepth			in	深度数据
	pImage			in	RGB彩色图数据
	vPointCloud		out	生成的三维点云数据
*/
void GeneratePointCloud( xn::DepthGenerator& rDepthGen,	const XnDepthPixel* pDepth,	const XnRGB24Pixel* pImage,	PointCloud<PointXYZRGB>& vPointCloud, Rect roiRect = Rect(0, 0, IMGWIDTH, IMGHEIGHT));




//全局变量
PointCloud<PointXYZRGB>	gPointCloud;
Point					gStartPt;
Point					gEndPt;
bool					gSelectFlag;

int ROI_leftup_x=0;
int ROI_leftup_y=0;
int ROI_rightdown_x=0;
int ROI_rightdown_y=0;
std::vector < int > counter_container;
geometry_msgs::Point bottle_p;

int key = 0;
//声明OpenNI变量
XnStatus	resultStat = XN_STATUS_OK;
xn::Context		kinectCont;
DepthMetaData	depthMD;
ImageMetaData	imageMD;

//声明OpenCV变量
Mat		imgDepth16u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
Mat		imgRGB8u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
Mat		imgDepthShow = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
Mat		imgRGBShow = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
Mat		imgDepthBg16u;			//深度背景图，一定要为16位位深
bool	loadBg = true;
DepthGenerator	depthGen;
ImageGenerator	imageGen;
Mat imgDiff16u;
int kinect_start()
{

	namedWindow("RGB");
	namedWindow("Depth");
	namedWindow("cover_depth");  //lzc
 	if(BG == 0)imgDepthBg16u = imread("/home/vision/moveit/DepthBg16u.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	imshow("RGB1", imgRGBShow);                     //lzc
	imshow("Depth1", imgDepthShow);                 //lzc
	key = waitKey(20);

	//声明OpenNI的各个节点
	resultStat = kinectCont.Init();
	CheckOpenNIError(resultStat, "OpenNI Context Initial...\t");
	/*//播放已录制文件
	resultStat = kinectCont.OpenFileRecording("Files\\Record_32.oni");
	CheckOpenNIError(resultStat, "Read record file...\t");*/


	resultStat = depthGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create depth generator...\t");

	resultStat = imageGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create image generator...\t");

	//初始化各个节点
	XnMapOutputMode	mapMode;
	mapMode.nXRes = IMGWIDTH;
	mapMode.nYRes = IMGHEIGHT;
	mapMode.nFPS = 30;
	resultStat = depthGen.SetMapOutputMode(mapMode);
	CheckOpenNIError(resultStat, "Set depth generator output mode...\t");
	resultStat = imageGen.SetMapOutputMode(mapMode);
	CheckOpenNIError(resultStat, "Set image generator output mode...\t");
	depthGen.GetAlternativeViewPointCap().SetViewPoint(imageGen);

	//开始生成数据
	resultStat = kinectCont.StartGeneratingAll();
	//开始更新数据
	resultStat = kinectCont.WaitAndUpdateAll();

	imgDiff16u = Mat::zeros(imgDepth16u.size(), CV_16U);
}

int test2()
{


	if ((key != 27) && !(resultStat = kinectCont.WaitAndUpdateAll()))
	{
		
		if(key!=-1) {ROS_INFO("%d",(int)key);ros::shutdown();}
		//获取数据
		depthGen.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);
ROS_INFO(":%d",(int)key);
		//转换为OpenCV的Mat格式
		memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);ROS_INFO(":%d",(int)key);
		memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0); ROS_INFO(":%d",(int)key);
		//保存深度背景图
		if (BG)
		{
			imwrite("DepthBg16u.png", imgDepth16u);
			imgDepthBg16u = imgDepth16u.clone();ROS_INFO (":%d",(int)key);
		}else if(imgDepthBg16u.empty())
		{
			imshow("RGB1", imgRGBShow);
			imshow("Depth1", imgDepthShow);
			key = waitKey(20);ROS_INFO(":%d",(int)key);
			return 0;
		}ROS_INFO("%d",(int)key);
		double timeNode = (double)getTickCount();		
		//背景差求得盒子在深度图中的区域
		for(int irow = 0; irow < imgDepth16u.rows; irow++)
		{
			for(int icol = 0; icol < imgDepth16u.cols; icol++)
				imgDiff16u.ptr<ushort>(irow)[icol] = imgDepth16u.ptr<ushort>(irow)[icol] > 0 && imgDepthBg16u.ptr<ushort>(irow)[icol] > 0 ? 
														abs((short)imgDepth16u.ptr<ushort>(irow)[icol] - imgDepthBg16u.ptr<ushort>(irow)[icol]) : 0;
		}
	
		for(int irow = 0; irow < imgDepth16u.rows; irow++)
		{
			for(int icol = 0; icol < imgDepth16u.cols; icol++)
                             {
                               if ((icol>=0 && icol<=512)&& (irow>=64 && irow <=352) )
                                   {
                                      if (fabs(imgDiff16u.ptr<ushort>(irow)[icol])<100)

                                          {imgDiff16u.ptr<ushort>(irow)[icol]=0;}
                                   }
                               else
                                   {imgDiff16u.ptr<ushort>(irow)[icol]=0;}
                             }

		}
		ROS_INFO("%d",(int)key);
		Mat	imgDiff8u = Mat::zeros(imgDepthShow.size(), CV_8U);
		imgDiff16u.convertTo(imgDiff8u, CV_8U, 255/4096.0);
		threshold(imgDiff8u, imgDiff8u, 60 / 4096.0 * 255, 255, CV_THRESH_BINARY);  //orginal:60
		erode(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 4);
		dilate(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 5);
//////////////lzc
/*
		Mat	bottle_cover8u = Mat::zeros(imgDepthShow.size(), CV_8U);   //lzc
		imgDiff16u.convertTo(bottle_cover8u, CV_8U, 255/4096.0);
		threshold(bottle_cover8u, bottle_cover8u, 60 / 4096.0 * 255, 255, CV_THRESH_BINARY);
		erode(bottle_cover8u, bottle_cover8u, Mat(), Point(1,1), 4);
		dilate(bottle_cover8u, bottle_cover8u, Mat(), Point(1,1), 5);
*/

		vector<vector<Point> >	contours;
		vector<Vec4i>			hierarchy;
		findContours(imgDiff8u, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);



//added by zhang yu mei for angel

	vector<vector<Point> >::const_iterator itcon = contours.begin();
	float angl;
	
	
		//最大连通域
		int maxConnectDomainIn = 0;
		double maxArea = 0;
		for(int i = 0; i < contours.size(); i++)
		{
			double tempArea = contourArea(contours[i]);
			if(maxArea < tempArea)
			{
				maxConnectDomainIn = i;
				maxArea = tempArea;
			}
		}



		if(maxArea < 10)
		{
			cout << "Warning: bottle indentify failed!" << endl;
			return 0;
		}
		if(maxArea > 0)
		{
			//根据ROI对深度图进行处理
			imgDiff8u = Mat::zeros(imgDiff8u.size(), CV_8U);
            Mat imgDiff8u_ROI(imgDiff8u,Rect(0, 64, 512, 352));
			drawContours(imgDiff8u, contours, maxConnectDomainIn, Scalar(255), CV_FILLED);
		}

		RotatedRect rRect = minAreaRect(Mat(contours[maxConnectDomainIn]));
		Point2f vertices[4];
		rRect.points(vertices);
		for (int i = 0; i < 4; ++i)
			line(imgRGBShow, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 2, 8, 0);//»­×îÐ¡Íâ°üŸØÐÎ
		ROS_INFO("width, height%d,%d",(int)rRect.size.width,(int)rRect.size.height);
		gwidth = (int)rRect.size.width<(int)rRect.size.height? (int)rRect.size.width:(int)rRect.size.height;
		//srvsend = false;
		if(rRect.size.width> rRect.size.height)
		{
		    Point2f pttA[2];
			pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
			pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
			pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
			pttA[1].y = (vertices[2].y + vertices[3].y) / 2;
			Point2f pttCent;
			pttCent.x = (pttA[0].x + pttA[1].x) / 2;
			pttCent.y = (pttA[0].y + pttA[1].y) / 2;
			int x=pttCent.x,miny=pttCent.y ;
			line(imgRGBShow, Point2f(x-10,miny) ,Point2f(x+10,miny) , Scalar(0,255,0));
			line(imgRGBShow, Point2f(x,miny-10) ,Point2f(x,miny+10) , Scalar(0,255,0));
			line(imgRGBShow, pttA[0], pttA[1], CV_RGB(255, 0, 0), 2, 8, 0);
			circle(imgRGBShow, pttCent, 2, Scalar(0), 3);
		 	angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));

		}
		else
		{
			Point2f ptt[2];
			ptt[0].x = (vertices[1].x + vertices[2].x) / 2;
			ptt[0].y = (vertices[1].y + vertices[2].y) / 2;
			ptt[1].x = (vertices[0].x + vertices[3].x) / 2;
			ptt[1].y = (vertices[0].y + vertices[3].y) / 2;
			Point2f pttCen;
			pttCen.x = (ptt[0].x + ptt[1].x) / 2;
			pttCen.y = (ptt[0].y + ptt[1].y) / 2;
			int x=pttCen.x,miny=pttCen.y ;
			line(imgRGBShow, Point2f(x-10,miny) ,Point2f(x+10,miny) , Scalar(0,255,0));
			line(imgRGBShow, Point2f(x,miny-10) ,Point2f(x,miny+10) , Scalar(0,255,0));
			line(imgRGBShow, ptt[0], ptt[1], CV_RGB(255, 0, 0), 2, 8, 0);
			circle(imgRGBShow, pttCen, 2, Scalar(0), 3);
			 angl = atan2((ptt[1].y - ptt[0].y), (ptt[1].x - ptt[0].x));

		}
		gangel = 0.4-angl;
		ROS_INFO( "before angl:%f rad %f", gangel,angl * 180 / CV_PI);

		int w = depthMD.FullXRes(), h = depthMD.FullYRes();
                int counter=1;
                bottle_p.x=0;
                bottle_p.y=0;
		
		for(int irow = 0; irow < imgDepth16u.rows; irow++)
		{
			for(int icol = 0; icol < imgDepth16u.cols; icol++)
				{

                                  imgDepth16u.ptr<ushort>(irow)[icol] = imgDiff8u.ptr<uchar>(irow)[icol] ? imgDepth16u.ptr<ushort>(irow)[icol] : 0;
                                  if (imgDepth16u.ptr<ushort>(irow)[icol]>0)
                                      {
                                        bottle_p.x += irow;
                                        bottle_p.y += icol;
                                        counter++;
                                      }
                                }
                }

		bottle_p.x=bottle_p.x/counter;
		bottle_p.y=bottle_p.y/counter;
		XnPoint3D	pt;
		pt.X = bottle_p.y;
		pt.Y = bottle_p.x;
		pt.Z = *((unsigned short*)imgDepth16u.data + (unsigned int)pt.Y * imgDepth16u.cols + (unsigned int)pt.X);
//imgDepth16u.ptr<ushort>((unsigned int)pt.Y)[(unsigned int)pt.X];
		
		
		XnPoint3D 	Pt3D;
		depthGen.ConvertProjectiveToRealWorld( 1, &pt, &Pt3D);
		ROS_ERROR("depth:%f,%f,%f",Pt3D.X,Pt3D.Y,Pt3D.Z);

		srvpose.position.z = Pt3D.Z/1000.;
		srvpose.position.x = Pt3D.X/1000.;
		srvpose.position.y = Pt3D.Y/1000.;
		
		srvsend = setsrvPose();

		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);

		imwrite("d.png", imgDepth16u);
		drawObject(bottle_p.y,bottle_p.x,imgRGBShow);

///////////////////////////////////////////////////////////////////////////PCL//////////////////////////////////////////////////////////////
		//计算点云数据
		gPointCloud.clear();
		GeneratePointCloud(depthGen, (XnDepthPixel*)imgDepth16u.data, imageGen.GetRGB24ImageMap(), gPointCloud);
		char filename[200];
		sprintf(filename, "(%f,%f,%f)-(%f,%f,%f).pcd",pt.X, pt.Y, pt.Z, Pt3D.X,Pt3D.Y,Pt3D.Z);
		//pcl::io::savePCDFileASCII (filename, gPointCloud);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		printf("total cost time：%fms\n", ((double)getTickCount() - timeNode)*1000/getTickFrequency());
		imshow("RGB2", imgRGBShow);
		imshow("Depth2", imgDepthShow);
		key = waitKey(20);

	}



	return 0;
}
int kinect_close()
{
	destroyAllWindows();
	kinectCont.StopGeneratingAll();
	kinectCont.Release();
}
void CheckOpenNIError( XnStatus result, string status )  
{   
	if( result != XN_STATUS_OK )   
		cerr << status << " Error: " << xnGetStatusString( result ) << endl;  
} 

void GeneratePointCloud( xn::DepthGenerator& rDepthGen,	const XnDepthPixel* pDepth,	const XnRGB24Pixel* pImage,	PointCloud<PointXYZRGB>& vPointCloud, Rect roiRect)
{
	// 1. number of point is the number of 2D image pixel
	xn::DepthMetaData mDepthMD;
	rDepthGen.GetMetaData( mDepthMD );
	unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

	// 2. build the data structure for convert
	XnPoint3D* pDepthPointSet = new XnPoint3D[ uPointNum ];
	unsigned int i, j, idxShift, idx;
	for( j = roiRect.y; j < roiRect.height; ++j )
	{
		idxShift = j * mDepthMD.FullXRes();
		for( i = roiRect.x; i < roiRect.width; ++i )
		{
			idx = idxShift + i;
			pDepthPointSet[idx].X = i;
			pDepthPointSet[idx].Y = j;
			pDepthPointSet[idx].Z = pDepth[idx];
		}
	}

	// 3. un-project points to real world
	XnPoint3D* p3DPointSet = new XnPoint3D[ uPointNum ];
	rDepthGen.ConvertProjectiveToRealWorld( uPointNum, pDepthPointSet, p3DPointSet );
	delete[] pDepthPointSet;

	// 4. build point cloud
	for( i = 0; i < uPointNum; ++ i )
	{
		// skip the depth 0 points
		if( p3DPointSet[i].Z == 0 )
			continue;
		PointXYZRGB	tempPt;
		tempPt.x = p3DPointSet[i].X;
		tempPt.y = p3DPointSet[i].Y;
		tempPt.z = p3DPointSet[i].Z;
		tempPt.r = 0;	//pImage[i].nRed;
		tempPt.g = 0;	//pImage[i].nGreen;
		tempPt.b = 255;	//pImage[i].nBlue;
		vPointCloud.push_back(tempPt);
	}
	delete[] p3DPointSet;
}



