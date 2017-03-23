/*
	工程功能：检测盒子的三维坐标及方向
	编者：白
	最后修改时间：2016.05.03 16：57
*/

#include <XnCppWrapper.h>  
#include <iostream>  
#include <fstream>
#include <iomanip>  
#include <vector>  
#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include<geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>  //lzc
#include <pcl/point_types.h>
#include <ros/ros.h>
#define IMGWIDTH	640
#define IMGHEIGHT	480
#define BG
using namespace std;
using namespace cv;
using namespace xn;
using namespace pcl;
using namespace Eigen;

Matrix<float,4,4> mat_kinect;
tf2::Matrix3x3 mat3;
geometry_msgs::Pose srvpose;
bool srvsend;
geometry_msgs::Pose curcarpose;
const static double PI = 3.1415926;
cv::Mat roi= imread("/home/vision/moveit/BgROI.bmp", CV_LOAD_IMAGE_GRAYSCALE);
/*
float T_data[4][4] = {		0.241919591653694,		0.844294602673505,		0.592067560553167,		-610.386467528557,
						1.04930561312756,		-0.131409689525925,		-0.200188098893888,		607.274109082033,
						0.104642093167853,		-0.650161998230167,		0.764883731680464,		-2176.08013959110,
						0,						0,						0,						1	};

*/

float T_data[4][4] = {0.934706405019371,-0.0310988990465522,-0.0327737298510710,-448.758019800802,
0.0227847091817027,0.369347497142335,0.971448423146522,-1426.77660142682,
0.0183591814219336,0.926810225979939,-0.395723932345384,1894.41655107448,
0,0,0,1};


Mat	T4x4_32F(4, 4, CV_32FC1, T_data);

bool ConverKinectToWorld(cv::Point3f &PtInKinect, cv::Mat &T4x4_32F)
{
	int const	TSize = 4; 
	if(T4x4_32F.rows != T4x4_32F.cols && T4x4_32F.rows != TSize)
		return false;
	float	m_pt_k_data[4];
	m_pt_k_data[0] = PtInKinect.x;
	m_pt_k_data[1] = PtInKinect.y;
	m_pt_k_data[2] = PtInKinect.z;
	m_pt_k_data[3] = 1;

	cv::Mat	m_pt_k(4, 1, CV_32FC1, m_pt_k_data);
	cv::Mat	m_pt_w = cv::Mat::zeros(4, 1, CV_32FC1);
	
	m_pt_w = T4x4_32F * m_pt_k;
	srvpose.position.x = m_pt_w.ptr<float>(0)[0];
	srvpose.position.y = m_pt_w.ptr<float>(1)[0]; 
	srvpose.position.z = m_pt_w.ptr<float>(2)[0];

	return true;
}


bool setsrvPosition()
{
	ROS_ERROR("before rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);
	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;
	//if(srvpose.position.z>2 ||srvpose.position.z < 1.2 ) return false;
	//ROS_INFO("DEPTH : %f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);

	Point3f ptKinect, ptWorld;
	ptKinect = Point3f(srvpose.position.x, srvpose.position.y, srvpose.position.z);
	ConverKinectToWorld(ptKinect, T4x4_32F);

	ROS_ERROR("after to world rotation:%f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);

       	return true;

}
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
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
/*
	函数功能：检查错误，并翻译错误提示
	输入参数：
	result		in	运行结果
	status		in	运行的函数
*/
void CheckOpenNIError( XnStatus result, string status );

/*
	函数功能：OpenNI录制主函数
*/
void Record_main();

/*
	函数功能：计算三维点云数据
	输入参数：
	rDepthGen		in	深度数据生成节点
	pDepth			in	深度数据
	pImage			in	RGB彩色图数据
	vPointCloud		out	生成的三维点云数据
*/
void GeneratePointCloud( xn::DepthGenerator& rDepthGen,	const XnDepthPixel* pDepth,	const XnRGB24Pixel* pImage,	PointCloud<PointXYZRGB>& vPointCloud, Rect roiRect = Rect(0, 0, IMGWIDTH, IMGHEIGHT));

/*
	函数功能：输出SDL崩溃错误提示
*/
void sdldie(const char *msg);

/*
	函数功能：检查SDL错误
*/
void checkSDLError(int line = -1);

/*
	函数功能：鼠标事件响应函数
*/
void OnMouse(int event, int x, int y, int flags, void *param);

void QRDecomposition(double* coeffM, double* D, int nSize);
bool Householder(double const*coeffM, double const* D, double* B, int nSize);
/*
	使用示例
	int n=3;
	double a[] = {4, 5, 11, 10, 3, 5, 7, 40, -38};
	double b[] = {47, 31, -27};
	double* d=new double[n]();
	QRDecomposition(a,d,n);
	Householder(a,d,b,n);
	结果在b中
*/

/*
	函数功能：判断两个面是否为盒子的前平面和上平面
	参数说明：
	n1				in		平面1法向量（1*3）的数据指针
	n2				in		平面2法向量（1*3）的数据指针
	n3				out		向量n1、n2的法向量（1*3）的数据指针
	minAngle		in		两个平面交线方向 与 Z轴的最小夹角
	minOrthogonality in		最小正交性值
*/
bool CanBeUsed(double* n1, double* n2, double* n3, double minAngle, double orthogonalityThresh);

/*
	函数功能：获取吸取点的位置和姿态
	参数说明：
	planeStat			in		三个平面的提取状态
	planeParams			in		三个平面的参数
	planeCenter			in		三个平面的中心点
	refOrient_1			in		竖直方向的参考向量
	refOrient_2			in		水平方向的参考向量
	maxAngle			in		最大夹角
	orthogonalityThresh	in		两个平面的正交性阈值
	absorbPt			out		计算的吸取点位置
	planesIndex			out		吸取点相邻两个平面索引值，对应于planeParams
	linesOrientation	out		交线的方向向量
*/
bool GetAbsorbPoint(bool* planeStat, double* planeParams, PointXYZ* planeCenter, double* refOrient_1, double* refOrient_2, double maxAngle, double orthogonalityThresh, PointXYZ* absorbPt, int* planesIndex, double* linesOrientation);

/*
	函数功能：定位盒子位置
	参数说明：
	depthGen			in		OpenNI深度数据生成器
	imageGen			in		OpenNI彩色数据生成器
	imgWidth			in		图像宽度
	imgHeight			in		图像高度
	imgDepthBg16u		in		深度背景图
	refOrient_1			in		竖直方向的参考向量
	refOrient_2			in		水平方向的参考向量
	maxAngle			in		最大夹角
	orthogonalityThresh	in		两个平面的正交性阈值
	absorbPt			out		计算的吸取点位置
	planesNormal		out		各个面的法向量
	planesIndex			out		吸取点相邻两个平面索引值，对应于planeParams
	linesOrientation	out		交线的方向向量
*/
bool	BoxIndentify(xn::DepthGenerator &depthGen, xn::ImageGenerator &imageGen, int imgWidth, int imgHeight, Mat &imgDepthBg16u, double* refOrient_1, double* refOrient_2, double maxAngle, double orthogonalityThresh, PointXYZ* absorbPt,double* planesNormal, int* planesIndex, double* linesOrientation, Mat &imgBgPreROI);

//全局变量
PointCloud<PointXYZRGB>	gPointCloud;
Point					gStartPt;
Point					gEndPt;
bool					gSelectFlag;
//声明OpenNI变量
XnStatus	resultStat = XN_STATUS_OK;
xn::Context		kinectCont;

Mat		imgDepthBg16u ;
DepthGenerator	depthGen;
ImageGenerator	imageGen;
DepthMetaData	depthMD;
ImageMetaData	imageMD;
Mat		imgDepth16u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
Mat		imgRGB8u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
Mat		imgRGBShow = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
Mat		imgDiff16u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
XnMapOutputMode	mapMode;
bool kinect_init()
{
#ifndef BG
	imgDepthBg16u = imread("/home/vision/moveit/DepthBg500.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
#endif
		//声明OpenNI的各个节点
	resultStat = kinectCont.Init();
	CheckOpenNIError(resultStat, "OpenNI Context Initial...\t");
	//播放已录制文件
	/*resultStat = kinectCont.OpenFileRecording("Files\\Record_502.oni");
	CheckOpenNIError(resultStat, "Read record file...\t");*/

	resultStat = depthGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create depth generator...\t");

	resultStat = imageGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create image generator...\t");

	//初始化各个节点

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
	return true;
}
bool kinect_close()
{
	destroyAllWindows();
	kinectCont.StopGeneratingAll();
	kinectCont.Release();
}
int test2( )
{
	int key = 0;
	 
	namedWindow("Depth");
	//setMouseCallback("Depth", OnMouse);

#ifdef BG
	int cnt = 0;
#endif
	if ((key != 27) && !(resultStat = kinectCont.WaitAndUpdateAll()))
	{

		//获取数据
		depthGen.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);
		//转换为OpenCV的Mat格式
		memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
		memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);
		#ifdef BG
		if(cnt>500) {imwrite("/home/vision/moveit/DepthBg500.png",imgDepth16u); ros::shutdown();}
		imwrite("halfd.bmp",imgRGBShow);
		imwrite("/home/vision/moveit/DepthBg500.png",imgDepth16u);
		cnt++;
		return 0;

		#endif
		imshow("Image", imgRGBShow);
		imwrite("halfd.bmp",imgRGBShow);
		key = waitKey(10);
		if( !(resultStat = kinectCont.WaitAndUpdateAll()))
		{
			double		refOrient_1[3] = {0.03,-1,0.35};
			double		refOrient_2[3] = {0.06,0.4,1.0};
			double		maxAngle = 30 * 3.1415926 / 180.0;
			double		orthogonalityThresh = 0.36;
			PointXYZ	absorbPt[3];
			double*		pPlanesNormal = new double[3*3];
			int			planesIndex[3*2] = {0, 0, 0, 0, 0, 0};
			double		linesOrientation[3*3];
			double		timeNode = (double)getTickCount();
			bool st = BoxIndentify(depthGen, imageGen, IMGWIDTH, IMGHEIGHT, imgDepthBg16u, refOrient_1, refOrient_2, maxAngle, orthogonalityThresh, absorbPt, pPlanesNormal, planesIndex, linesOrientation, roi);
			//结果在absorbPt[0],相邻面索引为planesIndex[0]、planesIndex[1],相邻面法向量在pPlanesNormal，交线方向向量在linesOrientation[0]
			if (st)
			{
				printf("\n\t盒子定位成功！耗时为%3.0fms,详细结果为：\n位置：\t(%3.3f,%3.3f,%3.3f)\n相邻面法向量为：N1 = (%3.3f,%3.3f,%3.3f)  N2 = (%3.3f,%3.3f,%3.3f)\n交线方向向量为：(%3.3f,%3.3f,%3.3f)\n",
					((double)getTickCount() - timeNode) * 1000 / getTickFrequency(),
					absorbPt[0].x, absorbPt[0].y, absorbPt[0].z,
					pPlanesNormal[planesIndex[0]*3], pPlanesNormal[planesIndex[0]*3 + 1], pPlanesNormal[planesIndex[0]*3 + 2],
					pPlanesNormal[planesIndex[1]*3], pPlanesNormal[planesIndex[1]*3 + 1], pPlanesNormal[planesIndex[1]*3 + 2],
					linesOrientation[0], linesOrientation[1], linesOrientation[2]);

				srvpose.position.z = absorbPt[0].z;
				srvpose.position.x = absorbPt[0].x;
				srvpose.position.y = absorbPt[0].y;
							   				
				srvsend=setsrvPosition();
				
				tf2::Vector3 v1(pPlanesNormal[planesIndex[0]*3], pPlanesNormal[planesIndex[0]*3 + 1], pPlanesNormal[planesIndex[0]*3 + 2]),
						v2(pPlanesNormal[planesIndex[1]*3], pPlanesNormal[planesIndex[1]*3 + 1], pPlanesNormal[planesIndex[1]*3 + 2]);
				setsrvOrientation(v1,v2);
			}
			else
				printf("\n\t盒子定位失败!!!");
			delete[] pPlanesNormal;

		}
		else if (key == 'B')		//更新深度图背景
		{
			//imgDepthBg16u = imgDepth16u.clone();
			//printf("\n通知：深度图背景已更新！");
		}			 

	}
	return 0;
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
	//io::savePCDFileASCII("BoxPointCloud.pcd", gPointCloud);
	delete[] p3DPointSet;
}

void OnMouse(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		gStartPt = Point(x, y);
		gSelectFlag = true;
		break;
	case EVENT_MOUSEMOVE:
		if(flags == EVENT_FLAG_LBUTTON)
			gEndPt = Point(x, y);			
		break;
	case EVENT_LBUTTONUP:
		gEndPt = Point(x, y);
		break;
	case EVENT_RBUTTONDOWN:
		gSelectFlag = false;		//取消区域选择
		break;
	default:
		break;
	}
}

void QRDecomposition(double* coeffM, double* D, int nSize)
{
	double temp1,temp2;
	double* temp=new double[nSize];
	for(int i=0;i<nSize-1;i++)
	{
		temp1=0;
		for(int j=i;j<nSize;j++)
			temp1+=(*(coeffM+j*nSize+i))*(*(coeffM+j*nSize+i));

		if(*(coeffM+nSize*i+i)>0)
			temp1=-sqrt(temp1);
		else 
			temp1=sqrt(temp1);

		*(D+i)=temp1;//存储主元
		*(coeffM+i*nSize+i)-=temp1;

		temp2=0;
		for(int j=i; j<=nSize-1; j++)
			temp2+=(*(coeffM+nSize*j+i))*(*(coeffM+nSize*j+i));
		temp2= sqrt(temp2);
		for(int j=i; j<=nSize-1; j++)
			*(coeffM+nSize*j+i)=(*(coeffM+nSize*j+i))/temp2 ;

		for(int j=i+1;j<nSize;j++) 
		{ 				
			for(int k=i; k<nSize; k++)
			{
				temp2=0 ;
				for(int l=i; l<nSize; l++)
					temp2+=(*(coeffM+nSize*k+i))*(*(coeffM+nSize*l+i))*(*(coeffM+nSize*l+j));
				*(temp+k)=(*(coeffM+k*nSize+j))-2*temp2;
			}
			for(int k=i; k<nSize; k++)
				*(coeffM+k*nSize+j)=*(temp+k);
		}
	}
	*(D+nSize-1)=*(coeffM+nSize*nSize-1); 
	delete[] temp;
}

bool Householder(double const*coeffM, double const* D, double* B, int nSize)
{
	double doubleTemp;
	double* temp=new double[nSize];

	for(int i=0; i<nSize-1; i++)
	{
		for(int j=i; j<nSize; j++)
		{
			doubleTemp=0;
			for(int k=i;k<nSize; k++)
				doubleTemp+=(*(coeffM+nSize*k+i))*(*(coeffM+nSize*j+i))*(*(B+k));
			*(temp+j)=*(B+j)-2*doubleTemp;
		}
		for(int j=i; j<nSize; j++)
			*(B+j)=*(temp+j);
	}

	for(int i=nSize-1; i>=0; i--)
	{
		for(int j=nSize-1; j!=i;--j)
			*(B+i)-=(*(B+j))*(*(coeffM+i*nSize+j));
		*(B+i)/=(*(D+i));
	}

	delete[] temp;
	return true;
}

bool CanBeUsed(double* n1, double* n2, double* n3, double minAngle, double orthogonalityThresh)
{
	n3[0] = n1[1] * n2[2] - n1[2] * n2[1];
	n3[1] = n1[2] * n2[0] - n1[0] * n2[2];
	n3[2] = n1[0] * n2[1] - n1[1] * n2[0];
	// 求向量n3与Z轴的夹角
	double	z = n3[2] > 0 ? 1 : -1;
	double	cos_theta = n3[2] * z / (sqrtf(n3[0]*n3[0] + n3[1]*n3[1] + n3[2]*n3[2]));
	//n1与n2的正交性
	double  orthogonality = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
	return	/*cos_theta < cos(minAngle) && */abs(orthogonality) < orthogonalityThresh? true : false;
}

bool GetAbsorbPoint(bool* planeStat, double* planeParams, PointXYZ* planeCenter, double* refOrient_1, double* refOrient_2, double maxAngle, double orthogonalityThresh, PointXYZ* absorbPt, int* planesIndex, double* linesOrientation)
{
	//初始化输出变量
	int numPlane = 3;			//面的个数
	int numPt = 2;				//吸取点的个数
	for(int i = 0; i < numPt; i++)
		absorbPt[i] = PointXYZ(0, 0, 0);
	memset(planesIndex, 0, numPt * 2 * sizeof(int));
	memset(linesOrientation, 0, numPt * 3 * sizeof(double));

	//寻找法向量与参考方向向量夹角小于阈值的平面
	//int selPlaneIndex = -1;
	int	selCounter = 0;
	int plaIndex[3] = {-1, -1, -1};			//将被使用的平面的索引,-1表示空
	for (int i = 0; i < numPlane; i++)			//寻找上平面
	{
		if(planeStat[i])
		{
			double crossV[3] = {0, 0, 0};
			crossV[0] = refOrient_1[1]*planeParams[i*4+2] - refOrient_1[2]*planeParams[i*4+1];
			crossV[1] = refOrient_1[2]*planeParams[i*4+0] - refOrient_1[0]*planeParams[i*4+2];
			crossV[2] = refOrient_1[0]*planeParams[i*4+1] - refOrient_1[1]*planeParams[i*4+0];
			double	lenC = sqrt(crossV[0]*crossV[0] + crossV[1]*crossV[1] + crossV[2]*crossV[2]);
			double	lenB = sqrt(planeParams[i*4]*planeParams[i*4] + planeParams[i*4+1]*planeParams[i*4+1] + planeParams[i*4+2]*planeParams[i*4+2]);
			double	lenA = sqrt(refOrient_1[0]*refOrient_1[0] + refOrient_1[1]*refOrient_1[1] + refOrient_1[2]*refOrient_1[2]);
			double	sinTheta = lenC / (lenA * lenB);
			if(sinTheta < sin(maxAngle))		
			{
				//selPlaneIndex = i;
				plaIndex[selCounter++] = i;
				break;
			}
		}
	}
	if(selCounter > 0)		//当找到上平面时
	{
		for(int i = 0; i < numPlane; i++)		//寻找前平面
		{
			if(planeStat[i] && i != planesIndex[selCounter-1])
			{
				double crossV[3] = {0, 0, 0};
				crossV[0] = refOrient_2[1]*planeParams[i*4+2] - refOrient_2[2]*planeParams[i*4+1];
				crossV[1] = refOrient_2[2]*planeParams[i*4+0] - refOrient_2[0]*planeParams[i*4+2];
				crossV[2] = refOrient_2[0]*planeParams[i*4+1] - refOrient_2[1]*planeParams[i*4+0];
				double	lenC = sqrt(crossV[0]*crossV[0] + crossV[1]*crossV[1] + crossV[2]*crossV[2]);
				double	lenB = sqrt(planeParams[i*4]*planeParams[i*4] + planeParams[i*4+1]*planeParams[i*4+1] + planeParams[i*4+2]*planeParams[i*4+2]);
				double	lenA = sqrt(refOrient_2[0]*refOrient_2[0] + refOrient_2[1]*refOrient_2[1] + refOrient_2[2]*refOrient_2[2]);
				double	sinTheta = lenC / (lenA * lenB);
				if(sinTheta < sin(maxAngle))		
				{
					plaIndex[selCounter++] = i;
					break;
				}
			}
		}
	}
	switch(selCounter)
	{
	case 0:		//没有上平面，没有前平面
		return false;
		for(int i = 0; i < numPlane; i++)
		{
			if(planeStat[i])
				plaIndex[selCounter++] = i;
		}
		break;
	case 1:		//有上平面或前平面
		for(int i = 0; i < numPlane; i++)
		{
			if(i != plaIndex[0] && planeStat[i])
				plaIndex[selCounter++] = i;
		}
		break;
	case 2:		//有上平面，有前平面
		
		break;
	default:
		break;
	}

	//求交线及中心点
	int resultCounter = 0;			//结果计数器

	for(int itr = 1; itr < selCounter; itr++)
	{
		//初始化变量
		int		p0 = plaIndex[0];
		int		p1 = plaIndex[itr];
		//测试两个平面是否正交
		double orthogonality = planeParams[p0*4] * planeParams[p1*4] + planeParams[p0*4 + 1] * planeParams[p1*4 + 1] + planeParams[p0*4 + 2] * planeParams[p1*4 + 2];
		if(abs(orthogonality) > orthogonalityThresh)
			continue;
		//求交线的方向向量
		double	n2[4] = {0,0,0,0};		
		n2[0] = planeParams[p0*4 + 1] * planeParams[p1*4 + 2] - planeParams[p0*4 + 2] * planeParams[p1*4 + 1];
		n2[1] = planeParams[p0*4 + 2] * planeParams[p1*4 + 0] - planeParams[p0*4 + 0] * planeParams[p1*4 + 2];
		n2[2] = planeParams[p0*4 + 0] * planeParams[p1*4 + 1] - planeParams[p0*4 + 1] * planeParams[p1*4 + 0];
		n2[3] = (n2[0]*planeCenter[p0].x + n2[1]*planeCenter[p0].y + n2[2]*planeCenter[p0].z);
		//求交线上中点的坐标
		// A * X = B
		double	A[3*3];		//系数矩阵
		double	B[3];
		int		nSize = 3;

		memcpy((double*)A, (double*)planeParams + p0*4, nSize*sizeof(double));
		B[0] = -planeParams[p0*4 + nSize];
		memcpy(&A[nSize], (double*)planeParams + p1*4, nSize*sizeof(double));
		B[1] = -planeParams[p1*4 + nSize];

		/*for(int i = 0; i < nSize - 1; i++)
		{
			memcpy((double*)A + i*nSize*sizeof(double), (double*)planeParams + plaIndex[i]*4, nSize*sizeof(double));
			B[i] = -planeParams[plaIndex[i]*4 + nSize];
		}*/
		memcpy(&A[2*nSize], n2, nSize*sizeof(double));
		B[2] = n2[3];
		//求解
		double* D= new double[nSize];
		QRDecomposition(A,D,nSize);
		bool st1 = Householder(A,D,B,nSize);
		delete[] D;

		//输出结果
		absorbPt[resultCounter] = PointXYZ(B[0], B[1], B[2]);
		planesIndex[resultCounter*2 + 0] = plaIndex[0];
		planesIndex[resultCounter*2 + 1] = plaIndex[itr];
		memcpy(linesOrientation + resultCounter*nSize, n2, nSize*sizeof(double));
		resultCounter++;
	}
	return resultCounter > 0 ? true : false;
}

bool	BoxIndentify(xn::DepthGenerator &depthGen, xn::ImageGenerator &imageGen, int imgWidth, int imgHeight, Mat &imgDepthBg16u, double* refOrient_1, double* refOrient_2, double maxAngle, double orthogonalityThresh, PointXYZ* absorbPt,double* planesNormal, int* planesIndex, double* linesOrientation, Mat &imgBgPreROI)
{
	//初始化变量
	DepthMetaData	depthMD;
	ImageMetaData	imageMD;
	Mat		imgDepth16u = Mat::zeros(imgHeight, imgWidth, CV_16U);
	Mat		imgRGB8u = Mat::zeros(imgHeight, imgWidth, CV_8UC3);
	Mat		imgDiff16u = Mat::zeros(imgHeight, imgWidth, CV_16U);
	int		pointSizeThresh = 200;				//平面提取时，三维点书目的最小阈值
	//获取数据
	depthGen.GetMetaData(depthMD);
	imageGen.GetMetaData(imageMD);
	//转换为OpenCV的Mat格式
	memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
	memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
	//深度图滤波
	Mat		imgDepth32f = Mat::zeros(imgDepth16u.size(), CV_32F);
	Mat		imgDepthBuf32f = Mat::zeros(imgDepth16u.size(), CV_32F);
	imgDepth16u.convertTo(imgDepth32f, CV_32F, 1<<15);
	bilateralFilter(imgDepth32f, imgDepthBuf32f, 5, 37, 97);
	imgDepthBuf32f.convertTo(imgDepth16u, CV_16U, 1.0/(1<<15));

	//背景差求得盒子在深度图中的区域
	for(int irow = 0; irow < imgDepth16u.rows; irow++)
	{
		for(int icol = 0; icol < imgDepth16u.cols; icol++)
			imgDiff16u.ptr<ushort>(irow)[icol] = imgDepth16u.ptr<ushort>(irow)[icol] > 0 && imgBgPreROI.ptr<uchar>(irow)[icol] > 0? 
													abs((short)imgDepth16u.ptr<ushort>(irow)[icol] - imgDepthBg16u.ptr<ushort>(irow)[icol]) : 0;
		//	imgDiff16u.ptr<ushort>(irow)[icol] = imgDepth16u.ptr<ushort>(irow)[icol] > 0 && imgDepthBg16u.ptr<ushort>(irow)[icol] > 0  && imgBgPreROI.ptr<uchar>(irow)[icol] > 0 ? 
		//											abs((short)imgDepth16u.ptr<ushort>(irow)[icol] - imgDepthBg16u.ptr<ushort>(irow)[icol]) : 0;

	}	
	
	Mat	imgDiff8u = Mat::zeros(imgDiff16u.size(), CV_8U);
	imgDiff16u.convertTo(imgDiff8u, CV_8U, 255/4096.0);
	threshold(imgDiff8u, imgDiff8u, 60 / 4096.0 * 255, 255, CV_THRESH_BINARY);

	erode(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 4);
	dilate(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 5);
	imshow("Diff", imgDiff8u);
	waitKey(10);
	vector<vector<Point> >	contours;
	vector<Vec4i>			hierarchy;
	findContours(imgDiff8u, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
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
	if(maxArea > 0)
	{
		//根据ROI对深度图进行处理
		imgDiff8u = Mat::zeros(imgDiff8u.size(), CV_8U);
		drawContours(imgDiff8u, contours, maxConnectDomainIn, Scalar(255), CV_FILLED);
	}
	int w = depthMD.FullXRes(), h = depthMD.FullYRes();
	for(int irow = 0; irow < imgDepth16u.rows; irow++)
	{
		for(int icol = 0; icol < imgDepth16u.cols; icol++)
			imgDepth16u.ptr<ushort>(irow)[icol] = imgDiff8u.ptr<uchar>(irow)[icol] ? imgDepth16u.ptr<ushort>(irow)[icol] : 0;
	}

	//计算点云数据
	gPointCloud.clear();
	GeneratePointCloud(depthGen, (XnDepthPixel*)imgDepth16u.data, imageGen.GetRGB24ImageMap(), gPointCloud);
	//io::savePCDFileASCII("pointcloudcache.pcd", gPointCloud);
	if(gPointCloud.size() < pointSizeThresh)
	{
		cout << "\nFail:\tThe size of point cloud is too few!";
		return false;
	}
	//io::savePCDFileASCII("pointcloud.pcd", gPointCloud);
	//点云面片处理
	double		planeParams[4*4] = {};					//面片参数,4面*4参
	bool		planeStat[3] = {false, false, false};	//平面X提取状态
	PointXYZ	planeCenter[3];							//平面X的中心点

	ModelCoefficients::Ptr	coefficients(new ModelCoefficients);
	PointIndices::Ptr		inliersPlane_1(new PointIndices);
	ExtractIndices<PointXYZRGB>		extract;			//点提取对象
	//创建分隔对象
	SACSegmentation<PointXYZRGB>	seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setDistanceThreshold(8);
	seg.setInputCloud(gPointCloud.makeShared());
	seg.segment(*inliersPlane_1, *coefficients);
	//判断已提取的平面的参数，确定平面是否提取成功
	
	//求平面1的中心点
	if(coefficients->values.size() == 4)
	{
		planeStat[0] = true;
		//	输出已提取的平面模型的参数
		for(int i = 0; i < coefficients->values.size(); i++)
			planeParams[i] = coefficients->values[i];
		memcpy(planesNormal, planeParams, 3 * sizeof(double));
		//计算平面1的中心点
		planeCenter[0] = PointXYZ(0,0,0);
		for(int i = 0; i < inliersPlane_1->indices.size(); i++)
		{
			planeCenter[0].x += gPointCloud[ inliersPlane_1->indices[i] ].x / inliersPlane_1->indices.size();
			planeCenter[0].y += gPointCloud[ inliersPlane_1->indices[i] ].y / inliersPlane_1->indices.size();
			planeCenter[0].z += gPointCloud[ inliersPlane_1->indices[i] ].z / inliersPlane_1->indices.size();
		}
		//标记面片1
		for(int i = 0; i < inliersPlane_1->indices.size(); i++)
		{
			gPointCloud[ inliersPlane_1->indices[i] ].r = 255;
			gPointCloud[ inliersPlane_1->indices[i] ].g = 255;
			gPointCloud[ inliersPlane_1->indices[i] ].b = 0;
		}
		//printf("\n%.3f,%.3f,%.3f",planeParams[0], planeParams[1], planeParams[2]);
		//io::savePCDFileASCII("Plane_1.pcd", gPointCloud);
	}
	else
	{
		cout << "\nFail:\tExtract plane-1 failed!";
		return false;
	}
	//	对剩余点进行平面提取
	extract.setInputCloud(gPointCloud.makeShared());
	extract.setIndices(inliersPlane_1);
	extract.setNegative(true);
	PointCloud<PointXYZRGB>::Ptr	pResidueCloud_1(new PointCloud<PointXYZRGB>());
	extract.filter(*pResidueCloud_1);
	//检查剩余点的个数，太少时，则不进行平面提取，直接结束定位操作
	if(pResidueCloud_1->size() < pointSizeThresh)
	{
		cout << "\nFail:\tExtract plane-2 failed!";		
		return false;					//	定位失败，退出
	}

	PointIndices::Ptr				inliersPlane_2(new PointIndices);
	bool	segFinishFlag = false;		//分割结束标志量
	do
	{
		inliersPlane_2->indices.clear();
		coefficients->values.clear();
		seg.setInputCloud(pResidueCloud_1);
		seg.segment(*inliersPlane_2, *coefficients);

		/*for(int iNum = 0; iNum < inliersPlane_2->indices.size(); iNum++)
		{
			(*pResidueCloud_1)[ inliersPlane_2->indices[iNum] ].r = 255;
		}
		io::savePCDFileASCII("ResiduePC.pcd", *pResidueCloud_1);*/

		double orthogonality = coefficients->values[0] * planeParams[0] + coefficients->values[1] * planeParams[1] + coefficients->values[2] * planeParams[2]; 
		if (abs(orthogonality) < orthogonalityThresh)
		{
			segFinishFlag = true;
		}
		else
		{
			segFinishFlag = false;
			extract.setInputCloud(pResidueCloud_1);
			extract.setIndices(inliersPlane_2);
			extract.setNegative(true);
			PointCloud<PointXYZRGB>::Ptr	pResidueCloudTemp(new PointCloud<PointXYZRGB>());
			extract.filter(*pResidueCloudTemp);
			pResidueCloud_1->clear();
			swap(*pResidueCloud_1, *pResidueCloudTemp);
			pResidueCloudTemp->clear();
			//检查剩余点的个数，太少时，则不进行平面提取，直接结束定位操作
			if(pResidueCloud_1->size() < pointSizeThresh)
			{
				cout << "\nFail:\tExtract plane-2 failed!";
				return false;					//	定位失败，退出
			}
		}
		

	}while(!segFinishFlag);

	// 判断提取的平面2是否提取成功
	if (coefficients->values.size() == 4)
	{
		planeStat[1] = true;
		//	输出已提取的平面模型的参数
		for(int i = 0; i < coefficients->values.size(); i++)
			planeParams[4 + i] = coefficients->values[i];
		memcpy(&(planesNormal[3]), &(planeParams[4]), 3 * sizeof(double));

		//计算平面2的中心点
		planeCenter[1] = PointXYZ(0,0,0);
		for(int i = 0; i < inliersPlane_2->indices.size(); i++)
		{
			planeCenter[1].x += (*pResidueCloud_1)[ inliersPlane_2->indices[i] ].x / inliersPlane_2->indices.size();
			planeCenter[1].y += (*pResidueCloud_1)[ inliersPlane_2->indices[i] ].y / inliersPlane_2->indices.size();
			planeCenter[1].z += (*pResidueCloud_1)[ inliersPlane_2->indices[i] ].z / inliersPlane_2->indices.size();			
		}

		//printf("\n%.3f,%.3f,%.3f",planeParams[4], planeParams[5], planeParams[6]);
		//标记面片2
		/*for(int i = 0; i < inliersPlane_2->indices.size(); i++)
		{
			(*pResidueCloud_1)[ inliersPlane_2->indices[i] ].r = 255;
			(*pResidueCloud_1)[ inliersPlane_2->indices[i] ].g = 0;
			(*pResidueCloud_1)[ inliersPlane_2->indices[i] ].b = 0;
		}
		io::savePCDFileASCII("Plane_2.pcd", *pResidueCloud_1);*/
	}
	else
	{
		cout << "\nFail:\tExtract plane-2 failed!";
		return false;
	}

	//	再对剩余点进行平面提取
	PointCloud<PointXYZRGB>::Ptr	pResidueCloud_2(new PointCloud<PointXYZRGB>());
	extract.setInputCloud(pResidueCloud_1);
	extract.setIndices(inliersPlane_2);
	extract.setNegative(true);
	extract.filter(*pResidueCloud_2);
	//检查剩余点的个数，太少时，则不进行平面提取，直接结束定位操作
	if(pResidueCloud_2->size() > pointSizeThresh)
	{
		PointIndices::Ptr		inliersPlane_3(new PointIndices);
		coefficients->values.clear();
		seg.setInputCloud(pResidueCloud_2);
		seg.segment(*inliersPlane_3, *coefficients);
		// 判断提取的平面3是否提取成功
		if (coefficients->values.size() == 4)
		{
			planeStat[2] = true;
			//	输出已提取的平面模型的参数
			for(int i = 0; i < coefficients->values.size(); i++)
				planeParams[4*2 + i] = coefficients->values[i];
			memcpy(&(planesNormal[6]), &(planeParams[8]), 3 * sizeof(double));

			//计算平面2的中心点
			planeCenter[2] = PointXYZ(0,0,0);
			for(int i = 0; i < inliersPlane_3->indices.size(); i++)
			{
				planeCenter[2].x += (*pResidueCloud_2)[ inliersPlane_3->indices[i] ].x / inliersPlane_3->indices.size();
				planeCenter[2].y += (*pResidueCloud_2)[ inliersPlane_3->indices[i] ].y / inliersPlane_3->indices.size();
				planeCenter[2].z += (*pResidueCloud_2)[ inliersPlane_3->indices[i] ].z / inliersPlane_3->indices.size();			
			}
			//标记面片3
			/*for(int i = 0; i < inliersPlane_3->indices.size(); i++)
			{
			(*pResidueCloud_2)[ inliersPlane_3->indices[i] ].r = 0;
			(*pResidueCloud_2)[ inliersPlane_3->indices[i] ].g = 255;
			(*pResidueCloud_2)[ inliersPlane_3->indices[i] ].b = 0;
			}
			io::savePCDFileASCII("Plane_3.pcd", *pResidueCloud_2);*/

		}
	}

	/*for (int ip = 0; ip < 3; ip++)
	{
		for(int it = 0; it < 4; it++)
			printf("\t%3.3f", planeParams[ip * 4 + it]);
		printf("\n");
	}*/

	//由三个面的中心点、法向量，计算得出适合的抓取点和相邻两个面的法向量
	//调用函数
	bool		st2 = GetAbsorbPoint(planeStat, planeParams, planeCenter, refOrient_1, refOrient_2, maxAngle, orthogonalityThresh, absorbPt, planesIndex, linesOrientation);
	if(st2)
	{
		//添加中点坐标突出显示 及 直线位置
		for(int ix = -3; ix < 4; ix++)
		{
			for(int iy = -3; iy < 4; iy++)
			{
				for(int iz = -3; iz < 4; iz++)
				{
					PointXYZRGB	pt2(255, 0 , 0);
					pt2.x = planeCenter[0].x + ix;
					pt2.y = planeCenter[0].y + iy;
					pt2.z = planeCenter[0].z + iz;
					gPointCloud.push_back(pt2);
				}
			}
		}
		//交线
		for(int i = -200; i < 201; i++)
		{				
			PointXYZRGB	pt2(0, 255 , 0);
			pt2.x = absorbPt[0].x + i * linesOrientation[0];
			pt2.y = absorbPt[0].y + i * linesOrientation[1];
			pt2.z = absorbPt[0].z + i * linesOrientation[2];
			gPointCloud.push_back(pt2);
			/*PointXYZRGB	pt(0, 255 , 0);
			pt.x = absorbPt[1].x + i * linesOrientation[3];
			pt.y = absorbPt[1].y + i * linesOrientation[4];
			pt.z = absorbPt[1].z + i * linesOrientation[5];
			gPointCloud.push_back(pt);*/
		}
		//平面1
		for(int i = -20; i < 21; i++)
		{
			PointXYZRGB	pt(255, 255 , 0);
			pt.x = absorbPt[0].x + i * planeParams[4*planesIndex[0]];
			pt.y = absorbPt[0].y + i * planeParams[4*planesIndex[0] + 1];
			pt.z = absorbPt[0].z + i * planeParams[4*planesIndex[0] + 2];
			gPointCloud.push_back(pt);
			/*PointXYZRGB	pt2(255, 255 , 0);
			pt2.x = absorbPt[1].x + i * planeParams[4*planesIndex[2]];
			pt2.y = absorbPt[1].y + i * planeParams[4*planesIndex[2] + 1];
			pt2.z = absorbPt[1].z + i * planeParams[4*planesIndex[2] + 2];
			gPointCloud.push_back(pt2);*/
		}
		//平面2
		for(int i = -20; i < 21; i++)
		{
			PointXYZRGB	pt(255, 0, 0);
			pt.x = absorbPt[0].x + i * planeParams[4*planesIndex[1]];
			pt.y = absorbPt[0].y + i * planeParams[4*planesIndex[1] + 1];
			pt.z = absorbPt[0].z + i * planeParams[4*planesIndex[1] + 2];
			gPointCloud.push_back(pt);
			/*PointXYZRGB	pt2(255, 0, 0);
			pt2.x = absorbPt[1].x + i * planeParams[4*planesIndex[3]];
			pt2.y = absorbPt[1].y + i * planeParams[4*planesIndex[3] + 1];
			pt2.z = absorbPt[1].z + i * planeParams[4*planesIndex[3] + 2];
			gPointCloud.push_back(pt2);*/
		}
		//存储PCD文件
		time_t	timer;
		tm*		pTimeInfo;
		time(&timer);
		pTimeInfo = localtime(&timer);
		char	pcdFilename[200];
		 
		sprintf(pcdFilename, "/home/vision/moveit/%d.pcd",
			(int)absorbPt[0].z/100 );
		ROS_WARN("/home/vision/moveit/%d.pcd", (int)absorbPt[0].z/100);
		io::savePCDFileASCII(pcdFilename, gPointCloud);

		Mat		imgDepthShow = Mat::zeros(imgHeight, imgWidth, CV_16U);
		Mat		imgRGBShow = Mat::zeros(imgHeight, imgWidth, CV_8UC3);
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);
		imshow("Image", imgRGBShow);
		imshow("Depth", imgDepthShow);
		waitKey(10);
		return true;
	}
	else
	{
		printf("\n警告：盒子定位失败！\t");
		return false;
	}
}
