/*
	工程功能：检测盒子的三维坐标及方向
*/

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

#define IMGWIDTH	640
#define IMGHEIGHT	480

using namespace std;
using namespace cv;
using namespace xn;
using namespace pcl;

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

/*
	函数功能：解线性方程组(n*n)
*/
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
	输入参数：
		n1		<in>	平面1法向量（1*3）的数据指针
		n2		<in>	平面2法向量（1*3）的数据指针
		n3		<out>	向量n1、n2的法向量（1*3）的数据指针
*/
bool CanBeUsed(double* n1, double* n2, double* n3, double maxAngle);

//全局变量
PointCloud<PointXYZRGB>	gPointCloud;
Point					gStartPt;
Point					gEndPt;
bool					gSelectFlag;

int test2()
{
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
	Mat		imgDepthBg16u = imread("bg30.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);		//读取背景深度图，一定要为16位深
	namedWindow("RGB");
	namedWindow("Depth");

	//声明OpenNI的各个节点
	resultStat = kinectCont.Init();
	CheckOpenNIError(resultStat, "OpenNI Context Initial...\t");
	//播放已录制文件
	//resultStat = kinectCont.OpenFileRecording("Files\\Record_32.oni");
	//CheckOpenNIError(resultStat, "Read record file...\t");

	DepthGenerator	depthGen;
	resultStat = depthGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create depth generator...\t");
	ImageGenerator	imageGen;
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

	Mat imgDiff16u = Mat::zeros(imgDepth16u.size(), CV_16U);

	while ((key != 27) && !(resultStat = kinectCont.WaitAndUpdateAll()))
	{
		double timeNode = (double)getTickCount();
		//获取数据
		depthGen.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);

		//转换为OpenCV的Mat格式
		memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);
		memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);
		//背景差求得盒子在深度图中的区域
		
		imshow("RGB", imgRGBShow);
		imshow("Depth", imgDepthShow);
		key = waitKey(20);

	}

	destroyAllWindows();
	kinectCont.StopGeneratingAll();
	kinectCont.Shutdown();

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
	delete[] p3DPointSet;
}
/*
void sdldie(const char *msg)
{
	printf("%s: %s\n", msg, SDL_GetError());
	SDL_Quit();
	exit(1);
}

void checkSDLError(int line)
{
#ifndef NDEBUG
	const char *error = SDL_GetError();
	if (*error != '\0')
	{
		printf("SDL Error: %s\n", error);
		if (line != -1)
			printf(" + line: %i\n", line);
		SDL_ClearError();
	}
#endif
}
*/
/*
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
*/

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


bool CanBeUsed(double* n1, double* n2, double* n3, double maxAngle)
{
	n3[0] = n1[1] * n2[2] - n1[2] * n2[1];
	n3[1] = n1[2] * n2[0] - n1[0] * n2[2];
	n3[2] = n1[0] * n2[1] - n1[1] * n2[0];
	// 求向量n3与Z轴的夹角
	double	z = n3[2] > 0 ? 1 : -1;
	double	cos_theta = n3[2] * z / (sqrtf(n3[0]*n3[0] + n3[1]*n3[1] + n3[2]*n3[2]));
	return	cos_theta < cos(maxAngle) ? true : false;
}
