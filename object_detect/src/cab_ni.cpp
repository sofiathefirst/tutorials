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
#include "object_detect/cabinetPose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include<geometry_msgs/Pose.h>
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
ros::Publisher positionpub;
Matrix<float,4,4> mat_kinect;
tf2::Matrix3x3 mat3;
geometry_msgs::Pose srvpose;
bool srvsend;
const static double PI = 3.1415926;
void seeV3(tf2::Vector3 v)
{
	//v.normalize();
	ROS_INFO("tf2:ad:Vector3 :X=%f,%f,%f",v.getX(),v.getY(),v.getZ());
}

bool setsrvPosition()
{
	if(isnan(srvpose.position.x) || isnan(srvpose.position.y) || isnan(srvpose.position.z))
		return false;
	//if(srvpose.position.z>2 ||srvpose.position.z < 1.2 ) return false;
	//ROS_INFO("DEPTH : %f,%f,%f",srvpose.position.x,srvpose.position.y,srvpose.position.z);

	Matrix<float,4,1> Pr;
	Matrix<float,4,1> Pc;
	Pc << 	srvpose.position.x,
	  	srvpose.position.y,
	  	srvpose.position.z,
		    1;

	Pr = mat_kinect *Pc;

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
	Mat		imgDepthBg16u;			//深度背景图，一定要为16位位深
	bool	loadBg = true;

	imgDepthBg16u = imread("DepthBg.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

	namedWindow("RGB");
	namedWindow("Depth");

	//声明OpenNI的各个节点
	resultStat = kinectCont.Init();
	CheckOpenNIError(resultStat, "OpenNI Context Initial...\t");
	/*//播放已录制文件
	resultStat = kinectCont.OpenFileRecording("Files\\Record_32.oni");
	CheckOpenNIError(resultStat, "Read record file...\t");*/

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
		//获取数据
		depthGen.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);
		//转换为OpenCV的Mat格式
		memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);
		memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);
		//保存深度背景图
		if (BG)
		{
			imwrite("DepthBg16u.png", imgDepth16u);
			imgDepthBg16u = imgDepth16u.clone();
		}else if(imgDepthBg16u.empty())
		{
			imshow("RGB", imgRGBShow);
			imshow("Depth", imgDepthShow);
			key = waitKey(20);
			continue;
		}
		double timeNode = (double)getTickCount();		
		//背景差求得盒子在深度图中的区域
		for(int irow = 0; irow < imgDepth16u.rows; irow++)
		{
			for(int icol = 0; icol < imgDepth16u.cols; icol++)
				imgDiff16u.ptr<ushort>(irow)[icol] = imgDepth16u.ptr<ushort>(irow)[icol] > 0 && imgDepthBg16u.ptr<ushort>(irow)[icol] > 0 ? 
														abs((short)imgDepth16u.ptr<ushort>(irow)[icol] - imgDepthBg16u.ptr<ushort>(irow)[icol]) : 0;
		}		
		Mat	imgDiff8u = Mat::zeros(imgDepthShow.size(), CV_8U);
		imgDiff16u.convertTo(imgDiff8u, CV_8U, 255/4096.0);
		threshold(imgDiff8u, imgDiff8u, 60 / 4096.0 * 255, 255, CV_THRESH_BINARY);
		erode(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 4);
		dilate(imgDiff8u, imgDiff8u, Mat(), Point(1,1), 5);
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
		if(maxArea < 10)
		{
			cout << "Warning: Box indentify failed!" << endl;
			continue;
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
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);

		//计算点云数据
		gPointCloud.clear();
		GeneratePointCloud(depthGen, (XnDepthPixel*)imgDepth16u.data, imageGen.GetRGB24ImageMap(), gPointCloud);
		//点云面片处理
		double			planeParams[4*3] = {};		//面片的参数
		ModelCoefficients::Ptr	coefficients(new ModelCoefficients);
		PointIndices::Ptr		inliersPlane_1(new PointIndices);
		ExtractIndices<PointXYZRGB>		extract;			//点提取对象
		//创建分隔对象
		SACSegmentation<PointXYZRGB>	seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SACMODEL_PLANE);
		seg.setMethodType(SAC_RANSAC);
		seg.setDistanceThreshold(5);
		seg.setInputCloud(gPointCloud.makeShared());
		seg.segment(*inliersPlane_1, *coefficients);
		//判断已提取的平面的参数，确定平面是否提取成功

		//	输出已提取的平面模型的参数
		for(int i = 0; i < coefficients->values.size(); i++)
		{
			planeParams[i] = coefficients->values[i];
		}
		//求平面1的中心点
		extract.setInputCloud(gPointCloud.makeShared());
		extract.setIndices(inliersPlane_1);
		extract.setNegative(false);
		PointCloud<PointXYZRGB>::Ptr	pPlane1(new PointCloud<PointXYZRGB>());
		extract.filter(*pPlane1);
		PointXYZ		cp(0, 0, 0);			//平面1的中心点
		for(int i = 0; i < pPlane1->size(); i++)
		{
			cp.x += (*pPlane1)[i].x / pPlane1->size();
			cp.y += (*pPlane1)[i].y / pPlane1->size();
			cp.z += (*pPlane1)[i].z / pPlane1->size();
		}

		//	对剩余点进行平面提取
		extract.setInputCloud(gPointCloud.makeShared());
		extract.setIndices(inliersPlane_1);
		extract.setNegative(true);
		PointCloud<PointXYZRGB>::Ptr	pResidueCloud_1(new PointCloud<PointXYZRGB>());
		extract.filter(*pResidueCloud_1);
		PointIndices::Ptr		inliersPlane_2(new PointIndices);
		seg.setInputCloud(pResidueCloud_1);
		seg.segment(*inliersPlane_2, *coefficients);
		// 判断提取的平面2是否可用
		double	n2[4] = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};		//平面2的法向量
		bool	st1 = CanBeUsed(planeParams, n2, (double*)planeParams + 4*2, 50/180.0*3.1415926);
		if(!st1)			//偏转角度大于阈值角度，重新提取平面
		{
			//	再对剩余点进行平面提取
			pResidueCloud_1->clear();
			inliersPlane_2->indices.clear();
			extract.setInputCloud(pResidueCloud_1);
			extract.setIndices(inliersPlane_2);
			extract.setNegative(true);
			extract.filter(*pResidueCloud_1);
			seg.setInputCloud(pResidueCloud_1);
			seg.segment(*inliersPlane_2, *coefficients);
			n2[0] = coefficients->values[0];
			n2[1] = coefficients->values[1];
			n2[2] = coefficients->values[2];
			n2[3] = coefficients->values[3];
			st1 = CanBeUsed(planeParams, n2, (double*)planeParams + 4*2, 40/180.0*3.1415926);
		}
		if(st1)
		{
			memcpy((double*)planeParams + 4, n2, sizeof(double)*4);
			//求解盒子前平面与上平面交线的中点
			planeParams[11] = -(planeParams[4*2]*cp.x + planeParams[4*2 + 1]*cp.y + planeParams[4*2 + 2]*cp.z);
			double	A[3*3];
			double	B[3];
			int nSize = 3;
			for(int i = 0; i < nSize; i++)
			{
				memcpy((double*)A + i*nSize, (double*)planeParams + i*4, nSize * sizeof(double));
				B[i] = -planeParams[i*4 + nSize];
			}
			double* D= new double[nSize]();
			QRDecomposition(A,D,nSize);
			st1 = Householder(A,D,B,nSize);
			delete[] D;
			if(st1)
			{	printf("\n盒子上平面与前平面的交线中点坐标为：\t(%.4f,%.4f,%.4f)\n两个面的朝向分别为\n\tN1=(%.4f,%.4f,%.4f)\tN2=(%.4f,%.4f,%.4f)\n", 
							B[0], B[1], B[2], planeParams[0], planeParams[1], planeParams[2], planeParams[4], planeParams[5], planeParams[6]);
				srvpose.position.z = B[2];
				srvpose.position.x = B[0];
				srvpose.position.y = B[1];
							   				
				srvsend=setsrvPosition();
				positionpub.publish(srvpose);	

				
			//	ÒÑÑ¡ÔñµãÔÆ
			extract.setInputCloud(gPointCloud.makeShared());
			extract.setIndices(inliersPlane_1);
			extract.setNegative(false);
			PointCloud<PointXYZRGB>::Ptr	pCloudPlane(new PointCloud<PointXYZRGB>());
			extract.filter(*pCloudPlane);
			
			extract.setInputCloud(pResidueCloud_1);
			extract.setIndices(inliersPlane_2);
			extract.setNegative(false);
			PointCloud<PointXYZRGB>::Ptr	pCloudPlaneTemp(new PointCloud<PointXYZRGB>());
			extract.filter(*pCloudPlaneTemp);
			pCloudPlane->insert(pCloudPlane->end(), pCloudPlaneTemp->begin(), pCloudPlaneTemp->end());

			//ÌíŒÓÖÐµã×ø±êÍ»³öÏÔÊŸ Œ° Ö±ÏßÎ»ÖÃ
			for(int ix = -3; ix < 4; ix++)
			{
				for(int iy = -3; iy < 4; iy++)
				{
					for(int iz = -3; iz < 4; iz++)
					{
						PointXYZRGB	pt(255, 0 , 0);
						pt.x = B[0] + ix;
						pt.y = B[1] + iy;
						pt.z = B[2] + iz;
						pCloudPlane->push_back(pt);
						PointXYZRGB	pt2(255, 0 , 0);
						pt2.x = cp.x + ix;
						pt2.y = cp.y + iy;
						pt2.z = cp.z + iz;
						pCloudPlane->push_back(pt2);
					}
				}
			}
			double	length3 = sqrt(planeParams[4*2] * planeParams[4*2] + planeParams[4*2+1] * planeParams[4*2+1] + planeParams[4*2+2] * planeParams[4*2+2]);
			double n3[3] = {planeParams[4*2]/length3, planeParams[4*2+1]/length3, planeParams[4*2+1]/length3};

			for(int i = -10; i < 11; i++)
			{
				PointXYZRGB	pt(0, 255 , 0);
				pt.x = B[0] + i * n3[0];
				pt.y = B[1] + i * n3[1];
				pt.z = B[2] + i * n3[2];
				pCloudPlane->push_back(pt);
			}

			//ŽæŽ¢PCDÎÄŒþ
			time_t	timer;
			tm*		pTimeInfo;
			time(&timer);
			pTimeInfo = localtime(&timer);
			char	pcdFilename[200];
			///sprintf(pcdFilename, "P(%3.0f, %3.0f, %3.0f) N1(%3.0f, %3.0f, %3.0f) N2(%3.0f, %3.0f, %3.0f)---%.2d%.2d.%.2d.pcd", B[0], B[1], B[2], planeParams[0], planeParams[1], planeParams[2], planeParams[4], planeParams[5], planeParams[6],pTimeInfo->tm_mon+1, pTimeInfo->tm_mday, pTimeInfo->tm_hour);
			io::savePCDFileASCII("cabni1.pcd", *pCloudPlane);
			//io::savePCDFileASCII("cabni2.pcd", *pCloudPlane);
			}
			else
			{
				
			}

		}
		else
			printf("\n警告：盒子定位失败！\t");
		printf("盒子定位耗时为：%fms\n", ((double)getTickCount() - timeNode)*1000/getTickFrequency());
		imshow("RGB", imgRGBShow);
		imshow("Depth", imgDepthShow);
		key = waitKey(20);

	}

	destroyAllWindows();
	kinectCont.StopGeneratingAll();
	kinectCont.Release();

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

