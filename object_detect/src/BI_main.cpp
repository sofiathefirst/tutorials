/*
	���̹��ܣ������ӵ���ά���꼰����
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
	�������ܣ������󣬲����������ʾ
	���������
	result		in	���н��
	status		in	���еĺ���
*/
void CheckOpenNIError( XnStatus result, string status );

/*
	�������ܣ�������ά��������
	���������
	rDepthGen		in	����������ɽڵ�
	pDepth			in	�������
	pImage			in	RGB��ɫͼ����
	vPointCloud		out	���ɵ���ά��������
*/
void GeneratePointCloud( xn::DepthGenerator& rDepthGen,	const XnDepthPixel* pDepth,	const XnRGB24Pixel* pImage,	PointCloud<PointXYZRGB>& vPointCloud, Rect roiRect = Rect(0, 0, IMGWIDTH, IMGHEIGHT));

/*
	�������ܣ������Է�����(n*n)
*/
void QRDecomposition(double* coeffM, double* D, int nSize);
bool Householder(double const*coeffM, double const* D, double* B, int nSize);
/*
	ʹ��ʾ��
	int n=3;
	double a[] = {4, 5, 11, 10, 3, 5, 7, 40, -38};
	double b[] = {47, 31, -27};
	double* d=new double[n]();
	QRDecomposition(a,d,n);
	Householder(a,d,b,n);
	�����b��
*/

/*
	�������ܣ��ж��������Ƿ�Ϊ���ӵ�ǰƽ�����ƽ��
	���������
		n1		<in>	ƽ��1��������1*3��������ָ��
		n2		<in>	ƽ��2��������1*3��������ָ��
		n3		<out>	����n1��n2�ķ�������1*3��������ָ��
*/
bool CanBeUsed(double* n1, double* n2, double* n3, double maxAngle);

//ȫ�ֱ���
PointCloud<PointXYZRGB>	gPointCloud;
Point					gStartPt;
Point					gEndPt;
bool					gSelectFlag;

int test2()
{
	int key = 0;
	//����OpenNI����
	XnStatus	resultStat = XN_STATUS_OK;
	xn::Context		kinectCont;
	DepthMetaData	depthMD;
	ImageMetaData	imageMD;

	//����OpenCV����
	Mat		imgDepth16u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
	Mat		imgRGB8u = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
	Mat		imgDepthShow = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_16U);
	Mat		imgRGBShow = Mat::zeros(IMGHEIGHT, IMGWIDTH, CV_8UC3);
	Mat		imgDepthBg16u = imread("bg30.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);		//��ȡ�������ͼ��һ��ҪΪ16λ��
	namedWindow("RGB");
	namedWindow("Depth");

	//����OpenNI�ĸ����ڵ�
	resultStat = kinectCont.Init();
	CheckOpenNIError(resultStat, "OpenNI Context Initial...\t");
	//������¼���ļ�
	resultStat = kinectCont.OpenFileRecording("Record_32.oni");
	CheckOpenNIError(resultStat, "Read record file...\t");

	DepthGenerator	depthGen;
	resultStat = depthGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create depth generator...\t");
	ImageGenerator	imageGen;
	resultStat = imageGen.Create(kinectCont);
	CheckOpenNIError(resultStat, "Create image generator...\t");

	//��ʼ�������ڵ�
	XnMapOutputMode	mapMode;
	mapMode.nXRes = IMGWIDTH;
	mapMode.nYRes = IMGHEIGHT;
	mapMode.nFPS = 30;
	resultStat = depthGen.SetMapOutputMode(mapMode);
	CheckOpenNIError(resultStat, "Set depth generator output mode...\t");
	resultStat = imageGen.SetMapOutputMode(mapMode);
	CheckOpenNIError(resultStat, "Set image generator output mode...\t");
	depthGen.GetAlternativeViewPointCap().SetViewPoint(imageGen);

	//��ʼ��������
	resultStat = kinectCont.StartGeneratingAll();
	//��ʼ��������
	resultStat = kinectCont.WaitAndUpdateAll();

	Mat imgDiff16u = Mat::zeros(imgDepth16u.size(), CV_16U);

	while ((key != 27) && !(resultStat = kinectCont.WaitAndUpdateAll()))
	{
		double timeNode = (double)getTickCount();
		//��ȡ����
		depthGen.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);

		//ת��ΪOpenCV��Mat��ʽ
		memcpy(imgRGB8u.data, imageMD.Data(), IMGWIDTH*IMGHEIGHT*3);
		cvtColor(imgRGB8u, imgRGBShow, CV_RGB2BGR);
		memcpy(imgDepth16u.data, depthMD.Data(), IMGWIDTH*IMGHEIGHT*2);
		imgDepth16u.convertTo(imgDepthShow, CV_8U, 255/4096.0);
		//��������ú��������ͼ�е�����
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
		//�����ͨ��
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
			//����ROI�����ͼ���д���
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

		//�����������
		gPointCloud.clear();
		GeneratePointCloud(depthGen, (XnDepthPixel*)imgDepth16u.data, imageGen.GetRGB24ImageMap(), gPointCloud);
		//������Ƭ����
		double			planeParams[4*3] = {};		//��Ƭ�Ĳ���
		ModelCoefficients::Ptr	coefficients(new ModelCoefficients);
		PointIndices::Ptr		inliersPlane_1(new PointIndices);
		ExtractIndices<PointXYZRGB>		extract;			//����ȡ����
		//�����ָ�����
		SACSegmentation<PointXYZRGB>	seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SACMODEL_PLANE);
		seg.setMethodType(SAC_RANSAC);
		seg.setDistanceThreshold(5);
		seg.setInputCloud(gPointCloud.makeShared());
		seg.segment(*inliersPlane_1, *coefficients);
		//�ж�����ȡ��ƽ��Ĳ�����ȷ��ƽ���Ƿ���ȡ�ɹ�

		//	�������ȡ��ƽ��ģ�͵Ĳ���
		for(int i = 0; i < coefficients->values.size(); i++)
		{
			planeParams[i] = coefficients->values[i];
		}
		//��ƽ��1�����ĵ�
		extract.setInputCloud(gPointCloud.makeShared());
		extract.setIndices(inliersPlane_1);
		extract.setNegative(false);
		PointCloud<PointXYZRGB>::Ptr	pPlane1(new PointCloud<PointXYZRGB>());
		extract.filter(*pPlane1);
		PointXYZ		cp(0, 0, 0);			//ƽ��1�����ĵ�
		for(int i = 0; i < pPlane1->size(); i++)
		{
			cp.x += (*pPlane1)[i].x / pPlane1->size();
			cp.y += (*pPlane1)[i].y / pPlane1->size();
			cp.z += (*pPlane1)[i].z / pPlane1->size();
		}

		//	��ʣ������ƽ����ȡ
		extract.setInputCloud(gPointCloud.makeShared());
		extract.setIndices(inliersPlane_1);
		extract.setNegative(true);
		PointCloud<PointXYZRGB>::Ptr	pResidueCloud_1(new PointCloud<PointXYZRGB>());
		extract.filter(*pResidueCloud_1);
		PointIndices::Ptr		inliersPlane_2(new PointIndices);
		seg.setInputCloud(pResidueCloud_1);
		seg.segment(*inliersPlane_2, *coefficients);
		// �ж���ȡ��ƽ��2�Ƿ����
		double	n2[4] = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};		//ƽ��2�ķ�����
		bool	st1 = CanBeUsed(planeParams, n2, (double*)planeParams + 4*2, 50/180.0*3.1415926);
		if(!st1)			//ƫת�Ƕȴ�����ֵ�Ƕȣ�������ȡƽ��
		{
			//	�ٶ�ʣ������ƽ����ȡ
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
			//������ǰƽ������ƽ�潻�ߵ��е�
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
				printf("\n������ƽ����ǰƽ��Ľ����е�����Ϊ��\t(%.4f,%.4f,%.4f)\n������ĳ���ֱ�Ϊ\n\tN1=(%.4f,%.4f,%.4f)\tN2=(%.4f,%.4f,%.4f)\n", 
							B[0], B[1], B[2], planeParams[0], planeParams[1], planeParams[2], planeParams[4], planeParams[5], planeParams[6]);

			//	��ѡ�����
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
/*
			//����е�����ͻ����ʾ �� ֱ��λ��
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

			//�洢PCD�ļ�
			time_t	timer;
			tm*		pTimeInfo;
			time(&timer);
			pTimeInfo = localtime(&timer);
			char	pcdFilename[200];
			sprintf(pcdFilename, "P(%3.0f, %3.0f, %3.0f) N1(%3.0f, %3.0f, %3.0f) N2(%3.0f, %3.0f, %3.0f)---%.2d%.2d.%.2d.pcd", B[0], B[1], B[2], planeParams[0], planeParams[1], planeParams[2], planeParams[4], planeParams[5], planeParams[6],pTimeInfo->tm_mon+1, pTimeInfo->tm_mday, pTimeInfo->tm_hour);
			io::savePCDFileASCII(pcdFilename, *pCloudPlane);
*/
		}
		else
			printf("\n���棺���Ӷ�λʧ�ܣ�\t");
		printf("���Ӷ�λ��ʱΪ��%fms\n", ((double)getTickCount() - timeNode)*1000/getTickFrequency());
		imshow("RGB", imgRGBShow);
		imshow("Depth", imgDepthShow);
		key = waitKey(20);

	}

	destroyAllWindows();
	kinectCont.StopGeneratingAll();
	//kinectCont.Shutdown();

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

		*(D+i)=temp1;//�洢��Ԫ
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
	// ������n3��Z��ļн�
	double	z = n3[2] > 0 ? 1 : -1;
	double	cos_theta = n3[2] * z / (sqrtf(n3[0]*n3[0] + n3[1]*n3[1] + n3[2]*n3[2]));
	return	cos_theta < cos(maxAngle) ? true : false;
}
