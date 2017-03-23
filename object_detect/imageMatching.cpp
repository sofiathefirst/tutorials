/*--- Author:Yang Xianghong    Function: Corner detection and image matching---*/

#include <opencv2\opencv.hpp>
#include <opencv2\nonfree\nonfree.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\legacy\legacy.hpp>
#include <iostream>
#include <Windows.h>
using namespace std;
using namespace cv;

/*********************公共变量*************************/
//检测器
GoodFeaturesToTrackDetector detector(1000, 0.01, 10.0);//返回的最大特征点数目，质量等级，两点之间的最小允许距离
//提取器
SiftDescriptorExtractor extractor;
//第一个以及第二个最近邻之间的最大比率
float ratio = 0.65f;
//是否再次优化F矩阵
bool refineF = true;
//到极线的最小矩离
double distance = 3.0;
//置信等级（概率）
double confidence = 0.99;
/**********************************************************************
* 函数名:ratioTest
* 参  数:matches
* 返  回:移除的匹配数量
* 说  明:对当前匹配进行筛选，最优匹配和次优匹配响应强度大于ratio的匹配以及
*  孤立的匹配。
***********************************************************************/
int ratioTest(std::vector<std::vector<cv::DMatch>>& matches)
{
	int removed = 0;
	std::vector<std::vector<cv::DMatch>>::iterator matchIt = matches.begin();
	for (; matchIt != matches.end(); matchIt++)
	{
		if (matchIt->size()  > 1)
		{
			//移除不合格比率的匹配（问题出现在这里）
			if ((*matchIt)[0].distance / (*matchIt)[1].distance > ratio)
			{
				matchIt->clear();
				removed++;
			}
		}
		else
		{
			//移除孤立的匹配
			matchIt->clear();
			removed++;
		}
	}
	return removed;
}
/**********************************************************************
* 函数名:symmetryTest
* 参  数:matches1:左匹配
*       matches2:右匹配
*       symMatche:输出的对称匹配
* 返  回:无
* 说  明:对左、右匹配进行检查，输出对称的匹配。
***********************************************************************/
void symmetryTest(std::vector<std::vector<cv::DMatch>>& matches1,
	std::vector<std::vector<cv::DMatch>>& matches2, std::vector<cv::DMatch>& symMatches)
{
	//遍历左匹配
	for (auto &leftMatchRef : matches1)
	{
		if (leftMatchRef.size() < 2)
			continue;
		//遍历右匹配
		for (auto &rightMatchRef : matches2)
		{
			if (rightMatchRef.size() < 2)
				continue;
			//对称性测试
			if ((leftMatchRef[0].queryIdx == rightMatchRef[0].trainIdx) &&
				(leftMatchRef[0].trainIdx == rightMatchRef[0].queryIdx))
			{
				symMatches.push_back(cv::DMatch(leftMatchRef[0].queryIdx, leftMatchRef[0].trainIdx,
					leftMatchRef[0].distance));
				break;
			}
		}
	}
}

/**********************************************************************
* 函数名:ransacTest
* 参  数:matches:当前匹配（输入）
*       keypoints1:图像1检测到的关键点（输入）
*       keypoints2:图像2检测到的关键点（输入）
*       outMatches:完成测试的匹配（输出）
* 返  回:基础矩阵
* 说  明:对当前匹配进行RANSAC测试，计算基础矩阵，同时返回通过测试的匹配
***********************************************************************/
cv::Mat ransacTest(
	const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& outMatches)
{
	//将Keypoints转换到Point2f
	std::vector<cv::Point2f>points1, points2;
	for (auto &matchesRef : matches)
	{
		//左图像关键点
		float x = keypoints1[matchesRef.queryIdx].pt.x;
		float y = keypoints1[matchesRef.queryIdx].pt.y;
		points1.push_back(cv::Point2f(x, y));
		//右图像关键点
		x = keypoints2[matchesRef.trainIdx].pt.x;
		y = keypoints2[matchesRef.trainIdx].pt.y;
		points2.push_back(cv::Point2f(x, y));
	}
	//基于RANSAC计算F矩阵
	//cout << "p1: " << points1.size() << endl;
	//cout << "p2: " << points2.size() << endl;
	std::vector<uchar>inlines(points1.size(), 0);
	cv::Mat fundemental = cv::findFundamentalMat(
		cv::Mat(points1),
		cv::Mat(points2),//匹配点
		inlines,        //匹配状态：inlier或者outlier
		CV_FM_RANSAC,  //RANSAC方法
		3.0,       //到极线的距离
		confidence  //置信概率
		);
	//提取通过的匹配
	std::vector<uchar>::const_iterator itIn = inlines.begin();
	std::vector<cv::DMatch>::const_iterator itM = matches.begin();
	//遍历所有匹配
	for (; itIn != inlines.end(); itIn++, itM++)
	{
		if (*itIn)
		{
			outMatches.push_back(*itM);
		}
	}
	//二次拟合
	if (refineF)
	{
		//F矩阵将使用所有接受的匹配重新计算
		//装换KeyPoint类型到Point2f
		//准备计算最终的F矩阵
		points1.clear();
		points2.clear();
		for (auto &matchRef : outMatches)
		{
			//得到左边特征点坐标
			float x = keypoints1[matchRef.queryIdx].pt.x;
			float y = keypoints1[matchRef.queryIdx].pt.y;
			points1.push_back(cv::Point2f(x, y));
			//得到右边特征点的坐标
			x = keypoints2[matchRef.trainIdx].pt.x;
			y = keypoints2[matchRef.trainIdx].pt.y;
			points2.push_back(cv::Point2f(x, y));
		}
		//从所有接受的匹配中计算8点F
		fundemental = cv::findFundamentalMat(
			cv::Mat(points1),
			cv::Mat(points2),
			CV_FM_8POINT
			);

		//cout << "Points" << points1 << endl;
		//cout << "Points" << points2 << endl;

	}
	return fundemental;
}
/**********************************************************************
* 函数名:match
* 参  数:image1 :图像1（输入）
*        image2:图像2（输入）
*        matches:经过多重测试剩下的高质量的匹配（输出）
*        keypoints1:用于保存图像1检测到的关键点（输出）
*        keypoints2:用于保存图像2检测到的关键点（输出）
* 返  回:基础矩阵
* 说  明:对输出的两幅图像进行特征检测、计算描述子，进而使用BruteForceMatcher
* 进行匹配，对初始得到的匹配关系再依次进行比率测试、对称测试最后进行Ransac
* 验证，并得到两个相机的基础矩阵。
***********************************************************************/
cv::Mat match(cv::Mat &image1, cv::Mat &image2, std::vector<cv::DMatch>&matches,
	std::vector<cv::KeyPoint>&keypoints1, std::vector<cv::KeyPoint>&keypoints2)
{
	//1.b 计算SURF描述子
	cv::Mat descriptors1, descriptors2;
	extractor.compute(image1, keypoints1, descriptors1);
	extractor.compute(image2, keypoints2, descriptors2);
	//2 匹配两幅图像的描述子
	//2.a创建匹配器
	cv::BruteForceMatcher<cv::L2<float>>matcher;
	//cv::FlannBasedMatcher matcher;

	//2.b计算图1->图2，图2->图1 的k最近邻（k=2）
	std::vector<std::vector<cv::DMatch>>matcher1;
	std::vector<std::vector<cv::DMatch>>matcher2;
	//这里调用了knnMatch
	matcher.knnMatch(descriptors1, descriptors2, matcher1, 2);
	matcher.knnMatch(descriptors2, descriptors1, matcher2, 2);

	//3.比率测式
	int removed = ratioTest(matcher1);
	removed = ratioTest(matcher2);
	//4.对称性测试
	std::vector<cv::DMatch>symMatches;
	symmetryTest(matcher1, matcher2, symMatches);
	//5.RANSAC最终验证
	cv::Mat fundemental = ransacTest(symMatches, keypoints1, keypoints2, matches);
	return fundemental;
}
/**********************************************************************
* 函数名:Thin
* 参  数:src :图像1（输入）
*        dst:图像2（输出）
*       iterations（输出）
* 返  回:void
* 说  明:细化图像
***********************************************************************/
void Thin(const Mat&src, Mat&dst, const int iterations)
{
	assert(src.data != NULL);
	assert(dst.data != NULL);
	const int height = src.rows - 1;
	const int width = src.cols - 1;

	//拷贝一个数组给另一个数组
	if (src.data != dst.data)
	{
		src.copyTo(dst);
	}
	int n = 0, i = 0, j = 0;
	Mat tmpImg;
	uchar *pU, *pC, *pD;
	bool isFinished = false;

	for (n = 0; n<iterations; n++)
	{
		dst.copyTo(tmpImg);
		isFinished = false;   //一次 先行后列扫描 开始
		//扫描过程一 开始
		for (i = 1; i<height; i++)
		{
			pU = tmpImg.ptr<uchar>(i - 1);
			pC = tmpImg.ptr<uchar>(i);
			pD = tmpImg.ptr<uchar>(i + 1);
			for (int j = 1; j<width; j++)
			{
				if (pC[j] > 0)
				{
					int ap = 0;
					int p2 = (pU[j] >0);
					int p3 = (pU[j + 1] >0);
					if (p2 == 0 && p3 == 1)
					{
						ap++;
					}
					int p4 = (pC[j + 1] >0);
					if (p3 == 0 && p4 == 1)
					{
						ap++;
					}
					int p5 = (pD[j + 1] >0);
					if (p4 == 0 && p5 == 1)
					{
						ap++;
					}
					int p6 = (pD[j] >0);
					if (p5 == 0 && p6 == 1)
					{
						ap++;
					}
					int p7 = (pD[j - 1] >0);
					if (p6 == 0 && p7 == 1)
					{
						ap++;
					}
					int p8 = (pC[j - 1] >0);
					if (p7 == 0 && p8 == 1)
					{
						ap++;
					}
					int p9 = (pU[j - 1] >0);
					if (p8 == 0 && p9 == 1)
					{
						ap++;
					}
					if (p9 == 0 && p2 == 1)
					{
						ap++;
					}
					if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)>1 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)<7)
					{
						if (ap == 1)
						{
							if ((p2*p4*p6 == 0) && (p4*p6*p8 == 0))
							{
								dst.ptr<uchar>(i)[j] = 0;
								isFinished = true;
							}

							//   if((p2*p4*p8==0)&&(p2*p6*p8==0))
							//    {                           
							//         dst.ptr<uchar>(i)[j]=0;
							//         isFinished =TRUE;                            
							//    }

						}
					}
				}

			} //扫描过程一 结束

			dst.copyTo(tmpImg);
			//扫描过程二 开始
			for (i = 1; i<height; i++)  //一次 先行后列扫描 开始
			{
				pU = tmpImg.ptr<uchar>(i - 1);
				pC = tmpImg.ptr<uchar>(i);
				pD = tmpImg.ptr<uchar>(i + 1);
				for (int j = 1; j<width; j++)
				{
					if (pC[j] > 0)
					{
						int ap = 0;
						int p2 = (pU[j] >0);
						int p3 = (pU[j + 1] >0);
						if (p2 == 0 && p3 == 1)
						{
							ap++;
						}
						int p4 = (pC[j + 1] >0);
						if (p3 == 0 && p4 == 1)
						{
							ap++;
						}
						int p5 = (pD[j + 1] >0);
						if (p4 == 0 && p5 == 1)
						{
							ap++;
						}
						int p6 = (pD[j] >0);
						if (p5 == 0 && p6 == 1)
						{
							ap++;
						}
						int p7 = (pD[j - 1] >0);
						if (p6 == 0 && p7 == 1)
						{
							ap++;
						}
						int p8 = (pC[j - 1] >0);
						if (p7 == 0 && p8 == 1)
						{
							ap++;
						}
						int p9 = (pU[j - 1] >0);
						if (p8 == 0 && p9 == 1)
						{
							ap++;
						}
						if (p9 == 0 && p2 == 1)

						{
							ap++;
						}
						if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)>1 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)<7)
						{
							if (ap == 1)
							{
								//   if((p2*p4*p6==0)&&(p4*p6*p8==0))
								//   {                           
								//         dst.ptr<uchar>(i)[j]=0;
								//         isFinished =TRUE;                            
								//    }

								if ((p2*p4*p8 == 0) && (p2*p6*p8 == 0))
								{
									dst.ptr<uchar>(i)[j] = 0;
									isFinished = true;
								}

							}
						}
					}

				}

			} //一次 先行后列扫描完成          
			//如果在扫描过程中没有删除点，则提前退出
			if (isFinished == false)
			{
				break;
			}
		}

	}


}
/**********************************************************************
* 函数名:houghLine
* 参  数:image:图像（输入）
*        lines:保存检测到的直线（输出）
* 返  回:void
* 说  明:对图像进行处理之后进行Hough概率直线检测
***********************************************************************/
void houghLine(cv::Mat &image, std::vector<Vec4i>&lines)
{
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);//转为灰度图像
	//imshow("2.灰度视频", gray);

	Mat Gauss;
	GaussianBlur(gray, Gauss, Size(5, 5), 1.5, 1.5);//高斯滤波

	Mat Bin;
	threshold(Gauss, Bin, 100, 255, THRESH_BINARY);//beijginhie,mubiaobai,180
	//imshow("Binarization Image", Bin);

	Mat element0 = getStructuringElement(0, Size(5, 5), Point(-1, -1));//0代表矩形
	dilate(Bin, Bin, element0);
	//imshow("膨胀", Bin);

	Mat Edge;
	Canny(Bin, Edge, 40, 150, 3);//边缘检测,threshold1和threshold2 当中的小阈值用来控制边缘连接，大的阈值用来控制强边缘的初始分割。3代表内核算子大小	
	//imshow("膨胀之后边缘检测", Edge);

	Mat pFrMat;
	Mat element1 = getStructuringElement(0, Size(20, 1), Point(2, 0));//0代表矩形，
	morphologyEx(Edge, pFrMat, MORPH_CLOSE, element1);
	//imshow("CLOSE", pFrMat);

	Mat element2 = getStructuringElement(0, Size(5, 5), Point(-1, -1));//0代表矩形
	dilate(pFrMat, pFrMat, element2);
	//imshow("dilate", pFrMat);

	Mat element3 = getStructuringElement(0, Size(3, 3), Point(-1, -1));//0代表矩形
	erode(pFrMat, pFrMat, element3);
	//imshow("erode", pFrMat);

	Mat thin(pFrMat.size(), CV_8UC1, Scalar(0));
	Thin(pFrMat, thin, 20);
	//imshow("thin", thin);

	HoughLinesP(thin, lines, 1, CV_PI / 360, 10, 10, 50);//thin为输入图像，要求是8位单通道图像,
}
/**********************************************************************
* 函数名:removekeypoints
* 参  数:image1 :图像1（输入）
*        keypoints1:用于保存图像检测到的关键点（输入）
*         lines:直线（输出）
* 返  回:基础矩阵
* 说  明:去掉不在直线上的角点--以便做匹配时减小计算量
***********************************************************************/
void removekeypoints(cv::Mat &image, std::vector<cv::KeyPoint>&keypoints, std::vector<Vec4i>&lines)
{
	assert(keypoints.size() > 0);
	assert(lines.size() > 0);
	//循环判断角点是否在直线上
	for (vector<KeyPoint>::const_iterator itm = keypoints.begin(); itm != keypoints.end();)//左图中的每一个匹配点
	{
		float x1 = (*itm).pt.x;
		float y1 = itm->pt.y;

		double dist = 0;
		double mindist = INFINITY;
		for (vector<Vec4i>::iterator itn = lines.begin(); itn != lines.end(); ++itn)//图中的每一条直线
		{
			Point pt1((*itn)[0], (*itn)[1]);
			Point pt2((*itn)[2], (*itn)[3]);
			if ((pt2.x - pt1.x) == 0)//如果该直线是垂直的，没有斜率
			{
				dist = double(abs(x1 - pt1.x));
				//std::cout << "dist:" << dist << endl;

			}
			else
			{
				//求直线的斜率y=kx+b
				double k = double(pt2.y - pt1.y) / (pt2.x - pt1.x);    //k=(y2-y1)/(x2-x1)
				double b = double(pt1.y - k*pt1.x);   //b=y-kx
				dist = double(abs(k*x1 - y1 + b) / sqrt(k*k + (-1)*(-1)));
				//std::cout << "dist:" << dist << endl;					  
			}
			if (dist<mindist)
			{
				mindist = dist;
			}
		}
		if (mindist >10)//在直线上
		{
			//cout << "mindist:　" << mindist << endl;	
			itm = keypoints.erase(itm);
		}
		else
		{
			itm++;
		}
	}
	//std::cout << "new keypoints.size()" << keypoints.size() << endl;
}
int main()
{
	VideoCapture pCapture1("d:/img/a.avi");
	VideoCapture pCapture2("d:/img/b.avi");

	if (!pCapture1.isOpened())
	{
		cout << "could not open the video" << endl;
		return -1;
	}
	if (!pCapture2.isOpened())
	{
		cout << "could not open the video" << endl;
		return -1;
	}
	Sleep(800);
	int framenum = 0;
	while (true)
	{

		Mat image1;
		Mat image2;
		pCapture1 >> image1;
		pCapture2 >> image2;

		if (image1.empty())
		{
			cout << "over" << endl;
			break;
		}
		//imshow("1号", image1);

		if (image2.empty())
		{
			cout << "over" << endl;
			break;
		}
		//imshow("2号", image2);
		framenum++;
		cout << "framenum:" << framenum << endl;

		//2.检测
		std::vector<cv::DMatch> matches;
		std::vector<KeyPoint> keypoints1, keypoints2;
		//检测SURF特征点
		detector.detect(image1, keypoints1);
		//std::cout << "keypoints1.size()" << keypoints1.size() << endl;
		detector.detect(image2, keypoints2);
		//std::cout << "keypoints2.size()" << keypoints2.size() << endl;

		//直线检测
		std::vector<Vec4i> lines1, lines2;
		//std::cout << "图像1中直线个数" << endl;
		houghLine(image1, lines1);
		//std::cout << "图像2中直线个数" << endl;
		houghLine(image2, lines2);

		//remove不符合条件的角点
		removekeypoints(image1, keypoints1, lines1);
		removekeypoints(image2, keypoints2, lines2);

		match(image1, image2, matches, keypoints1, keypoints2);
		//std::cout << "matches.size() " << matches.size() << endl;

		Mat imgMatch1;
		drawMatches(image1, keypoints1, image2, keypoints2, matches, imgMatch1, Scalar::all(-1), Scalar::all(-1), vector<char>(), 2);
		imshow("匹配效果图", imgMatch1);
		waitKey(10);
	}
	return 0;
}
