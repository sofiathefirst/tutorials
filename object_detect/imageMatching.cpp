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

/*********************��������*************************/
//�����
GoodFeaturesToTrackDetector detector(1000, 0.01, 10.0);//���ص������������Ŀ�������ȼ�������֮�����С�������
//��ȡ��
SiftDescriptorExtractor extractor;
//��һ���Լ��ڶ��������֮���������
float ratio = 0.65f;
//�Ƿ��ٴ��Ż�F����
bool refineF = true;
//�����ߵ���С����
double distance = 3.0;
//���ŵȼ������ʣ�
double confidence = 0.99;
/**********************************************************************
* ������:ratioTest
* ��  ��:matches
* ��  ��:�Ƴ���ƥ������
* ˵  ��:�Ե�ǰƥ�����ɸѡ������ƥ��ʹ���ƥ����Ӧǿ�ȴ���ratio��ƥ���Լ�
*  ������ƥ�䡣
***********************************************************************/
int ratioTest(std::vector<std::vector<cv::DMatch>>& matches)
{
	int removed = 0;
	std::vector<std::vector<cv::DMatch>>::iterator matchIt = matches.begin();
	for (; matchIt != matches.end(); matchIt++)
	{
		if (matchIt->size()  > 1)
		{
			//�Ƴ����ϸ���ʵ�ƥ�䣨������������
			if ((*matchIt)[0].distance / (*matchIt)[1].distance > ratio)
			{
				matchIt->clear();
				removed++;
			}
		}
		else
		{
			//�Ƴ�������ƥ��
			matchIt->clear();
			removed++;
		}
	}
	return removed;
}
/**********************************************************************
* ������:symmetryTest
* ��  ��:matches1:��ƥ��
*       matches2:��ƥ��
*       symMatche:����ĶԳ�ƥ��
* ��  ��:��
* ˵  ��:������ƥ����м�飬����ԳƵ�ƥ�䡣
***********************************************************************/
void symmetryTest(std::vector<std::vector<cv::DMatch>>& matches1,
	std::vector<std::vector<cv::DMatch>>& matches2, std::vector<cv::DMatch>& symMatches)
{
	//������ƥ��
	for (auto &leftMatchRef : matches1)
	{
		if (leftMatchRef.size() < 2)
			continue;
		//������ƥ��
		for (auto &rightMatchRef : matches2)
		{
			if (rightMatchRef.size() < 2)
				continue;
			//�Գ��Բ���
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
* ������:ransacTest
* ��  ��:matches:��ǰƥ�䣨���룩
*       keypoints1:ͼ��1��⵽�Ĺؼ��㣨���룩
*       keypoints2:ͼ��2��⵽�Ĺؼ��㣨���룩
*       outMatches:��ɲ��Ե�ƥ�䣨�����
* ��  ��:��������
* ˵  ��:�Ե�ǰƥ�����RANSAC���ԣ������������ͬʱ����ͨ�����Ե�ƥ��
***********************************************************************/
cv::Mat ransacTest(
	const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& outMatches)
{
	//��Keypointsת����Point2f
	std::vector<cv::Point2f>points1, points2;
	for (auto &matchesRef : matches)
	{
		//��ͼ��ؼ���
		float x = keypoints1[matchesRef.queryIdx].pt.x;
		float y = keypoints1[matchesRef.queryIdx].pt.y;
		points1.push_back(cv::Point2f(x, y));
		//��ͼ��ؼ���
		x = keypoints2[matchesRef.trainIdx].pt.x;
		y = keypoints2[matchesRef.trainIdx].pt.y;
		points2.push_back(cv::Point2f(x, y));
	}
	//����RANSAC����F����
	//cout << "p1: " << points1.size() << endl;
	//cout << "p2: " << points2.size() << endl;
	std::vector<uchar>inlines(points1.size(), 0);
	cv::Mat fundemental = cv::findFundamentalMat(
		cv::Mat(points1),
		cv::Mat(points2),//ƥ���
		inlines,        //ƥ��״̬��inlier����outlier
		CV_FM_RANSAC,  //RANSAC����
		3.0,       //�����ߵľ���
		confidence  //���Ÿ���
		);
	//��ȡͨ����ƥ��
	std::vector<uchar>::const_iterator itIn = inlines.begin();
	std::vector<cv::DMatch>::const_iterator itM = matches.begin();
	//��������ƥ��
	for (; itIn != inlines.end(); itIn++, itM++)
	{
		if (*itIn)
		{
			outMatches.push_back(*itM);
		}
	}
	//�������
	if (refineF)
	{
		//F����ʹ�����н��ܵ�ƥ�����¼���
		//װ��KeyPoint���͵�Point2f
		//׼���������յ�F����
		points1.clear();
		points2.clear();
		for (auto &matchRef : outMatches)
		{
			//�õ��������������
			float x = keypoints1[matchRef.queryIdx].pt.x;
			float y = keypoints1[matchRef.queryIdx].pt.y;
			points1.push_back(cv::Point2f(x, y));
			//�õ��ұ������������
			x = keypoints2[matchRef.trainIdx].pt.x;
			y = keypoints2[matchRef.trainIdx].pt.y;
			points2.push_back(cv::Point2f(x, y));
		}
		//�����н��ܵ�ƥ���м���8��F
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
* ������:match
* ��  ��:image1 :ͼ��1�����룩
*        image2:ͼ��2�����룩
*        matches:�������ز���ʣ�µĸ�������ƥ�䣨�����
*        keypoints1:���ڱ���ͼ��1��⵽�Ĺؼ��㣨�����
*        keypoints2:���ڱ���ͼ��2��⵽�Ĺؼ��㣨�����
* ��  ��:��������
* ˵  ��:�����������ͼ�����������⡢���������ӣ�����ʹ��BruteForceMatcher
* ����ƥ�䣬�Գ�ʼ�õ���ƥ���ϵ�����ν��б��ʲ��ԡ��ԳƲ���������Ransac
* ��֤�����õ���������Ļ�������
***********************************************************************/
cv::Mat match(cv::Mat &image1, cv::Mat &image2, std::vector<cv::DMatch>&matches,
	std::vector<cv::KeyPoint>&keypoints1, std::vector<cv::KeyPoint>&keypoints2)
{
	//1.b ����SURF������
	cv::Mat descriptors1, descriptors2;
	extractor.compute(image1, keypoints1, descriptors1);
	extractor.compute(image2, keypoints2, descriptors2);
	//2 ƥ������ͼ���������
	//2.a����ƥ����
	cv::BruteForceMatcher<cv::L2<float>>matcher;
	//cv::FlannBasedMatcher matcher;

	//2.b����ͼ1->ͼ2��ͼ2->ͼ1 ��k����ڣ�k=2��
	std::vector<std::vector<cv::DMatch>>matcher1;
	std::vector<std::vector<cv::DMatch>>matcher2;
	//���������knnMatch
	matcher.knnMatch(descriptors1, descriptors2, matcher1, 2);
	matcher.knnMatch(descriptors2, descriptors1, matcher2, 2);

	//3.���ʲ�ʽ
	int removed = ratioTest(matcher1);
	removed = ratioTest(matcher2);
	//4.�Գ��Բ���
	std::vector<cv::DMatch>symMatches;
	symmetryTest(matcher1, matcher2, symMatches);
	//5.RANSAC������֤
	cv::Mat fundemental = ransacTest(symMatches, keypoints1, keypoints2, matches);
	return fundemental;
}
/**********************************************************************
* ������:Thin
* ��  ��:src :ͼ��1�����룩
*        dst:ͼ��2�������
*       iterations�������
* ��  ��:void
* ˵  ��:ϸ��ͼ��
***********************************************************************/
void Thin(const Mat&src, Mat&dst, const int iterations)
{
	assert(src.data != NULL);
	assert(dst.data != NULL);
	const int height = src.rows - 1;
	const int width = src.cols - 1;

	//����һ���������һ������
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
		isFinished = false;   //һ�� ���к���ɨ�� ��ʼ
		//ɨ�����һ ��ʼ
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

			} //ɨ�����һ ����

			dst.copyTo(tmpImg);
			//ɨ����̶� ��ʼ
			for (i = 1; i<height; i++)  //һ�� ���к���ɨ�� ��ʼ
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

			} //һ�� ���к���ɨ�����          
			//�����ɨ�������û��ɾ���㣬����ǰ�˳�
			if (isFinished == false)
			{
				break;
			}
		}

	}


}
/**********************************************************************
* ������:houghLine
* ��  ��:image:ͼ�����룩
*        lines:�����⵽��ֱ�ߣ������
* ��  ��:void
* ˵  ��:��ͼ����д���֮�����Hough����ֱ�߼��
***********************************************************************/
void houghLine(cv::Mat &image, std::vector<Vec4i>&lines)
{
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);//תΪ�Ҷ�ͼ��
	//imshow("2.�Ҷ���Ƶ", gray);

	Mat Gauss;
	GaussianBlur(gray, Gauss, Size(5, 5), 1.5, 1.5);//��˹�˲�

	Mat Bin;
	threshold(Gauss, Bin, 100, 255, THRESH_BINARY);//beijginhie,mubiaobai,180
	//imshow("Binarization Image", Bin);

	Mat element0 = getStructuringElement(0, Size(5, 5), Point(-1, -1));//0�������
	dilate(Bin, Bin, element0);
	//imshow("����", Bin);

	Mat Edge;
	Canny(Bin, Edge, 40, 150, 3);//��Ե���,threshold1��threshold2 ���е�С��ֵ�������Ʊ�Ե���ӣ������ֵ��������ǿ��Ե�ĳ�ʼ�ָ3�����ں����Ӵ�С	
	//imshow("����֮���Ե���", Edge);

	Mat pFrMat;
	Mat element1 = getStructuringElement(0, Size(20, 1), Point(2, 0));//0������Σ�
	morphologyEx(Edge, pFrMat, MORPH_CLOSE, element1);
	//imshow("CLOSE", pFrMat);

	Mat element2 = getStructuringElement(0, Size(5, 5), Point(-1, -1));//0�������
	dilate(pFrMat, pFrMat, element2);
	//imshow("dilate", pFrMat);

	Mat element3 = getStructuringElement(0, Size(3, 3), Point(-1, -1));//0�������
	erode(pFrMat, pFrMat, element3);
	//imshow("erode", pFrMat);

	Mat thin(pFrMat.size(), CV_8UC1, Scalar(0));
	Thin(pFrMat, thin, 20);
	//imshow("thin", thin);

	HoughLinesP(thin, lines, 1, CV_PI / 360, 10, 10, 50);//thinΪ����ͼ��Ҫ����8λ��ͨ��ͼ��,
}
/**********************************************************************
* ������:removekeypoints
* ��  ��:image1 :ͼ��1�����룩
*        keypoints1:���ڱ���ͼ���⵽�Ĺؼ��㣨���룩
*         lines:ֱ�ߣ������
* ��  ��:��������
* ˵  ��:ȥ������ֱ���ϵĽǵ�--�Ա���ƥ��ʱ��С������
***********************************************************************/
void removekeypoints(cv::Mat &image, std::vector<cv::KeyPoint>&keypoints, std::vector<Vec4i>&lines)
{
	assert(keypoints.size() > 0);
	assert(lines.size() > 0);
	//ѭ���жϽǵ��Ƿ���ֱ����
	for (vector<KeyPoint>::const_iterator itm = keypoints.begin(); itm != keypoints.end();)//��ͼ�е�ÿһ��ƥ���
	{
		float x1 = (*itm).pt.x;
		float y1 = itm->pt.y;

		double dist = 0;
		double mindist = INFINITY;
		for (vector<Vec4i>::iterator itn = lines.begin(); itn != lines.end(); ++itn)//ͼ�е�ÿһ��ֱ��
		{
			Point pt1((*itn)[0], (*itn)[1]);
			Point pt2((*itn)[2], (*itn)[3]);
			if ((pt2.x - pt1.x) == 0)//�����ֱ���Ǵ�ֱ�ģ�û��б��
			{
				dist = double(abs(x1 - pt1.x));
				//std::cout << "dist:" << dist << endl;

			}
			else
			{
				//��ֱ�ߵ�б��y=kx+b
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
		if (mindist >10)//��ֱ����
		{
			//cout << "mindist:��" << mindist << endl;	
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
		//imshow("1��", image1);

		if (image2.empty())
		{
			cout << "over" << endl;
			break;
		}
		//imshow("2��", image2);
		framenum++;
		cout << "framenum:" << framenum << endl;

		//2.���
		std::vector<cv::DMatch> matches;
		std::vector<KeyPoint> keypoints1, keypoints2;
		//���SURF������
		detector.detect(image1, keypoints1);
		//std::cout << "keypoints1.size()" << keypoints1.size() << endl;
		detector.detect(image2, keypoints2);
		//std::cout << "keypoints2.size()" << keypoints2.size() << endl;

		//ֱ�߼��
		std::vector<Vec4i> lines1, lines2;
		//std::cout << "ͼ��1��ֱ�߸���" << endl;
		houghLine(image1, lines1);
		//std::cout << "ͼ��2��ֱ�߸���" << endl;
		houghLine(image2, lines2);

		//remove�����������Ľǵ�
		removekeypoints(image1, keypoints1, lines1);
		removekeypoints(image2, keypoints2, lines2);

		match(image1, image2, matches, keypoints1, keypoints2);
		//std::cout << "matches.size() " << matches.size() << endl;

		Mat imgMatch1;
		drawMatches(image1, keypoints1, image2, keypoints2, matches, imgMatch1, Scalar::all(-1), Scalar::all(-1), vector<char>(), 2);
		imshow("ƥ��Ч��ͼ", imgMatch1);
		waitKey(10);
	}
	return 0;
}
