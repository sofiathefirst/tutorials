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
int num = 0;
Mat pFrame,pFrame0;
VideoWriter video_write("VideoTest_03.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480));
  void Drandirection(Mat&pFrame,vector<RotatedRect>& mr0)
  {
	cout << "mr0.size(): " << mr0.size() << endl;
	double radio1 = 0, radio2 = 0;
	if (mr0.size()< 2)
	{
		if (mr0.size()>2)
			cout << "The circle num is > 2" << endl;
		else
			cout << "The circle num is < 2" << endl;
	}
	else
	{
		radio1 = double(mr0[0].size.width) / double(mr0[0].size.height);
		radio2 = double(mr0[1].size.width) / double(mr0[1].size.height);
		if (radio1>1)radio1 = 1 / radio1;
		if (radio2 > 1)radio2 = 1 / radio2;
		if (radio2 > radio1)
		{
			line(pFrame, Point(mr0[1].center.x, mr0[1].center.y), Point(mr0[0].center.x, mr0[0].center.y), Scalar(255, 0, 0), 1);   //
			circle(pFrame, Point(mr0[1].center.x, mr0[1].center.y), 5, Scalar(100, 100, 255), 2);
		}
		else
		{
			line(pFrame, Point(mr0[0].center.x,mr0[0].center.y), Point(mr0[1].center.x,mr0[1].center.y), Scalar(255, 0, 0),1);
			circle(pFrame, Point(mr0[0].center.x,mr0[1].center.y), 5, Scalar(100, 100, 255), 2);
		}
	}
  }
void Thin(const Mat&src, Mat&dst, const int iterations)
{
	const int height = src.rows - 1;
	const int width = src.cols - 1;

	//¿œ±ŽÒ»žöÊý×éžøÁíÒ»žöÊý×é
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
		isFinished = false;   //Ò»ŽÎ ÏÈÐÐºóÁÐÉšÃè ¿ªÊŒ
		//ÉšÃè¹ý³ÌÒ» ¿ªÊŒ
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

			} //ÉšÃè¹ý³ÌÒ» œáÊø


			dst.copyTo(tmpImg);
			//ÉšÃè¹ý³Ì¶þ ¿ªÊŒ
			for (i = 1; i<height; i++)  //Ò»ŽÎ ÏÈÐÐºóÁÐÉšÃè ¿ªÊŒ
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

			} //Ò»ŽÎ ÏÈÐÐºóÁÐÉšÃèÍê³É          
			//Èç¹ûÔÚÉšÃè¹ý³ÌÖÐÃ»ÓÐÉŸ³ýµã£¬ÔòÌáÇ°ÍË³ö
			if (isFinished == false)
			{
				break;
			}
		}
	}
}

  void Filter(Mat&img, const double n, const double m)
  {
	assert(img.data != NULL);
	for (int i = 0; i < img.rows; ++i)
	{
		uchar*data = img.ptr<uchar>(i);
		for (int j = 0; j < img.cols; ++j)
		{
			if (double(data[j]) > n && double(data[j])<m)
				data[j] = 255;
			else
				data[j] = 0;
		}
	}
  }
  void And(Mat&img1, Mat&img2, Mat&out)
  {
	for (int i = 0; i < img1.rows; ++i)
	{
		uchar*data1 = img1.ptr<uchar>(i);
		uchar*data2 = img2.ptr<uchar>(i);
		uchar*data3 = out.ptr<uchar>(i);
		for (int j = 0; j < img1.cols; ++j)
		{
			if (data1[j]>0 && data2[j] > 0)
				data3[j] = 255;
			else
				data3[j] = 0;
		}
	}
  }
  void Filterbylengthandarea(vector<vector<Point> >&contoures,int minl, int maxl, int mina, int maxa)
  {
	vector<vector<Point> >::iterator it = contoures.begin();
	while (it != contoures.end())
	{
		/*RotatedRect mr;
		mr = minAreaRect(Mat(*it));*/		double len = arcLength(*it, 1);
		if (len>minl && len< maxl)
		{
			double area = cv::contourArea(*it);
			if (area > mina && area < maxa)
			{
				it++;
			}
			else
				it= contoures.erase(it);

		}
		else
			it= contoures.erase(it);
	}
  }
  void select(vector<vector<Point> >&lcontoures, vector<vector<Point> >&abcontoures, vector<RotatedRect>&mr0)
  {
	vector<vector<Point> >::iterator itl = lcontoures.begin();
	
	int mini = 0;
	//vector<RotatedRect> mr0;
	while (itl != lcontoures.end())
	{
		double area = cv::contourArea(*itl);
		RotatedRect mr;
		mr = minAreaRect(Mat(*itl));
		vector<vector<Point> >::iterator  itab = abcontoures.begin();
		double mindis = 0.0;
		int i = 0;
		while (itab != abcontoures.end())
		{
			double dist = pointPolygonTest(*itab, mr.center, 1);
			if (dist > mindis)
			{
				mindis = dist;
				mini = i;
			}
			i++;
			itab++;
		}

		if (mindis < 50)
		{
			//ellipse(pFrame, mr, Scalar(255, 0, 0), 2);
			//cv::drawContours(pFrame, abcontoures, mini, Scalar(0, 100, 255), 2);
			//std::cout << mindis << endl;
			mr0.push_back(mr);
		}
		//ellipse(pFrame, mr, Scalar(255, 0, 0), 2);
		itl++;
	}
  }
  
  void DataProcessing(Mat &pFrame,Mat &pFrame0,Mat &pFrame1)
  {
    Mat pFrameecopy;
    pFrame.copyTo(pFrameecopy);

		Mat pLab0;
		Mat pLab;
		Mat pLab1;
		cvtColor(pFrame0, pLab0, CV_BGR2Lab);
		cvtColor(pFrame, pLab, CV_BGR2Lab);
		cvtColor(pFrame1, pLab1, CV_BGR2Lab);


		cv::GaussianBlur(pLab0, pLab0, Size(5, 5), 0, 0, BORDER_DEFAULT);
		cv::GaussianBlur(pLab, pLab, Size(5, 5), 0, 0, BORDER_DEFAULT);
		cv::GaussianBlur(pLab1, pLab1, Size(5, 5), 0, 0, BORDER_DEFAULT);

		Mat DiffMat10, DiffMat11, DiffMat12;
		Mat DiffMat20, DiffMat21, DiffMat22;
		vector<Mat> lab0;
		vector<Mat> lab;
		vector<Mat> lab1;
		split(pLab0, lab0);
		split(pLab, lab);
		split(pLab1, lab1);

		absdiff(lab[0], lab0[0], DiffMat10);
		absdiff(lab[1], lab0[1], DiffMat11);
		absdiff(lab[2], lab0[2], DiffMat12);

		absdiff(lab1[0], lab[0], DiffMat20);
		absdiff(lab1[1], lab[1], DiffMat21);
		absdiff(lab1[2], lab[2], DiffMat22);


		morphologyEx(DiffMat10, DiffMat10, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));
		morphologyEx(DiffMat11, DiffMat11, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));
		morphologyEx(DiffMat12, DiffMat12, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));

		morphologyEx(DiffMat20, DiffMat20, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));
		morphologyEx(DiffMat21, DiffMat21, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));
		morphologyEx(DiffMat22, DiffMat22, MORPH_CLOSE, Mat(5, 5, CV_8U, Scalar(1)));


		threshold(DiffMat10, DiffMat10, 20, 255, CV_THRESH_BINARY);
		threshold(DiffMat11, DiffMat11, 20, 255, CV_THRESH_BINARY);
		threshold(DiffMat12, DiffMat12, 20, 255, CV_THRESH_BINARY);
		dilate(DiffMat10, DiffMat10, Mat(7, 7, CV_8U, Scalar(1)));

		threshold(DiffMat20, DiffMat20, 20, 255, CV_THRESH_BINARY);
		threshold(DiffMat21, DiffMat21, 20, 255, CV_THRESH_BINARY);
		threshold(DiffMat22, DiffMat22, 20, 255, CV_THRESH_BINARY);
		dilate(DiffMat20, DiffMat20, Mat(7, 7, CV_8U, Scalar(1)));


		Mat andl(DiffMat10.size(), CV_8UC1, Scalar(0));
		And(DiffMat10, DiffMat20, andl);

		////	erode(and, and, Mat(5, 5, CV_8U, Scalar(1)));
		//morphologyEx(and, and, MORPH_CLOSE, Mat(7, 7, CV_8U, Scalar(1)));


		vector<vector<Point> > contours;
		findContours(andl, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		cout << contours.size() << endl;
		drawContours(pFrameecopy, contours, -1, Scalar(0, 0, 255), 1);

//		pFrame.copyTo(pFrame0);
//		pFrame1.copyTo(pFrame);


		cv::namedWindow("DiffMat10");
		cv::imshow("DiffMat10", DiffMat10);
		cv::waitKey(3);
		cv::namedWindow("DiffMat11");
		cv::imshow("DiffMat11", DiffMat11);
		cv::waitKey(3);
		cv::namedWindow("DiffMat12");
		cv::imshow("DiffMat12", DiffMat11);
		cv::waitKey(3);

		cv::namedWindow("DiffMat20");
		cv::imshow("DiffMat20", DiffMat20);
		cv::waitKey(3);
		cv::namedWindow("DiffMat21");
		cv::imshow("DiffMat21", DiffMat21);
		cv::waitKey(3);
		cv::namedWindow("DiffMat22");
		cv::imshow("DiffMat22", DiffMat22);
		cv::waitKey(3);

		cv::namedWindow("and");
		cv::imshow("and", andl);
		cv::waitKey(3);

		cv::namedWindow("pFrame pLab");
		cv::imshow("pFrame pLab", pLab1);
		cv::waitKey(3);

		cv::namedWindow("pFrameecopy");
		cv::imshow("pFrameecopy", pFrameecopy);
		cv::waitKey(3);
	}
  

  void imageCb(const sensor_msgs::ImageConstPtr& msg) 	  //callback function defination
  { 
    num++;
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
    //Mat pFrame = cv_ptr->image;
    if(num <=3)
      return;
    if(num == 4)
      pFrame0 = cv_ptr->image;
    else if(num == 5)
      pFrame = cv_ptr->image;
    else
    {
      Mat pFrame1 = cv_ptr->image;
      if (pFrame1.empty())
      {
        ROS_INFO("THE FRAME IS EMPTY");
      }
      else
      {
        ROS_INFO("START TO PROCESSING THE DATA");
        DataProcessing(pFrame,pFrame0,pFrame1);
        pFrame.copyTo(pFrame0);
        pFrame1.copyTo(pFrame);
      }
    }
}

    

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detect");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  

  ros::Rate loop_rate(100);
  image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &imageCb); 			                                    

  ros::spin();

  
  return 0;
}



