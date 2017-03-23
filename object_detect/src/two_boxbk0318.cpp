/*--- Author:Yang Xianghong    Function: Corner detection and image matching---*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <object_detect/cam2position.h>
#include <sensor_msgs/image_encodings.h>
#include<iostream>
#include<opencv2/opencv.hpp>
//#include<fstream>
#include <stdio.h>
#include <std_msgs/Time.h>
#include"CornerMatch.h"
//#include"kmeans.h"
using namespace cv;
using namespace std;
Point pixell(162,193);
Point pixelr (152, 213);
double flx = 631.6730;
double fly = 629.2484;
double clx = 292.0061;
double cly = 229.1424;

double frx = 630.8853;
double fry = 630.1457;
double crx = 328.7070;
double cry = 229.2420;
cv::Point3f pcenter3send;
 std_msgs::Time time;


 // while(ros::Time::now() - time.data < ros::Duration(0.05))
/*
//the folowing code by zhang yu mei 20160315 , start



*/
ros::Publisher vspub;
geometry_msgs::Twist tposition;
bool msend = false;
bool getTWOPositionPose(object_detect::cam2position::Request &req , object_detect::cam2position::Response &res)
{
	if(req.flag2 && msend)
	{
		tposition.linear.x = pcenter3send.x;
		tposition.linear.y = pcenter3send.y;
		tposition.linear.z = pcenter3send.z;
		res.position2 = tposition;
		msend = false;
		return true;
	}
	return false;
} 


//the  above code by zhang yu mei 20160315 , end

/***************************************** new camera calibration data2**************************************************

double rotation[9] = { 0.999501461929381, - 0.00722997032746972, - 0.0307336156365171,
0.00761272385306140,	0.999894692311411,	0.0123551901242102,
0.0306410514925042, - 0.0125829971204105,	0.999451246508253 };

double translation[3] = { -32.4595383501705, - 0.694839133078388,	0.0167157354909775 };

double _D2[5] = { 0.0937452796855751, -0.0600991169664922, -0.000495549677604535, - 0.00427390007261944, 0 };
double _D1[5] = { 0.107371462983369, -0.119853681527916, -0.000179766750871017, - 0.00401057973647961, 0 };


double _M2[9] = { 500.472920273042, 0, 321.309929906660, 0, 500.870936762893, 282.428923591351, 0, 0, 1 };
double _M1[9] = { 492.921814710618, 0, 332.190730309682, 0, 494.197584008819, 236.449439749013, 0, 0, 1 };

/************************************************************************************************************/

double rotation[9] = { 0.999706256203138, - 0.0235807114995707,	0.00559922793601769,
0.0235686938341842,	0.999719793162599,	0.00220268696835813,
- 0.00564959991999057, - 0.00207007345379294,	0.999981898244483 };

double translation[3] = { -32.1696656435154,	0.0525646734170617,	0.335717389360114 };

double _D2[5] = { 0.0781865513660357, -0.0587577350036247, -0.000888238501669840, - 0.00373645548587948, 0 };

double _D1[5] = { 0.0683718721032674, -0.0323286356614776, -0.000339387098187202, - 0.00230205450913859, 0 };


double _M2[9] = { 489.429536469700, 0, 320.674943613416, 0, 489.409323813696, 278.009460040028, 0, 0, 1 };
double _M1[9] = { 486.409228580560, 0, 329.638253671028, 0, 486.853609795593, 272.198145574015, 0, 0, 1 };

Mat img1;
Mat img2;



Point3d Calc_worldPoint1(const Point &pl, const Point &pr);
bool Calc_worldPoint2(std::vector<cv::Point2f>&pl, std::vector<cv::Point2f>&pr,
	const Mat&Q, const Mat&P2, 
	std::vector<cv::Point3f>&pw);

bool Calc_worldPoint3(const Mat&Q, const Mat&P2);
int Filter3D(std::vector<cv::Point3f>&pw);
bool Center3D(std::vector<cv::Point3f>&pw, cv::Point3f&pcenter,int&Idix);

cv::Scalar colorTab[] =     //ÒòÎª×î¶àÖ»ÓÐ5Àà£¬ËùÒÔ×î¶àÒ²ŸÍžø5žöÑÕÉ«
{
cv::Scalar(0, 0, 255),
cv::Scalar(0, 255, 0),
cv::Scalar(255, 50, 100),
cv::Scalar(255, 0, 255),
cv::Scalar(0, 255, 255)
};
bool PrintMat(const Mat&m)
{
	if (m.data != NULL)
	{
		for (int i = 0; i < m.rows; ++i)
		{
			for (int j = 0; j < m.cols; ++j)
				cout << m.at<double>(i, j) << "\t";
			cout << endl;
		}
	}
	return true;
}

bool Center2D(std::vector<cv::Point2f>&pw, cv::Point2f&pcenter, int&Idix);
int cnt = 0;
bool test()
{

	if (img1.empty())
	{
		//std::cout << "over" << std::endl;
		return false;
	}

	if (img2.empty())
	{
		//std::cout << "over" << std::endl;
		return false;
	}
	//ROS_INFO("%d,%d",img1.cols,img2.rows);
	cv::Size imgsize (img1.cols, img1.rows);

	/*************************** stereo rectification is started *********************/
	Mat R = Mat(3, 3, CV_64F, rotation);
	Mat T = Mat(3, 1, CV_64F, translation);
	Mat D1 = Mat(5, 1, CV_64F, _D1);
	Mat D2 = Mat(5, 1, CV_64F, _D2);
	Mat M1 = Mat(3, 3, CV_64F, _M1);
	Mat M2 = Mat(3, 3, CV_64F, _M2);
	Mat R1, R2, P1, P2, Q;
	stereoRectify(M1, D1, M2, D2, imgsize, R, T, R1, R2, P1, P2, Q, 0, -1, imgsize, 0, 0);

	Mat map11, map12;
	Mat map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_32FC1, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_32FC1, map21, map22);
	Mat img11;
	Mat img12;
	remap(img1, img11, map11, map12, INTER_LINEAR);
	remap(img2, img12, map21, map22, INTER_LINEAR);

	Mat pairs = Mat(img11.rows, img11.cols * 2, CV_8UC3, Scalar(0, 0, 0));
	for (int i = 0; i < img11.cols; i++)
	{
		img11.col(i).copyTo(pairs.col(i));
		img12.col(i).copyTo(pairs.col(img11.cols + i));
	}
	/*************************** stereo rectification is ended *********************/


	/*************************** corner dection and match *********************/
	std::vector<cv::Point2f> point1;
	std::vector<cv::Point2f> point2;
	CornerMatch decte_match;
	decte_match.corner_match(img11, img12, point1, point2);
	/*************************************************************/


	std::vector<cv::Point3f> pw;
	Calc_worldPoint2(point1, point2, Q, P2, pw);	
	if (pw.size()<1) {return false;}
	//Calc_worldPoint3(Q, P2);
	int Idx=Filter3D(pw);
	if (pw.size()<1) {return false;}
	cv::Point2f pcenter;
	
	int idx,idx3;
	cv::Point3f pcenter3;
	msend = false;
	Center3D(pw, pcenter3, idx3);
	if (pw.size()<1) {return false;}
	Center2D(point1, pcenter, idx);

	if (pw.size()>0)
	{
		cv::circle(pairs, point1[idx3], 10, cv::Scalar(255, 0, 0), 2);

		std::cout << "[ " << pcenter3.x << ", " << pcenter3.y << ", " << pcenter3.z << " ]" <<ros::Time::now()-time.data<< endl;
		time.data=ros::Time::now();
	}
	

	
	if(pcenter3.z > 30 && pcenter3.z < 400)
	{msend = true; pcenter3send = pcenter3;
	tposition.linear.x = pcenter3send.x;
	tposition.linear.y = pcenter3send.y;
	tposition.linear.z = pcenter3send.z;
	vspub.publish(tposition);}
	for (int j = 0; j < img1.rows; j += 50)
	{
		line(img11, cvPoint(0, j), cvPoint(img1.cols, j), CV_RGB(255, 0, 0));
		line(pairs, cvPoint(0, j), cvPoint(pairs.cols, j), CV_RGB(255, 0, 0));
	}
	for (int j = 0; j <img1.rows; j += 50)
		{line(img12, cvPoint(0, j), cvPoint(img1.cols, j), CV_RGB(255, 0, 0));}
	imshow("pairs", pairs);

	waitKey(3);
	return true;

}
bool Center2D(std::vector<cv::Point2f>&pw, cv::Point2f&pcenter, int&Idix)
{
	if (pw.size()<1) {return false;}
	std::vector<cv::Point2f>::iterator it = pw.begin();
	double*Dist = new double[pw.size()];
	int Idx = 0, i = 0;
	double sumx = 0, sumy = 0;
	while (it != pw.end())
	{
		sumx += it->x;
		sumy += it->y;
		it++;
	}
	sumx = sumx / pw.size();
	sumy = sumy / pw.size();
	it = pw.begin();
	double minerror = 1000000000;
	while (it != pw.end())
	{
		double error = sqrt((it->x - sumx)*(it->x - sumx) + (it->y - sumy)*(it->y - sumy)); 
		if (error < minerror)
		{
			minerror = error;
			Idix = i;
			pcenter.x = it->x;
			pcenter.y = it->y;
		}
		i++;
		it++;
	}
	return true;
}
bool Center3D(std::vector<cv::Point3f>&pw, cv::Point3f&pcenter, int&Idix)
{
	if (pw.size()<1) {return false;}
	std::vector<cv::Point3f>::iterator it = pw.begin();
	double*Dist = new double[pw.size()];
	int Idx = 0, i = 0;
	double sumx = 0, sumy = 0, sumz = 0;
	while (it != pw.end())
	{
		sumx += it->x;
		sumy += it->y;
		sumz += it->z;
		it++;
	}
	sumx = sumx / pw.size();
	sumy = sumy / pw.size();
	sumz = sumz / pw.size();
	it = pw.begin();
	double minerror = 1000000000;
	while (it != pw.end())
	{
		double error =sqrt((it->x - sumx)*(it->x - sumx) + (it->y - sumy)*(it->y - sumy) + (it->z - sumz)*(it->z - sumz));
		if (error < minerror)
		{
			minerror = error;
			Idix = i;

		}
		i++;
		it++;
	}
	pcenter.x = sumx;
	pcenter.y = sumy;
	pcenter.z = sumz;
	return true;

}
int Filter3D(std::vector<cv::Point3f>&pw)
{
	if (pw.size()<1) {return -1;}
	std::vector<cv::Point3f>::iterator it = pw.begin();
	double*Dist = new double[pw.size()];
	int Idx = 0,i=0;
	double mindist = 100000000;
	while (it != pw.end())
	{
		double dist = sqrt(it->x*it->x + it->y*it->y + it->z*it->z);
		if (dist < mindist)
		{
			mindist = dist;
			Idx = i;
		}
		Dist[i++] = dist;
		it++;
	}
	delete[]Dist;
	return Idx;
}
bool Calc_worldPoint2(std::vector<cv::Point2f>&pl, std::vector<cv::Point2f>&pr, const Mat&Q, const Mat&P2,std::vector<cv::Point3f>&pw)
{
	if (pl.size() <= 0 || pr.size() <= 0)
	{
		///std::cout << "pl.size() <= 0 || pr.size() <= 0" << std::endl;
		return false;
	}
	double x1 = Q.at<double>(0, 3);
	double x2 = Q.at<double>(1, 3);
	double x3 = Q.at<double>(2, 3);
	double x4 = Q.at<double>(3, 2);
	double x5 = Q.at<double>(3, 3);

	double Tx = -1 / x4;
	double f = x3;
	double cx = -x1;
	double cy = -x2;
	double cx2 = P2.at<double>(0, 2);

	std::vector<cv::Point2f>::iterator itpL = pl.begin();
	std::vector<cv::Point2f>::iterator itpR = pr.begin();
	double ul, vl, ur, vr;

	while(itpL!=pl.end()&&itpR!=pr.end())
	{
		ul = itpL->x;
		vl=itpL->y;
		ur=itpR->x;
		vr=itpR->y;
		double d1 = ((ul - cx) - (ur - cx2));
		double Z = f*Tx / d1;
		double X = Z*(ul - cx) / f;
		double Y = Z*(vl - cy) / f;

		double d = (ul)-(ur);
		double W = -x4*d - x5;
		cv::Point3f Pw;

		Pw.x =X;
		Pw.y = Y;
		Pw.z = Z;

		pw.push_back(Pw);
		itpL++;
		itpR++;
	}
	return true;

}

Point3d Calc_worldPoint1(const Point &pl, const Point &pr)
{
	
	Point3d Pw (0,0,0);
	double a = (pl.x - clx)*rotation[0] / flx + (pl.y - cly)*rotation[1] / fly + rotation[2];
	double b = (pl.x - clx)*rotation[6] / flx + (pl.y - cly)*rotation[7] / fly + rotation[8];
	double m = frx*translation[0] - (pr.x - crx)*translation[2];
	double n = b*(pr.x - crx) - frx*a;
	Pw.z= m / n;
	Pw.x = Pw.z*(pl.x - clx) / flx;
	Pw.y = Pw.z*(pl.y - cly) / fly;
	return Pw;
}




Mat map1;
Mat map2;
												

  bool imageCb1(const sensor_msgs::ImageConstPtr& msg)								  //callback function defination
  {//ROS_WARN("bb");

	if (!msg)
	{
		ROS_WARN("MSG NULL");
		return false;
	}

	cv_bridge::CvImagePtr cv_ptr;													   
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}



	cv_ptr->image.copyTo(img1);	
	if( !img1.empty() && !img2.empty())
		test();
	return true;

  }

  bool imageCb2(const sensor_msgs::ImageConstPtr& msg)								  //callback function defination
    {//ROS_WARN("aa");
	if (!msg)
	{
		ROS_WARN("MSG NULL");
		return false;
	}

	cv_bridge::CvImagePtr cv_ptr;													   
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}



	cv_ptr->image.copyTo(img2);	
	if( !img1.empty() && !img2.empty())
		test();
	return true;
    }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_detect");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	image_transport::Subscriber image_sub1, image_sub2;
	time.data=ros::Time::now();
	//pMOG = new BackgroundSubtractorMOG();
	ros::Rate loop_rate(10);
	
	image_sub1 = it_.subscribe("/usb_cam1/image_raw", 1, imageCb1);

	image_sub2 = it_.subscribe("/usb_cam2/image_raw", 1, imageCb2);
	//ros::MultiThreadedSpinner spinner(3);
	ros::ServiceServer service = nh_.advertiseService("/srvtwocamPosition",getTWOPositionPose);
	 vspub = nh_.advertise<geometry_msgs::Twist> ("/cam2postion_pub", 100,true);
	//ros::MultiThreadedSpinner spinner(3);
	//spinner.spin();	
	//ros::spinOnce();

	int count = 0;
	while (ros::ok())
	{ 
		
		ros::spinOnce();
		loop_rate.sleep();

	++count;
	}

  return 0;
}


