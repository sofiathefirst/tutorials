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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"          //bzj
#include "object_detect/PointVector.h"	  //bzj
#include <vector>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include "contentFinder.h"
#include "colorhistogram.h"  
  using namespace pcl;
  using namespace pcl::io;
  using namespace cv;
  using namespace std;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool receive = false;
ros::Subscriber object_pcl;

ros::Publisher object_center_pub;   //zxx
ros::Publisher object_center_2d_pub;   //zxx
vector<Point2f> center2ds;
//zxx
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
//pcl::PointCloud<pcl::PointXYZ> cloud; 
  sensor_msgs::Image image_;
  pcl::fromROSMsg (*input, cloud);
  receive = true;

  float dep = cloud.points[164*cloud.width+370].z;
  float x = cloud.points[164*cloud.width+370].x;
  float y = cloud.points[164*cloud.width+370].y;
      ROS_INFO("DEPTH : %f,%f,%f",x,y,dep);
//  cloud.points.at<float>(256,64);369.90
}
  string intToString(int number)
  {
	std::stringstream ss;
	ss << number;
	return ss.str();
  }


void minEnclosingRect(Mat &src,Mat &res,int N);
Mat color;
	//void computeAndJustice(Mat &src,RotatedRect &rRect,Point2f vertices[]);
/*ÔÚÖ÷º¯ÊýÖÐ£¬ÎÒÃÇ¶ÔŒžÖÖ·šÏàÍ¶Ó°Ö±·œÍŒµÄ·œ·šœøÐÐÁË¶Ô±È£ºÖ»ÓÃ»Ò¶ÈÍŒÏñµÄÖ±·œÍŒ£»ÓÃ²ÊÉ«ÍŒÏñÖ±·œÍŒ£»ÒÔŒ°HSV¿ÕŒäÖÐÉ«ÍŒÐÅÏ¢µÄÖ±·œÍŒ¡£*/ 
    int test()  
	{  
		

		center2ds.clear();		
		char filename[20];
		
		Mat colortempl;
		ColorHistogram hc;  
		ContentFinder finder;

		vector<Mat> shist;

		for(int i=1;i<=3;i++)
		{
			sprintf(filename,"/home/lzc/moveit/src/object_detect/src/tp2/%d.bmp",i);
			colortempl=imread(filename,1);
			if(colortempl.empty())
			{
				cout<<"Ä£°åÎÞÍŒÏñ"<<endl;
				return -1;
			}		
			//imshow("ÔŽ²ÊÉ«Ä£°åÍŒÏñ",colortempl);  					
			colortempl = hc.colorReduce(colortempl,32);//ÎªÁËŒõÉÙŒÆËãÁ¿£¬Ê¹ÓÃcolorReduceº¯Êý 
			Mat hist=hc.getHistogram(colortempl);//»ñÈ¡Ä£°åÖ±·œÍŒ,histÊÇÈýÎ¬µÄ
			shist.push_back(finder.setHistogram(hist));  //œ«Ä£°åÖ±·œÍŒhist¹éÒ»»¯£¬²¢ŽæÈëshist
			
		}
		//while (true)
		{
			//nFrmNum++;			
			//pCapture >> color;
			if (color.empty())
			{
				cout << "over" << endl;
				return -1;
			}
			cout<<"\t"<<endl;
			//cout<<"µÚ"<<nFrmNum<<"Ö¡ÍŒÏñ£º"<<endl;
			imshow("ÔŽ²ÊÉ«ÍŒÏñ", color);
			Mat colorreduce = hc.colorReduce(color,32);//ÎªÁËŒõÉÙŒÆËãÁ¿£¬Ê¹ÓÃcolorReduceº¯Êý 
			
			int thr=700;
			Mat mat0;
			Mat mat1;
			Mat mat2;
			//Mat mat3;
			//Mat mat4;
			//Mat mat5;
			vector<int> countIDvector;
			for(int i=0;i<3;i++)
			{
							
				//»ñÈ¡¶þÖµ»¯µÄ·ŽÏòÍ¶Ó°Ö±·œÍŒ  			
				finder.setThreshold(0.05f); 
				Mat result1=finder.find(colorreduce,shist[i]);				 
			
				//Í³ŒÆ¶þÖµ»¯·ŽÏòÍ¶Ó°ÍŒÏñÖÐµÄ°×É«ÏñËØžöÊý			
				Mat_<uchar>::iterator itstart=result1.begin<uchar>();
				Mat_<uchar>::iterator itend = result1.end<uchar>(); 
				int sumcount=0;
				for (; itstart!=itend; ++itstart)
				{
					if((*itstart)>0)
						sumcount+=1;//¶þÖµ»¯ºó£¬ÏñËØµãÊÇ0»òÕß255
				}	
				//imshow("result1",result1);
				//µ±ÍŒÏñresult1ÖÐ°×É«ÏñËØµÄžöÊý³¬¹ýthrµÄžöÊý£¬ËµÃ÷ÓÐÒªÆ¥ÅäµÄÎïÌåŽæÔÚ¡£
				//œ«ÕâÐ©ÅäÉÏµÄÄ£°å±àºÅŽæÈëcountIDvector
				//œ«result1ŽæÈëresults£¬Áô×ÅºóÓÃ
				if(sumcount>thr)
				{		
					countIDvector.push_back(i);
					if(0==i)
					{
						mat0=result1;					
					}
					else if(1==i)
					{
						mat1=result1;						
					}
					else if(2==i)
					{
						mat2=result1;					
					}
					
					
				}

	            //ÏÂÃæ¶Ô¶þÖµ»¯µÄÍŒÏñresult1×öÅòÕÍž¯ÊŽÔËËã£¬Œì²â±ßÔµ£¬ÕÒ×îÐ¡±ßœçŸØÐÎ£¬ÓÃ³€¿í±ÈÀýÇø·ÖÍ¬ÖÖÑÕÉ«µÄ²»Í¬ÎïÌå£¬ÕÒÖÐÐÄÖáºÍÖÐÐÄµã
				
			}
			//cout<<"countIDvector.size():"<<countIDvector.size()<<endl;
			if(countIDvector.size()==0)
			{
				cout<<"ÍŒÏñÖÐÃ»ÓÐÓëÄ£°å¿âÖÐÏàÆ¥ÅäµÄÎïÆ·"<<endl;
			}
			//ÈýÕÅÄ£°å¶ŒÆ¥ÅäÍêÁË£¬Èç¹ûÃ¿ÕÅÄ£°å¶ŒÆ¥ÅäÉÏÁË£¬ËµÃ÷ÍŒÖÐÓÐ¶àžöÑÕÉ«µÄÎïÌå
			//±éÀúcountIDvectorÖÐiµÄÖµÓÐŒžžö£¬·Ö±ðÊÇÊ²ÃŽ
			vector<int>::iterator itID=countIDvector.begin();
			for(;itID!=countIDvector.end();itID++)
			{
				if((*itID)==0)
				{
					cout<<"ÍŒÏñÖÐÓÐºìÉ«µÄÎïÌå"<<endl;
					//ŽŠÀíÓëºìÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃºìÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat0,(*itID));		
				}
					
				else if((*itID)==1)
				{				
					cout<<"ÍŒÏñÖÐÓÐÀ¶É«µÄÎïÌå"<<endl;
					//ŽŠÀíÓëºÚÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃºÚÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat1,(*itID));					
				}

				else if((*itID)==2)
				{
					cout<<"ÍŒÏñÖÐÓÐÂÌÉ«µÄÎïÌå"<<endl;
					//ŽŠÀíÓë»ÆÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃ»ÆÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat2,(*itID));
				}	
			}
			imshow("œá¹û",color);
			waitKey(33); 
		}        
        return 0;  
    }  

	void minEnclosingRect(Mat &src,Mat &res,int N)
	{
		Mat pFrame=res;
		//ÊýÑ§ÐÎÌ¬Ñ§ÔËËãœøÐÐÅòÕÍž¯ÊŽ£¬œ«Í¬Ò»Ä¿±êÇøÓòœøÐÐÈÚºÏ
		Mat element1 = getStructuringElement(0, Size(20,20), Point(-1,-1));
		morphologyEx(pFrame, pFrame, MORPH_CLOSE, element1);//±ÕÔËËã

		//ŒìË÷ÍŒÏñÖÐËùÓÐÍ¬Ò»ÖÖÑÕÉ«µÄÂÖÀª
		vector<vector<Point> > contours;
		findContours(pFrame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
		// Eliminate too short or too long contours  
		int cmin = 120;  // minimum contour length  //Change these two parameters
		int cmax = 1000; // maximum contour length  
		float angl; 
		char buf[20];
		vector<vector<Point> >::iterator itc = contours.begin();
		while (itc != contours.end())
		{

			if (itc->size() < cmin || itc->size() > cmax)
				itc = contours.erase(itc);
			else
				++itc;
		}
		//ÍŒÖÐÍ¬Ò»ÖÖÑÕÉ«ÎïÌåµÄÃ¿Ò»žö×îÐ¡°üº¬ŸØÐÎ
		vector<vector<Point> >::iterator itcon = contours.begin();		
		for (; itcon != contours.end(); ++itcon)
		{
			RotatedRect rRect = minAreaRect(Mat(*itcon));// minAreaRect:Ñ°ÕÒ°üÎ§Ãæ»ý×îÐ¡µÄŸØÐÎ
			Point2f vertices[4];
			rRect.points(vertices);
			
			Point2f pttA[2];//ÓÃÓÚŽæ·ÅÖÐÐÄÖáµÄ×óÓÒÁœžö¶Ëµã
			Point2f pttCent;//ÓÃÓÚŽæ·ÅÖÐÐÄ×ø±ê
			double ratio;//ÓÃÓÚŽæ·Å³€¿í±ÈÀý
			if(rRect.size.width> rRect.size.height)
			{			
				pttA[0].x = (vertices[0].x + vertices[1].x) / 2;
				pttA[0].y = (vertices[0].y + vertices[1].y) / 2;
				pttA[1].x = (vertices[2].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[2].y + vertices[3].y) / 2;	
				ratio=rRect.size.width/rRect.size.height;
			}
			else
			{				
				pttA[0].x = (vertices[1].x + vertices[2].x) / 2;
				pttA[0].y = (vertices[1].y + vertices[2].y) / 2;
				pttA[1].x = (vertices[0].x + vertices[3].x) / 2;
				pttA[1].y = (vertices[0].y + vertices[3].y) / 2;	
				ratio=rRect.size.height/rRect.size.width;
			}	
			//cout<<"ratio:"<<ratio<<endl;
				if(0==N)
				{
					if( ratio>10.0&&ratio<28.0)
					{
						cout<<"žÃÎïÌåÊÇÒ»Ö»ºìÉ«±Ê"<<endl;
						for (int i = 0; i < 4; ++i)
						{
							line( src, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);//»­ºÖÉ«ÎïÌåµÄ×îÐ¡Íâ°üŸØÐÎ	
						}
						line(src, pttA[0], pttA[1], CV_RGB(255,255, 255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
						pttCent.x = (pttA[0].x + pttA[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
						pttCent.y = (pttA[0].y + pttA[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
						circle(src, pttCent, 2,CV_RGB(255, 255, 255), 3);//ÓÃÂÌÉ«µÄµã»­³öÖÐÐÄ×ø±êµã
						cout<<"ÖÐÐÄ×ø±ê£º"<<pttCent.x<<","<<pttCent.y<<endl;
						//ŒÆËãÆ«×ªœÇ
						angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));//Çó·ŽÕýÇÐº¯Êý
						cout<<"Ðý×ªœÇ£º"<<angl<<" »¡¶È"<<endl;
						center2ds.push_back(pttCent);
						
					}
				}
				else if(1==N)
				{				
					if(ratio>10.0&&ratio<24.0)
					{
						cout<<"žÃÎïÌåÊÇÒ»Ö§À¶É«±Ê"<<endl;
						for (int i = 0; i < 4; ++i)
						{
							line( src, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 0, 255), 2, 8, 0);//»­À¶É«ÎïÌåµÄ×îÐ¡Íâ°üŸØÐÎ	
						}
						line(src, pttA[0], pttA[1], CV_RGB(255, 255,255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
						pttCent.x = (pttA[0].x + pttA[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
						pttCent.y = (pttA[0].y + pttA[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
						circle(src, pttCent, 2,CV_RGB(255, 255, 255), 3);//ÓÃ°×É«µÄµã»­³öÖÐÐÄ×ø±êµã
						cout<<"ÖÐÐÄ×ø±ê£º"<<pttCent<<endl;
						//ŒÆËãÆ«×ªœÇ
						angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));//Çó·ŽÕýÇÐº¯Êý
						cout<<"Ðý×ªœÇ£º"<<angl<<" »¡¶È"<<endl;
						center2ds.push_back(pttCent);
					}
				
				}
				else if(2==N)
				{
					
					if( ratio>10.0&&ratio<24.0)
					{
						cout<<"žÃÎïÌåÊÇÒ»Ö»ÂÌÉ«µÄ±Ê"<<endl;
						for (int i = 0; i < 4; ++i)
						{
							line( src, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255,0), 2, 8, 0);//»­ÂÌÉ«ÎïÌåµÄ×îÐ¡Íâ°üŸØÐÎ
						}
						line(src, pttA[0], pttA[1], CV_RGB(255, 255, 255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
						pttCent.x = (pttA[0].x + pttA[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
						pttCent.y = (pttA[0].y + pttA[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
						circle(src, pttCent, 2,CV_RGB(255, 255, 255), 3);//ÓÃ°×É«µÄµã»­³öÖÐÐÄ×ø±êµã
						cout<<"ÖÐÐÄ×ø±ê£º"<<pttCent<<endl;
						//ŒÆËãÆ«×ªœÇ
						angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));//Çó·ŽÕýÇÐº¯Êý
						cout<<"Ðý×ªœÇ£º"<<angl<<" »¡¶È"<<endl;
						center2ds.push_back(pttCent);
					}
				}
				
				
			
		}

	}
	
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
												   
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::FileStorage fs("box_ref.xml", cv::FileStorage::WRITE);
    fs<<"box_ref"<<cv_ptr->image;

cv::imwrite("desktopbg.bmp",cv_ptr->image);
/*
 //   cv::namedWindow(OPENCV_WINDOW_1);
  //  cv::imshow(OPENCV_WINDOW_1, cv_ptr->image);
  //  cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("box_ref.xml", cv::FileStorage::READ); 
    fs["box_ref"]>>color;


    ref = color - cv_ptr->image ;
    ref.convertTo(color, CV_32FC1);
    test();

//    cv::FileStorage f("box.xml", cv::FileStorage::WRITE);
 //   f<<"box"<<ref;
//    Mat ROI(ref,Rect(140, 100, 500, 300));
*/
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "desktop_detect");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  ros::Rate loop_rate(100);
  image_sub_ = it_.subscribe("/camera/depth_registered/image", 1, imageCb);      
  object_pcl = nh_.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  object_center_2d_pub = nh_.advertise<object_detect::PointVector>("/object_center_2d_pub",1);        //zxx
  object_center_pub = nh_.advertise<object_detect::PointVector>("/object_center_pub",1);        //zxx
 // object_corner_left_pub = nh_.advertise<object_detect::PointVector>("/object_corner_left_pub",1);        //zxx 
  //object_corner_right_pub = nh_.advertise<object_detect::PointVector>("/object_corner_right_pub",1);        //zxx       
  //object_corner_left_2d_pub = nh_.advertise<object_detect::PointVector>("/object_corner_left_2d_pub",1);        //zxx 
  //object_corner_right_2d_pub = nh_.advertise<object_detect::PointVector>("/object_corner_right_2d_pub",1);        //zxx         
  ros::spin();
  
  return 0;
}
