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
#include "object_detect/contentFinder.h" 
#include "object_detect/colorhistogram.h"  
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
						cout<<"ÖÐÐÄ×ø±ê£º"<<pttCent<<endl;
						//ŒÆËãÆ«×ªœÇ
						angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));//Çó·ŽÕýÇÐº¯Êý
						cout<<"Ðý×ªœÇ£º"<<angl<<" »¡¶È"<<endl;
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
					}
				
					//else if( ratio>1.3&&ratio<3.2)
					//{
					//	cout<<"žÃÎïÌåÊÇÒ»žöºÚÉ«Ò£¿ØÆ÷"<<endl;
					//	for (int i = 0; i < 4; ++i)
					//	{
					//		line( src, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 0, 0), 2, 8, 0);//»­ºÚÎïÌåµÄ×îÐ¡Íâ°üŸØÐÎ	
					//	}
					//	line(src, pttA[0], pttA[1], CV_RGB(0, 255, 0), 2, 8, 0);//ÓÃÂÌÉ«µÄÏß»­³öÖÐÐÄÖá
					//	pttCent.x = (pttA[0].x + pttA[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
					//	pttCent.y = (pttA[0].y + pttA[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
					//	circle(src, pttCent, 2,CV_RGB(0, 255, 0), 3);//ÓÃÂÌÉ«µÄµã»­³öÖÐÐÄ×ø±êµã
					//	cout<<"ÖÐÐÄ×ø±ê£º"<<pttCent<<endl;
					//	//ŒÆËãÆ«×ªœÇ
					//	angl = atan2((pttA[1].y - pttA[0].y), (pttA[1].x - pttA[0].x));//Çó·ŽÕýÇÐº¯Êý
					//	cout<<"Ðý×ªœÇ£º"<<angl<<" »¡¶È"<<endl;
					//}

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
					}
				}
				
			
		}

	}
	
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
			sprintf(filename,"tp2/%d.bmp",i);
			Mat tt = imread(filename);
//imwrite("aaimg/color.bmp",color);
			//imshow("aa",tt);
			waitKey(3);
			colortempl=imread(filename,1);
			if(colortempl.empty())
			{
				cout<<"colortempl.empty"<<endl;
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
			imshow("color", color);
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
				cout<<"countIDvector.size()=0"<<endl;
			}
			//ÈýÕÅÄ£°å¶ŒÆ¥ÅäÍêÁË£¬Èç¹ûÃ¿ÕÅÄ£°å¶ŒÆ¥ÅäÉÏÁË£¬ËµÃ÷ÍŒÖÐÓÐ¶àžöÑÕÉ«µÄÎïÌå
			//±éÀúcountIDvectorÖÐiµÄÖµÓÐŒžžö£¬·Ö±ðÊÇÊ²ÃŽ
			vector<int>::iterator itID=countIDvector.begin();
			for(;itID!=countIDvector.end();itID++)
			{
				if((*itID)==0)
				{
					cout<<"(*itID)==0"<<endl;
					//ŽŠÀíÓëºìÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃºìÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat0,(*itID));		
				}
					
				else if((*itID)==1)
				{				
					cout<<"(*itID)==1"<<endl;
					//ŽŠÀíÓëºÚÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃºÚÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat1,(*itID));					
				}

				else if((*itID)==2)
				{
					cout<<"(*itID)==2"<<endl;
					//ŽŠÀíÓë»ÆÉ«Ä£°åÏàÆ¥ÅäµÄ¶þÖµ»¯ÍŒÏñ£¬ÕÒµœÂÖÀª£¬ÓÃ»ÆÉ«Ïß»­³ö×îÐ¡ŸØÐÎ¿ò
					minEnclosingRect(color,mat2,(*itID));
				}	
			}
			//if (color
			imshow("color",color);
			waitKey(3); 
		}        
        return 0;  
    }  
int test2()  
{  
	
	//while (true)
	{
		
		Mat image=color;
		
		Mat hsv;
		cvtColor( image, hsv, CV_BGR2HSV );
		//imshow("HSV",hsv);

		Mat gray;
		cvtColor(image,gray,CV_BGR2GRAY);
	

		Mat mat1;
		gray.copyTo(mat1);
		Mat mat2;
		gray.copyTo(mat2);
		Mat mat3;
		gray.copyTo(mat3);
		Mat mat4;
		gray.copyTo(mat4);
		Mat mat5;
		gray.copyTo(mat5);
	
		Mat_<uchar>::iterator it1 = mat1.begin<uchar>();
		Mat_<uchar>::iterator it2 = mat2.begin<uchar>();
		Mat_<uchar>::iterator it3 = mat3.begin<uchar>();
		Mat_<uchar>::iterator it4 = mat4.begin<uchar>();
		Mat_<uchar>::iterator it5 = mat5.begin<uchar>();


		Mat_<Vec3b>::iterator ithsv = hsv.begin<Vec3b>();
		for(;ithsv!=hsv.end<Vec3b>();++ithsv,++it1,++it2,++it3,++it4,++it5)
		{
			double H = ((*ithsv).val[0])*2;
			double S = ((*ithsv).val[1])/255.0;
			double V = ((*ithsv).val[2])/255.0; 

			if((H>230&&H<260)&&(S>=0.35&&S<=1.0)&&(V>=0.35&&V<=1.0))//×ÏÉ«
			{				
		
					(*it1)=255;
			}
			else
			{
		
					(*it1)=0;
			}
	

			if((H>120&&H<170)&&(S>=0.25&&S<=1.0)&&(V>=0.25&&V<=1.0))//ÂÌÉ«
			{		
				
				(*it2)=255;
				
			}
			else
			{
				(*it2)=0;
				
			}
			if((H>340&&H<360)&&(S>=0.35&&S<=1.0)&&(V>=0.35&&V<=1.0))//ºìÉ«
			{		
				(*it3)=255;
				
			}
			else
			{
				(*it3)=0;
				
			}
			if((H>30&&H<50)&&(S>=0.35&&S<=1.0)&&(V>=0.35&&V<=1.0))//»ÆÉ«
			{			
				(*it4)=255;

			}
			else
			{
				
				(*it4)=0;

			}
			if((H>190&&H<220)&&(S>=0.35&&S<=1.0)&&(V>=0.35&&V<=1.0))//ÌìÀ¶É«
			{			
				
				(*it5)=255;

			}
			else
			{
				
				(*it5)=0;

			}
		}
		//ÊýÑ§ÐÎÌ¬Ñ§ÔËËãœøÐÐÅòÕÍž¯ÊŽ£¬œ«Í¬Ò»Ä¿±êÇøÓòœøÐÐÈÚºÏ
		Mat element1 = getStructuringElement(0, Size(15,15), Point(-1,-1));
		morphologyEx(mat1,mat1, MORPH_CLOSE, element1);//±ÕÔËËã
		morphologyEx(mat2,mat2, MORPH_CLOSE, element1);//±ÕÔËËã
		morphologyEx(mat3,mat3, MORPH_CLOSE, element1);//±ÕÔËËã
		morphologyEx(mat4,mat4, MORPH_CLOSE, element1);//±ÕÔËËã
		morphologyEx(mat5,mat5, MORPH_CLOSE, element1);//±ÕÔËËã
		imshow("ZiSe",mat1);
		imshow("Green",mat2);
		imshow("Red",mat3);
		imshow("Yellow",mat4);
		imshow("Blue",mat5);
		int cmin = 100;  // minimum contour length  //Change these two parameters
		int cmax = 1500; // maximum contour length  
		
		vector<vector<Point> > contours1;
		findContours(mat1, contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
		vector<vector<Point> >::iterator itc1 = contours1.begin();
		while (itc1 != contours1.end())
		{

			if (itc1->size() < cmin || itc1->size() > cmax)
			itc1 = contours1.erase(itc1);
			else
			++itc1;
		}

		vector<vector<Point> > contours2;
		findContours(mat2, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		vector<vector<Point> >::iterator itc2 = contours2.begin();
		while (itc2 != contours2.end())
		{

			if (itc2->size() < cmin || itc2->size() > cmax)
				itc2 = contours2.erase(itc2);
			else
				++itc2;
		}

		vector<vector<Point> > contours3;
		findContours(mat3, contours3, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
		vector<vector<Point> >::iterator itc3 = contours3.begin();
		while (itc3 != contours3.end())
		{

			if (itc3->size() < cmin || itc3->size() > cmax)
				itc3 = contours3.erase(itc3);
			else
				++itc3;
		}

		vector<vector<Point> > contours4;
		findContours(mat4, contours4, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
		vector<vector<Point> >::iterator itc4 = contours4.begin();
		while (itc4 != contours4.end())
		{

			if (itc4->size() < cmin || itc4->size() > cmax)
				itc4 = contours4.erase(itc4);
			else
				++itc4;
		}
		vector<vector<Point> > contours5;
		findContours(mat5, contours5, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
		vector<vector<Point> >::iterator itc5 = contours5.begin();
		while (itc5 != contours5.end())
		{

			if (itc5->size() < cmin || itc5->size() > cmax)
				itc5 = contours5.erase(itc5);
			else
				++itc5;
		}
		
		vector<vector<Point> >::iterator itcon1 = contours1.begin();		
		for (; itcon1 != contours1.end(); ++itcon1)
		{
			RotatedRect rRect1 = minAreaRect(Mat(*itcon1));
			Point2f vertices1[4];
			rRect1.points(vertices1);

			Point2f ptt1[2];
			Point2f pttCent1;
			double ratio1;
			if(rRect1.size.width> rRect1.size.height)
			{			
				ptt1[0].x = (vertices1[0].x + vertices1[1].x) / 2;
				ptt1[0].y = (vertices1[0].y + vertices1[1].y) / 2;
				ptt1[1].x = (vertices1[2].x + vertices1[3].x) / 2;
				ptt1[1].y = (vertices1[2].y + vertices1[3].y) / 2;	
				ratio1=rRect1.size.width/rRect1.size.height;
			}
			else
			{				
				ptt1[0].x = (vertices1[1].x + vertices1[2].x) / 2;
				ptt1[0].y = (vertices1[1].y + vertices1[2].y) / 2;
				ptt1[1].x = (vertices1[0].x + vertices1[3].x) / 2;
				ptt1[1].y = (vertices1[0].y + vertices1[3].y) / 2;	
				ratio1=rRect1.size.height/rRect1.size.width;
			}	
			//cout<<"ratio1:"<<ratio1<<endl;
			for (int i = 0; i < 4; ++i)
			{
				line( image, vertices1[i], vertices1[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);	
			}
			line(image, ptt1[0], ptt1[1], CV_RGB(255,255, 255), 2, 8, 0);
			pttCent1.x = (ptt1[0].x + ptt1[1].x) / 2;
			pttCent1.y = (ptt1[0].y + ptt1[1].y) / 2;
			circle(image, pttCent1, 2,CV_RGB(255, 255, 255), 3);
			float angl1; 
			angl1 = atan2((ptt1[1].y - ptt1[0].y), (ptt1[1].x - ptt1[0].x));
			if(ratio1>10.0&&ratio1<28.0)
			{
				
				cout<<" Pen of Purple: "<<endl;
				cout<<"Center coordinate£º"<<pttCent1<<endl;				
				cout<<"Angle of rotation£º"<<angl1<<" »¡¶È"<<endl;
			}
			else
			{
				cout<<"ÓÐÖØµþ»òÕßœô°€µÄ×ÏÉ«ÎïÌå£¡£šÒ²ÓÐ¿ÉÄÜÊÇžÉÈÅ£©"<<endl;
			}
			
		}
		
		vector<vector<Point> >::iterator itcon2 = contours2.begin();		
		for (; itcon2 != contours2.end(); ++itcon2)
		{
			RotatedRect rRect2 = minAreaRect(Mat(*itcon2));
			Point2f vertices2[4];
			rRect2.points(vertices2);
			Point2f ptt2[2];
			Point2f pttCent2;
			double ratio2;
			if(rRect2.size.width> rRect2.size.height)
			{			
				ptt2[0].x = (vertices2[0].x + vertices2[1].x) / 2;
				ptt2[0].y = (vertices2[0].y + vertices2[1].y) / 2;
				ptt2[1].x = (vertices2[2].x + vertices2[3].x) / 2;
				ptt2[1].y = (vertices2[2].y + vertices2[3].y) / 2;	
				ratio2=rRect2.size.width/rRect2.size.height;
			}
			else
			{				
				ptt2[0].x = (vertices2[1].x + vertices2[2].x) / 2;
				ptt2[0].y = (vertices2[1].y + vertices2[2].y) / 2;
				ptt2[1].x = (vertices2[0].x + vertices2[3].x) / 2;
				ptt2[1].y = (vertices2[0].y + vertices2[3].y) / 2;	
				ratio2=rRect2.size.height/rRect2.size.width;
			}	
			//cout<<"ratio1:"<<ratio1<<endl;
			for (int i = 0; i < 4; ++i)
			{
				line( image, vertices2[i], vertices2[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);	
			}
			line(image, ptt2[0], ptt2[1], CV_RGB(255,255, 255), 2, 8, 0);
			pttCent2.x = (ptt2[0].x + ptt2[1].x) / 2;
			pttCent2.y = (ptt2[0].y + ptt2[1].y) / 2;
			circle(image, pttCent2, 2,CV_RGB(255, 255, 255), 3);
			float angl2; 
			angl2 = atan2((ptt2[1].y - ptt2[0].y), (ptt2[1].x - ptt2[0].x));

			if(ratio2>10.0&&ratio2<28.0)
			{
				
				cout<<"Pen of Green"<<endl;
				cout<<"Center coordinate£º"<<pttCent2<<endl;			
				cout<<"Angle of rotation£º"<<angl2<<" »¡¶È"<<endl;
			}
			else
			{
				cout<<"ÓÐÖØµþ»òÕßœô°€µÄÂÌÉ«ÎïÌå£¡£šÒ²ÓÐ¿ÉÄÜÊÇžÉÈÅ£©"<<endl;
			}
		}
		vector<vector<Point> >::iterator itcon3 = contours3.begin();		
		for (; itcon3 != contours3.end(); ++itcon3)
		{
			RotatedRect rRect3 = minAreaRect(Mat(*itcon3));
			Point2f vertices3[4];
			rRect3.points(vertices3);
			Point2f ptt3[2];//ÓÃÓÚŽæ·ÅÖÐÐÄÖáµÄ×óÓÒÁœžö¶Ëµã
			Point2f pttCent3;//ÓÃÓÚŽæ·ÅÖÐÐÄ×ø±ê
			double ratio3;//ÓÃÓÚŽæ·Å³€¿í±ÈÀý
			if(rRect3.size.width> rRect3.size.height)
			{			
				ptt3[0].x = (vertices3[0].x + vertices3[1].x) / 2;
				ptt3[0].y = (vertices3[0].y + vertices3[1].y) / 2;
				ptt3[1].x = (vertices3[2].x + vertices3[3].x) / 2;
				ptt3[1].y = (vertices3[2].y + vertices3[3].y) / 2;	
				ratio3=rRect3.size.width/rRect3.size.height;
			}
			else
			{				
				ptt3[0].x = (vertices3[1].x + vertices3[2].x) / 2;
				ptt3[0].y = (vertices3[1].y + vertices3[2].y) / 2;
				ptt3[1].x = (vertices3[0].x + vertices3[3].x) / 2;
				ptt3[1].y = (vertices3[0].y + vertices3[3].y) / 2;	
				ratio3=rRect3.size.height/rRect3.size.width;
			}	
			//cout<<"ratio1:"<<ratio1<<endl;
			for (int i = 0; i < 4; ++i)
			{
				line( image, vertices3[i], vertices3[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);	
			}
			line(image, ptt3[0], ptt3[1], CV_RGB(255,255, 255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
			pttCent3.x = (ptt3[0].x + ptt3[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
			pttCent3.y = (ptt3[0].y + ptt3[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
			circle(image, pttCent3, 2,CV_RGB(255, 255, 255), 3);//ÓÃÂÌÉ«µÄµã»­³öÖÐÐÄ×ø±êµã
			//ŒÆËãÆ«×ªœÇ
			float angl3; 
			angl3 = atan2((ptt3[1].y - ptt3[0].y), (ptt3[1].x - ptt3[0].x));//Çó·ŽÕýÇÐº¯Êý

			if(ratio3>10.0&&ratio3<28.0)
			{
				
				cout<<"Pen of Red:"<<endl;
				cout<<"Center coordinate£º"<<pttCent3<<endl;	
				cout<<"Angle of rotation£º"<<angl3<<" »¡¶È"<<endl;
			}
			else
			{
				cout<<"ÓÐÖØµþ»òÕßœô°€µÄºìÉ«ÎïÌå£¡£šÒ²ÓÐ¿ÉÄÜÊÇžÉÈÅ£©"<<endl;
			}
		}
		vector<vector<Point> >::iterator itcon4 = contours4.begin();		
		for (; itcon4 != contours4.end(); ++itcon4)
		{
			RotatedRect rRect4 = minAreaRect(Mat(*itcon4));
			Point2f vertices4[4];
			rRect4.points(vertices4);
			Point2f ptt4[2];//ÓÃÓÚŽæ·ÅÖÐÐÄÖáµÄ×óÓÒÁœžö¶Ëµã
			Point2f pttCent4;//ÓÃÓÚŽæ·ÅÖÐÐÄ×ø±ê
			double ratio4;//ÓÃÓÚŽæ·Å³€¿í±ÈÀý
			if(rRect4.size.width> rRect4.size.height)
			{			
				ptt4[0].x = (vertices4[0].x + vertices4[1].x) / 2;
				ptt4[0].y = (vertices4[0].y + vertices4[1].y) / 2;
				ptt4[1].x = (vertices4[2].x + vertices4[3].x) / 2;
				ptt4[1].y = (vertices4[2].y + vertices4[3].y) / 2;	
				ratio4=rRect4.size.width/rRect4.size.height;
			}
			else
			{				
				ptt4[0].x = (vertices4[1].x + vertices4[2].x) / 2;
				ptt4[0].y = (vertices4[1].y + vertices4[2].y) / 2;
				ptt4[1].x = (vertices4[0].x + vertices4[3].x) / 2;
				ptt4[1].y = (vertices4[0].y + vertices4[3].y) / 2;	
				ratio4=rRect4.size.height/rRect4.size.width;
			}	
			//cout<<"ratio1:"<<ratio1<<endl;
			for (int i = 0; i < 4; ++i)
			{
				line( image, vertices4[i], vertices4[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);	
			}
			line(image, ptt4[0], ptt4[1], CV_RGB(255,255, 255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
			pttCent4.x = (ptt4[0].x + ptt4[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
			pttCent4.y = (ptt4[0].y + ptt4[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
			circle(image, pttCent4, 2,CV_RGB(255, 255, 255), 3);//ÓÃÂÌÉ«µÄµã»­³öÖÐÐÄ×ø±êµã
			//ŒÆËãÆ«×ªœÇ
			float angl4; 
			angl4 = atan2((ptt4[1].y - ptt4[0].y), (ptt4[1].x - ptt4[0].x));//Çó·ŽÕýÇÐº¯Êý

			if(ratio4>10.0&&ratio4<28.0)
			{			
				cout<<"Pen of Yellow:"<<endl;
				cout<<"Center coordinate£º"<<pttCent4<<endl;		
				cout<<"Angle of rotation£º"<<angl4<<" »¡¶È"<<endl;
			}
			else
			{
				cout<<"ÓÐÖØµþ»òÕßœô°€µÄ»ÆÉ«ÎïÌå£¡£šÒ²ÓÐ¿ÉÄÜÊÇžÉÈÅ£©"<<endl;
			}
		}
		vector<vector<Point> >::iterator itcon5 = contours5.begin();		
		for (; itcon5 != contours5.end(); ++itcon5)
		{
			RotatedRect rRect5 = minAreaRect(Mat(*itcon5));
			Point2f vertices5[4];
			rRect5.points(vertices5);
			Point2f ptt5[2];//ÓÃÓÚŽæ·ÅÖÐÐÄÖáµÄ×óÓÒÁœžö¶Ëµã
			Point2f pttCent5;//ÓÃÓÚŽæ·ÅÖÐÐÄ×ø±ê
			double ratio5;//ÓÃÓÚŽæ·Å³€¿í±ÈÀý
			if(rRect5.size.width> rRect5.size.height)
			{			
				ptt5[0].x = (vertices5[0].x + vertices5[1].x) / 2;
				ptt5[0].y = (vertices5[0].y + vertices5[1].y) / 2;
				ptt5[1].x = (vertices5[2].x + vertices5[3].x) / 2;
				ptt5[1].y = (vertices5[2].y + vertices5[3].y) / 2;	
				ratio5=rRect5.size.width/rRect5.size.height;
			}
			else
			{				
				ptt5[0].x = (vertices5[1].x + vertices5[2].x) / 2;
				ptt5[0].y = (vertices5[1].y + vertices5[2].y) / 2;
				ptt5[1].x = (vertices5[0].x + vertices5[3].x) / 2;
				ptt5[1].y = (vertices5[0].y + vertices5[3].y) / 2;	
				ratio5=rRect5.size.height/rRect5.size.width;
			}	
			//cout<<"ratio1:"<<ratio1<<endl;
			for (int i = 0; i < 4; ++i)
			{
				line( image, vertices5[i], vertices5[(i + 1) % 4], CV_RGB(255, 0,0), 2, 8, 0);	
			}
			line(image, ptt5[0], ptt5[1], CV_RGB(255,255, 255), 2, 8, 0);//ÓÃ°×É«µÄÏß»­³öÖÐÐÄÖá
			pttCent5.x = (ptt5[0].x + ptt5[1].x) / 2;//ŒÆËãÖÐÐÄÖáx×ø±ê
			pttCent5.y = (ptt5[0].y + ptt5[1].y) / 2;//ŒÆËãÖÐÐÄÖáy×ø±ê
			circle(image, pttCent5, 2,CV_RGB(255, 255, 255), 3);//ÓÃÂÌÉ«µÄµã»­³öÖÐÐÄ×ø±êµã
			//ŒÆËãÆ«×ªœÇ
			float angl5; 
			angl5 = atan2((ptt5[1].y - ptt5[0].y), (ptt5[1].x - ptt5[0].x));//Çó·ŽÕýÇÐº¯Êý

			if(ratio5>10.0&&ratio5<28.0)
			{
				
				cout<<"Pen of Blue:"<<endl;
				cout<<"Center coordinate£º"<<pttCent5<<endl;
				
				cout<<"Angle of rotation£º"<<angl5<<" »¡¶È"<<endl;
			}
			else
			{
				cout<<"ÓÐÖØµþ»òÕßœô°€µÄÀ¶É«ÎïÌå£¡£šÒ²ÓÐ¿ÉÄÜÊÇžÉÈÅ£©"<<endl;
			}
		}
		imshow("image finally",image);
		waitKey(13);
	}

	return 0;
}
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
												   
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::image_encodings::TYPE_32FC1);   			//convert ROS image to CV image and make copy of it storing in cv_ptr(a pointer)
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   // cv::FileStorage fs("box_ref.xml", cv::FileStorage::WRITE);
   // fs<<"box_ref"<<cv_ptr->image;

cv::imwrite("dest1.bmp",cv_ptr->image);
    cv::namedWindow("dest1.bmp");
    cv::imshow("dest1.bmp", cv_ptr->image);
    cv::waitKey(3);

    cv::Mat ref;
    cv::FileStorage fs("box_ref.xml", cv::FileStorage::READ); 
    fs["box_ref"]>>ref;


   // color = ref - cv_ptr->image ;
	color = cv_ptr->image;
   // ref.convertTo(color, CV_32FC1);
    if(color.data!=NULL )
    {test2();}
//    cv::FileStorage f("box.xml", cv::FileStorage::WRITE);
 //   f<<"box"<<ref;
//    Mat ROI(ref,Rect(140, 100, 500, 300));

  }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiple_detect");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  ros::Rate loop_rate(100);
 // image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, imageCb);   
  image_sub_ = it_.subscribe("/usb_cam1/image_raw", 1, imageCb); 
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
