#if!defined COLORHISTOGRAM  
#define COLORHISTOGRAM  

#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>  
#include <algorithm>
#include <iostream>  
#include <vector>
using namespace cv;  
using namespace std;
class ColorHistogram  
{  
private:  
	int histSize[3];  
	float hranges[2];  
	const float* ranges[3];  
	int channels[3];  
public:  

	//构造函数  
	ColorHistogram()  
	{  
		histSize[0]= histSize[1]= histSize[2]= 256;  
		hranges[0] = 0.0;  
		hranges[1] = 255.0;  
		ranges[0] = hranges;  
		ranges[1] = hranges;  
		ranges[2] = hranges;  
		channels[0] = 0;  
		channels[1] = 1;  
		channels[2] = 2;  
	}  

	//计算彩色图像直方图  
	Mat getHistogram(const Mat& image)  
	{  
		Mat hist;  

		//BGR直方图  
		hranges[0]= 0.0;      
		hranges[1]= 255.0;  
		channels[0]= 0;   
		channels[1]= 1;   
		channels[2]= 2;   

		//计算  
		calcHist(&image,1,channels,Mat(),hist,3,histSize,ranges);  
		return hist;  
	}  

	//计算颜色的直方图  
	Mat getHueHistogram(const Mat &image)  
	{  
		Mat hist;  
		Mat hue;  
		//转换到HSV空间  
		cvtColor(image,hue,CV_BGR2HSV);  

		//设置1维直方图使用的参数  
		hranges[0] = 0.0;  
		hranges[1] = 180.0;  
		channels[0] = 0;  
		//计算直方图  
		calcHist(&hue,1,channels,Mat(),hist,1,histSize,ranges);  
		return hist;  

	}  

	//减少颜色  
	Mat colorReduce(const Mat &image,int div = 64)  
	{  
		int n = static_cast<int>(log(static_cast<double>(div))/log(2.0));  
		uchar mask = 0xFF<<n;  
		Mat_<Vec3b>::const_iterator it = image.begin<Vec3b>();  
		Mat_<Vec3b>::const_iterator itend = image.end<Vec3b>();  
		//设置输出图像  
		Mat result(image.rows,image.cols,image.type());  
		Mat_<Vec3b>::iterator itr = result.begin<Vec3b>();  
		for(;it != itend;++it,++itr)  
		{  
			(*itr)[0] = ((*it)[0]&mask) + div/2;  
			(*itr)[1] = ((*it)[1]&mask) + div/2;  
			(*itr)[2] = ((*it)[2]&mask) + div/2;  
		}  
		return result;  
	}  

};  


#endif 