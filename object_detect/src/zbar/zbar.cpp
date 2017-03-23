#include<iostream>
#include<string>
#include <opencv2/opencv.hpp>
//#include"ImageProcess.h"
#include <zbar.h>
#include <ros/ros.h>
#include <limits.h>
//#include"udpClient.h"
using namespace std;
using namespace cv;
using namespace zbar;


bool BHuangFuzzyThreshold(const cv::Mat&image, cv::Mat&binary);

void scanTest(cv::Mat&src, std::string& codeNum);

std::string Scan(cv::Mat src);
bool rotation(cv::Mat&in, cv::Mat&out, const cv::Point2f center,
	const double angle, const cv::Size rec_size);
bool getHistGram(const cv::Mat&gray, int HistGram[256]);

int maint(cv::Mat src)
{	
	
	
	if (src.empty())
	{
		std::cout << "over" << std::endl;
		return -3;
	}

	std::string codeNum ="NULL";
	scanTest(src, codeNum);
	
	istringstream iss(codeNum);
	int id ;
	iss>>id;
	cout << "Num: " << codeNum <<id << endl;
	imshow("src", src);
	waitKey(1);
	if(id>0) return id;
	else return 0;
	

	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "twodcode");
	ros::NodeHandle node_handle;  
	
	VideoCapture pCapture2(0);
	if (!pCapture2.isOpened())
	{
		std::cerr << "could not open the video" << std::endl;
		return -2;
	}

	int id = 0,preid= 0;
	int cnt =5;
	cv::Mat src;
	while(true)
	{
		if (pCapture2.isOpened())
		{
			pCapture2 >> src; 
			id = maint(src);
		}
	}
	return id;
	ros::spin();
	cv::destroyAllWindows();
	ros::shutdown();  
	return 0;
}


void scanTest(cv::Mat&src, std::string& codeNum)
{
	cv::Mat gray;
	cvtColor(src, gray, CV_BGR2GRAY);
	cv::imshow("gray", gray);
	cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0);
	//cv::Mat gamaMat;
	//gama(gray, gamaMat, 3, 1);
	//cv::imshow("gamaMat", gamaMat);


	cv::Mat binary;
	BHuangFuzzyThreshold(gray, binary);
	cv::erode(binary, binary, cv::Mat(7, 7, CV_8U, cv::Scalar(1)));
	cv::imshow("binary", binary);

	///absdiff(src, backMat, gamaMat);
	//cv::Mat logMat = logTransform3(gamaMat,1);
	//cv::equalizeHist(src, src);
	//cv::equalizeHist(gray, gray);


	/*cv::Mat binary;
	cv::threshold(gray, binary, 160, 255, cv::THRESH_BINARY);
	cv::imshow("binary", binary);*/

	std::vector<cv::KeyPoint> keypoints;
	////检测SURF特征点
	//cv::SiftFeatureDetector detector2(400);
	cv::FastFeatureDetector detector(20);
	//cv::FastFeatureDetector detector2(40);
	//cv::GoodFeaturesToTrackDetector detector(500, 0.05, 5.0);//返回的最大特征点数目，质量等级，两点之间的最小允许距离
	//提取器cv::FastFeatureDetector 


	//detector.detect(gray, keypoints);
	////二维点云滤波，运行时间0.004s
	//cv::Mat filter(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
	//std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
	//while (itk != keypoints.end())
	//{
	//	cv::circle(src, cv::Point(itk->pt.x, itk->pt.y), 3, cv::Scalar(0, 0, 255), 1);
	//	uchar*data = filter.ptr((int)itk->pt.y);
	//	data[(int)itk->pt.x] = 255;
	//	itk++;
	//}

	std::vector < std::vector <cv::Point > > bcontours;//Cun chu luan kuo 
	cv::findContours(binary, bcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	if (bcontours.size() <= 0)
		return;
	//double length = cv::arcLength(bcontours[0], 0);
	std::vector<cv::Mat> sub_Mat;
	std::vector<cv::RotatedRect> Vmr;
	std::vector < std::vector <cv::Point > >::iterator itb = bcontours.begin();
	while (itb != bcontours.end())
	{
		cv::RotatedRect mr = cv::minAreaRect(cv::Mat(*itb));
		cv::Rect r0 = mr.boundingRect();
		if (mr.size.width> 400 || mr.size.width<50 || mr.size.height>400 || mr.size.height<50)
			itb = bcontours.erase(itb);
		else
		{
			//cout << mr.size.width << "\t" << mr.size.height << endl;
			cv::Mat code_sub_Mat;
			cv::getRectSubPix(gray, Size(r0.width, r0.height), mr.center, code_sub_Mat);
			detector.detect(code_sub_Mat, keypoints);
			std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
			while (itk != keypoints.end())
			{
				cv::circle(code_sub_Mat, cv::Point(itk->pt.x, itk->pt.y), 3, cv::Scalar(0, 0, 255), 1);
				itk++;
			}
			if (keypoints.size() < 100)
				itb = bcontours.erase(itb);
			else
			{
				//cout << keypoints.size() << endl;
				//cv::imshow("code_rot_Mat", code_sub_Mat);
				//waitKey(300);
				cv::getRectSubPix(gray, Size(r0.width, r0.height), mr.center, code_sub_Mat);
				sub_Mat.push_back(code_sub_Mat);
				Vmr.push_back(mr);
				itb++;
			}
		}
	}
	cv::drawContours(src, bcontours, -1, cv::Scalar(0, 0, 255), 2);
	std::vector<cv::Mat>::iterator itm = sub_Mat.begin();
	int iter = 0;
	while (itm != sub_Mat.end())
	{
		cv::Mat rot_Mat;
		rotation(*itm, rot_Mat, cv::Point2f(itm->cols / 2, itm->rows / 2),
			Vmr[iter].angle, cv::Size(itm->cols, itm->rows));
		string s = Scan(rot_Mat);
		if (s == "NULL")
			std::cout << "there has no qrcode!" << std::endl;
		else
		{
			
			codeNum =s;
			
		}
		
		itm++;
	}

}



std::string Scan(cv::Mat src)
{
	//// create a reader
	ImageScanner scanner;
	//// configure the reader
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	// obtain image data
	const void *raw = NULL;
	int width = src.cols;
	int height = src.rows;
	raw = src.data;
	// wrap image data
	zbar::Image image(width, height, "Y800", raw, width * height);
	// scan the image for barcodes
	int n = scanner.scan(image);

	std::string strTemp = "";
	//zbar_symbol_t 
	/*extern int zbar_symbol_get_loc_x(const zbar_symbol_t *symbol,
	unsigned index);*/

	// extract results

	//int i = 0;
	//for (Image::SymbolIterator symbol = image.symbol_begin();
	//	symbol != image.symbol_end();
	//	++symbol,i++) {
	//	// do something useful with results
	//	strTemp = strTemp + symbol->get_data() + ";";
	//	cout << "decoded " << symbol->get_type_name()
	//		<< " symbol \"" << symbol->get_data() << '"' << "\t";
	//}

	//std::cout << "n:" << n << std::endl;
	// clean up
	//if (image.symb)
	std::string str = "NULL";
	if (n > 0)
	{
		str = image.symbol_begin()->get_data();
	}
	image.set_data(NULL, 0);
	return str;
}


bool getHistGram(const cv::Mat&gray, int HistGram[256])
{
	if (gray.data == NULL || gray.channels() != 1)
		return false;
	for (int i = 0; i < gray.rows; ++i)
	{
		const uchar*data = gray.ptr<uchar>(i);
		for (int j = 0; j < gray.cols; ++j)
		{
			int v = int(data[j]);
			HistGram[v]++;
		}
	}
	return true;
}

bool BHuangFuzzyThreshold(const cv::Mat&image, cv::Mat&binary)
{
	if (image.data == NULL)
		return false;
	cv::Mat gray;
	if (1 != image.channels())
		cv::cvtColor(image, gray, CV_BGR2GRAY);
	else
		image.copyTo(gray);
	int HistGram[256] = { 0 };
	getHistGram(gray, HistGram);
	int thresholdvalue = 0;

	int MinValue, MaxValue;
	for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);
	for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--);
	if (MaxValue == MinValue) return false;        // 图像中只有一个颜色             
	if (MinValue + 1 == MaxValue) // 图像中只有二个颜色
	{
		thresholdvalue = MinValue;
		cv::threshold(gray, binary, thresholdvalue, 255, cv::THRESH_BINARY);
		return true;
	}

	////#define INFTY 2147483647

	int X, Y;
	int First = MinValue, Last = MaxValue;
	int Threshold = -1;
	double BestEntropy = 2147483647, Entropy;
	//   找到第一个和最后一个非0的色阶值
	//for (First = 0; First < 256 && HistGram[First] == 0; First++);
	//for (Last = 255; Last > First && HistGram[Last] == 0; Last--);
	//if (First == Last) return First;                // 图像中只有一个颜色
	//if (First + 1 == Last) return First;            // 图像中只有二个颜色

	// 计算累计直方图以及对应的带权重的累计直方图
	//int* S = new int[Last + 1+100];
	//int* W = new int[Last + 1+100];            // 对于特大图，此数组的保存数据可能会超出int的表示范围，可以考虑用long类型来代替
	int S[256] = { 0 };
	int W[256] = { 0 };
	S[0] = HistGram[0];
	W[0] = 0;
	for (Y = 1; Y < 256; Y++)
	{
		S[Y] = S[Y - 1] + HistGram[Y];
		W[Y] = W[Y - 1] + Y * HistGram[Y];
	}

	// 建立公式（4）及（6）所用的查找表
	//double* Smu = new double[Last + 1 - First+20000];
	//for (Y = 1; Y < (Last + 1 - First); Y++)
	//{
	//	double mu = 1 / (1 + (double)Y / (Last - First));               // 公式（4）
	//	Smu[Y] = -mu * log(mu) - (1 - mu) * log(1 - mu);      // 公式（6）
	//}

	// 迭代计算最佳阈值
	for (Y = First; Y <= Last; Y++)
	{
		Entropy = 0;
		double mu0 = (double)W[Y] / S[Y];             // 公式17
		for (X = First; X <= Y; X++)
		{
			double mu = 1 / (1 + double(abs(X - mu0)) / double(Last - First));
			double smu = -mu * log(mu) - (1 - mu) * log(1 - mu);
			Entropy += smu * HistGram[X];
		}

		mu0 = (double)(W[Last] - W[Y]) / (S[Last] - S[Y]);  // 公式18
		for (X = Y + 1; X <= Last; X++)
		{
			double mu = 1 / (1 + double(abs(X - mu0)) / double(Last - First));
			double smu = -mu * log(mu) - (1 - mu) * log(1 - mu);
			Entropy += smu* HistGram[X];       // 公式8
		}


		if (BestEntropy > Entropy)
		{
			BestEntropy = Entropy;      // 取最小熵处为最佳阈值
			thresholdvalue = Y;
		}

	}
	cv::threshold(gray, binary, thresholdvalue, 255, cv::THRESH_BINARY);
	return true;
}

bool rotation(cv::Mat&in, cv::Mat&out, const cv::Point2f center,
	const double angle, const cv::Size rec_size)
{
	Mat in_large;
	in_large.create(int(in.rows*1.5), int(in.cols*1.5), in.type());


	float x = in_large.cols / 2 - center.x > 0 ? in_large.cols / 2 - center.x : 0;
	float y = in_large.rows / 2 - center.y > 0 ? in_large.rows / 2 - center.y : 0;

	float width = x + in.cols < in_large.cols ? in.cols : in_large.cols - x;
	float height = y + in.rows < in_large.rows ? in.rows : in_large.rows - y;

	/*assert(width == in.cols);
	assert(height == in.rows);*/

	if (width != in.cols || height != in.rows) return false;

	Mat imageRoi = in_large(Rect_<float>(x, y, width, height));
	addWeighted(imageRoi, 0, in, 1, 0, imageRoi);
	//jiang kong tu xiang yu yuan tu xiang die jia
	//	imwrite("imageRoi.BMP", imageRoi);

	Point2f new_ceter(in_large.cols / 2.0f, in_large.rows / 2.0f);
	Mat rot_mat = getRotationMatrix2D(new_ceter, angle, 1);

	Mat mat_rotated;
	warpAffine(in_large, mat_rotated, rot_mat, Size(in_large.cols, in_large.rows), CV_INTER_CUBIC);

	Mat sub_Mat;
	cv::getRectSubPix(mat_rotated, Size(rec_size.width, rec_size.height), new_ceter, sub_Mat);

	if (sub_Mat.cols < sub_Mat.rows)
	{

		Mat Rot90;
		int r = sub_Mat.rows;
		int c = sub_Mat.cols;
		Rot90.create(c, r, CV_8UC3);

		Vec3b S;
		for (int i = 0; i < Rot90.rows; ++i)
		{
			//uchar*dataR = Rot90.ptr<uchar>(i);
			for (int j = 0; j< Rot90.cols; ++j)
			{

				//S= sub_Mat.at<Vec3b>(r-1, i);
				S = sub_Mat.at<Vec3b>(j, c - 1);
				//S[0] = 0;
				//S[1] = 0;
				//S[2] = 255;
				Rot90.at<Vec3b>(i, j) = S;
				//uchar*dataS = sub_Mat.ptr<uchar>(r);
				//dataR[j] = dataS[0];
				//dataR[j] = 0;
			}
			c--;
			//	r = sub_Mat.rows;
		}
		/*cout << sub_Mat.channels() << endl;
		namedWindow("Rot90", 0);
		imshow("Rot90", Rot90);
		waitKey(0);*/
		out = Rot90;

	}
	else
		out = sub_Mat;

	/*namedWindow("Mat_rotated", 0);
	imshow("Mat_rotated", mat_rotated);
	waitKey(0);*/

	return true;
}
