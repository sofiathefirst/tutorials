
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdio>
using namespace std;
using namespace cv;
VideoCapture  camCapRight;

int main()
{
		//camCapLeft.open(1);
		camCapRight.open(0);
		if(!camCapRight.isOpened())
		{
		    printf("Failed to connect the RIGHT camera!\n");
		    return -1;
		}

		camCapRight.set(CV_CAP_PROP_FRAME_WIDTH,800);
		camCapRight.set(CV_CAP_PROP_FRAME_HEIGHT,600);
		Mat m;
		while(1)
		{
			camCapRight>>m;
			imshow("13",m);
			waitKey(1);
		}
std::cout<<"234\n";
        return 0;
}
