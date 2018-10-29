#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
int main()
{
	Mat img = imread("lena.jpg");
	if (img.empty())
		std::cout << "failed to open img.jpg" << std::endl;
	else
		std::cout << "img.jpg loaded OK" << std::endl;
	Mat img2 = img;
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", img2);
	waitKey(0);
	return 0;
}