#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

void loadImageSet(Mat* image_set, int Length);

int main()
{
	Mat image_array[1] = {};
	loadImageSet(image_array, 1);
	if (image_array[0].empty())
		std::cout << "failed to open img.jpg" << std::endl;
	else
		std::cout << "img.jpg loaded OK" << std::endl;
	Mat img2 = image_array[0];
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", img2);
	waitKey(0);
	return 0;
}

void loadImageSet(Mat* image_set, int Length) {
	String temp;
	for (int i = 0; i < Length; i++) {
		temp = to_string(i) + ".jpg";
	std::cout << temp << std::endl;
		image_set[i] = imread(temp);
	}
}