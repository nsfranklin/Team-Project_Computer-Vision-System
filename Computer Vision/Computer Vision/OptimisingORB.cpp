#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <mysqlx/xdevapi.h>
#include <jdbc/mysql_connection.h>
#include <jdbc/cppconn/driver.h>
#include <jdbc/cppconn/exception.h>
#include <jdbc/cppconn/resultset.h>
#include <jdbc/cppconn/statement.h>
#include <jdbc/cppconn/prepared_statement.h>
#include <thread>
#include <time.h>
#include <windows.h>
#include <boost/algorithm/string.hpp>
#include <string>
#include <boost/progress.hpp>

using namespace std;
using namespace cv;
using namespace ::mysqlx;
using namespace boost;

void keypointTesting(vector<Mat> image_set);
void loadTestImageSet(vector<Mat> *image_set, int Length);



int main() {

	vector<Mat> image_set;

	loadTestImageSet(&image_set, 18);

	std::cout << "Images Loaded" << std::endl;

	keypointTesting(image_set);

	std::cout << "Finished Computation" << std::endl;


	waitKey(0);

}


void keypointTesting(vector<Mat> image_set){
	int nfeature = 500; //max number of feature to keep
	float scaleFactor = 1.25f; //Effectly this is a measure of precision
	int edgeThreshold = 31; //Rather clear. How much of the edge should be ignored for feature detection
	int firstLevel = 0; //Don't really understand this at the moment. Seems to be useful in optimising.
	int nlevels = 8;
	int WTA_K = 2;
	int scoreType = cv::ORB::HARRIS_SCORE; //This is used to rank features to determine the best. A faster alternative is FAST_SCORE
	int patchSize = 31; //This size of patches by BRIEF when descripting and matching keypoints found by FAST
	int fastThreshold = 20; //default seems to be 20. It is the threshold in intinity different between the pixel at the center and the points round it.
	Mat mask = Mat(); 
	vector<vector<KeyPoint>> temp;
	vector<Mat> vecDescriptors;
	Mat DescriptorTemp;

	std::cout << "Finished Computation" << std::endl;

	Ptr<ORB> orb = cv::ORB::create(nfeature, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

	std::cout << "Constructing Detector. Detecting Keypoints" << std::endl;

	boost::progress_timer timer;

	orb->detect(image_set, temp, mask); //detect is used as it can take a vector<Mat> rather than detectAndCompute
	std::cout << "Detection Completed" << std::endl;
	std::cout << "Computing Descriptors" << std::endl;
	orb->compute(image_set, temp, vecDescriptors);
	std::cout << "Descriptors Computed" << std::endl;

	std::cout << to_string(timer.elapsed()) << std::endl;
	Mat outImage;
	String outPath;

	for (int i = 0; i < image_set.size(); i++) {
										//output image with rich keypoints
		int flags = DrawMatchesFlags::DEFAULT + DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
		drawKeypoints(image_set[i], temp[i], outImage , Scalar::all(-1), flags);
		//namedWindow(to_string(i), WINDOW_NORMAL);
		//resizeWindow(to_string(i), 2880, 2160);
		outPath = "C:\\Users\\Nick\\Desktop\\ImageOutput\\";
		outPath = outPath + to_string(i) + ".png";
		imwrite(outPath , outImage);
		//imshow(to_string(i), outImage);
	}
}

void loadTestImageSet(vector<Mat> *image_set, int Length) {
	String temp;
	vector<Mat> temp2;

	for (int i = 1; i < Length + 1; i++) {
		temp = to_string(i) + ".png";
		temp2.push_back(imread(temp));
		std::cout << "Image Loaded: " << i << std::endl;
	}
	*image_set = temp2;
	return;
}