#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;

void loadImageSet(Mat* image_set, int Length);
void loadImageSet(Mat* image_set, int Length, char prefix);

int main()
{
	Mat image_array[2] = {};
	loadImageSet(image_array, 2);
	if (image_array[0].empty())
		std::cout << "failed to open img.jpg" << std::endl;
	else
		std::cout << "img.jpg loaded OK" << std::endl;
	Mat img2 = image_array[0];
	Mat img1 = image_array[1];
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", img2);
	namedWindow("image2", WINDOW_NORMAL);
	imshow("image2", image_array[1]);
	waitKey(0);
	return 0;
}


void loadImageSet(Mat* image_set, int Length, char prefix) {  //used to disingue image sets. relation to the same listing
	String temp;											  //c is intened for chessboard calibration images
	for (int i = 0; i < Length; i++) {
		temp = prefix + to_string(i) + ".jpg";
	std::cout << temp << std::endl;
		image_set[i] = imread(temp);
	}
}

void loadImageSet(Mat* image_set, int Length) { 
	String temp;
	for (int i = 0; i < Length; i++) {
		temp = to_string(i) + ".jpg";
		std::cout << temp << std::endl;
		image_set[i] = imread(temp);
	}
}

void featureMatching(Mat image_set) {
	int nfeature = 500; //max number of feature to keep
	float scaleFactor = 1.2f; //Effectly this is a measure of precision
	int edgeThreshold = 20; //Rather clear. How much of the edge should be ignored for feature detection
	int firstLevel = 0; //Don't really understand this at the moment. Seems to be useful in optimising.
	int WTA_K = 2 ; //  ''
	int scoreType = cv::ORB::HARRIS_SCORE; //This is used to rank features to determine the best. A faster alternative is FAST_SCORE
	int patchSize = 31; //This size of patches by BRIEF when descripting and matching keypoints found by FAST
	int fastThreshold = 20; //default seems to be 20. It is the threshold in intinity different between the pixel at the center and the points round it.

	vector<KeyPoint> keypoints;

	Ptr<ORB> orb = cv::ORB::create(nfeature, scaleFactor, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

	orb->detect(image_set, keypoints);



}


/*  Work in Progress

bool cameraCalibration(Mat image_set[]) {
	
bool methodSuccess;
Size imageSetSize = cv::Size(image_set[0].size().width, image_set[0].size().height);                                 //all the images need to be of a fixed resolution
Size chessboardSize = cv::Size(7, 9);
vector<vector<Point2f>> cornersMatrix;
vector<Point2f> cornersTemp;
TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON)
int count = 0;

for (;;){
	methodSuccess = findChessboardCorners(image_set[count], chessboardSize, cornersTemp, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

	count = count + 1;
}	

	calculateCameraMatrix();

	calibrateCamera(cornersMatrix,,imageSetSize, cameraMatrix, distCoeffs, rvec, tvec,stdDevIntrinsics, stdDevExtrinsics, perViewErrors, flags=0, criteria); //calibrate camera is used to calculate the camera matrix and distortion coefficient.
	return false;
}

void calculateCameraMatrix() {



}

void create

void generateSparcePointCloud() {



	triangulatePoints(cam1ProjectionMatrix, cam2ProjectionMatrix, cam1Points, cam2Points, Output);

}

*/