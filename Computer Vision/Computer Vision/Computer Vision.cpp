#include "stdafx.h"
#include<opencv2/opencv.hpp>
#include<iostream>

#include<mysqlx/xdevapi.h>

#include <jdbc/mysql_connection.h>
#include <jdbc/cppconn/driver.h>
#include <jdbc/cppconn/exception.h>
#include <jdbc/cppconn/resultset.h>
#include <jdbc/cppconn/statement.h>

using namespace std;
using namespace cv;
using namespace ::mysqlx;

void loadImageSet(vector<Mat> *image_set, int Length);
void loadImageSet(vector<Mat> *image_set, int Length, char prefix);
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec);
void objToMySQL(vector<Mat> *image_set);

int main()
{
	time_t start = time(NULL);
	vector<Mat> image_array = {};
	loadImageSet(&image_array, 3);
	objToMySQL(&image_array);
	if (image_array.empty())
		std::cout << "Failed to load image set" << std::endl; //the start of error handeling
	else
		std::cout << "Image Set Loaded" << std::endl;

	vector<vector<KeyPoint>> KeyPoints;   

	featureMatching(image_array, &KeyPoints);
	std::cout << "Keypoint Detection complete" << std::endl;

	Mat sampleImage = image_array[0];								//sample to show keypoints
	std::cout << "Sample Image Loaded" << std::endl;
	Mat sampleWithKeyPoints;										//output image with rich keypoints
	int flags = DrawMatchesFlags::DEFAULT + DrawMatchesFlags::DRAW_RICH_KEYPOINTS; 
	drawKeypoints(sampleImage, KeyPoints[0], sampleWithKeyPoints, Scalar::all(-1), flags);
	namedWindow("image", WINDOW_NORMAL);
	imshow("image", sampleWithKeyPoints);

	std::cout << "Keypoints detected in image 0.jgp:" << std::endl;
	std::cout << KeyPoints[0].size() << std::endl;

	waitKey(0);


	//cameraCalibration();
	// undistortPoints();
	// triangulatePoints();


	//printf("time elapsed: %d\n", (time(NULL) - start));

	return 0;
}

void objToMySQL(vector<Mat> *image_set) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::Statement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		/* Connect to the MySQL test database */
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
		}
		

		stmt = con->createStatement();
		stmt->executeQuery("SELECT * AS _message");
	


		//delete res;
		delete stmt;
		delete con;
		

	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
	}
}

void loadImageSetFromDatabase(vector<Mat> *image_set, char prefix, int ListingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::Statement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		/* Connect to the MySQL test database */
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
		}


		stmt = con->createStatement();
		stmt->executeQuery("SELECT  AS _message");  //selected images for a required mesh



		//delete res;
		delete stmt;
		delete con;
	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
	}
}


void loadImageSet(vector<Mat> *image_set, int Length, char prefix) {  //used to disingue image sets. relation to the same listing
	String temp;													  //c is intened for chessboard calibration images
	vector<Mat> temp2;
	
	for (int i = 0; i < Length; i++) {
		temp = prefix + to_string(i) + ".jpg";
		temp2.push_back(imread(temp));
	}

	*image_set = temp2;
	return;
}

void loadImageSet(vector<Mat> *image_set, int Length) { 
	String temp;
	vector<Mat> temp2;

	for (int i = 0; i < Length; i++) {
		temp = to_string(i) + ".jpg";
		std::cout << temp << std::endl;
		temp2.push_back(imread(temp));
	}
	*image_set = temp2;
	return;
}

void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec) {
	int nfeature = 500; //max number of feature to keep
	float scaleFactor = 1.2f; //Effectly this is a measure of precision
	int edgeThreshold = 20; //Rather clear. How much of the edge should be ignored for feature detection
	int firstLevel = 0; //Don't really understand this at the moment. Seems to be useful in optimising.
	int WTA_K = 2 ; //  ''
	int scoreType = cv::ORB::HARRIS_SCORE; //This is used to rank features to determine the best. A faster alternative is FAST_SCORE
	int patchSize = 31; //This size of patches by BRIEF when descripting and matching keypoints found by FAST
	int fastThreshold = 20; //default seems to be 20. It is the threshold in intinity different between the pixel at the center and the points round it.
	Mat mask = Mat(); //option part of detect. strange its still a needed parameter
	vector<vector<KeyPoint>> temp;

	Ptr<ORB> orb = cv::ORB::create(nfeature, scaleFactor, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

	std::cout << "Detector Constructed" << std::endl;

	orb->detect(image_set, temp, mask);
	*keyPointVec = temp;

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