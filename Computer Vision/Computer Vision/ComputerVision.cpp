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
#include <fstream>

using namespace std;
using namespace cv;
using namespace ::mysqlx;
using namespace boost;

void loadImageSet(vector<Mat> *image_set, int Length);
void loadImageSet(vector<Mat> *image_set, int Length, char prefix);
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec);
void objToMySQL(String filename, int ListingID, int ModelID);
bool objToMySQL(int ListingID);
void insertImages(vector<Mat> *image_set, int listingID, int length);
void loadImageSetFromDatabase(vector<Mat> *image_set, int ListingID, bool isCalibration);
int determinePending();
bool MeshXYZToOBJ(int ListingID);
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners, int patternType);
void generateSparcePointCloud();
void setStateAvailable(int ListingID);
bool checkCameraParameters(int listingID);
bool cameraCalibration(float focusLength, float sensorWidth, int listingID); //the lenght and width are in mm
bool triangulatePoints();
int loadCameraDetails(int listingID, float &focusLength, float &sensorWidth);
void loadLocalCalibration(int listingID, Mat cameraMatrix);
int findCameraID(int listingID);
void setStatePending(int listingID);
void dummyTriangulatePoints(int listingID);
void dummyDensification();
void setListingStateFailed(int listingID);
bool checkForLocalCalibration(int listingID);
bool saveCameraDetails(int listingID, Mat cameraMatrix, Mat distCoeffs);

int main()
{
	bool successful = true;
	int pendingListingID = -1;
	vector<Mat> image_array = {};
	vector<vector<KeyPoint>> KeyPoints;
	vector<vector<KeyPoint>> undistortedKeyPoints;
	float focusLength; 
	float sensorWidth; 
	String failureReason = "";

	while (true) {

		std::cout << "Determined Pending Listing" << std::endl;

		pendingListingID = determinePending(); //returns the pending ID. If there are no pending IDs -1 is returned.

		if (pendingListingID != -1) {
			if (loadCameraDetails(pendingListingID, focusLength, sensorWidth) > 0) {
				if (cameraCalibration(focusLength, sensorWidth, pendingListingID)) { //checks for local calibration. Then calibrates if not present.
					loadImageSetFromDatabase(&image_array, pendingListingID, false); //Parameters: The image array to load them into, the Listing ID for the Image
					if (image_array.empty() || image_array.size() < 25 || image_array.size() > 40) {

						featureMatching(image_array, &KeyPoints);
						std::cout << "Keypointed and Feature Detection complete" << std::endl;

						Mat sampleImage = image_array[2];								//sample to show keypoints
						Mat sampleWithKeyPoints;										//output image with rich keypoints
						//drawKeypoints(sampleImage, KeyPoints[2], sampleWithKeyPoints, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
						//namedWindow("image", WINDOW_NORMAL);
						//imshow("image", sampleWithKeyPoints);
						//undistortAllPoints(KeyPoints, undistortedKeyPoints , vecPending[0]); //undistort point has a bug so undistorting the image will be used at an earlier point 

						dummyTriangulatePoints(pendingListingID);
						dummyDensification();

						MeshXYZToOBJ(pendingListingID);

						successful = objToMySQL(pendingListingID);

						std::cout << "Keypointed and Feature Detection complete" << std::endl;
					}
					else{
						successful = false; //Image load failed
						failureReason = "Image load Failed";
					}
				}else{
					successful = false; // cameraCalibration Failed
					failureReason = "Calibration Failed";
				}
			}
			else {
				successful = false; // camera details not present
				failureReason = "Camera Details not present";
			}

			image_array.clear();
			KeyPoints.clear();
			undistortedKeyPoints.clear();

			if (successful) {
				setStateAvailable(pendingListingID);
			}
			else {
				std::cout << failureReason << std::endl;
				setListingStateFailed(pendingListingID);
			}
			pendingListingID = -1;
			successful = true;

		}
		else {
			std::cout << "Sleeping Nothing Pending" << std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //sleeps for 5 seconds then checks again.
		}

		//break; //TEMP BREAKCLAUSE FOR TESTING

		cv::waitKey(0);
	}
}

bool checkForLocalCalibration(int listingID) {
	return false;
}
bool saveCameraDetails(int listingID, Mat cameraMatrix, Mat distCoeffs) {

	return true;
}
int determinePending() { //returns list of pending listingID. Pending if the state is pending and the camer
	int counter = 0;
	int pendingID;
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");

		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
		stmt = con->prepareStatement("SELECT ListingID FROM Product WHERE State = 'pending' AND CameraID IS NOT NULL LIMIT 1");
		res = stmt->executeQuery();

		if (res->next()) {
			std::cout << "Listing: " << res->getInt(1) << " pending." << std::endl;
			pendingID = res->getInt(1);
			counter++;
		}

		delete res;
		delete stmt;
		delete con;
	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
		return -1;
	}

	if (counter == 0) {
		std::cout << "No pending listings" << std::endl;
		return -1;
	}
	return pendingID;
}	//adds the listingID of currently pending listing that need there models to be generated
void setListingStateFailed(int listingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		/* Create a connection */
		driver = get_driver_instance();
		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}

		vector<uchar> buf = {};
		stmt = con->prepareStatement("UPDATE `cTeamTeamProjectDatabase`.`Product` SET `State` = 'failed' WHERE (`ListingID` = ?)");

		stmt->setInt(1, listingID);
		stmt->execute();
		std::cout << listingID << " state set to failed" << std::endl;

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
void setStateAvailable(int listingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
		stmt = con->prepareStatement("UPDATE Product SET State = 'available' WHERE ListingID = ? AND State = 'pending';");
		stmt->setInt(1,listingID);
		res = stmt->executeQuery();
		delete res;
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
void setStatePending(int listingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
			std::cout << "Set Test Listing to pending" << std::endl;

		stmt = con->prepareStatement("UPDATE Product SET State = 'pending' WHERE ListingID = ? AND State = 'available';");
		stmt->setInt(1, listingID);
		res = stmt->executeQuery();
		delete res;
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
int loadCameraDetails(int listingID, float &focusLength, float &sensorWidth) {
	int cameraID = findCameraID(listingID);
	float tempFocusLength = 0;
	float tempSensorWidth = 0;
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
		stmt = con->prepareStatement("SELECT FocusLength, SensorSize FROM CameraDetails WHERE CameraID = ? AND FocusLength IS NOT NULL AND SensorSize IS NOT NULL");
		stmt->setInt(1, cameraID);
		res = stmt->executeQuery();

		if (res->next()) {
			tempSensorWidth = (float)res->getDouble("FocusLength");
			tempFocusLength = (float)res->getDouble("SensorSize");
		}
		else {
			std::cout << "Camera Details NULL" << std::endl;
			return -1; //If no camera details are present
		}

		delete res;
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
	if (cameraID == -1) {//If the camera is -1. This shouldn't be possible but redendancy is error prevention seems prudent.
		return -1; //
	}
	else {
		focusLength = tempFocusLength;
		sensorWidth = tempSensorWidth;
		std::cout <<"Focus Length " << tempFocusLength << " Sensor Width " << tempSensorWidth << std::endl;
		return cameraID;
	}
}
bool checkCameraParameters(int listingID) { //checks for camera information in database
	if (findCameraID(listingID) != -1){
		return true;
	}
	else 
	{
		return false;
	}
}
void insertImages(vector<Mat> *image_set, int listingID, int length) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		/* Create a connection */
		driver = get_driver_instance();
		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}

		vector<uchar> buf = {};
		stmt = con->prepareStatement("INSERT INTO Image(ImageID, ImageBlob, listingID) VALUES(? , ? , ? )");

		if (image_set->empty()) {
			std::cout << "IMAGE SET NULL!" << std::endl;
		}

		for(int i = 1; i < length + 1; i++) {
			imencode(".jpg", image_set->at(i - 1), buf); //defi
			std::cout << "encoded" << std::endl;
			std::string bufstr;
			for (auto letter : buf)
				bufstr += letter;
			std::cout << "streamed" << std::endl;
			std::istringstream str(bufstr);
			stmt = con->prepareStatement("INSERT INTO Image(ImageID, ImageBlob, listingID) VALUES(? , ? , ? )");
			stmt->setInt(1, i);
			stmt->setBlob(2, &str);
			stmt->setInt(3, listingID);
			stmt->execute();
		}

		delete stmt;
		delete con;

	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
		return;
	}
} //Inserts images into DB for testing  
void loadImageSetFromDatabase(vector<Mat> *image_set, int tableID, bool isCalibration) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;
		std::string table , column;

		driver = get_driver_instance();
		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
		if (isCalibration) {
			stmt = con->prepareStatement("SELECT ImageBlob FROM CaliImage WHERE CameraID = ?");
		}
		else {
			stmt = con->prepareStatement("SELECT ImageBlob FROM Image WHERE ListingID = ? LIMIT 3");
		}
		std::cout << "Prepared Statement" << std::endl;
		stmt->setInt(1, tableID); //either a camera or listing ID
		std::cout << "Set ListingID" << std::endl;
		std::cout << "Executing Query (May be slow)" << std::endl;
		res = stmt->executeQuery();  //Selected images for a required mesh
		Mat temp;
		int flag = IMREAD_UNCHANGED; //Decode Blob Flag

		char * charbuf; 
		vector<uchar> vecChar;
		size_t blobSize = 100;
		int count = 0;

		while (res->next()) {
			std::cout << "Image Loaded: " << count << std::endl;
			std::istream *buf = res->getBlob("ImageBlob");
			buf->seekg(0, std::ios::end);
			blobSize = buf->tellg();
			buf->seekg(0, std::ios::beg);
			charbuf = new char[blobSize];
			buf->read(charbuf, blobSize);
			vecChar.assign(charbuf, charbuf + blobSize);
			temp = imdecode(vecChar, flag);
			image_set->push_back(temp);
			count = count + 1;
		}

		delete res;
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
	
	for (int i = 0 ; i < Length; i++) {
		temp = prefix + to_string(i) + ".jpg";
		temp2.push_back(imread(temp));
		std::cout << "Image Loaded: " << prefix << i << std::endl;

	}

	*image_set = temp2;
	return;
}
void loadImageSet(vector<Mat> *image_set, int Length) { 
	String temp;
	vector<Mat> temp2;

	for (int i = 0; i < Length; i++) {
		temp = to_string(i) + ".png";
		temp2.push_back(imread(temp));
		std::cout << "Image Loaded: " << i << std::endl;

	}
	*image_set = temp2;
	return;
}
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec) {
	int nfeature = 4000; //max number of feature to keep
	float scaleFactor = 1.2f; //Effectly this is a measure of precision
	int edgeThreshold = 20; //Rather clear. How much of the edge should be ignored for feature detection
	int firstLevel = 0; //Don't really understand this at the moment. Seems to be useful in optimising.
	int nlevels = 8;
	int WTA_K = 2 ; //  
	cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; //This is used to rank features to determine the best. A faster alternative is FAST_SCORE
	int patchSize = 31; //This size of patches by BRIEF when descripting and matching keypoints found by FAST
	int fastThreshold = 20; //default seems to be 20. It is the threshold in intinity different between the pixel at the center and the points round it.
	Mat mask = Mat(); //option part of detect. strange its still a needed parameter
	vector<vector<KeyPoint>> temp;
	vector<Mat> vecDescriptors;
	Mat DescriptorTemp;

	Ptr<ORB> orb = cv::ORB::create(nfeature, scaleFactor, nlevels , edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

	std::cout << "Constructing Detector. Detecting Keypoints" << std::endl;

	orb->detect(image_set, temp, mask); //detect is used as it can take a vector<Mat> rather than detectAndCompute
	*keyPointVec = temp;
	std::cout << "Detection Completed" << std::endl;
	std::cout << "Computing Descriptors" << std::endl;
	orb->compute(image_set, temp, vecDescriptors);
	std::cout << "Descriptors Computed" << std::endl;

	FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));; //This sets up flann to work with ORB discriptors 

	std::vector<vector<DMatch>> vecGoodMatches;
	std::vector<DMatch> GoodMatch;
		
	int j = 0;
	for (int i = 0 ; i < vecDescriptors.size() ; i++) {
		for (j = i ; j < vecDescriptors.size() ; j++) { //further Optimisation! 
			if (i != j) {  //Matching identical descriptors is pointless.
				std::cout << "finding matches:" << i << "," << j << std::endl;
				matcher.match(vecDescriptors[i], vecDescriptors[j], GoodMatch);
				vecGoodMatches.push_back(GoodMatch);
			}
		}
	}

	Mat imageMatches;

	//drawMatches(image_set[0], temp[0], image_set[1], temp[1], vecGoodMatches[0], imageMatches, -1, -1, vector<char>() , DrawMatchesFlags::DEFAULT);

	//namedWindow("Some OK Matches", WINDOW_NORMAL);
	
	//imshow("Some OK Matches", imageMatches);
}
bool cameraCalibration(float focusLength, float sensorWidth, int listingID) {
	bool localCalibrationExists = false;
	localCalibrationExists = checkForLocalCalibration(listingID);
	
	if (!localCalibrationExists) {
		std::cout << "Starting calibration on listing " << listingID << std::endl;
		vector<Mat> image_set;
		bool methodSuccess;
		Size imageSetSize;                               //all the images need to be of a fixed resolution
		Size chessboardSize = cv::Size(7, 9);
		int squareSize = 20; //20mm default. https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf
		vector<vector<Point2f>> cornersMatrix;
		vector<Point2f> cornersTemp;
		TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON);
		int count = 0;
		int cameraID = findCameraID(listingID);

		loadImageSetFromDatabase(&image_set, cameraID, true);
		std::cout << image_set.size() << std::endl;
		if (!(image_set.size() > 10 || image_set.size() < 8)) {
			imageSetSize = cv::Size(image_set[0].size().width, image_set[0].size().height);
			for (int i = 0; i < 10; i++) {
				std::cout << "Finding Chessboard: " << i << std::endl;
				methodSuccess = findChessboardCorners(image_set[count], chessboardSize, cornersTemp, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
				cornersMatrix.push_back(cornersTemp);
				if (methodSuccess) {
					std::cout << "Chessboard Detected" << std::endl;
				}
				else
				{
					std::cout << "Chessboard Not Detected " << std::endl;
					cornersMatrix.push_back({});
				}
				count = count + 1;
			}

			float focalPixelLength = (focusLength / sensorWidth) * image_set[0].size().width;
			vector<vector<Point3f>> worldSpacePoints(1); //the cordinates of the world space points of the calibration pattern

			calcBoardCornerPositions(chessboardSize, squareSize, worldSpacePoints[0], 1);

			std::cout << "Constructing World Space Points Array" << std::endl;
			worldSpacePoints[0][chessboardSize.width - 1].x = worldSpacePoints[0][0].x + squareSize;
			worldSpacePoints.resize(cornersMatrix.size(), worldSpacePoints[0]);

			Mat cameraMatrix;
			Mat distCoeffs = Mat::zeros(8, 1, CV_64F); //documentation notes this as an output only
			vector<Mat>  rvec, tvec;  //Output
			int flag = 0;

			calibrateCamera(worldSpacePoints, cornersMatrix, imageSetSize, cameraMatrix, distCoeffs, rvec, tvec, flag); //calibrate camera is used to calculate the camera matrix and distortion coefficient.
			return saveCameraDetails(listingID, cameraMatrix, distCoeffs);
		}
		else {
			return false;
		}
	}
	else {
		return true;
	}

}
bool MeshXYZToOBJ(int ListingID) {
	std::string cmdFrag1 = "cd C:\\Program Files\\VCG\\MeshLab && meshlabserver -i c:\\MeshingFolder\\";
	std::string cmdFrag3 = ".xyz -o c:\\MeshingFolder\\";
	std::string cmdFrag5 = ".obj -s c:\\MeshingFolder\\MeshingScript.mlx";

	int code;
	std::string listingIDString = to_string(ListingID);
	std::string cmdString = cmdFrag1 + listingIDString + cmdFrag3 + listingIDString + cmdFrag5;
	const char* cmdCString = cmdString.c_str();
	std::cout << "Meshing " << ListingID << ".xyz" << std::endl;
	code = system(cmdCString);

	printf("Mesh Lab Exit Code: %d.\n", code);

	if (code == 0) {
		std::cout << "Meshing Successful" << std::endl;

		return true;
	}
	else {
		std::cout << "Meshing failed. Error Code: " << code << std::endl;
		return false;
	}
}
int findCameraID(int listingID) {
	int cameraID = -1;
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}
		stmt = con->prepareStatement("SELECT CameraID FROM Product WHERE ListingID = ? AND CameraID IS NOT NULL");
		stmt->setInt(1, listingID);
		res = stmt->executeQuery();

		int count = 0;
		while (res->next()) {
			cameraID = res->getInt(1);
			std::cout << "Listing " << listingID << " has Camera ID " << cameraID << std::endl;
		}
		delete res;
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
	if (cameraID == -1) {
		return -1;
	}
	else {
		return cameraID;
	}
}
void loadLocalCalibration(int listingID, Mat cameraMatrix) {
	int cameraID = findCameraID(listingID);
}
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners, int patternType) { //modifided from template method provided at: https://docs.opencv.org/trunk/d4/d94/tutorial_camera_calibration.html
	corners.clear();
	switch (patternType)
	{
	case 1: //chessboard
	case 2: //Circles_Grid
		std::cout << "Constructing Corner Array" << std::endl;
		for (int i = 0; i < boardSize.height; ++i) {
			for (int j = 0; j < boardSize.width; ++j) {
				corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
				
			}
		}
		break;
	case 3: //Asymmetric_Circles_Grid
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
		break;
	default:
		break;
	}
}
void undistortAllPoints(vector<vector<KeyPoint>> &keypoints, vector<vector<KeyPoint>> &undistortedKeypoints, int listingID) {
	vector<KeyPoint> temp = {};
	Mat cameraMatrix, distortionCoeff;
	loadLocalCalibration(listingID, cameraMatrix);
	for (int i = 0 ; i < 1; i++) {
		undistortedKeypoints.push_back(temp);
		//undistortPoints(keypoints[i],undistortedKeypoints[i], cameraMatrix, distortionCoeff);
	}

}
bool triangulatePoints() {
	return true;
}
void dummyTriangulatePoints(int listingID) {
	std::cout << "Dummy Triangulate Coping bunny.xyz" << std::endl;
	String fileOutName = to_string(listingID) + ".xyz";
	
	String fileOutPath = "c:\\MeshingFolder\\" + fileOutName;

	
	ifstream source("bunny.xyz", ios::binary);
	ofstream dest(fileOutPath);

	istreambuf_iterator<char> begin_source(source);
	istreambuf_iterator<char> end_source;
	ostreambuf_iterator<char> begin_dest(dest);
	copy(begin_source, end_source, begin_dest);

	source.close();
	dest.close();
}
void dummyDensification() {
	std::cout << "Dummy Densify Called" << std::endl;
}
void generateSparcePointCloud() {

	InputArray cam1ProjectionMatrix = {};
	InputArray cam2ProjectionMatrix = {};
	InputArray cam1Points = {};
	InputArray cam2Points = {};
	OutputArray Output = {}; //outputs and array of 4D points


	triangulatePoints(cam1ProjectionMatrix, cam2ProjectionMatrix, cam1Points, cam2Points, Output);

}

//Utility function version
void objToMySQL(String filename, int listingID, int modelID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::Statement *stmt;

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}


		std::ifstream inObj(filename);
		std::string data((std::istreambuf_iterator<char>(inObj)),
			(std::istreambuf_iterator<char>()));

		std::cout << data << std::endl;

		std::string sqlInsert = "INSERT INTO Model (ModelID, ModelString, ListingID) VALUES (" + to_string(modelID);
		sqlInsert = sqlInsert + ",\"" + data;
		sqlInsert = sqlInsert + "\"," + to_string(listingID) + ")";
	

		stmt = con->createStatement();
		stmt->execute(sqlInsert.c_str());
		inObj.close();

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
bool objToMySQL(int listingID) {
	
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::Statement *stmt;
		
		String listingIDSTR = to_string(listingID);

		/* Create a connection */
		driver = get_driver_instance();

		//std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			//std::cout << "Connection Open" << std::endl;
		}

		String filename = "c:\\MeshingFolder\\" + listingIDSTR;
		filename = filename + ".obj";


		std::ifstream inObj(filename);
		std::string data((std::istreambuf_iterator<char>(inObj)),(std::istreambuf_iterator<char>()));

		std::cout << filename << " " << data.size() << std::endl;

		std::string sqlInsert = "INSERT INTO Model (ModelID, ModelString, ListingID) VALUES (" + listingIDSTR;
		sqlInsert = sqlInsert + ",\"" + data;
		sqlInsert = sqlInsert + "\"," + listingIDSTR + ")";
		stmt = con->createStatement();

		if (data.size() > 20) { //prevents empty object being inserted into the DB 
			stmt->execute(sqlInsert.c_str());
			std::cout << "Inserted " << listingID << ".obj into DB" << std::endl;
		}
		else {
			std::cout << "Object empty not inserted" << std::endl;
			setListingStateFailed(listingID);
			return false;
		}
		inObj.close();
		std::cout << "Object instream closed" << std::endl;


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
	return true;
}

