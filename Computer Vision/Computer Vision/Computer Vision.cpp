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

using namespace std;
using namespace cv;
using namespace ::mysqlx;
using namespace boost;

void loadImageSet(vector<Mat> *image_set, int Length);
void loadImageSet(vector<Mat> *image_set, int Length, char prefix);
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec);
void objToMySQL(String filename, int ListingID, int ModelID);
void insertImages(vector<Mat> *image_set, int listingID, int length);
void loadImageSetFromDatabase(vector<Mat> *image_set, int ListingID, bool isCalibration);
bool determinePending(std::vector<int> &vecPending);
bool MeshXYZToOBJ(int ListingID);
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners, int patternType);
void generateSparcePointCloud();
void setStateAvailable(int ListingID);
bool checkCameraParameters(int listingID);
bool cameraCalibration(float focusLength, float sensorWidth, int listingID); //the lenght and width are in mm
bool triangulatePoints();
bool checkLocalCalibration(int cameraID);
void loadLocalCalibration(int listingID, Mat cameraMatrix);
int findCameraID(int listingID);
void setStatePending(int listingID);
void undistortAllImages();

int main()
{

	vector<int> vecPending;
	vector<Mat> image_array = {};
	vector<vector<KeyPoint>> KeyPoints;
	vector<vector<KeyPoint>> undistortedKeyPoints;
	float focusLength = 4.4f; //TEMP value the value of my phone. Also a typical focus length for a mobile phone camera.
	float sensorWidth = 6.17f; //also the value of my mobile phone.

	while(true) {

		if (determinePending(vecPending)) {

			cameraCalibration(focusLength, sensorWidth, vecPending[0]);//checks for local calibration. Then calibrates if not present.
			loadImageSetFromDatabase(&image_array, vecPending[0], false); //Parameters: The image array to load them into, the Listing ID for the Image

			if (image_array.empty())
				std::cout << "Failed to load image set" << std::endl;
			else
				std::cout << "Image Set Loaded" << std::endl;

			undistortAllImages(&image_array);

			featureMatching(image_array, &KeyPoints);
			std::cout << "Keypoint Detection complete" << std::endl;

			Mat sampleImage = image_array[2];								//sample to show keypoints

			Mat sampleWithKeyPoints;										//output image with rich keypoints
			int flags = DrawMatchesFlags::DEFAULT + DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
			drawKeypoints(sampleImage, KeyPoints[2], sampleWithKeyPoints, Scalar::all(-1), flags);
			namedWindow("image", WINDOW_NORMAL);
			imshow("image", sampleWithKeyPoints);

			//undistortAllPoints(KeyPoints, undistortedKeyPoints , vecPending[0]); //undistort point has a bug so undistorting the image will be used at an earlier point 
			dummyTriangulatePoints();

			std::cout << "Keypoints detected in image" << std::endl;
			std::cout << KeyPoints[0].size() << std::endl;

			//setStateAvailable(vecPending[0]);
			vecPending.erase(vecPending.begin());

		}
		else {
			std::cout << "Sleeping Nothing Pending" << std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //sleeps for 5 seconds then checks again.
		}

		break; //TEMP BREAKCLAUSE FOR TESTING
	}

	cv::waitKey(0);

	return 0;
}
void setStateAvailable(int listingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
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

void undistortAllImages(vector<Mat> *ImageSet) {

}

void setStatePending(int listingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
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
bool checkLocalCalibration(int cameraID) {//checks for local calibration files for a camera. creates dir if not present
	if (CreateDirectory(dirs , NULL) || ERROR_ALREADY_EXISTS == GetLastError())
	{
		return false; //If you need to make the dir then there is no saved data.
	}
	else
	{
		String filePath = "../camera_calibrations/" + to_string(cameraID) + ".txt";
		ifstream inf(filePath);
		if (!inf) {
			return false;
		}
		else {
			return true;
		}
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
bool determinePending(std::vector<int> &vecPending) { //returns list of pending listingID. Pending if the state is pending and the camer
	int pendingID;
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
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
		stmt = con->prepareStatement("SELECT ListingID FROM Product WHERE State = 'pending' AND CameraID IS NOT NULL");
		res = stmt->executeQuery();

		int count = 0;
		while (res->next()) {
			std::cout << "Listing: " << res->getInt(1) << " pending." << std::endl;
			pendingID = res->getInt(1);

			if (vecPending.empty()) {
				vecPending.push_back(pendingID);
				std::cout << res->getInt(1) << std::endl;
			}
			else if (pendingID > vecPending.back()) {
				vecPending.push_back(pendingID);
			}
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

	if (vecPending.empty()) {
		return false;
	}
	return true;
}	//adds the listingID of currently pending listing that need there models to be generated
void insertImages(vector<Mat> *image_set, int listingID, int length) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		/* Create a connection */
		driver = get_driver_instance();
		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		/* Connect to the MySQL test database */
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
		}

		vector<uchar> buf = {};
		stmt = con->prepareStatement("INSERT INTO CaliImage(ImageID, ImageBlob, CameraID) VALUES(? , ? , ? )");

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
			stmt = con->prepareStatement("INSERT INTO CaliImage(ImageID, ImageBlob, CameraID) VALUES(? , ? , ? )");
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
		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
		}
		if (isCalibration) {
			stmt = con->prepareStatement("SELECT ImageBlob FROM CaliImage WHERE CameraID = ?");
		}
		else {
			stmt = con->prepareStatement("SELECT ImageBlob FROM Image WHERE ListingID = ? and ImageID < 6");
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
		temp = to_string(i) + ".jpg";
		temp2.push_back(imread(temp));
		std::cout << "Image Loaded: " << i << std::endl;

	}
	*image_set = temp2;
	return;
}
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec) {
	int nfeature = 500; //max number of feature to keep
	float scaleFactor = 1.2f; //Effectly this is a measure of precision
	int edgeThreshold = 20; //Rather clear. How much of the edge should be ignored for feature detection
	int firstLevel = 0; //Don't really understand this at the moment. Seems to be useful in optimising.
	int nlevels = 8;
	int WTA_K = 2 ; //  ''
	int scoreType = cv::ORB::HARRIS_SCORE; //This is used to rank features to determine the best. A faster alternative is FAST_SCORE
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
	FlannBasedMatcher matcher;
	std::vector<vector<DMatch>> vecGoodMatches;
	std::vector<DMatch> GoodMatch;
		
	int j = 0;
	for (int i = 0 ; i < vecDescriptors.size() ; i++) {
		for (j = i ; j < vecDescriptors.size() ; j++) { //further Optimisation! 
			if (i != j) {  //Matching identical descriptors is pointless.
				std::cout << "finding matches:" << i << "," << j << std::endl;
				if (vecDescriptors[i].type() != CV_32F) {
					vecDescriptors[i].convertTo(vecDescriptors[i], CV_32F);
				}
				if (vecDescriptors[j].type() != CV_32F) {
					vecDescriptors[j].convertTo(vecDescriptors[j], CV_32F);
				}
				matcher.match(vecDescriptors[i], vecDescriptors[j], GoodMatch);
				vecGoodMatches.push_back(GoodMatch);
			}
		}
	}

	Mat imageMatches;

	drawMatches(image_set[0], temp[0], image_set[1], temp[1], vecGoodMatches[0], imageMatches, -1, -1, vector<char>() , 2);

	namedWindow("Some OK Matches", WINDOW_NORMAL);
	
	imshow("Some OK Matches", imageMatches);
}
bool cameraCalibration(float focusLength, float sensorWidth, int listingID) {

	std::cout << "starting calibration" << std::endl;
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
	if (!checkLocalCalibration(cameraID)) {
		loadImageSetFromDatabase(&image_set, cameraID, true);
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
		float focalPixelLength = (focusLength / sensorWidth) * image_set[0].size().width;   //Focus Length in 4.4mm Sensor Width in MM 6.17 of my Sony XZ Premium
		vector<vector<Point3f>> worldSpacePoints(1); //the cordinates of the world space points of the calibration pattern
		calcBoardCornerPositions(chessboardSize, squareSize, worldSpacePoints[0], 1);
		std::cout << "Constructing World Space Points Array" << std::endl;
		worldSpacePoints[0][chessboardSize.width - 1].x = worldSpacePoints[0][0].x + squareSize;
		worldSpacePoints.resize(cornersMatrix.size(), worldSpacePoints[0]);

		Mat cameraMatrix;
		Mat distCoeffs = Mat::zeros(8, 1, CV_64F); //documentation notes this as an output only
		vector<Mat>  rvec, tvec;  //Output
		int flag = 0;

		//OutputArray stdDevIntrinsics = {}; //Output
		//OutputArray stdDevExtrinsics = {}; //Output
		//OutputArray perViewErrors = {}; //Output

		//calculateCameraMatrix();

		calibrateCamera(worldSpacePoints, cornersMatrix, imageSetSize, cameraMatrix, distCoeffs, rvec, tvec, flag); //calibrate camera is used to calculate the camera matrix and distortion coefficient.

		ofstream calibrationFile;
		std::string fileName = to_string(cameraID) + ".txt";
		calibrationFile.open(fileName);


	}
	else {
	}
	return false;
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
	cin.ignore();
	if (code == 0) {
		return true;
	}
	else {
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

		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		/* Connect to the MySQL test database */
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
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
bool dummyTriangulatePoints() {

}

bool dummyGenerateSparcePointCloud() {
	
}

void generateSparcePointCloud() {

	InputArray cam1ProjectionMatrix = {};
	InputArray cam2ProjectionMatrix = {};
	InputArray cam1Points = {};
	InputArray cam2Points = {};
	OutputArray Output = {}; //outputs and array of 4D points


	triangulatePoints(cam1ProjectionMatrix, cam2ProjectionMatrix, cam1Points, cam2Points, Output);

}
void objToMySQL(String filename, int listingID, int modelID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::Statement *stmt;

		/* Create a connection */
		driver = get_driver_instance();

		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		/* Connect to the MySQL test database */
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
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

