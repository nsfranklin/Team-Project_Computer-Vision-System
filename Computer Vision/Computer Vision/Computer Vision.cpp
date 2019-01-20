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

using namespace std;
using namespace cv;
using namespace ::mysqlx;

void loadImageSet(vector<Mat> *image_set, int Length);
void loadImageSet(vector<Mat> *image_set, int Length, char prefix);
void featureMatching(vector<Mat> image_set, vector<vector<KeyPoint>> *keyPointVec);
void objToMySQL(String filename);
void insertImages(vector<Mat> *image_set, int listingID, int length);
void loadImageSetFromDatabase(vector<Mat> *image_set, int ListingID);
bool determinePending(std::vector<int> &vecPending);
bool MeshXYZToOBJ(int ListingID);
void calculateCameraMatrix();
void generateSparcePointCloud();
void setStateAvailable(int ListingID);
bool checkCamera(int ListingID);
bool cameraCalibration(vector<Mat> image_set, float focusLength, float sensorWidth); //the lenght and width are in mm

int main()
{
	vector<int> vecPending;
	vector<Mat> image_array = {};
	vector<Mat> calibrationSet = {};
	vector<vector<KeyPoint>> KeyPoints;
	float focusLength = 4.4f; //TEMP value the value of my phone. Also a typical focus length for a mobile phone camera.
	float sensorWidth = 6.17f; //also the value of my mobile phone.

	while(true) {

		if (determinePending(vecPending) && checkCamera(vecPending[0])) {
		
			loadImageSetFromDatabase(&image_array, vecPending[0]); //Parameters: The image array to load them into, the Listing ID for the Image
			loadImageSet(&calibrationSet, 10 ,'c');


			if (image_array.empty())
				std::cout << "Failed to load image set" << std::endl;
			else
				std::cout << "Image Set Loaded" << std::endl;

			featureMatching(image_array, &KeyPoints);
			std::cout << "Keypoint Detection complete" << std::endl;

			cameraCalibration(calibrationSet , focusLength, sensorWidth); //para if camera is present
			
			// undistortPoints();
			// triangulatePoints();

			Mat sampleImage = image_array[2];								//sample to show keypoints
			std::cout << "Sample Image Loaded" << std::endl;
			Mat sampleWithKeyPoints;										//output image with rich keypoints
			int flags = DrawMatchesFlags::DEFAULT + DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
			drawKeypoints(sampleImage, KeyPoints[2], sampleWithKeyPoints, Scalar::all(-1), flags);
			namedWindow("image", WINDOW_NORMAL);
			imshow("image", sampleWithKeyPoints);

			std::cout << "Keypoints detected in image" << std::endl;
			std::cout << KeyPoints[0].size() << std::endl;

			setStateAvailable(vecPending[0]);
			vecPending.erase(vecPending.begin());

		}
		else {
			std::cout << "Sleeping Nothing Pending" << std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //sleeps for 5 seconds then checks again.
		}

		//break; //TEMP BREAKCLAUSE FOR TESTING
	}

	return 0;
}

void setStateAvailable(int listingID) {

}

bool checkCamera(int ListingID) { //Check if there is camera information in the DB for the users listing.
	return true;
}

bool determinePending(std::vector<int> &vecPending) {
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
		stmt = con->prepareStatement("SELECT ListingID FROM Product WHERE State = 'pending'");
		res = stmt->executeQuery();

		int count = 0;
		while (res->next()) {
			std::cout << "Listing: " << res->getInt(1) << " pending." << std::endl;
			std::cout << "enters res loop" << std::endl;
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

}//adds the listingID of currently pending listing that need there models to be generated
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
		stmt = con->prepareStatement("INSERT INTO Image(ImageID, ImageBlob, ListingID) VALUES(? , ? , ? )");

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
			stmt = con->prepareStatement("INSERT INTO Image(ImageID, ImageBlob, ListingID) VALUES(? , ? , ? )");
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
}  
void objToMySQL(String filename) {
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
		
		stmt = con->createStatement();
		stmt->executeQuery("SELECT * AS _message");

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

void loadImageSetFromDatabase(vector<Mat> *image_set, int ListingID) {
	try {
		sql::Driver *driver;
		sql::Connection *con;
		sql::PreparedStatement *stmt;
		sql::ResultSet *res;


		driver = get_driver_instance();
		std::cout << "Attempting to Connect" << std::endl;
		con = driver->connect("cteamteamprojectdatabase.csed5aholavi.eu-west-2.rds.amazonaws.com:3306", "nsfranklin", "KEigQqfLiLKy2kXzdwzN");
		con->setSchema("cTeamTeamProjectDatabase");
		if (!(con->isClosed())) {
			std::cout << "Connection Open" << std::endl;
		}
		stmt = con->prepareStatement("SELECT ImageBlob FROM Image WHERE ListingID = ? AND ImageID < 6");  //Used to restrict the number of images loaded for testing
		std::cout << "Prepared Statement" << std::endl;
		stmt->setInt(1, ListingID);
		std::cout << "Set ListingID" << std::endl;
		std::cout << "Executing Query (May be slow)" << std::endl;
		res = stmt->executeQuery();  //selected images for a required mesh
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

bool cameraCalibration(vector<Mat> image_set, float focusLength, float sensorWidth) {
	timespec start;
	timespec end;

	std::cout << "starting calibration" << std::endl;

	bool methodSuccess;
	Size imageSetSize = cv::Size(image_set[0].size().width, image_set[0].size().height);                                 //all the images need to be of a fixed resolution
	Size chessboardSize = cv::Size(7, 9);
	vector<vector<Point2f>> cornersMatrix;
	vector<Point2f> cornersTemp;
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON);
	int count = 0;

	for (int i = 0 ; i < 10 ; i++){
		std::cout << "Finding Chessboard: " << i << std::endl;
		clock_gettime(CLOCK_MONOTONIC, &start);

		methodSuccess = findChessboardCorners(image_set[count], chessboardSize, cornersTemp, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

		clock_gettime(CLOCK_MONOTONIC, &end);

		count = count + 1;


	}	
	drawChessboardCorners(image_set[9], chessboardSize, cornersMatrix[9], methodSuccess);



	float focalPixelLength = (focusLength / sensorWidth) * image_set[0].size().width;   //Focus Length in 4.4mm Sensor Width in MM 6.17 of my Sony XZ Premium

	InputArrayOfArrays imagePoints = {};
	InputOutputArray cameraMatrix = {};
	InputOutputArray distCoeffs = {}; //documentation notes this as an output only
	OutputArrayOfArrays rvec = {};  //Output
	OutputArrayOfArrays tvec = {}; //Output
	int flag = 0;
	OutputArray stdDevIntrinsics = {}; //Output
	OutputArray stdDevExtrinsics = {}; //Output
	OutputArray perViewErrors = {}; //Output


	//calculateCameraMatrix();
		
	//calibrateCamera(cornersMatrix, imagePoints,imageSetSize, cameraMatrix, distCoeffs, rvec, tvec, stdDevIntrinsics, stdDevExtrinsics, perViewErrors, flag, criteria); //calibrate camera is used to calculate the camera matrix and distortion coefficient.
		
	return false;
}

void calculateCameraMatrix() {

}

void generateSparcePointCloud() {

	InputArray cam1ProjectionMatrix = {};
	InputArray cam2ProjectionMatrix = {};
	InputArray cam1Points = {};
	InputArray cam2Points = {};
	OutputArray Output = {}; //outputs and array of 4D points


	triangulatePoints(cam1ProjectionMatrix, cam2ProjectionMatrix, cam1Points, cam2Points, Output);

}

