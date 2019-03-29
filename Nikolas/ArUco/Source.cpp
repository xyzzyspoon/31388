#include "opencv2\core.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\aruco.hpp"
#include "opencv2\calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.020f; //meters
const float arucoSquareDimension = 0.1016f; //meters : Change based on ArUCo printout size.
const Size chessboardDimensions = Size(9, 7);

void createArucoMarkers() //Creates ArUco Markers so that they are later detectable via camera.
{
	Mat outputMarker; //Stores an ArUco marker.
	Ptr <aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50); //Creates dicitionary of 50 4x4 ArUco markers.

	for (int i = 0; i < 50; i++) //Grabs individual markers from dictionary, names them, and then pushes them to our function library.
	{
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1); //Puts current marker from library into output marker.
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg"; //Creates name for current ArUco marker.
		imwrite(convert.str(), outputMarker); //Writes it to our function library under our new name.
	}
}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) //Defines known structure of our calibration chessboard.
{
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f)); //Populates corners vector with Point3f(each point has 3 axis) variables following the structure of the chessboard drawing.
		}
	}
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) //Find and visualize chessboard corners.
{
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++) //Cycles through all saved calibration images and returns found corners to pointBuf vector array.
	{
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, Size(9, 7), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //

		if (found) //If computer can see the corner grid, it writes all the x-y coordinates to our pointBuf array.
		{
			allFoundCorners.push_back(pointBuf);
		}

		if (showResults) //If true, writes corners from pointBuf to our iterator.
		{
			drawChessboardCorners(*iter, Size(9, 7), pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey(0);
		}
	}
}

void reorientRobot(vector<double> markerPose, Vec3d translationVectors, Vec3d rotationVectors, vector<double> * robotPose)
{
	Mat rotation;
	Mat cameraRotation;
	Mat cameraRotationVectors;
	Mat translationVectorsMat(3,1,DataType<double>::type);
	Mat cameraTranslationVectors(3, 1, DataType<double>::type);
	//vector<double> robotPose = {0,0,0,0,0,0};

	Rodrigues(rotationVectors, rotation); // Calculates markerPose rotation matrix.
	cameraRotation = rotation.t();  // Calculates Camera rotation matrix.

	Rodrigues(rotation, cameraRotationVectors); // Calculates Camera Rotation Vectors.
	

	translationVectorsMat.at<double>(0, 0) = translationVectors[0];
	
	translationVectorsMat.at<double>(1, 0) = translationVectors[1];
	

	translationVectorsMat.at<double>(2, 0) = translationVectors[2];
	
	cameraTranslationVectors = -rotation.t() * translationVectorsMat; // Calculates Camera Translation Vector
	*robotPose = { cameraTranslationVectors.at<double>(0, 0) + markerPose[0],  cameraTranslationVectors.at<double>(1,0) + markerPose[1],  cameraTranslationVectors.at<double>(2,0), cameraRotationVectors.at<double>(0, 0), cameraRotationVectors.at<double>(1,0), cameraRotationVectors.at<double>(2,0)};
	return;
}


void localizeRobot(Vec3d translationVectors, Vec3d rotationVectors, int markerIds, vector<double> * robotPose)
{
	//vector<double> robotPose = {0,0,0,0,0,0};
	vector<double> markerPose = {0,0,0,0,0,0};
	switch (markerIds)
	{
	case 1:
		markerPose = {3,3.5,0,0,0,0};
		reorientRobot(markerPose, translationVectors, rotationVectors, robotPose);
		break;
	case 2:
		markerPose = {5,7.6,0,0,0,0};
		reorientRobot(markerPose, translationVectors, rotationVectors, robotPose);
		break;
	case 3:
		markerPose = {7.6,3.6,0,0,0,0};
		reorientRobot(markerPose, translationVectors, rotationVectors, robotPose);
		break;
	case 4:
		markerPose = {0,0,0,0,0,0};
		reorientRobot(markerPose, translationVectors, rotationVectors, robotPose);
		break;
	case 5:
		markerPose = {5.1,9.2,0,0,0,0};
		reorientRobot(markerPose, translationVectors, rotationVectors, robotPose);
		break;
	default:
		break;
	}
	return;
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimensions)
{
	Mat frame; //Webcam feed.
	vector<int> markerIds; //ArUco markers.
	vector<vector<Point2f>> markerCorners, rejectedCandidates; //x,y positions of found square intersections on image.
	vector<double> robotPose = { 0,0,0,0,0,0 };

	aruco::DetectorParameters parameters; //

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50); //Creates dictionary of ArUco markers for detection.

	VideoCapture vid(0); //Opens our webcam.
	if (!vid.isOpened()) //Breaks function if video fails.
	{
		return -1;
	}
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
	vector<Vec3d> rotationVectors, translationVectors; //3D image translations to 2D.

	while (true)
	{
		if (!vid.read(frame)) //Breaks loop if camera fails.
			break;

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds); //Finds markers in camera feed and relates them to an ID in our library.
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors); //Having seen marker, function attempt to determine 3D pose of our marker.

		for (int i = 0; i < markerIds.size(); i++) //Draws a 3D axis over top of each ArUco marker in the frame, and localizes robot in world frame.
		{
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
			//cout << translationVectors[i] << rotationVectors[i] << markerIds[i] << "\n"<< endl;
			localizeRobot(translationVectors[i], rotationVectors[i], markerIds[i], &robotPose); //STILL REQUIRES A VARIABLE TO WRITE ANSWER TO!!!!
			cout << "Robot Pose Relative to Camera: \n" ;
			for (int i = 0; i < robotPose.size(); i++)
			{
				 cout << robotPose[i] << "\t";
			}
			cout << endl;
		}
		imshow("Webcam", frame);
		if (waitKey(30) >= 0) //Prevents infinite errors that may occur.
			break;
	}
	return 1;
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)// Takes "found" calibration images and calibrates camera using them.
{
	vector<vector<Point2f>> checkerboardImageSpacePoints; //Found corner intersections on calibration images.
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false); //Finds all calibration corners on a verified passed image of checkerboard.

	vector<vector<Point3f>> worldSpaceCornerPoints(1); //Point3f equivalents of our found calibration Point2f intersections.

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]); //Defines known structure of our calibration chessboard for our Point3f vector equivalent.
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]); //Resizes our Point3f vector grid to the same size as our Point2f vector grid.

	vector<Mat> rVectors, tVectors;
	distanceCoefficients = Mat::zeros(8, 1, CV_64F);
	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors); //Using the Point3f and Point2f grids, this function populates cameraMatrix to tVectors with information for use in future functions.
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) //Saves information of our calibration (cameraMatrix, distanceCoefficients) to a file that we name. 
{
	ofstream outStream(name);
	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) //Saving cameraMatrix values to chosen file.
		{
			for (int c = 0; c < columns; c++)
			{
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) //Saving distanceCoefficients values to chosen file.
		{
			for (int c = 0; c < columns; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}
		outStream.close(); //Stop writing to file.
		return true;
	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) //Takes saved text file and extract calibration values found during saveCameraCalibration() to cameraMatrix and distanceCoefficients.
{
	ifstream inStream(name);
	if (inStream)
	{
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++) // Writes camera calibration value from file to cameraMatrix.
		{
			for (int c = 0; c < columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}
		//Distance Coefficients
		inStream >> rows;
		inStream >> columns;

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++) // Writes camera calibration value from file to distanceCoefficients.
		{
			for (int c = 0; c < columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << "\\in";
			}
		}
		inStream.close();
		return true;
	}
	return false;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients) //Goes through the procedure of calling functions and taking in data manually to write out a calibration file.
{
	Mat frame; //Video feed.
	Mat drawToFrame;

	vector<Mat> savedImages; //Saved calibration images.
	vector<vector<Point2f>> markerCorners, rejectedCandidates; //Corners of our chessboard calibration.

	VideoCapture vid(0); //Begins video loop.
	if (!vid.isOpened())
	{
		return;
	}

	int framesPerSecond = 20;
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	while (true)
	{
		if (!vid.read(frame)) //If video fails, end program.
			break;

		vector<Vec2f> foundPoints;
		bool found = false;
		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK); //Searches image for chessboard corners and writes to foundPoints.
		frame.copyTo(drawToFrame); //Copies frame video feed to drawToFrame video feed.
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found); //Displays found corners visually over "frame" footage.
		if (found) //Show regular or altered video depending on if corners found.
			imshow("Webcam", drawToFrame);
		else
			imshow("Webcam", frame);
		char character = waitKey(1000 / framesPerSecond);

		switch (character)
		{
		case ' ':
			//saving image
			if (found)
			{
				Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
			}
			break;
		case 13:
			//start calibraton
			if (savedImages.size() > 25)
			{
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
				saveCameraCalibration("Camera Calibration Matrix", cameraMatrix, distanceCoefficients);
			}
			break;
		case 27:
			return;
			break;
		}
	}
	return;
}



int main(int argv, char** argc)
{

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;

	//createArucoMarkers();
	//cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
	loadCameraCalibration("Camera Calibration Matrix", cameraMatrix, distanceCoefficients);
	startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);

	return 0;
}