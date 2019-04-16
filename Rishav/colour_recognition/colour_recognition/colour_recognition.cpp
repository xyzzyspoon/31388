/*
This program performs real time object tracking using colour recognition.
We are tracking bright orange and bright green objects.
The program takes in live video feed in RGB. Then converts it into the HSV format.
Using the threshold values mentioned for each colour (bright orange and bright green), it forms a binary image.
The objects that we are detecting are white and everything else is black.
Then, we use a "Bitwise And" operation - we multiply the binary image by our original image to obtain another view where we just visualise the coloured objects.
Lastly we draw bounding boxes for our objects.
Each box will have a label that mentions the corresponding colour of the object.
The program also computes the centroids of the bounding boxes.
*/

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <string>

using namespace std;
using namespace cv;

// Window names
char original_image_window[] = "The Watcher in RGB";
char HSV_image_window[] = "The Watcher in HSV";
char orange_mask_window[] = "The Watcher - Orange Mask";
char green_mask_window[] = "The Watcher - Green Mask";
char detected_object_window[] = "The Watcher Detected the Object";

// VideoCapture class - class used to get video stream from a video camera
// When we create a VideoCapture object, we have to mention the id of the camera
// The device IDs are integers starting from 0 (first device) to the number of cameras we have
VideoCapture webcam_stream(0); // Our VideoCapture object

// Function prototypes
void live_video(Mat live_image, Mat live_image_HSV); // Function to start and end the live video feed
void orange_object(Mat live_image, Mat live_image_HSV); // Function to detect orange objects
void green_object(Mat live_image, Mat live_image_HSV); // Function to detect green objects

int main()
{	
	Mat live_image; // Matrix to store the live images from the camera
	Mat live_image_HSV; // Matrix to store the live images in HSV format
	
	live_video(live_image, live_image_HSV);

	return 0;
}

void live_video(Mat live_image, Mat live_image_HSV)
{
	int delay = 50; // Time interval (in milliseconds) at which images are taken

	// Videos are sequences of images
	// Hence we put our commands in a infinite loop
	while (1)
	{
		webcam_stream.read(live_image); // Storing our video feed
		
		cout << "I am the Watcher." << endl;
		cout << "It is my task to note all events of significance in the DTU Robocup." << endl;
		cout << "Only to note them and never interfere." << endl;
		cout << "This task brings me great joy and great sorrow." << endl;

		cout << " " << endl;

		imshow(original_image_window, live_image);
		
		cvtColor(live_image, live_image_HSV, COLOR_RGB2HSV); // Converting the RGB image taken into HSV format
		imshow(HSV_image_window, live_image_HSV);

		orange_object(live_image, live_image_HSV);
		green_object(live_image, live_image_HSV);

		// The argument of waitKey mentions the delay
		// The ineqaulity is for checking if any key is pressed
		// as waitKey() also stores the ASCII value of the key being pressed
		if (waitKey(delay) >= 0)
		{
			break;
		}
	}

	return;
}

void orange_object(Mat live_image, Mat live_image_HSV)
{	
	// PART 1 - OBJECT DETECTION

	Mat live_mask_orange; // Stores the binary mask for the orange object
	Mat detected_objects_orange; // Stores the final orange object that we detected

	// Lower and upper thresholds for the orange objects
	// MODIFY AFTER EVERY CALIBRATION
	Scalar orange_lower = Scalar(95, 70, 100);
	Scalar orange_upper = Scalar(255, 255, 255);

	inRange(live_image_HSV, orange_lower, orange_upper, live_mask_orange); // Thresholding for the orange objects

	// PART 2 - NOISE REMOVAL

	// Variable for Morphological Operation
	Mat structuring_element = getStructuringElement(0, Size(8, 8), Point(-1, -1)); // Stores the structuring element that we use for Morphological Operations

	erode(live_mask_orange, live_mask_orange, structuring_element, Point(-1, -1)); // To remove noise from our image
	imshow(orange_mask_window, live_mask_orange);

	bitwise_and(live_image, live_image, detected_objects_orange, live_mask_orange); // Bitwise AND operation will show only the orange object (if accurately calibrated)
	imshow(detected_object_window, detected_objects_orange);

	// PART 3 - DETECTING OBJECT BOUNDARIES

	// Variables for computing the contours of the thresholded images
	vector<vector<Point>> live_image_contours; // Stores each contour as a vector of points
	vector<Vec4i> hierarchy; // Vector to store information about on image topology
	int retrieval_mode = 0; // To select the contour retrieval mode
	int approximate_method = 1; // To select the contour approximation method

	findContours(live_mask_orange, live_image_contours, hierarchy, retrieval_mode, approximate_method); // Computing object boundaries

	// PART 4 - DRAWING THE BOUNDING BOXES

	// Variable for storing the points of the rectangle
	vector<Rect> bounding_rectangle(live_image_contours.size()); // To store the points for the rectangle

	// Variables for drawing the rectangle
	Scalar box_colour = Scalar(0, 255, 255); // Colour of the box - Yellow
	int line_thickness = 4; // Thickness of the lines of the box
	int line_type = 8; // 8-connected line
	int shift = 0;
	
	// Variables for drawing the lines in the rectangle
	Scalar line_colour = Scalar(0, 255, 255); // Colour of the line - Yellow

	// Varibles for writing the label for the boxes
	int font = 1;
	int font_face = 1;
	Scalar text_colour = Scalar(0, 255, 255);  // Colour of the text - Yellow
	int text_thickness = 1; // Thickness of the text
	bool bottom_left_origin = false; // Puts the label on the top left corner of the box
	string my_text = "Orange Object"; // The text that we want to use for our label

	// Variables used for drawing the circles at the bounding box corners
	int radius = 7; // Radius of the circle
	int circle_thickness = -1; // Draws a filled circle

	for (int i = 0; i < live_image_contours.size(); i++)
	{
		bounding_rectangle[i] = boundingRect(live_image_contours[i]); // Computing the points for the rectangle using our image contours
	}

	// Following is a 2D array to store the co-ordinates of the corner points of the bounding boxes
	// The format that the points are stored in are:
	// 1st row - Top left corner
	// 2nd row - Top right corner
	// 3rd row - Bottom right corner
	// 4th row - Bottom left corner
	int ordered_corners[4][2]; 

	// Drawing the bounding boxes and the correspoding labels for the objects
	for (int i = 0; i < live_image_contours.size(); i++)
	{	
		Moments moment = moments(live_image_contours[i]);
		double orange_objects_area = moment.m00;
		double threshold_contour_area = 300; // We want to draw bounding boxes for contour of specific sizes only


		if (orange_objects_area > threshold_contour_area)
		{	
			cout << "But now, by telling you about the orange objects I am breaking my most sacred vow." << endl;
			cout << "Know all of you that Mia is approaching an orange object." << endl;

			// Drawing the bounding boxes for objects above a certain contour area
			rectangle(live_image, bounding_rectangle[i].tl(), bounding_rectangle[i].br(), box_colour, line_thickness, line_type, shift);
			// Writing labels for those bounding boxes
			putText(live_image, my_text, bounding_rectangle[i].br(), font, font_face, text_colour, text_thickness, line_type, bottom_left_origin);
			// tl() - top left corner
			// br() - top right corner

			cout << "Orange Object" << endl;

			cout << "Top left corner : " << bounding_rectangle[i].tl() << " " << "Bottom right corner: " << bounding_rectangle[i].br() << "\n";

			// Finding the height and the width of the bounding box
			int box_width = bounding_rectangle[i].width;
			int box_height = bounding_rectangle[i].height;

			// Co-ordinates of the top left corner
			int tl_x = bounding_rectangle[i].tl().x;
			int tl_y = bounding_rectangle[i].tl().y;
			cout << "TL X: " << tl_x << " " << "TL Y: " << tl_y << "\n";
			Point top_left = Point(tl_x, tl_y);

			// Saving the co-ordinates
			ordered_corners[0][0] = tl_x;
			ordered_corners[0][1] = tl_y;

			Scalar tl_colour = Scalar(0, 0, 255); // The top left corner is red
			circle(live_image, Point(tl_x, tl_y), radius, tl_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the top left corner

			// Co-ordinates of the top right corner
			int tr_x = tl_x + box_width;
			int tr_y = tl_y;
			cout << "TR X: " << tr_x << " " << "TR Y: " << tr_y << "\n";
			Point top_right = Point(tr_x, tr_y);

			// Saving the co-ordinates
			ordered_corners[1][0] = tr_x;
			ordered_corners[1][1] = tr_y;

			Scalar tr_colour = Scalar(255, 0, 0); // The top right corner is blue
			circle(live_image, Point(tr_x, tr_y), radius, tr_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the top right corner

			// Co-ordinates of the bottom right corner
			int br_x = tl_x + box_width;
			int br_y = tl_y + box_height;
			cout << "BR X: " << br_x << " " << "BR Y: " << br_y << "\n";
			Point bottom_right = Point(br_x, br_y);

			// Saving the co-ordinates
			ordered_corners[2][0] = br_x;
			ordered_corners[2][1] = br_y;

			Scalar br_colour = Scalar(180, 150, 255); // The bottom right corner is pink
			circle(live_image, Point(br_x, br_y), radius, br_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the bottom right corner

			// Co-ordinates of the bottom left corner
			int bl_x = tl_x;
			int bl_y = tl_y + box_height;
			cout << "BL X: " << bl_x << " " << "BL Y: " << bl_y << "\n";
			Point bottom_left = Point(bl_x, bl_y);

			// Saving the co-ordinates
			ordered_corners[3][0] = bl_x;
			ordered_corners[3][1] = bl_y;

			Scalar bl_colour = Scalar(255, 255, 0); // The bottom right corner is green-ish blue
			circle(live_image, Point(bl_x, bl_y), radius, bl_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the bottom left corner

			// Finding the centroid using the moments() method
			// Centroid is given by the relations, Cx = M10/M00 and Cy = M01/M00
			double c_x = moment.m10/moment.m00;
			double c_y = moment.m01 / moment.m00;
			Point centroid = Point(c_x, c_y);
			cout << "Centroid x: " << c_x << " " <<"Centroid y: " << c_y << endl;
			Scalar centroid_colour = Scalar(255, 255, 255); // The centroid is white
			circle(live_image, Point(c_x, c_y), radius, centroid_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the centroid
			
			line(live_image, top_left, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect top left corner and centroid
			line(live_image, top_right, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect top right corner and centroid
			line(live_image, bottom_right, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect bottom right corner and centroid
			line(live_image, bottom_left, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect bottom left corner and centroid
		
			cout << endl;
		}
	}

	imshow(original_image_window, live_image);

	return;
}

void green_object(Mat live_image, Mat live_image_HSV)
{
	// PART 1 - OBJECT DETECTION

	Mat live_mask_green; // Stores the binary mask for the green object
	Mat detected_objects_green; // Stores the final green object that we detected

	// Lower and upper thresholds for the green objects
	Scalar green_lower = Scalar(40, 65, 35);
	Scalar green_upper = Scalar(255, 255, 255);

	inRange(live_image_HSV, green_lower, green_upper, live_mask_green); // Detecting green objects

	// PART 2 - NOISE REMOVAL

	// Variable for Morphological Operation
	Mat structuring_element = getStructuringElement(0, Size(8, 8), Point(-1, -1)); // Stores the structuring element that we use for Morphological Operations

	erode(live_mask_green, live_mask_green, structuring_element, Point(-1, -1)); // To remove noise from our image
	imshow(green_mask_window, live_mask_green);

	bitwise_and(live_image, live_image, detected_objects_green, live_mask_green); // Bitwise AND operation will show only the green object (if accurately calibrated)
	imshow(detected_object_window, detected_objects_green);

	// PART 3 - DETECTING OBJECT BOUNDARIES

	// Variables for computing the contours of the thresholded images
	vector<vector<Point>> live_image_contours; // Stores each contour as a vector of points
	vector<Vec4i> hierarchy; // Vector to store information about on image topology
	int retrieval_mode = 0; // To select the contour retrieval mode
	int approximate_method = 1; // To select the contour approximation method

	findContours(live_mask_green, live_image_contours, hierarchy, retrieval_mode, approximate_method); // Computing object boundaries

	// PART 4 - DRAWING THE BOUNDING BOXES

	// Variable for storing the points of the rectangle
	vector<Rect> bounding_rectangle(live_image_contours.size()); // To store the points for the rectangle

	// Variables for drawing the rectangle
	Scalar box_colour = Scalar(0, 255, 255); // Colour of the box - Yellow
	int line_thickness = 4; // Thickness of the lines of the box
	int line_type = 8; // 8-connected line
	int shift = 0;

	// Variables for drawing the lines in the rectangle
	Scalar line_colour = Scalar(0, 255, 255); // Colour of the line - Yellow

	// Varibles for writing the label for the boxes
	int font = 1;
	int font_face = 1;
	Scalar text_colour = Scalar(0, 255, 255);  // Colour of the text - Yellow
	int text_thickness = 1; // Thickness of the text
	bool bottom_left_origin = false; // Puts the label on the top left corner of the box
	string my_text = "Green Objects"; // The text that we want to use for our label

	// Variables used for drawing the circles at the bounding box corners
	int radius = 7; // Radius of the circle
	int circle_thickness = -1; // Draws a filled circle

	for (int i = 0; i < live_image_contours.size(); i++)
	{
		bounding_rectangle[i] = boundingRect(live_image_contours[i]); // Computing the points for the rectangle using our image contours
	}

	// Following is a 2D array to store the co-ordinates of the corner points of the bounding boxes
	// The format that the points are stored in are:
	// 1st row - Top left corner
	// 2nd row - Top right corner
	// 3rd row - Bottom right corner
	// 4th row - Bottom left corner
	int ordered_corners[4][2];

	// Drawing the bounding boxes and the correspoding labels for the objects
	for (int i = 0; i < live_image_contours.size(); i++)
	{	
		Moments moment = moments(live_image_contours[i]);
		double green_objects_area = moment.m00;
		double threshold_contour_area = 300; // We want to draw bounding boxes for contour of specific sizes only

		if (green_objects_area > threshold_contour_area)
		{	
			cout << "But now, by telling you about the green objects I am breaking my most sacred vow." << endl;
			cout << "Know all of you that Mia is approaching a green object." << endl;

			// Drawing the bounding boxes for objects above a certain contour area
			rectangle(live_image, bounding_rectangle[i].tl(), bounding_rectangle[i].br(), box_colour, line_thickness, line_type, shift);
			// Writing labels for those bounding boxes
			putText(live_image, my_text, bounding_rectangle[i].br(), font, font_face, text_colour, text_thickness, line_type, bottom_left_origin);
			// tl() - top left corner
			// br() - top right corner

			cout << "Green Object" << endl;

			cout << "Top left corner : " << bounding_rectangle[i].tl() << " " << "Bottom right corner: " << bounding_rectangle[i].br() << "\n";

			// Finding the height and the width of the bounding box
			int box_width = bounding_rectangle[i].width;
			int box_height = bounding_rectangle[i].height;

			// Co-ordinates of the top left corner
			int tl_x = bounding_rectangle[i].tl().x;
			int tl_y = bounding_rectangle[i].tl().y;
			cout << "TL X: " << tl_x << " " << "TL Y: " << tl_y << "\n";
			Point top_left = Point(tl_x, tl_y);

			// Saving the co-ordinates
			ordered_corners[0][0] = tl_x;
			ordered_corners[0][1] = tl_y;

			Scalar tl_colour = Scalar(0, 0, 255); // The top left corner is red
			circle(live_image, Point(tl_x, tl_y), radius, tl_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the top left corner

			// Co-ordinates of the top right corner
			int tr_x = tl_x + box_width;
			int tr_y = tl_y;
			cout << "TR X: " << tr_x << " " << "TR Y: " << tr_y << "\n";
			Point top_right = Point(tr_x, tr_y);

			// Saving the co-ordinates
			ordered_corners[1][0] = tr_x;
			ordered_corners[1][1] = tr_y;

			Scalar tr_colour = Scalar(255, 0, 0); // The top right corner is blue
			circle(live_image, Point(tr_x, tr_y), radius, tr_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the top right corner

			// Co-ordinates of the bottom right corner
			int br_x = tl_x + box_width;
			int br_y = tl_y + box_height;
			cout << "BR X: " << br_x << " " << "BR Y: " << br_y << "\n";
			Point bottom_right = Point(br_x, br_y);

			// Saving the co-ordinates
			ordered_corners[2][0] = br_x;
			ordered_corners[2][1] = br_y;

			Scalar br_colour = Scalar(180, 150, 255); // The bottom right corner is pink
			circle(live_image, Point(br_x, br_y), radius, br_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the bottom right corner

			// Co-ordinates of the bottom left corner
			int bl_x = tl_x;
			int bl_y = tl_y + box_height;
			cout << "BL X: " << bl_x << " " << "BL Y: " << bl_y << "\n";
			Point bottom_left = Point(bl_x, bl_y);

			// Saving the co-ordinates
			ordered_corners[3][0] = bl_x;
			ordered_corners[3][1] = bl_y;

			Scalar bl_colour = Scalar(255, 255, 0); // The bottom right corner is green-ish blue
			circle(live_image, Point(bl_x, bl_y), radius, bl_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the bottom left corner

			// Finding the centroid using the moments() method
			// Centroid is given by the relations, Cx = M10/M00 and Cy = M01/M00
			double c_x = moment.m10/moment.m00;
			double c_y = moment.m01/moment.m00;
			Point centroid = Point(c_x, c_y);
			cout << "Centroid x: " << c_x << " " << "Centroid y: " << c_y << endl;
			Scalar centroid_colour = Scalar(255, 255, 255); // The centroid is white
			circle(live_image, Point(c_x, c_y), radius, centroid_colour, circle_thickness, line_type, shift); // Drawing a filled circled for the centroid

			line(live_image, top_left, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect top left corner and centroid
			line(live_image, top_right, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect top right corner and centroid
			line(live_image, bottom_right, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect bottom right corner and centroid
			line(live_image, bottom_left, centroid, line_colour, line_thickness, line_type, shift); // Drawing line to connect bottom left corner and centroid

			cout << endl;
		}
	}

	imshow(original_image_window, live_image);
	
	return;
}