/*
This program serves as a framework for real-time object detection.
First, it takes in an RGB image. Then converts it into an HSV image.
Then we perform Thresholding on our HSV image using values obtained from trackbars.
On performing Thresholding we get the objects that we want to detect in white. The rest of the image should be black.
The rest of the noise in the image is reduced/gotten rid of using Morphological operations called Erosion and Dilation.
After that we find the object boundaries of our thresholded objects using Contouring.
This is followed by drawing contours for those thresholded objects.
In the final step we draw squares for our objects and label them.
*/

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2\core.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\calib3d.hpp"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

// Initialisations for the trackbars used in calibration
const int HSV_max_value = 255; // The maximum value of the HSV spectrum remains constant
// The minimum values for each spectrum of the image type
// This is what the user changes	
int h_min_value = 0;
int s_min_value = 0;
int v_min_value = 0;
// The maximum values for each spectrum of the image type
// This is what the user changes	
int h_max_value = 255;
int s_max_value = 255;
int v_max_value = 255;

// Trackbar name
char trackbar_h_min[] = "H Min. Val";
char trackbar_h_max[] = "H Max. Val";
char trackbar_s_min[] = "S Min. Val";
char trackbar_s_max[] = "S Max. Val";
char trackbar_v_min[] = "V Min. Val";
char trackbar_v_max[] = "V Max. Val";

// Window names
char original_image_window[] = "The Watcher in RGB";
char HSV_image_window[] = "The Watcher in HSV";
char thresholded_image_window[] = "The Watcher in Binary";
char denoised_image_window[] = "The Watcher Denoised";
char contoured_image_window[] = "The Watcher Contoured";
char bounded_box_window[] = "The Watcher Bounded Box";

// Variables for Morphological Operations
char erode_operation = 'e';
char dilate_operation = 'd';

// VideoCapture class - class used to get video stream from a video camera
// When we create a VideoCapture object, we have to mention the id of the camera
// The device IDs are integers starting from 0 (first device) to the number of cameras we have
VideoCapture webcam_stream(0); // Our VideoCapture object

// Function prototypes
void live_video(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised); // Function to start and end the live video feed
static void on_trackbar(int, void*); // Callback function using an anonymous inner class listener in Java
void create_trackbars(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised); // Function to create our trackbars 
void threshold_image(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised); // Function to perform Binary Thresholding
void perform_morphology(Mat live_image, Mat live_image_binary, Mat live_image_denoised, char operation_type); //Function to perform Morpholigical Operations for noise reduction/removal
void contouring(Mat live_image, Mat live_image_denoised); // Function to find and draw contours on our live feed
void draw_bounding_box(Mat live_image, vector<vector<Point>> live_image_contours);  // Functions to draw boxes for the detected objects

int main()
{
	Mat live_image; // Matrix to store the live images from the camera
	Mat live_image_HSV; // Matrix to store the live images in HSV format
	Mat live_image_binary; // Matrix to store the binary thresholded images
	Mat live_image_denoised; // Matrix to store the denoised thresholded image

	live_video(live_image, live_image_HSV, live_image_binary, live_image_denoised);

	return 0;
}

void live_video(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{
	int delay = 50; // Time interval (in milliseconds) at which images are taken

	// Videos are sequences of images
	// Hence we put our commands in a infinite loop
	while (1)
	{
		webcam_stream.read(live_image); // Storing our video feed

		cvtColor(live_image, live_image_HSV, COLOR_RGB2HSV); // Converting the RGB image taken into HSV format

		imshow(original_image_window, live_image);
		
		imshow(HSV_image_window, live_image_HSV);

		create_trackbars(live_image, live_image_HSV, live_image_binary, live_image_denoised);

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

static void on_trackbar(int, void*)
{

}

void create_trackbars(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{
	namedWindow(HSV_image_window, WINDOW_AUTOSIZE); // Creates and opens a window of a given name

	// Using the built-in function createTrackbar() to create our trackbars for calibration
	createTrackbar(trackbar_h_min, HSV_image_window, &h_min_value, HSV_max_value, on_trackbar); 
	createTrackbar(trackbar_h_max, HSV_image_window, &h_max_value, HSV_max_value, on_trackbar);

	createTrackbar(trackbar_s_min, HSV_image_window, &s_min_value, HSV_max_value, on_trackbar);
	createTrackbar(trackbar_s_max, HSV_image_window, &s_max_value, HSV_max_value, on_trackbar);

	createTrackbar(trackbar_v_min, HSV_image_window, &v_min_value, HSV_max_value, on_trackbar);
	createTrackbar(trackbar_v_max, HSV_image_window, &v_max_value, HSV_max_value, on_trackbar);

	threshold_image(live_image, live_image_HSV, live_image_binary, live_image_denoised);

	return;
}

void threshold_image(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{
	// Thresholding our images using the built-in function inRange()
	inRange(live_image_HSV, Scalar(h_min_value, s_min_value, v_min_value), Scalar(HSV_max_value, HSV_max_value, HSV_max_value), live_image_binary);
	imshow(thresholded_image_window, live_image_binary);

	perform_morphology(live_image, live_image_binary, live_image_denoised, erode_operation);
	//perform_morphology(live_image, live_image_binary, live_image_denoised, dilate_operation);

	return;
}

void perform_morphology(Mat live_image, Mat live_image_binary, Mat live_image_denoised, char operation_type)
{	
	// Variables for selecting our structuring element
	int shape = 0; // Rectangular structuring element
	int k_size = 8; // Size of the structuring element
	int x_anchor = -1; // x location of the anchor
	int y_anchor = -1; // y location of the anchor

	Mat structuring_element; // To store the Structuring Element to be applied to our Morphological Operation 

	// Using the built-in function getStructuringElement() to get the Morphological filter for our Erosion and Dilation operations
	structuring_element = getStructuringElement(shape, Size(k_size, k_size), Point(x_anchor, y_anchor)); 

	// Performs Erosion -> Reduces white spots/lines and increases the amount of black in the thresholded image
	if (operation_type == 'e')
	{
		erode(live_image_binary, live_image_denoised, structuring_element, Point(x_anchor, y_anchor));
	}
	// Performs Dilation -> Reduces black spots/lines and increases the amount of white in the thresholded image
	else if (operation_type == 'd')
	{
		dilate(live_image_binary, live_image_denoised, structuring_element, Point(x_anchor, y_anchor));
	}

	imshow(denoised_image_window, live_image_denoised);

	contouring(live_image, live_image_denoised);

	return;
}

void contouring(Mat live_image, Mat live_image_denoised)
{	
	// Variables for computing the contours of the thresholded images
	vector<vector<Point>> live_image_contours; // Stores each contour as a vector of points
	vector<Vec4i> hierarchy; // Vector to store information about on image topology
	int retrieval_mode = 0; // To select the contour retrieval mode
	int approximate_method = 1; // To select the contour approximation method

	// Variables for drawing the contours
	int contour_idx = -1; // Parameter indicating a contour to draw.If it is negative, all the contours are drawn
	Scalar contour_colour = Scalar(255, 0, 0); // Color of the contours - Blue
	int thickness = 2; // Contour thickness
	int line_type = 4; // 4-connected line.

	findContours(live_image_denoised, live_image_contours, hierarchy, retrieval_mode, approximate_method); // Computing the countours from our denoised binary image

	drawContours(live_image, live_image_contours, contour_idx, contour_colour, thickness, line_type, hierarchy); // Drawing the contours for our thresholded objects

	imshow(contoured_image_window, live_image);

	draw_bounding_box(live_image, live_image_contours);

	return;
}

void draw_bounding_box(Mat live_image, vector<vector<Point>> live_image_contours)
{
	// Variable for storing the points of the rectangle
	vector<Rect> bounding_rectangle(live_image_contours.size()); // To store the points for the rectangle

	// Variables for drawing the rectangle
	Scalar box_colour = Scalar(0, 255, 255); // Colour of the box - Yellow
	int line_thickness = 4; // Thickness of the lines of the box
	int line_type = 8; // 8-connected line
	int shift = 0;

	// Varibles for writing the label for the boxes
	int font = 1;
	int font_face = 1;
	Scalar text_colour = Scalar(0, 255, 255);  // Colour of the text - Yellow
	int text_thickness = 1; // Thickness of the text
	bool bottom_left_origin = false; // Puts the label on the top left corner of the box

	string my_text = "Objects"; // The text that we want to use for our label

	for (int i = 0; i < live_image_contours.size(); i++)
	{
		bounding_rectangle[i] = boundingRect(live_image_contours[i]); // Computing the points for the rectangle using our image contours
	}

	for (int i = 0; i < live_image_contours.size(); i++)
	{
		rectangle(live_image, bounding_rectangle[i].tl(), bounding_rectangle[i].br(), box_colour, line_thickness, line_type, shift);
		putText(live_image, my_text, bounding_rectangle[i].tl(), font, font_face, text_colour, text_thickness, line_type, bottom_left_origin);
		// tl() - top left corner
		// br() - top right corner
	}

	imshow(bounded_box_window, live_image);

	return;
}