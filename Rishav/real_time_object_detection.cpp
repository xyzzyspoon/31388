#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

// Initialisations for the trackbars used in calibration
const int HSV_max_value = 255; // The maximum value of the HSV spectrum remains constant
// The minimum values for each spectrum of the image type
// This is what the user changes	
int h_min_value = 0;
int s_min_value = 0;
int v_min_value = 0;

// Trackbar name
char trackbar_h[] = "H";
char trackbar_s[] = "S";
char trackbar_v[] = "V";

// Window names
char original_image_window[] = "The Watcher in RGB";
char HSV_image_window[] = "The Watcher in HSV";
char thresholded_image_window[] = "The Watcher in Binary";
char denoised_image_window[] = "The Watcher Denoised";

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
void create_trackbars(Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised); // Function to create our trackbars 
void threshold_image(Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised); // Function to perform Binary Thresholding
void perform_morphology(Mat live_image_binary, Mat live_image_denoised, char operation_type); //Function to perform Morpholigical Operations for noise reduction/removal

int main()
{
	Mat live_image; // Matrix to store the live images from the camera
	Mat live_image_HSV; // Matrix to store the live images in HSV format
	Mat live_image_binary; // Matrix to store the binary thresholded images
	Mat live_image_denoised; // Matrix to store the denoised image

	live_video(live_image, live_image_HSV, live_image_binary, live_image_denoised);

	return 0;
}

void live_video(Mat live_image, Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{
	// Mentions the time interval (in milliseconds) at which images are taken
	int delay = 50;

	// Videos are sequences of images
	// Hence we put our commands in a infinite loop
	while (1)
	{
		// Once we get the video stream, we put the image into a matrix
		// Using the read function from the VideoCapture class to store images into our matrix
		webcam_stream.read(live_image);

		// Converting the RGB image taken into HSV format
		cvtColor(live_image, live_image_HSV, COLOR_RGB2HSV);

		// Displaying the image that we are storing
		imshow(original_image_window, live_image);
		// Displaying the same image in the HSV format
		imshow(HSV_image_window, live_image_HSV);

		create_trackbars(live_image_HSV, live_image_binary, live_image_denoised);

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

void create_trackbars(Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{	
	namedWindow(HSV_image_window, WINDOW_AUTOSIZE); // Creates and opens a window of a given name

	/*
	Following is an explanation of the arguments of the function createTrackbar()
	Argument 1 - label TrackbarName
	Argument 2 - Name of window where we have the trackbar
	Argument 3 - Pointer to the minimum value cut-off value of the HSV spectrum.
	We use the pointer as this value is changed by the user and we do pass by reference to keep udating the value
	Argument 4 - The Trackbar values will be in the range from 0 to alpha_slider_max (the minimum limit is always zero)
	Argument 5 - Whenever the user moves the Trackbar, the callback function on_trackbar() is called
	*/
	
	createTrackbar(trackbar_h, HSV_image_window, &h_min_value, HSV_max_value, on_trackbar);
	createTrackbar(trackbar_s, HSV_image_window, &s_min_value, HSV_max_value, on_trackbar);
	createTrackbar(trackbar_v, HSV_image_window, &v_min_value, HSV_max_value, on_trackbar);

	threshold_image(live_image_HSV, live_image_binary, live_image_denoised);

	return;
}

void threshold_image(Mat live_image_HSV, Mat live_image_binary, Mat live_image_denoised)
{	
	/*
	Following is an explanation of the arguments of the function inRange()
	Argument 1 - Input image
	Argument 2 - Minimum intensity values
	Argument 3 - Maximum intensity values
	Argument 4 - Output image
	*/
	inRange(live_image_HSV, Scalar(h_min_value, s_min_value, v_min_value), Scalar(HSV_max_value, HSV_max_value, HSV_max_value), live_image_binary);
	imshow(thresholded_image_window, live_image_binary);

	perform_morphology(live_image_binary, live_image_denoised, dilate_operation);
	perform_morphology(live_image_binary, live_image_denoised, erode_operation);

	return;
}

void perform_morphology(Mat live_image_binary, Mat live_image_denoised, char operation_type)
{	
	/*
	Following is an explanation of the arguments of the function getStructuringElement()
	Argument 1 - int shape -> Element shape that could be one of the following:
				MORPH_RECT - a rectangular structuring element (0)
				MORPH_CROSS a cross-shaped structuring element (1)
				MORPH_ELLIPSE - an elliptic structuring element (2)
				CV_SHAPE_CUSTOM - custom structuring element
	Argument 2 - Size ksize –> Size of the structuring element.
	Argument 3 - Point anchor = Point(-1,-1)
				Anchor position within the element. The default value(-1, -1) means that the anchor is at the center.
				Note that only the shape of a cross - shaped element depends on the anchor position.
				In other cases the anchor just regulates how much the result of the morphological operation is shifted.
	*/

	int shape = 0; 
	int k_size = 8; 
	int x_anchor = -1; 
	int y_anchor = -1; 

	Mat structuring_element; // To store the Structuring Element to be applied to our Morphological Operation 

	structuring_element = getStructuringElement(shape, Size(k_size, k_size), Point(x_anchor, y_anchor));

	/*
	Following is an explanation of the arguments of the function erode() and dilate()
	Argument 1 - src -> Input image
	Argument 2 - dst -> Output image
	Argument 3 - element –> structuring element used for erosion
	Argument 4 - anchor –> 
				position of the anchor within the element
				default value(-1, -1) means that the anchor is at the element center
	Argument 5 - iterations –> number of times erosion is applied
	Argument 6 - borderType –> pixel extrapolation method (see borderInterpolate() for details).
	Argument 7 - borderValue – border value in case of a constant border (see createMorphologyFilter() for details)
	*/

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

	return;
}