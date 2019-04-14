/**
  * Lane_Detection_Using_Canny_Algorithm
  * 
  */
#pragma once

#include "opencv2/opencv.hpp" 
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;

#define RIO_X_DIVISION_RATION 0.25
#define RIO_Y_DIVISION_RATION 0.5

int width = 0;
int height = 0;



Mat grab_picture()
{
	//Initialise the image as a matrix container
	Mat src;
	raspicam::RaspiCam_Cv capture; // initialise the raspicam object

	capture.open(); // activate the raspicam object
	capture.grab(); //grab the scene using raspicam
	capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
	width = src.size().width;
	height = src.size().height;

	capture.release();
	return src;
}

Mat* get_top_RIO(Mat src)
{
	Rect l_side(0, 0, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
	Rect middle(width * RIO_X_DIVISION_RATION, 0, width * (1 - 2 * RIO_X_DIVISION_RATION), height * (1 - RIO_Y_DIVISION_RATION));
	Rect r_side(width * 0.75, 0, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
		
	
	Mat top[3];
	
	top[0] = src(l_side);
	top[1] = src(middle);
	top[2] = src(r_side);
		
	return top;
}

Mat* get_bottom_RIO(Mat src)
{
	Rect l_side_b(0, height * RIO_Y_DIVISION_RATION, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
	Rect middle_b(width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION, width * (1 - 2 * RIO_X_DIVISION_RATION), height * RIO_Y_DIVISION_RATION);
	Rect r_side_b(width * 0.75, height * RIO_Y_DIVISION_RATION, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
	
	Mat bottom[3];

	bottom[0] = src(l_side_b);
	bottom[1] = src(middle_b);
	bottom[2] = src(r_side_b);
	
	return bottom;
}

Mat* get_bicolor_bottom(Mat* bottom)
{
	cvtColor(bottom[0], bottom[0], CV_BGR2GRAY);
			
	cvtColor(bottom[1], bottom[1], CV_BGR2GRAY);
		
	cvtColor(bottom[2], bottom[2], CV_BGR2GRAY);

	int lowThreshold = 35;
	int ratio = 3;
	int kernel_size = 3;
	
	for(int i  = 0; i < 3; i++)
	{
		
		/// Reduce noise with a kernel 3x3
		blur(bottom[i], bottom[i], Size(3,3) );
	
		/// Canny detector
		Canny( bottom[i], bottom[i], lowThreshold, lowThreshold*ratio, kernel_size );
		
	}
	
	return bottom;	
}

int main()
{

	//Initialise the image as a matrix container
	Mat src;
	raspicam::RaspiCam_Cv capture; // initialise the raspicam object

	capture.open(); // activate the raspicam object

	while (1)
	{
		capture.grab(); //grab the scene using raspicam
		capture.retrieve(src); // retrieve the captured scene as an image and store it in matrix container
		int width = src.size().width;
		int height = src.size().height;
		//cout << width << " " << height << endl;
		
		Mat top[3];
		Mat bottom[3];
		
		Rect l_side(0, 0, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
		Rect middle(width * RIO_X_DIVISION_RATION, 0, width * (1 - 2 * RIO_X_DIVISION_RATION), height * (1 - RIO_Y_DIVISION_RATION));
		Rect r_side(width * 0.75, 0, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
		
		Rect l_side_b(0, height * RIO_Y_DIVISION_RATION, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
		Rect middle_b(width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION, width * (1 - 2 * RIO_X_DIVISION_RATION), height * RIO_Y_DIVISION_RATION);
		Rect r_side_b(width * 0.75, height * RIO_Y_DIVISION_RATION, width * RIO_X_DIVISION_RATION, height * RIO_Y_DIVISION_RATION);
		
		
		top[0] = src(l_side);
		top[1] = src(middle);
		top[2] = src(r_side);
		
		bottom[0] = src(l_side_b);
		bottom[1] = src(middle_b);
		bottom[2] = src(r_side_b);
		
		//Converting images to gray scale
		Mat temp;
		cvtColor(bottom[0], bottom[0], CV_BGR2GRAY);
		//bottom[0] = temp;
				
		cvtColor(bottom[1], bottom[1], CV_BGR2GRAY);
		//bottom[1] = temp;
		
		cvtColor(bottom[2], bottom[2], CV_BGR2GRAY);
		//bottom[2] = temp;
		
		int lowThreshold = 35;
		int ratio = 3;
		int kernel_size = 3;
		
		for(int i  = 0; i < 3; i++)
		{
			
			/// Reduce noise with a kernel 3x3
			blur(bottom[i], bottom[i], Size(3,3) );
			//bottom[i] = temp;

			/// Canny detector
			Canny( bottom[i], bottom[i], lowThreshold, lowThreshold*ratio, kernel_size );
		
		}
		
		
		
		
		//namedWindow("Src", WINDOW_AUTOSIZE);
		//imshow("Src", src);
		//waitKey(10);
		
		
		//namedWindow("Top[0]", WINDOW_AUTOSIZE);
		//imshow("Top[0]", top[0]);
		//waitKey(10);
		
		//namedWindow("Top[1]", WINDOW_AUTOSIZE);
		//imshow("Top[1]", top[1]);
		//waitKey(10);
		
		//namedWindow("Top[2]", WINDOW_AUTOSIZE);
		//imshow("Top[2]", top[2]);
		//waitKey(10);
		
		//namedWindow("bottom[0]", WINDOW_AUTOSIZE);
		imshow("bottom[0]", bottom[0]);
		waitKey(10);
		
		//namedWindow("bottom[1]", WINDOW_AUTOSIZE);
		imshow("bottom[1]", bottom[1]);
		waitKey(10);
		
		//namedWindow("bottom[2]", WINDOW_AUTOSIZE);
		imshow("bottom[2]", bottom[2]);
		waitKey(10);
	
	}
	capture.release(); // release the raspicam frame grabbing 
	return 0;
}

//// Compile :
/* g++ Edge_Detection_Using_RaspiCam.cpp -o edgedetect_opencv -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv` */

