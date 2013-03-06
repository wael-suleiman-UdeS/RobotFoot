//#include "opencv\cv.h"
//#include "opencv\highgui.h"
//
//using namespace cv;
//
//int main(){
//
////create Matrix to store image
//
//Mat image;
//
//// initialize capture
//
//VideoCapture cap;
//
//cap.open(0);
//
//// create window to show image namedWindow(“window”, CV_WINDOW_AUTOSIZE);
//
//while(1){
//
//// copy webcam stream to image
//
//cap>>image;
////print image to screen 
//imshow("window", image);
////delay 33ms 
//waitKey(33);
//
//}
//
//return 0; }


/*****************************************************************************************
*  Name    : Fast object tracking using the OpenCV library                               *
*  Author  : Lior Chen <chen.lior@gmail.com>                                             *
*  Notice  : Copyright (c) Jun 2010, Lior Chen, All Rights Reserved                      *
*          :                                                                             *
*  Site    : http://www.lirtex.com                                                       *
*  WebPage : http://www.lirtex.com/robotics/fast-object-tracking-robot-computer-vision   *
*          :                                                                             *
*  Version : 1.0                                                                         *
*  Notes   : By default this code will open the first connected camera.                  *
*          : In order to change to another camera, change                                *
*          : CvCapture* capture = cvCaptureFromCAM( 0 ); to 1,2,3, etc.                  *
*          : Also, the code is currently configured to tracking RED objects.             *
*          : This can be changed by changing the hsv_min and hsv_max vectors             *
*          :                                                                             *
*  License : This program is free software: you can redistribute it and/or modify        *
*          : it under the terms of the GNU General Public License as published by        *
*          : the Free Software Foundation, either version 3 of the License, or           *
*          : (at your option) any later version.                                         *
*          :                                                                             *
*          : This program is distributed in the hope that it will be useful,             *
*          : but WITHOUT ANY WARRANTY; without even the implied warranty of              *
*          : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
*          : GNU General Public License for more details.                                *
*          :                                                                             *
*          : You should have received a copy of the GNU General Public License           *
*          : along with this program.  If not, see <http://www.gnu.org/licenses/>        *
******************************************************************************************/
#include <opencv\cvaux.h>
#include <opencv\highgui.h>
#include <opencv\cxcore.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

// Global localisation algorithm : Detection of sphere aka (GlADoS)

int main(int argc, char* argv[])
{
    // Default capture size - 640x480
    CvSize size = cvSize(640,480);
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    CvCapture* capture = cvCaptureFromCAM( 0 );
    if( !capture )
    {
            fprintf( stderr, "ERROR: capture is NULL \n" );
            getchar();
            return -1;
    }
    // Create a window in which the captured images will be presented
    cvNamedWindow( "Camera", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "EdgeDetection", CV_WINDOW_NORMAL );

	// Minimum value for the filter
	int Hvalue = 151;
	int Svalue = 80;
	int Vvalue = 181;

	// Maximum value for the filter
	int Hvalue2 = 172;
	int Svalue2 = 255;
	int Vvalue2 = 255;

	// Value for the erosion
	int erosion = 100;
	// Value for the dilatation
	int dilatation = 255;

	// Trackball created for the calibration
	cvCreateTrackbar("H minimum","EdgeDetection",&Hvalue,255);
	cvCreateTrackbar("S minimum","EdgeDetection",&Svalue,255);
	cvCreateTrackbar("V minimum","EdgeDetection",&Vvalue,255);

	// Trackball created for the calibration
	cvCreateTrackbar("H maximum","EdgeDetection",&Hvalue2,255);
	cvCreateTrackbar("S maximum","EdgeDetection",&Svalue2,255);
	cvCreateTrackbar("V maximum","EdgeDetection",&Vvalue2,255);

	// Trackball created for the erosion
	cvCreateTrackbar("Erosion","EdgeDetection",&erosion,255);
	// Trackball created for the erosion
	cvCreateTrackbar("Dilatation","EdgeDetection",&dilatation,255);

	// Image
    IplImage*  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1);
    while(true)
    {
        // Get one frame
        IplImage* frame = cvQueryFrame( capture );
        if( !frame )
        {
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }

	    // Detect a red ball
		CvScalar hsv_min = cvScalar(Hvalue, Svalue, Vvalue, 0); //	Default value : 150, 84, 130, 0
		CvScalar hsv_max = cvScalar(Hvalue2, Svalue2, Vvalue2, 0);	// Default value : 358, 256, 255, 0
		// CvScalar (a,b,c,d) où a = [0-255], b = [0-255], c = [0-255], d = 0
		// Scalar (blue component, green component, red component, [alpha component]) - FOR CV_RGB

        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);

		cvErode(thresholded,thresholded,NULL,erosion/25.5);
		cvDilate(thresholded,thresholded,NULL,dilatation/25.5);
        // Memory for hough circles
        CvMemStorage* storage = cvCreateMemStorage(0);
        // hough detector works better with some smoothing of the image
        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
                                        thresholded->height/4, 100, 50, 10, 400);
        for (int i = 0; i < circles->total; i++)
        {
            float* p = (float*)cvGetSeqElem( circles, i );
            printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                    3, CV_RGB(0,255,0), -1, 8, 0 );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                    cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
		}

		/*
		//
		//  For testing purpose only
		//
		CvPoint pt1 = cvPoint(0,0);
		CvPoint pt2 = cvPoint(100,100);
		CvPoint pt3 = cvPoint(0,100);
		CvPoint pt4 = cvPoint(100,200);

		cvRectangle(hsv_frame, pt1, pt2, hsv_min, CV_FILLED);
		cvRectangle(hsv_frame, pt3, pt4, hsv_max, CV_FILLED);
		*/

        cvShowImage( "Camera", frame ); // Original stream with detected ball overlay
        cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
        cvShowImage( "After Color Filtering", thresholded ); // The stream after color filtering
        cvReleaseMemStorage(&storage);
        // Do not release the frame!
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cvWaitKey(10) & 255) == 27 ) break;
    }
     // Release the capture device housekeeping
     cvReleaseCapture( &capture );
     cvDestroyWindow( "mywindow" );
     return 0;
   }