#include "test.h"
#include "Camera.h"
#include "ColorFinder.h"

using namespace cv;
using namespace std;

void test::testPrimaire(){

	// initialize capture
    CvCapture* capture = cvCaptureFromCAM( 0 );
    if(!capture)
    {
        fprintf( stderr, "ERROR: capture is NULL \n" );
        getchar();
		exit;
    }
	// create window to show image namedWindow(“window”, CV_WINDOW_AUTOSIZE);
	cvNamedWindow( "window", CV_WINDOW_AUTOSIZE );

	while(true){
		// Get one frame
        IplImage* frame = cvQueryFrame(capture);
        if( !frame )
        {
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }
	//print image to screen 
	cvShowImage("window", frame);
	//delay 33ms 
	waitKey(33);

	if( (cvWaitKey(10) & 255) == 27 ) break;

	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("window");

}


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

// Global Localisation Algorithm : Detection of a Sphere aka (GlADoS)

void test::testModularite(){
	bool debug = true;

	CvPoint* ballPosition;
	if (!Camera::getInstance().initialize())
	{return;}
	ColorFinder *finder = new ColorFinder(215, 40, 163, 103);

	if (debug)
	{
		cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("HSV", CV_WINDOW_AUTOSIZE);
	}

	while(true)
	{
		Camera::getInstance().captureFrame();

		if (debug)
		{
			cvShowImage("RGB", Camera::getInstance().getFrame(Camera::ColorSpace::RGB));
			cvShowImage("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
		}

		ballPosition = finder->getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV));

		if (debug)
		{
			cvCircle(Camera::getInstance().getFrame(Camera::ColorSpace::RGB), cvPoint(cvRound(ballPosition->x), ballPosition->y),
				3, CV_RGB(0,255,0), -1, 8, 0 );

			cvShowImage("RGB", Camera::getInstance().getFrame(Camera::ColorSpace::RGB));
			cvShowImage("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
		}

		if( (cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
}

void test::trackBall()
{
    // Default capture size - 640x480
    CvSize size = cvSize(640,480);
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    CvCapture* capture = cvCaptureFromCAM( 1 );
    if( !capture )
    {
            fprintf( stderr, "ERROR: capture is NULL \n" );
            getchar();
			exit;
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
     cvDestroyAllWindows();
   }

   int test::cameraClibration(int argc, char* argv[]) {
	help();
	Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    for(int i = 0;;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }
      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;
		//Mat pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found)                // If done with success,
		{
                // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, CV_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }


    return 0;
   }