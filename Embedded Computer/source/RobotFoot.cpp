#include "ImageProcessing\Camera.h"
#include "ImageProcessing\ColorFinder.h"
#include "ImageProcessing\ObjectTracker.h"
#include "Utilities\XmlParser.h"

void testTracking(bool debug)
{
	XmlParser config;
	if (!config.loadFile("config.xml")) { return; }

	if (!Camera::getInstance().initialize(config)) {return;}

	cv::Point ballPosition;
	HSVcolor color(config, "red");
	CircleSpec circle(config, "red");
	ColorFinder finder(&color);

	ObjectTracker tracker(Camera::getInstance().getCenter());

	if (debug)
	{
		cv::namedWindow("BGR", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);
	}

	while(true)
	{
		Camera::getInstance().captureFrame();

		ballPosition = finder.getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV),
			circle);

		if (debug)
		{
			if (ballPosition.x > -1 && ballPosition.y > -1);
			cv::Scalar circleColor = cvScalar(255, 0, 0);
			cv::circle(Camera::getInstance().getFrame(Camera::ColorSpace::BGR), ballPosition, 5, circleColor);

			cv::imshow("BGR", Camera::getInstance().getFrame(Camera::ColorSpace::BGR));
			cv::imshow("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
		}

		tracker.track(ballPosition);

		if((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
}

int main()
{
	testTracking(true);
	return 0;
}