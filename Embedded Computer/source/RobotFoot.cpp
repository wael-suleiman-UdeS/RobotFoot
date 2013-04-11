#include "ImageProcessing\Camera.h"
#include "ImageProcessing\ColorFinder.h"
#include "ImageProcessing\ObjectTracker.h"

int main()
{
	testTracking();
	return 0;
}

void testTracking()
{
	// Ini file
	//////////////////////////////
	int cameraDeviceIndex = 0;

	// red color
	int hue = 215;
	int hueTolerance = 40;
	int saturation = 163;
	int brightness = 103;

	// red circle
	int erosionIterations = 0;
	int dilationIterations = 2;
	int smoothingApertureSize = 9;
	double resolutionDivisor = 2;
	double minDistance = 120;
	double edgeThreshold = 100;
	double centerThreshold = 10;
	double minRadius = 10;
	double maxRadius = 400;

	//////////////
	
	if (!Camera::getInstance().initialize(cameraDeviceIndex)) {return;}

	CvPoint ballPosition;
	HSVcolor color = {hue, hueTolerance, saturation, brightness};
	ColorFinder finder(&color);
	
	CircleSpec circle;
	circle.erosionIterations = erosionIterations;
	circle.dilationIterations = dilationIterations;
	circle.smoothingApertureSize = smoothingApertureSize;
	circle.resolutionDivisor = resolutionDivisor;
	circle.minDistance = minDistance;
	circle.edgeThreshold = edgeThreshold;
	circle.centerThreshold = centerThreshold;
	circle.minRadius = minRadius;
	circle.maxRadius = maxRadius;

	ObjectTracker tracker(Camera::getInstance().getCenter());

	while(true)
	{
		Camera::getInstance().captureFrame();

		ballPosition = finder.getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV),
			circle);

		tracker.track(ballPosition);

		if((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
}