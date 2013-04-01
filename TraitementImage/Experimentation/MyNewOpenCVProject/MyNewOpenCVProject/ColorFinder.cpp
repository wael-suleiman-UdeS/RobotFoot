#include "ColorFinder.h"

using namespace cv;

ColorFinder::ColorFinder()
{
	// todo : Configurer d'une autre façon. Retirer ce constructeur?
	ColorFinder(160, 10, 80, 180);
}

ColorFinder::ColorFinder(int hue, int hueTolerance, int saturation, int brightness)
{
	//todo : forcer les valeurs de 0 à 255. Classe IntRange ou fonction SetInRange? Nécessaire?
	_hue = hue;
	_hueTolerance = hueTolerance;
	_saturation = saturation;
	_brightness = brightness;

	CvSize frameSize = cvSize(WIDTH, HEIGHT);
	_resultFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 1);
}

ColorFinder::~ColorFinder()
{
}

void ColorFinder::Filter(IplImage* sourceFrame)
{
	CvScalar minHSV = cvScalar(_hue - _hueTolerance, _saturation, _brightness, MIN_VALUE);
	CvScalar maxHSV = cvScalar(_hue + _hueTolerance, MAX_VALUE, MAX_VALUE, MAX_VALUE);

	cvInRangeS(sourceFrame, minHSV, maxHSV, _resultFrame);
}

CvPoint* ColorFinder::GetCirclePosition(IplImage* frame)
{
	Filter(frame);
	ImageProcessing::Erode(_resultFrame, _resultFrame);
	ImageProcessing::Dilate(_resultFrame, _resultFrame);
	ImageProcessing::Smooth(_resultFrame, _resultFrame);

	// todo: choix du cercle (présentement le premier)
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* circles = cvHoughCircles(_resultFrame, storage, CV_HOUGH_GRADIENT, 2, _resultFrame->height/4,
		100, 50, 10, 400);

	CvPoint* circlePosition = new CvPoint();
	if (circles->total)
	{
		float* positionBuffer = (float*)cvGetSeqElem(circles, 0);
		
		circlePosition->x = positionBuffer[0];
		circlePosition->y = positionBuffer[1];
	}
	else
	{
		circlePosition->x = -1;
		circlePosition->y = -1;
	}

	return circlePosition;
}
