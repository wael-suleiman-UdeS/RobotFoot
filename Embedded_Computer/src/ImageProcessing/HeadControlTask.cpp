#include "ImageProcessing/HeadControlTask.h"

#include "Utilities/logger.h"
#include "ImageProcessing/Camera.h"

#include <boost/filesystem.hpp>
#include <boost/chrono.hpp>


/*!
 * \brief Constructor for the tracking of the ball
 *
 * \param threadManager : Use for starting this thread
 * \param config : The configuration file
 * \param PID : Activate the use of PID
 * \param debug : Acivate some debug option
 * \param colorName : The string of the color of the ball
 * \param mc : An instance of the micro controller
 */
HeadControlTask::HeadControlTask( ThreadManager *threadManager, const XmlParser &config, MotorControl mc ) : _threadManager(threadManager)
{
	// Initialize capture
	Logger::getInstance() << "Initializing capture device..." << std::endl;
	if (!Camera::getInstance().initialize(config))
	{
		Logger::getInstance() << "Error while initializing capture device." << std::endl;
		return;
	}

	_durationMean = 0;
	_durationIndex = 0;
	_guiEnabled = false;

	path basePath = XmlPath::Root / XmlPath::ImageProcessing;
	string colorName = config.getStringValue(basePath / XmlPath::ActiveColor);

	HSVcolor color(config, colorName);
	_circle = new CircleSpec(config, colorName);
	_finder = new ColorFinder(&color);
	_tracker = new ObjectTracker(&mc, Camera::getInstance().getCenter());
	_tracker.initializeHack(config); // todo: holy hack
	_tracker.initializeHackPID(config);

	if (_guiEnabled)
	{
		cv::namedWindow("BGR", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);
	}
}

HeadControlTask::~HeadControlTask()
{
	delete _circle;
	delete _finder;
	delete _tracker;
	cvDestroyAllWindows();
}

void HeadControlTask::start()
{
	Logger::getInstance() << "Tracking process started" << std::endl;
	while(true)
	{
      		boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
		Camera::getInstance().captureFrame();

		_ballPosition = _finder.getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV),
			_circle);

		//Logger::getInstance() << "Ball position: " << _ballPosition.x << ", " << _ballPosition.y << std::endl;

		if (_guiEnabled)
		{
			if (_ballPosition.x > -1 && _ballPosition.y > -1)
			{
				cv::Scalar circleColor = cvScalar(255, 0, 0);
				cv::circle(Camera::getInstance().getFrame(Camera::ColorSpace::BGR), _ballPosition, 5, circleColor);
			}

			cv::imshow("BGR", Camera::getInstance().getFrame(Camera::ColorSpace::BGR));
			cv::imshow("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
		}

		_tracker.trackPID(_ballPosition);		

		if((cvWaitKey(10) & 255) == 27) break;

      		boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
		if(_guiEnabled)
		{      		
			durationMean += sec.count();
		        durationIndex++;
		        if(!(durationIndex <= 99)) 
      			{
	      	  		std::cout << "took " << durationMean/100 << " seconds\n";
		  		durationMean = 0;
		    		durationIndex = 0;
      			}
		}
	}
}
