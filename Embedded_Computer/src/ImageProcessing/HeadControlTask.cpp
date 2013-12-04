#include "ImageProcessing/HeadControlTask.h"

#include "Utilities/logger.h"
#include "ImageProcessing/Camera.h"

#include <boost/filesystem.hpp>
#include <boost/chrono.hpp>

using boost::filesystem::path;

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
HeadControlTask::HeadControlTask(std::shared_ptr<ThreadManager> threadManager_ptr, const XmlParser &config, std::shared_ptr<MotorControl> mc_ptr) :
_threadManager(threadManager_ptr),
_config(config),
_mc(mc_ptr)
{
	// Initialize capture
	Logger::getInstance() << "Initializing capture device..." << std::endl;
	if (!Camera::getInstance().initialize(_config))
	{
		Logger::getInstance() << "Error while initializing capture device." << std::endl;
		std::exit(1);
	}

	_durationMean = 0;
	_durationIndex = 0;
	_guiEnabled = _config.getIntValue(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::ActivatedGUI);

	//string ballColor = _config.getStringValue(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::Color / XmlPath::BallColor);
	//string goalColor = _config.getStringValue(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::Color / XmlPath::GoalColor);

	_tracker = std::make_shared<ObjectTracker>(_threadManager, mc_ptr, Camera::getInstance().getCenter());	
    _tracker->initializeHack(_config); // todo: holy hack
	_tracker->initializeHackPID(_config);
	_lastColorName = "";
}

HeadControlTask::~HeadControlTask()
{
	cvDestroyAllWindows();
}

void HeadControlTask::run()
{
	Logger::getInstance() << "Tracking process started" << std::endl;
	try
	{
		if (_guiEnabled)
		{
			//cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);
			cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);
			cv::namedWindow("BGR", CV_WINDOW_AUTOSIZE);
		}


		while(true)
		{
			setColor();
			boost::this_thread::interruption_point();
	      	//boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
			Camera::getInstance().captureFrame();

			_ballPosition = _finder->getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV), _circle);

			if (_guiEnabled)
			{
				if (_ballPosition.x > -1 && _ballPosition.y > -1)
				{
					cv::Scalar circleColor = cvScalar(255, 0, 0);
					cv::circle(Camera::getInstance().getFrame(Camera::ColorSpace::BGR), _ballPosition, 5, circleColor);
				}

				cv::imshow("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
				cv::imshow("BGR", Camera::getInstance().getFrame(Camera::ColorSpace::BGR));
			}

			_tracker->track(_ballPosition);

            cvWaitKey(10);
		}
	}
	catch(const boost::thread_interrupted& e)
	{
		Logger::getInstance() << "Catch an exeption in HeadControlTask.run()" << std::endl;
	}
    _threadManager->end();
}

void HeadControlTask::setColor()
{
	string colorName = _mc->GetColorToTrack();

	if (colorName != _lastColorName)
	{
		_lastColorName = colorName;
		std::shared_ptr<HSVcolor> color(new HSVcolor(_config, colorName));
		_circle = std::make_shared<CircleSpec>(_config, colorName);
		_finder = std::make_shared<ColorFinder>(color);
	}
}
