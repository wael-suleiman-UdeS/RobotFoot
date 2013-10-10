#include "logger.h"

using std::string;
using std::ostream;

Logger* Logger::instance = &getInstance();

/** \brief Retreive the instance of the Logger (Singleton pattern)
 *
 *  \return Logger&: Instance of the Logger object
 *
 */
Logger& Logger::getInstance()
{
    if (instance == NULL) 
    {
       instance = new Logger();
    }
    return *instance;
}

/** \brief Retreive Logger instance and set the Log lvl for the next entry.
 *
 *  \return Logger&: Instance of the Logger object
 *
 */
Logger& Logger::getInstance(LogLvl lvl)
{
    instance->_currentLvl = lvl;
    return getInstance();
}

/** \brief Add an output stream object to the stream list
 *
 * \param stream ostream&: Output stream to add
 *
 */
void Logger::addStream(ostream& stream)
{
    _streams.push_front(&stream);
}

/** \brief Set the master log lvl for the whole session
 *
 *  \param Logger::LogLvl enum {DEBUG, INFO, WARN, ERROR, SHUTUP}
 *
 */
void Logger::setLogLvl(LogLvl lvl)
{
    _masterLvl = lvl;
}

/** \brief Set the master log lvl for the whole session
 *
 *  \param LogLvl string "DEBUG", "INFO", "WARN", "ERROR", "SHUTUP"
 *
 */
void Logger::setLogLvl(std::string lvl)
{
    if (lvl == "DEBUG")
    {
        _masterLvl = LogLvl::DEBUG; 
    }
    else if (lvl == "ERROR")
    {
        _masterLvl = LogLvl::ERROR;
    }
    else if (lvl == "WARN")
    {
        _masterLvl = LogLvl::WARN;
    }
    else if (lvl == "SHUTUP")
    {
        _masterLvl = LogLvl::SHUTUP;
    }
    else
    {
        _masterLvl = LogLvl::INFO;
    }
}

Logger& Logger::operator<<(ostream& (*endlPtr)(std::ostream&))
{
    if (_currentLvl >= _masterLvl)
    {
        for(auto stream = _streams.begin(); stream != _streams.end(); ++stream)
        {
            **stream << *endlPtr;
        }
        _timeStamp = true;
        _currentLvl = LogLvl::INFO;
    }
    return *this;
}

/** \brief Retrieve the current time in a readable format
 *
 * \return string: Retrieved time stamp
 *
 */
string Logger::timeStamp()
{
    char buffer[50];
    time_t currentTime= time(NULL);
    strftime(buffer, 50,"%c", localtime(&currentTime));
    return string(buffer);
}
