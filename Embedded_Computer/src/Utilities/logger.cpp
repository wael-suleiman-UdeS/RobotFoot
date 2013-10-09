#include "logger.h"

using std::string;
using std::ostream;

Logger* Logger::instance = &getInstance();

Logger& Logger::getInstance()
{
    if (instance == NULL) 
    {
       instance = new Logger();
    }
    return *instance;
}

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

void Logger::setLogLvl(LogLvl lvl)
{
    _masterLvl = lvl;
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
