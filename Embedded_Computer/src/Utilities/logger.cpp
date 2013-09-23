#include "logger.h"

using std::string;
using std::ostream;

/** \brief Retrieve the instance of the Logger instance
 *
 * \return Logger&: Instance of the Logger object
 *
 */
Logger& Logger::getInstance()
{
    static Logger instance;
    return instance;
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

Logger& Logger::operator<<(ostream& (*endlPtr)(std::ostream&))
{
    for(auto stream = _streams.begin(); stream != _streams.end(); ++stream)
    {
        **stream << *endlPtr;
    }
    _timeStamp = true;
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