/////////////////////////////////////////////////////////////////////
//
// Class Logger
// Author : Mickael Paradis
// Date : March 6th
// Description : Simple utility class to log data in osteam objects
//
/////////////////////////////////////////////////////////////////////
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <ostream>
#include <list>
#include <string>
#include <time.h>

class Logger
{
   public:
      static Logger& getInstance()
      {
          static Logger instance;
          return instance;
      }

      void addStream(std::ostream &stream)
      {
          _streams.push_front(&stream);
      }

      template <typename T>
      Logger &operator<<(const T &object)
      {
          for(auto stream_it = _streams.begin(); stream_it != _streams.end(); ++stream_it)
          {
             if (_timeStamp) **stream_it << timestamp() << ": ";

             **stream_it << object;
          }
          _timeStamp = false;
          return *this;
      }

      Logger &operator<<(std::ostream& (*endl_ptr)(std::ostream&))
      {
          for(auto stream_it = _streams.begin(); stream_it != _streams.end(); ++stream_it)
          {
             **stream_it << *endl_ptr;
          }
          _timeStamp = true;
          return *this;
      }

   private:
	  bool _timeStamp;
      std::list<std::ostream*> _streams;

      Logger(){ _timeStamp = true; }
      Logger(const Logger&);
      void operator=(const Logger&);

      std::string timeStamp()
      {
          char buffer[50];
          time_t timeCalendar;
          timeCalendar = time(NULL);
          strftime(buffer ,50 ,"%c" ,localtime(&timeCalendar));
          return std::string(buffer);
      }
};
#endif
