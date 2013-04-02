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
          m_streams.push_front(&stream);
      }

      template <typename T>
      Logger &operator<<(const T &object)
      {
          for(auto stream_it = m_streams.begin(); stream_it != m_streams.end(); ++stream_it)
          {
             if (m_timeStamp) **stream_it << timestamp() << ": ";

             **stream_it << object;
          }
          m_timeStamp = false;
          return *this;
      }

      Logger &operator<<(std::ostream& (*endl_ptr)(std::ostream&))
      {
          for(auto stream_it = m_streams.begin(); stream_it != m_streams.end(); ++stream_it)
          {
             **stream_it << *endl_ptr;
          }
          m_timeStamp = true;
          return *this;
      }

   private:
      Logger(){ m_timeStamp = true; }
      Logger(const Logger&);
      void operator=(const Logger&);

      std::string timestamp()
      {
          char buffer[50];
          time_t ltime; /* calendar time */
          ltime=time(NULL); /* get current cal time */
          strftime(buffer,50,"%c",localtime(&ltime));
          return std::string(buffer);
      }

      bool m_timeStamp;
      std::list<std::ostream*> m_streams;
};
#endif
