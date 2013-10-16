#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <ostream>
#include <list>
#include <time.h>
#include <boost/thread.hpp>

/** @addtogroup Utilities
 * @{
 */

/** \brief Class for logging data in ostream objects
 */
class Logger
{
   public:
      enum LogLvl
      {
		   DEBUG = 0,
         INFO = 1,
         WARN = 2,
         ERROR = 3,
         SHUTUP = 4
      };
 
      static Logger& getInstance();     
      
      static Logger& getInstance(LogLvl lvl);

      void addStream(std::ostream& stream);
      void setLogLvl(LogLvl lvl);
      void setLogLvl(std::string lvl);
	  
	  /** \brief Stream an object to each output stream in the stream list
	   *
	   * \param stream ostream&: Output stream to add
	   * \return Logger&: 
	   *
	   */
      template <typename T>
      Logger &operator<<(const T& object)
	  {
          if (_currentLvl >= _masterLvl)
          {
              for(auto stream = _streams.begin(); stream != _streams.end(); ++stream)
              {
                  if (_timeStamp)
                  {
                      **stream << timeStamp() << ": ";
                      _mutex.lock();
                  }
                  **stream << object;
              }
              _timeStamp = false;
          }
          return *this;
      }

      Logger &operator<<(std::ostream& (*endlPtr)(std::ostream&));

   private:
      bool _timeStamp;

      // Log lvl for the next stream. Return to default after std::endl.
      // Default = LogLvl::INFO
      LogLvl _currentLvl;

      // Log lvl of reference. Set with setLogLvl() method
      LogLvl _masterLvl;

      std::list<std::ostream*> _streams;

      Logger() { _timeStamp  = true;
                 _currentLvl = LogLvl::INFO;
                 _masterLvl  = LogLvl::INFO;}
      Logger(const Logger&);
      void operator=(const Logger&);
      static Logger* instance;
 
      std::string timeStamp();

      // Thread sync
      boost::mutex _mutex;
};
#endif
