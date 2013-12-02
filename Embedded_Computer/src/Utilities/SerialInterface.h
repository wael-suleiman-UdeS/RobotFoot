
/////////////////////////////////////////////////////////////////////
//
// Class SerialInterface
// Author : Mickael Paradis
// Date : April 11th
// Description : Serial Interface implemented with boost::asio
//
/////////////////////////////////////////////////////////////////////
#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <string>
#include <vector>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 

const unsigned int MAX_SIZE = 256; 

class SerialInterface
{
    public:
      SerialInterface(boost::asio::io_service& io_service,
                   const std::string &port_name,
                   unsigned int baud);
      ~SerialInterface();
      
      void start_read(std::function<void(std::vector<char>)> function);
      void write_async(std::vector<char>& command);
      
      void write(std::vector<char>& command); // Deprecated
      std::vector<char> read_sync(std::vector<char>& command); // Deprecated
    private:
      void read_async(std::function<void(std::vector<char>)> function);
      
      boost::asio::serial_port _serialPort;
      char _read_msg[MAX_SIZE];
};
#endif
