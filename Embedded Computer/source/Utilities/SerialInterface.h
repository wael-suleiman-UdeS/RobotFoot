
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

const unsigned int MAX_SIZE = 512; 

class SerialInterface
{
    public:
      SerialInterface(boost::asio::io_service& io_service,
                   const std::string &port_name,
                   unsigned int baud);
      ~SerialInterface();

      void write(std::vector<char> command);
      std::vector<char> read_sync(std::vector<char> command);
      void read_asyc(std::vector<char> command);

    private:
      boost::asio::io_service& _io_service;
      boost::asio::serial_port _serialPort;
};
#endif
