
/////////////////////////////////////////////////////////////////////
//
// Class USBInterface
// Author : Mickael Paradis
// Date : April 11th
// Description : Serial Interface implemented with boost::asio
//
/////////////////////////////////////////////////////////////////////
#ifndef USBINTERFACE_H
#define USBINTERFACE_H

#include <string>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 

const unsigned int MAX_SIZE = 512; 

class USBInterface
{
    public:
      USBInterface(boost::asio::io_service& io_service,
                   const std::string &port_name,
                   unsigned int baud);
      ~USBInterface();

      void write(const char *command);
      const char *read_sync(const char *command);
      void read_asyc(const char *command);

    private:
      boost::asio::io_service &_io_service;
      boost::asio::serial_port _serialPort; 
};
#endif
