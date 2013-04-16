#include <ostream>
#include <boost/regex.hpp>

#include "USBInterface.h"

USBInterface::USBInterface(boost::asio::io_service &io_service,
                           const std::string &port_name,
                           unsigned int baud)
: _io_service(io_service),
  _serialPort(io_service, port_name) 
{
    if(!_serialPort.is_open())
    {
         // TODO Logger
         std::cout << "Can't open the serial port" << port_name << std::endl;
         return;
    }
    boost::asio::serial_port_base::baud_rate baud_option(baud);
    _serialPort.set_option(baud_option);
}

USBInterface::~USBInterface()
{
    _serialPort.cancel();
    _serialPort.close();
}

void USBInterface::write(const char *command)
{
    boost::asio::streambuf streamBuffer;
    std::ostream os(&streamBuffer);
    os << command << "\r\n";
    
    boost::asio::write(_serialPort, streamBuffer);
}

const char *USBInterface::read_sync(const char *command)
{
    boost::asio::streambuf streamBuffer;
    std::ostream os(&streamBuffer);
    os << command << "\r\n";
    boost::asio::write(_serialPort, streamBuffer);

    boost::asio::streambuf readBuffer;
    boost::asio::read_until(_serialPort, readBuffer, "\r\n");

    return boost::asio::buffer_cast<const char*>(readBuffer.data()); 
}

void USBInterface::read_asyc(const char *command)
{
    // TODO
}


