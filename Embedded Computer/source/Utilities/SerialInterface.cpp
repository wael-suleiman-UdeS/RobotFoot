#include <ostream>
#include <boost/regex.hpp>

#include "SerialInterface.h"

SerialInterface::SerialInterface(boost::asio::io_service &io_service,
                           const std::string &port_name,
                           unsigned int baud)
: _io_service(io_service),
  _serialPort(io_service)
{
	try
	{
		_serialPort.open(port_name);
	}
	catch (std::exception &ex)
	{
		return;
	}
		if(!_serialPort.is_open())
		{
			 // TODO Logger
			 std::cout << "Can't open the serial port" << port_name << std::endl;
			 return;
		}
		boost::asio::serial_port_base::baud_rate baud_option(baud);
		_serialPort.set_option(baud_option);
}

SerialInterface::~SerialInterface()
{
	if(!_serialPort.is_open()) { return; }
    _serialPort.cancel();
    _serialPort.close();
}

void SerialInterface::write(std::vector<char> command)
{
	if(!_serialPort.is_open()) { return; }

    boost::asio::streambuf streamBuffer;
    std::ostream os(&streamBuffer);
    os.write(command.data(), command.size());
    os << "\r\n";

    boost::asio::write(_serialPort, streamBuffer);
}

std::vector<char> SerialInterface::read_sync(std::vector<char> command)
{
	if(!_serialPort.is_open()) { return std::vector<char>(); }

    boost::asio::streambuf streamBuffer;
    std::ostream os(&streamBuffer);
    os.write(command.data(), command.size());
    os << "\r\n";
    boost::asio::write(_serialPort, streamBuffer);

    streamBuffer.consume(streamBuffer.size());
    boost::asio::read_until(_serialPort, streamBuffer, "\r\n");

	//std::vector<char>(streamBuffer.data(), streamBuffer.data() 
	const char* start_ptr = boost::asio::buffer_cast<const char*>(streamBuffer.data());
	return std::vector<char>(start_ptr, start_ptr + streamBuffer.size());
}

void SerialInterface::read_asyc(std::vector<char> command)
{
	if(!_serialPort.is_open()) { return; }
    // TODO
}


