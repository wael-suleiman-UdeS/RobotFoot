#ifndef STM32F4_H
#define STM32F4_H

#include <string>
#include <cstdint>
#include <boost/asio.hpp>

#include "../Utilities/USBInterface.h"

class STM32F4
{
public:
	STM32F4(std::string portName, boost::asio::io_service& io);
	~STM32F4();
	void setMotor(std::uint8_t id, std::uint8_t value);

private:
	USBInterface _usb;
};

#endif // STM32F4_H
