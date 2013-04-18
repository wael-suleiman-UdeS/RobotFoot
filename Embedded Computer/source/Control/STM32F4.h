#ifndef STM32F4_H
#define STM32F4_H

#include <string>
#include <cstdint>
#include <boost/asio.hpp>

#include "../Utilities/SerialInterface.h"

class STM32F4
{
public:
	enum TorqueState { TorqueOff, TorqueOn };

	STM32F4(std::string portName, boost::asio::io_service& io);
	~STM32F4();
	void setMotor(std::uint8_t id, std::uint16_t value);

	void setTorque(std::uint8_t id, TorqueState state);

	std::uint16_t read(std::uint8_t id);

private:
	SerialInterface _usb;
};

#endif // STM32F4_H
