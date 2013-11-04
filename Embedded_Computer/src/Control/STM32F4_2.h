#ifndef STM32F4_H
#define STM32F4_H

#include "../Utilities/SerialInterface.h"

#include <cstdint>
#include <boost/asio.hpp>

class STM32F4
{
public:
	STM32F4(std::string portName, boost::asio::io_service& io);
	~STM32F4();
   
   void RegisterRead(/*lambda*/);

   void AddMsg(const std::vector<std::uint8_t>& msg);
   void SendMsg();
	
private:
	SerialInterface _usb;
   std::vector<char> _msg;
   //std::vector<std::shared_ptr<ComComponent>> _readComponent;
};

#endif // STM32F4_H
