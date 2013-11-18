#ifndef STM32F4_H
#define STM32F4_H

#include "Utilities/SerialInterface.h"

#include <cstdint>
#include <boost/asio.hpp>


class STM32F4
{
    public:
        STM32F4(std::string portName, boost::asio::io_service& io, std::function<void(std::vector<char>)> &function);
        ~STM32F4();

        void AddMsg(const std::vector<char>& msg);
        void SendMsg();
        void ReceiveMsg(const std::vector<char>& msg);

    private:
        SerialInterface _usb;
        std::share_ptr<std::function<void(std::vector<char>)>>  _callBackFunction;

        std::vector<char> _outBuffer;
        std::vector<char> _inBuffer;
};

#endif // STM32F4_H
