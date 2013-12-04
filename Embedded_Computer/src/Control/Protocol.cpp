#include "Control/Protocol.h"
#include "Utilities/logger.h"

void Protocol::Separate2Bytes(const std::uint16_t value, std::uint8_t& valueLSB, std::uint8_t& valueMSB)
{
    valueMSB = value >> 8;
    valueLSB = value;
}

void Protocol::Unify2Bytes(std::uint16_t& value, const std::uint8_t valueLSB, const std::uint8_t valueMSB)
{
    value = 0;
    value = valueMSB << 8;
    value = value | valueLSB;
}

char Protocol::CalculCheckSum(const std::vector<char>& msg)
{
    std::uint8_t checkSum(0);
    auto itr = msg.begin() + 2;

    for(; itr != msg.end(); ++itr)
    {
        checkSum += *(itr); 
    }

    return checkSum;
}

std::vector<char> Protocol::GenerateDataMsg(uint16le header, const std::vector<char>& data)
{
    std::vector<char> result;
    const uint16le size = data.size() + sizeof(uint16le);

    result.push_back(header.bytes[0]);
    result.push_back(header.bytes[1]);
    result.push_back(size.bytes[0]);
    result.push_back(size.bytes[1]);

    result.insert(result.end(), data.begin(), data.end());
    return result;
}

// Find first tag in vector and return iterator to the start of this tag
bool Protocol::FindMsgHeader(std::vector<char>::const_iterator &iterator, const std::vector<char> &msg, uint16le& header)
{
    if (iterator < msg.cbegin() || iterator > msg.cend())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Protocol : FindMsgHeader error, iterator not valid" << std::endl;
        iterator = msg.cend(); 
        return false;
    }
    else
    {
        auto end = msg.end() - 1;
        for (; iterator <= end; ++iterator)
        {
            header.bytes[0] = *iterator;
            header.bytes[1] = *(iterator+1);
            if (isTag(header))
                return true;         
        }
        iterator = msg.cend();
        return false;
    }
}

bool Protocol::isTag(uint16le header)
{
    // TODO Sry for who ever read this
    // Put headers in map and search with the map?
    return (header == MsgHeader     ||
            header == MotorHeader   ||
            header == GyroAccHeader ||
            header == ButtonHeader  ||
            header == PowerHeader   ||
            header == TorqueHeader  ||
            header == MotorRawHeader);
}

