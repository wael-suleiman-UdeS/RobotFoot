#include "Control/Protocol.h"

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

    for(; itr != msg.end()-1; ++itr)
    {
        checkSum += *(itr); 
    }

    return checkSum;
}

std::vector<char> Protocol::GenerateDataMsg(std::uint16_t header, const std::vector<char>& data)
{
    std::uint8_t headerMSB, headerLSB, sizeMSB, sizeLSB;
    std::vector<char> result;
    const std::uint16_t size = data.size() + sizeof(std::uint16_t);

    Separate2Bytes(header, headerLSB, headerMSB);
    Separate2Bytes(size, sizeLSB, sizeMSB);

    result.push_back(headerLSB);
    result.push_back(headerMSB);
    result.push_back(sizeLSB);
    result.push_back(sizeMSB);

    result.insert(result.end(), data.begin(), data.end());
    return result;
}

// Find first tag in vector and return iterator to the start of this tag
std::uint16_t Protocol::FindMsgHeader(std::vector<char>::const_iterator &iterator, const std::vector<char> &msg)
{
    std::uint16_t header = 0;
    for (; iterator != msg.end() - 1; ++iterator)
    {
       Unify2Bytes(header, *iterator, *(iterator+1));
       if (isTag(header))
           break;         
    }
    return header;
}

bool Protocol::isTag(std::uint16_t header)
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

void Protocol::ReadPacket(std::uint16_t header,
        std::vector<char>::const_iterator& mainItr, 
        const std::vector<char>::const_iterator& mainEnd,
        std::vector<char>& result)
{
    std::uint8_t headerMSB, headerLSB;
    std::uint16_t size(0);
    Separate2Bytes(header,headerLSB,headerMSB);

    for(;mainItr != mainEnd && mainItr+1 != mainEnd; ++mainItr)
    {
        if(headerLSB == *(mainItr) && headerMSB == *(mainItr+1))
        {
            mainItr+=2;
            char sizeLSB = *(mainItr);
            char sizeMSB = *(++mainItr); 
            Unify2Bytes(size, sizeLSB, sizeMSB);
            break;
        }
    }
    std::vector<char>::const_iterator packetBegin = ++mainItr;
    mainItr += size;
    result.assign(packetBegin,mainItr);
}

