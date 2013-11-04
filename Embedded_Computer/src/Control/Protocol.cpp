#include "Control/Protocol.h"


void Protocol::Separate2Bytes(const std::int16_t value, char& valueLSB, char& valueMSB)
{
    valueMSB = value >> 8;
    valueLSB = value;
}

char Protocol::CalculCheckSum(const std::vector<char>& msg)
{
    uint8_t checkSum(0);

    auto itr = msg.begin();
    auto itrSecond = itr+2;
    auto end = msg.end();

    while(itr != itrSecond && itr != end)
    {
        ++itr;
    }

    for(;itr != end ;++itr)
    {
        checkSum += *(itr); 
    }

    return checkSum;
}

void Protocol::GenerateDataMsg(std::int16_t header, const std::vector<char>& data, std::vector<char>& result)
{
    char headerMSB, headerLSB, sizeMSB, sizeLSB;

    const std::int16_t size = data.size();

    Separate2Bytes(header, headerLSB, headerMSB);
    Separate2Bytes(size, sizeLSB, sizeMSB);

    result.push_back(headerLSB);
    result.push_back(headerMSB);
    result.push_back(sizeLSB);
    result.push_back(sizeMSB);

    result.insert(result.end(), data.begin(), data.end());
}
