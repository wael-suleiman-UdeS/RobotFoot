#ifndef CORTEXM4_H
#define CORTEXM4_H

#include "herkulex.h"
class DataBuffer;

class CortexM4
{
public:
    CortexM4();
    ~CortexM4();

    void read();

private:
    void write( uint8_t* data, uint32_t n );

    void sendCommand( uint8_t* data, uint32_t n );

    friend class DataBuffer;
};

#endif // CORTEXM4_H
