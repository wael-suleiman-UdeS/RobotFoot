#include "CortexM4.h"

#include "usb_com.h"

//HACK
#include <cstdlib>

namespace{

const int MAX_LEN = 10;
uint8_t         circularBuff[MAX_LEN];
uint8_t*        pCircularBuff = circularBuff;
uint8_t* const  pEndBuff = pCircularBuff + MAX_LEN;

const uint8_t StartMsg = 0xFF;

enum CMD
{
    CMD_MOTOR_SET_POS = 1,
    CMD_MOTOR_READ_POS = 2,
    CMD_MOTOR_SET_TORQUE = 3

};

}

class DataBuffer
{
public:
    DataBuffer()
    {
        Reset();
    }

    ~DataBuffer(){}

    void Reset()
    {
        calculatedCheckSum = 0;
        checkSum = 0;
        msgNb = 0;
        isReadingMessage = false;
        iMsgNb = -1;
        pData = data;
    }

    void FillBuffer(uint8_t* pBuff, CortexM4* cortex)
    {
        if(!isReadingMessage)
            {
                if(*pBuff == StartMsg)
                {
                    isReadingMessage = true;
                }
            }
            else
            {
                if(iMsgNb == -1)
                {
                    checkSum = *pBuff;
                    iMsgNb++;
                }
                else if(iMsgNb == 0)
                {
                    iMsgNb = *pBuff;
                    msgNb = iMsgNb;
                    calculatedCheckSum = iMsgNb;
                    if(iMsgNb > MAX_LEN-3)
                    {
                        Reset();
                    }
                }
                else
                {
                    *pData = *pBuff;
                    pData++;
                    iMsgNb--;
                    calculatedCheckSum += *pBuff;

                    if(iMsgNb == 0)
                    {
                        if(checkSum == calculatedCheckSum)
                        {
                            cortex->sendCommand(data, msgNb);
                        }

                        Reset();
                    }
                }
            }
    }

private:
    uint8_t data[MAX_LEN-3];
    uint8_t calculatedCheckSum ;
    uint8_t checkSum;
    uint8_t msgNb;

    bool isReadingMessage;
    int iMsgNb;
    uint8_t* pData;
};


CortexM4::CortexM4()
{
    usb::init();
}

CortexM4::~CortexM4()
{

}

void CortexM4::read()
{
    DataBuffer dataBuff;

    while(1)
    {
        uint8_t* pTempBuff = pCircularBuff;
        pCircularBuff += usb::read(pCircularBuff, pEndBuff-pCircularBuff);

        for(;pTempBuff != pCircularBuff;pTempBuff++)
        {
            dataBuff.FillBuffer(pTempBuff, this);
        }

        if(pCircularBuff == pEndBuff)
        {
            pCircularBuff = circularBuff;
        }
    }
}

void CortexM4::write( uint8_t* data, uint32_t n )
{
    usb::write(data, n);
}

void CortexM4::sendCommand( uint8_t* data, uint32_t n )
{
    if( n == 0 ) return;

    switch ( data[0] )
    {
        case CMD_MOTOR_SET_POS:
            if( n >= 4 ) //TODO : Should be == instead
            {
                uint16_t pos = data[2];
                pos = pos << 8;
                pos |= data[3];
                Herkulex::GetInstance()->positionControl(data[1], pos, 70, 0x00);
            }
            break;
        case CMD_MOTOR_READ_POS:
            if( n >= 2 ) //TODO : Should be == instead
            {
                uint16_t pos = Herkulex::GetInstance()->getPos( data[1] );
                // TODO : Change write protocol
                uint32_t msgSize = 6;
                uint8_t msg[msgSize];

                msg[0] = CMD_MOTOR_READ_POS;
                msg[1] = data[1];
                msg[2] = pos >> 8;
                msg[3] = pos;
                msg[4] = '\r';
                msg[5] = '\n';

                CortexM4::write( msg, msgSize );
            }
            break;
        case CMD_MOTOR_SET_TORQUE:
            if( n >= 3 ) //TODO : Should be == instead
            {
                static const uint8_t toCMD[] =
                { TORQUE_FREE, BREAK_ON, TORQUE_ON };

                uint32_t const idx = data[2];

                if (idx < sizeof(toCMD))
                {
                    Herkulex::GetInstance()->setTorque(data[1], toCMD[idx]);
                }
            }
            break;
    }
}
