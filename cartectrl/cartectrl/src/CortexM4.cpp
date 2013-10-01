#include "CortexM4.h"

#include "usb_com.h"

//HACK
#include <cstdlib>

namespace{

const int MAX_LEN = 200;
uint8_t circularBuff[MAX_LEN];
uint8_t* pCircularBuff = circularBuff;
uint8_t* pEndBuff = pCircularBuff + MAX_LEN;

const uint8_t StartMsg = 0xFF;

enum CMD
{
    CMD_MOTOR_SET_POS = 1,
    CMD_MOTOR_READ_POS = 2,
    CMD_MOTOR_SET_TORQUE = 3

};

}

CortexM4::CortexM4()
{
    usb::init();
}

CortexM4::~CortexM4()
{

}

void CortexM4::read()
{
    uint8_t data[10];
    uint8_t calculatedCheckSum = 0;
    uint8_t checkSum = 0;
    uint8_t msgNb = 0;

    bool isReadingMessage = false;
    int iMsgNb = -1;
    uint8_t* pData = data;

    while(1)
    {
        //TODO : if usb::read return a lot of value, buffer can be overwrited
        uint8_t* pTempBuff = pCircularBuff;
        pCircularBuff += usb::read(pCircularBuff, pEndBuff-pCircularBuff);

        for(;pTempBuff != pCircularBuff;pTempBuff++)
        {
            if(!isReadingMessage)
            {
                if(*pTempBuff == StartMsg)
                {
                    isReadingMessage = true;
                }
            }
            else
            {
                if(iMsgNb == -1)
                {
                    checkSum = *pTempBuff;
                    iMsgNb++;
                }
                else if(iMsgNb == 0)
                {
                    iMsgNb = *pTempBuff;
                    msgNb = iMsgNb;
                    calculatedCheckSum = iMsgNb;
                }
                else
                {
                    *pData = *pTempBuff;
                    pData++;
                    iMsgNb--;
                    calculatedCheckSum += *pTempBuff;

                    if(iMsgNb == 0)
                    {
                        if(checkSum == calculatedCheckSum)
                        {
                            sendCommand(data, msgNb);
                        }
                        else
                        {
                            // TODO : dummy
                            isReadingMessage = false;
                        }
                        isReadingMessage = false;
                        iMsgNb = -1;
                        pData = data;
                    }
                }
            }
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
