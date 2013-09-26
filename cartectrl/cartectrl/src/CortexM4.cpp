#include "CortexM4.h"

#include "usb_com.h"

namespace{

const int MAX_LEN = 20;
uint8_t receivedMsg[MAX_LEN];

const uint8_t EndMsg1 = '\r';
const uint8_t EndMsg2 = '\n';

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
    int j = 0;
    bool end = false;
    bool isPreviousEndMsg1 = false;

    char p[4];
    while( !end )
    {
        uint32_t n = usb::read(p);

        for( unsigned int i = 0; i < n; i++ )
        {
            receivedMsg[j] = p[i];
            j++;
            if( p[i] == EndMsg1 )
            {
                isPreviousEndMsg1 = true;
            }
            else if ( isPreviousEndMsg1 && p[i] == EndMsg2)
            {
                isPreviousEndMsg1 = false;
                end = true;
                sendCommand ( receivedMsg, j-2 );
                break;
            }
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

                uint32_t msgSize = 6;
                uint8_t msg[msgSize];

                msg[0] = CMD_MOTOR_READ_POS;
                msg[1] = data[1];
                msg[2] = pos >> 8;
                msg[3] = pos;
                msg[4] = EndMsg1;
                msg[5] = EndMsg2;

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
