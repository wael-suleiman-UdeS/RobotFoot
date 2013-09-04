#include "UnitTestHerkulex.h"

#include "UnitTestOutput.h"

#include "herkulex.h"
#include "Tools.h"

namespace
{

    bool checkPosition( uint16_t ReadPosition, uint16_t DesiredPosition, uint16_t Threshold )
    {
        return DesiredPosition <= ReadPosition +  Threshold && DesiredPosition >= ReadPosition -  Threshold;
    }

}

UnitTestHerkulex::UnitTestHerkulex() :
    UnitTest( "Herkulex" )
{

}

UnitTestHerkulex::~UnitTestHerkulex()
{

}

bool UnitTestHerkulex::Test()
{
    const uint16_t DesiredPosition(900);
    const uint16_t Threshold = 3;
    const uint8_t Playtime = 100;

    Herkulex test = Herkulex();

    for( int i = Herkulex::ID_R_HIP_YAW; i < Herkulex::NUMBER_OF_JOINTS; i++ )
    {
        test.setTorque(i, TORQUE_ON);
    }

    for( int i = Herkulex::ID_R_HIP_YAW; i < Herkulex::NUMBER_OF_JOINTS; i++ )
    {
        test.positionControl(i, DesiredPosition, Playtime, 0x00);
    }

    Tools::Delay( Tools::DELAY_AROUND_1S*4 );

    UnitTestOutput::SendMessage( "\r\nPosition Test : (Position = " );
    //UnitTestOutput::SendMessage( DesiredPosition );
    UnitTestOutput::SendMessage( ")" );
    bool Result = true;

    for( int i = Herkulex::ID_R_HIP_YAW; i < Herkulex::NUMBER_OF_JOINTS; i++ )
    {
        UnitTestOutput::SendMessage( "\r\nMotor " );
        //UnitTestOutput::SendMessage( i );
        UnitTestOutput::SendMessage( "  : " );


        const uint16_t ReadPosition = test.getPos(i);
        bool CurrentResult = checkPosition( ReadPosition, DesiredPosition, Threshold );
        Result &= CurrentResult;

        if( CurrentResult )
        {
            UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_PASSED );
        }
        else
        {
            UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_FAILED );

        }
    }

    return Result;
}
