#include "UnitTestHerkulex.h"

#include "UnitTestOutput.h"

#include "herkulex.h"
#include "Tools.h"

UnitTestHerkulex::UnitTestHerkulex() :
    UnitTest( "Herkulex" )
{

}

UnitTestHerkulex::~UnitTestHerkulex()
{

}

bool UnitTestHerkulex::Test()
{
    const uint16_t DesiredPosition(999);
    const uint16_t Motor_ID(0xFD);


    Herkulex test = Herkulex();

    test.setTorque(Motor_ID, TORQUE_ON);

    test.positionControl(Motor_ID, DesiredPosition, 60, 0x00);
    Tools::Delay(40000000);

    UnitTestOutput::SendMessage( "\nPosition Test = " );
    if( DesiredPosition == test.getPos(Motor_ID) )
    {
        //UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_PASSED );
    }
    else
    {
        //UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_FAILED );
    }
}
