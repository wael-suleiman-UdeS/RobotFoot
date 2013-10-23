#include "EmergencyStop.h"

#include "Input.hpp"
#include "herkulex.h"

void EmergencyStop::operator()()
{
    Herkulex::GetInstance()->setTorque(BROADCAST_ID, TORQUE_FREE);
}
//------------------------------------------------------------------------------
void EmergencyStop::install()
{
    inputMan.handler(button::b3, event_type::pressed, EmergencyStop{});
}
//------------------------------------------------------------------------------
