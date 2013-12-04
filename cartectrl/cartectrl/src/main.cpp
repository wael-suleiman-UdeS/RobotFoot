#include "LEDs.hpp"
#include "vsense.hpp"
#include "Tools.h"
#include "USBProtocol.hpp"
#include "herkulex.h"
#include "UARTs.hpp"
//------------------------------------------------------------------------------

int main(void)
{
    Herkulex    herk    = {&UART3i};
    HerkulexMan herkman = {&herk};
    USBProtocol app     = {&herkman};

    herk.send_reboot(BROADCAST_ID);

    bsp::vsense.setLimits(7.2f, 9.5f);
    // wait
    Tools::Delay(Tools::DELAY_AROUND_1S/10);

    // Application loop
    app.loop();
}
//------------------------------------------------------------------------------
