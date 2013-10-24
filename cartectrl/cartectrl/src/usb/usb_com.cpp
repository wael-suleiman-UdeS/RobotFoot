/**
  ******************************************************************************
  * @file    usb_com.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V1.0.0
  * @date    2013-04-10
  * @brief   Allow communication through usb cdc
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "usb_com.hpp"
//------------------------------------------------------------------------------
#include "usbd_usr.h"
#include "usbd_cdc_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "miscutils.h"
#include "bsp/TimedTasks.hpp"
#include "LEDs.hpp"
//------------------------------------------------------------------------------

// This global object is used inside other USB functions, and seems to require
// alignment on a 4 byte boundary.
USB_OTG_CORE_HANDLE  USB_OTG_dev ALIGNED(4);

//------------------------------------------------------------------------------
namespace usb
{

static bool usb_enabled = false;
static bool rxed = false;


static auto usbTask = bsp::make_Task([]
{
    enum { reload = 5 };
    static unsigned LEDcnt;
    static unsigned cnt = reload;

    if (!cnt)
    {
        cnt = reload;
        static unsigned old_irq_cnt;

        auto new_irq_cnt = getSOFCount(); //usb_interrupt_cnt();

        // USB is considered enabled if some activity is detected from usb irq.
        bool new_usb_state = (new_irq_cnt != old_irq_cnt);

        if (new_usb_state and new_usb_state != usb_enabled)
        {
            // USB has just been enabled. Reset buffers.
            usb_reset_buffers();
        }

        usb_enabled = new_usb_state;
        old_irq_cnt = new_irq_cnt;
    }
    else
        --cnt;

    ++LEDcnt;

    if (usb_enabled)
    {
        if ((LEDcnt & 0x7F) == 0x60)
        {
            LED2.set(!rxed);
            rxed = false;
        }
        else if ((LEDcnt & 0x60) != 0x60)
        {
            LED2.set();
        }
    }
    else
    {
        // slow flashing
        LED2.set(LEDcnt & 0x200);
    }
});

int init()
{
    USBD_Init(  &USB_OTG_dev,
                USB_OTG_FS_CORE_ID,
                &USR_desc,
                &USBD_CDC_cb,
                &USR_cb);

    bsp::TimedTasks::GetInstance()->add(usbTask);

    return 0;   // Assume no error...
}


ssize_t write(const uint8_t *m, size_t s)
{
    if (!usb_enabled)
        return -1;

    return usb_tx(m, s);
}

ssize_t read(uint8_t *m, size_t s)
{
    if (!usb_enabled)
        return -1;

    auto ret = usb_rx(m, s);

    rxed = rxed or !!ret;
    return ret;
}

unsigned getSOFCount()
{
    return SOFCount;
}

bool enabled()
{
    return usb_enabled;
}

} // namespace usb


