#include "usb_com.h"
//------------------------------------------------------------------------------
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "miscutils.h"
//------------------------------------------------------------------------------

// This global object is used inside other USB functions, and seems to require
// alignment on a 4 byte boundary.
USB_OTG_CORE_HANDLE  USB_OTG_dev ALIGNED(4);

//------------------------------------------------------------------------------
namespace usb
{

int init()
{
    USBD_Init(  &USB_OTG_dev,
                USB_OTG_FS_CORE_ID,
                &USR_desc,
                &USBD_CDC_cb,
                &USR_cb);

    return 0;   // Assume no error...
}


int write(const uint8_t *m, size_t s)
{
    return 0;
}

int read(uint8_t *m, size_t s)
{
    return 0;
}

} // namespace usb


