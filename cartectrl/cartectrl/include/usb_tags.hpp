/**
  ******************************************************************************
  * @file    usb_tags.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-21
  * @brief   Centralized defition for USB tags
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef USB_TAG_HPP
#define USB_TAG_HPP
//------------------------------------------------------------------------------
#include "miscutils.h"
//------------------------------------------------------------------------------

enum class usb_tag : unsigned
{
    MO,
    GA,
    BU,
    PO,
    MR,
    TO
};

template <usb_tag> struct usb_tag_traits;

#define USB_TAG_TRAIT(X) \
  template <> \
 struct usb_tag_traits<usb_tag:: X >  \
{ \
    static constexpr const char *tagc = #X; \
    static constexpr uint16_t tag() \
    { \
       return (tagc[0] & 0xFF) | ((tagc[1] & 0xFF) << 8); \
    } \
}

USB_TAG_TRAIT(MO);
USB_TAG_TRAIT(GA);
USB_TAG_TRAIT(BU);
USB_TAG_TRAIT(PO);
USB_TAG_TRAIT(MR);
USB_TAG_TRAIT(TO);

#undef USB_TAG_TRAIT

struct message_header_base
{
    uint16le tag;
    uint16le size;
};


template <usb_tag msg>
 struct message_header : message_header_base
{
    message_header()
    {
        tag = usb_tag_traits<msg>::tag();
    }
};


template <usb_tag>
 struct usb_sub_message;


template <>
 struct usb_sub_message<usb_tag::MO>
{
    uint8_t  id;
    int16le  pos;
    uint8_t  dt;
    uint8_t  status[2];
    uint16le PWM;
    uint8_t  voltage;
    uint8_t  temperature;

    enum { inner_min_size = 6 };
};


template <>
 struct usb_sub_message<usb_tag::GA>
{
    int16le gyroscope[3];
    int16le accelerometer[3];
};

template <>
 struct usb_sub_message<usb_tag::BU>
{
    uint8_t id;
    uint8_t val;
};

template <>
 struct usb_sub_message<usb_tag::PO>
{
    uint8_t val;
};

template <>
 struct usb_sub_message<usb_tag::MR>
{
    uint8_t id;
    uint8_t cmd;
    uint8_t data[0];
};

template <>
 struct usb_sub_message<usb_tag::TO>
{
    uint8_t id;
    uint8_t val;
};

template <usb_tag utag>
 struct usb_message
{
    message_header<utag>  header;
    usb_sub_message<utag> data;

    enum { min_size = sizeof (usb_sub_message<utag>)};
    usb_message() { size(min_size); }

    void size(size_t s) { this->header.size = s + sizeof this->header.size; }
};



//------------------------------------------------------------------------------
#endif // USB_TAG_HPP

