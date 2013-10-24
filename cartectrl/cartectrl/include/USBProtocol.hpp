/**
  ******************************************************************************
  * @file    USBProtocol.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-21
  * @brief   Handles the USB protocol (application specific)
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef USBPROTOCOL_HPP
#define USBPROTOCOL_HPP
//------------------------------------------------------------------------------
#include "HerkulexMan.hpp"
#include <memory>
//------------------------------------------------------------------------------

class USBProtocol
{
public:
    USBProtocol(HerkulexMan *herkman);
    ~USBProtocol();

    void loop();

private:
    void decode(uint8_t* data, uint32_t n);

    // Forward decl
    class DataBuffer;
    friend class DataBuffer;

    HerkulexMan *herkman;

    std::unique_ptr <DataBuffer> dataBuff;

    enum : int { reload10ms = 10, reload50ms = 50, reload500ms = 500 };

    int timer = reload500ms;

    unsigned sof_counter = 0;
};

//------------------------------------------------------------------------------
#endif // USBPROTOCOL_HPP

