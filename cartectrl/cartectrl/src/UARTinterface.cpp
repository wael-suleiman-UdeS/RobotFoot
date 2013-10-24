/**
  ******************************************************************************
  * @file    UARTinterface.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-18
  * @brief
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "UARTinterface.hpp"
//------------------------------------------------------------------------------


struct UART_empty_RxHandler : UART_Callback
{
    void rx(int) override {}
};

static UART_empty_RxHandler empty_handler;

//------------------------------------------------------------------------------
UARTInterface::UARTInterface(UART_Callback *newhandler)
{
    callback(newhandler);
}
//------------------------------------------------------------------------------
void UARTInterface::callback(UART_Callback *newcb)
{
    cb = newcb ? newcb : &empty_handler;
}
//------------------------------------------------------------------------------
