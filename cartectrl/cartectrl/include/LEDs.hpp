/**
  ******************************************************************************
  * @file    LEDs.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 1.0
  * @date    2013-10-22
  * @brief   Contains declarations of LED objects.
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef LEDS_HPP
#define LEDS_HPP
//------------------------------------------------------------------------------
#include <LED.hpp>
//------------------------------------------------------------------------------

namespace bsp
{

#if STM32F4DISCOVERY_TEST
extern LED<gpio::d, 12> LED1;
extern LED<gpio::d, 13> LED2;
extern LED<gpio::d, 14> LED3;   // < Avoid use of these
extern LED<gpio::d, 15> LED4;   // <
#else
extern LED<gpio::e, 15> LED1;
extern LED<gpio::e, 14> LED2;
#endif

}

using bsp::LED1;
using bsp::LED2;
#if STM32F4DISCOVERY_TEST
using bsp::LED3;                // < Avoid use of these
using bsp::LED4;                // <
#endif


//------------------------------------------------------------------------------
#endif // LEDS_HPP
