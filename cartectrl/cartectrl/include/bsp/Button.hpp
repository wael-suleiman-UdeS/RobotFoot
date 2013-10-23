/**
  ******************************************************************************
  * @file    Button.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 1.0
  * @date    2013-10-22
  * @brief   Code to read I/Os as buttons (rather low level, should not be used
  *          by user-level code)
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef BUTTON_HPP
#define BUTTON_HPP
//------------------------------------------------------------------------------
#include "bsp/GPIO.hpp"
//------------------------------------------------------------------------------

namespace bsp
{

/** @addtogroup Constants
  * @{
  */

/**
  *  These constants are used with the Button ctor.
  */
enum class activelevel : bool
{
    low, high
};


/**
  * @}
  */


/** @addtogroup Button
  * @{
  */

/** @addtogroup Control
  * @{
  */

/**
  *  A Button uses the implementation of a GPIO_Pin, but is read-only.
  */
template <gpio g, std::uint32_t bit, activelevel level>
 class Button : private GPIO_Pin<g, bit>
{
    using Parent = GPIO_Pin<g, bit>;
public:
    /**
      * @brief  Button ctor
      * @param  None
      */
    Button() : Parent{pinfunction::input,
                      outtype::opendrain,
                      level == activelevel::high ?
                        pullresistor::pulldown : pullresistor::pullup,
                      iospeed::_2MHz}
    {}
    /**
      * @brief  Read the pin
      * @param  None
      * @return State of the button (relative to the selected active level)
      */
    bool state() const
    {
        return Parent::input() == static_cast<bool>(level);
    }
    /**
      * @brief  Read the pin
      * @return State of the button (relative to the selected active level)
      * @note   This conversion operator is handy when used as a condition
      *         inside an @c if / @c while etc...
      *
      *         @par
      *         Can be used like this:
      *         @code
      *         if (button)
      *         {
      *             // code to do when the button is in ACTIVE state
      *             // ...
      *         }
      *         @endcode
      */
    explicit operator bool() const { return this->state(); }
};

/**
  * @}
  */

/**
  * @}
  */


} //namespace bsp


//------------------------------------------------------------------------------
#endif // BUTTON_HPP
