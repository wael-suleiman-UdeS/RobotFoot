/**
  ******************************************************************************
  * @file    GPIO.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-10-21
  * @brief   Easy configuration and control of GPIOs
  ******************************************************************************
  */

#ifndef GPIO_HPP
#define GPIO_HPP
//------------------------------------------------------------------------------
#include <stm32f4xx.h>
#include <cstdint>
#include <utility>
//------------------------------------------------------------------------------

namespace bsp
{

/** @addtogroup Constants
  * @{
  */

/**
  *  These constants are used with the GPIO_Pin ctor. Their meaning should be
  *  obvious.
  */
enum class gpio : std::uint8_t
{
    a, b, c, d, e, f, g, h, i,
    n_gpios
};

enum class iospeed : std::uint8_t
{
    _2MHz, _25MHz, _50MHz, _100MHz
};

enum class outtype : bool
{
    pushpull, opendrain
};

enum class pullresistor : std::uint8_t
{
    none, pullup, pulldown
};

enum class pinfunction : std::uint8_t
{
    input, output, alternate, analog
};
/**
  * @}
  */


/**
  * @addtogroup Control
  * @{
  */

/**
  *  Base class, cannot be directly instanciated
  */
class GPIO_Pin_base
{
    template <gpio g, std::uint32_t bit>
     friend struct GPIO_Pin;
public:

    // Configure pin
    void init(pinfunction, outtype, pullresistor = pullresistor::none,
                                            iospeed = iospeed::_50MHz);


    /**
      * @brief  Set the pin state (on/off) or turn it on (default argument)
      * @param  b state or None
      * @return None
      * @note   This operation is atomic.
      */
    void set(bool b = true) { (&pgpio_->BSRRL)[!b] = (1u << bit_); }

    /**
      * @brief  Reset the pin state
      * @param  None
      * @return None
      * @note   This operation is atomic.
      */
    void reset()            { pgpio_->BSRRH = (1u << bit_); }

    /**
      * @brief  Query the output pin state
      * @param  None
      * @return State of the pin
      */
    bool state() const
    {
        return static_cast<bool>(pgpio_->ODR & (1u << bit_));
    }

    /**
      * @brief  Query the input pin state
      * @param  None
      * @return State of the pin
      */
    bool input() const
    {
        return static_cast<bool>(pgpio_->IDR & (1u << bit_));
    }

    /**
      * @brief  Toggle the pin state
      * @param  None
      * @return None
      * @note   This operation is NOT atomic: the state of the pin is queried,
      *         then the pin is set accordingly. If concurrent access to the pin
      *         is made, it is possible that the pin is not toggled; however, it
      *         will never affect the state of other port bits.
      */
    void toggle()           { set(!state()); }

private:
    GPIO_Pin_base(gpio gpio, uint16_t bit_);

private:
    void imp_init(gpio);

    GPIO_TypeDef *const pgpio_;
    uint16_t const      bit_;
};

/**
  *  class GPIO_Pin
  *  The dtor does not 'deinitialize' GPIOs, so one can exploit the side effects
  *  of the ctor to initialize a GPIO pin
  */

template <gpio g, std::uint32_t bit>
 struct GPIO_Pin : GPIO_Pin_base
{
    /**
      * @brief  Constructor of GPIO_Pin. If arguments are given, they are
      *         forwarded to @c init.
      * @param  args Potentials arguments to @c init.
      * @note   If no arguments are given, @c init is simply not invoked.
      */
    template <typename ...Args>
    GPIO_Pin(Args ...args) : GPIO_Pin_base{g, bit}
    {
        static_assert(g < gpio::n_gpios,
                      "GPIO must be between gpio::a and gpio::i");
        static_assert(bit < 16, "bit must be between 0..15");

        if (sizeof...(args))
            init(std::forward<Args>(args)...);
    }
};

/**
  * @}
  */


} // namespace bsp

//------------------------------------------------------------------------------
#endif // GPIO_HPP
