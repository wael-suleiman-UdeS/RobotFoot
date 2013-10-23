/**
  ******************************************************************************
  * @file    Input.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-10-22
  * @brief   Handle input
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef INPUT_HPP
#define INPUT_HPP
//------------------------------------------------------------------------------
#include <functional>
#include <cstdint>
#include <cstdlib>
//------------------------------------------------------------------------------


enum class button : std::uint8_t
{
    b1, b2, b3,
    // aliases, from the naming of buttons on the board
    s2=b1, s3=b2, s4=b3
};

enum class event_type : std::uint8_t
{
    down, up, toggled,
    pressed, released
};

enum {num_event_type = static_cast<std::uint8_t>(event_type::released) + 1};
// forward declarations
struct Button_info;

class Input
{
    friend class InputMan_imp;

    bool old_input = false;
    bool cur_input = false;
    Button_info &b;

    bool (Input::*readmptr)() const = &Input::isDown;
private:
    Input(Button_info &b) : b(b) {}
public:
    // be reassured, the copy contructor is available.
    Input(const Input &)            = default;
    Input &operator=(const Input &) = default;

    Input &read();
    Input &configure(event_type e)
    {
        switch (e)
        {
        case event_type::down:     readmptr = &Input::isDown;     break;
        case event_type::up:       readmptr = &Input::isUp;       break;
        case event_type::toggled:  readmptr = &Input::isToggled;  break;
        case event_type::pressed:  readmptr = &Input::isPressed;  break;
        case event_type::released: readmptr = &Input::isReleased; break;
        default:                   std::abort();
        }
        return *this;
    }
    operator bool() const { return (this->*readmptr)(); }

    bool isDown() const     { return cur_input; }
    bool isUp() const       { return !isDown(); }
    bool isToggled() const  { return cur_input != old_input; }
    bool isPressed() const  { return isToggled() && isDown(); }
    bool isReleased() const { return isToggled() && isUp(); }
};

class InputMan
{
public:
    using button_handler = std::function<void ()>;

    virtual auto handler(button, event_type) const -> button_handler = 0;
    virtual void handler(button, event_type, button_handler)         = 0;

    virtual auto input(button b) -> ::Input = 0;

    inline void resetHandler(button b, event_type h, button_handler bh= []{})
    {
        return handler(b, h, std::move(bh));
    }

};

extern InputMan &inputMan;

//------------------------------------------------------------------------------
#endif // INPUT_HPP

