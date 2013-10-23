/**
  ******************************************************************************
  * @file    Input.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-10-22
  * @brief   ...
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "Input.hpp"
//------------------------------------------------------------------------------
#include <array>
#include <tuple>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include "bsp/Button.hpp"
//------------------------------------------------------------------------------

using std::array;
using std::move;
using std::tuple;
using std::make_tuple;
using std::tuple_size;
using std::tie;
using std::forward_as_tuple;

using bsp::activelevel;
using bsp::Button;
using bsp::gpio;



static auto Buttons = make_tuple
(
    Button<gpio::c, 1, activelevel::low> {},
    Button<gpio::c, 2, activelevel::low> {},
    Button<gpio::c, 3, activelevel::low> {}
);

enum { num_buttons = tuple_size<decltype(Buttons)>::value };

struct Button_info
{
    bool newval = false;
    bool oldval = false;

    array<InputMan::button_handler, num_event_type> handlers =
    {{
        []{}, // empty downHandler
        []{}, // empty upHandler
        []{}, // empty toggleHandler
        []{}, // empty pressedHandler
        []{}  // empty releasedHandler
    }};
    enum {
        i_downHandler,
        i_upHandler,
        i_toggleHandler,
        i_pressedHandler,
        i_releasedHandler
    };
    inline bool stateChanged() const { return newval != oldval; }
};

static array<Button_info, num_buttons> Buttons_state {};

//------------------------------------------------------------------------------
template <size_t N>
 struct button_reader
{
    template <typename Tuple, typename Array>
     static void read(const Tuple &t, Array &a)
    {
        auto &e = std::get<N>(a);
        e.oldval = e.newval;
        e.newval = std::get<N>(t).state();
        button_reader<N-1>::read(t,a);
    }
};
//------------------------------------------------------------------------------
template <>
 struct button_reader<0>
{
    template <typename Tuple, typename Array>
     static void read(const Tuple &t, Array &a)
    {
        enum {N};
        auto &e = std::get<N>(a);
        e.oldval = e.newval;
        e.newval = std::get<N>(t).state();
    }
};
//------------------------------------------------------------------------------
static
class InputMan_imp : public InputMan
{
public:
    InputMan_imp()
    {
        init();
    }
private:
    void init()
    {
        // Code from EmergencyStop.cpp

        // Might eventually use the Systick timer, and the timing might be
        // shared by anything that needs a specific timing
        RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5EN, ENABLE);
        TIM_DeInit(TIM5);

        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Period = 1680000; // hundredth of a second
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
        TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

        TIM_Cmd(TIM5, ENABLE);
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    Button_info &access(button b)
    {
        return Buttons_state.at(static_cast<int>(b));
    }

    Button_info const &access(button b) const
    {
        return Buttons_state.at(static_cast<int>(b));
    }

    auto handler(button b, event_type e) const override -> button_handler
    {
        return access(b).handlers.at(static_cast<int>(e));
    }

    void handler(button b, event_type e, button_handler bh) override
    {
        access(b).handlers.at(static_cast<int>(e)) = move(bh);
    }

    auto input(button b) override -> ::Input
    {
        return {access(b)};
    }

}
inman;

InputMan &inputMan = inman;


//------------------------------------------------------------------------------
Input &Input::read()
{
    std::swap(old_input, cur_input);
    cur_input = b.newval;
    return *this;
}
//------------------------------------------------------------------------------
extern "C" void TIM5_IRQHandler(void)
{
    // Acknowledge the IRQ.
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);

    // Read all buttons
    button_reader<num_buttons-1>::read(Buttons, Buttons_state);

    // call handlers
    for (auto &e : Buttons_state)
    {
        if (e.newval)
        {
            e.handlers.at(Button_info::i_downHandler)();
            if (e.stateChanged())
                e.handlers.at(Button_info::i_pressedHandler)();
        }
        else
        {
            e.handlers.at(Button_info::i_upHandler)();
            if (e.stateChanged())
                e.handlers.at(Button_info::i_releasedHandler)();
        }
    }
}

//------------------------------------------------------------------------------

