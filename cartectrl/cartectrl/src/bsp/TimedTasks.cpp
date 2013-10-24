/**
  ******************************************************************************
  * @file    TimedTasks.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-27
  * @brief   Do something in a timely fashion
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "bsp/TimedTasks.hpp"
#include "misc.h"
//------------------------------------------------------------------------------

namespace bsp
{

static auto emptyTask = funcref<void()>::make([] {});


class TimedTasks_imp : public TimedTasks
{
    basic_task *task = nullptr;
public:
    TimedTasks_imp()
    {
        SysTick->VAL  = SysTick->LOAD = 168000-1;
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk
                      | SysTick_CTRL_CLKSOURCE_Msk;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
        NVIC_InitTypeDef nvic[1];
        nvic->NVIC_IRQChannel = SysTick_IRQn;
        nvic->NVIC_IRQChannelPreemptionPriority = 3;
        nvic->NVIC_IRQChannelSubPriority = 1;
        nvic->NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(nvic);
    }

    void call()
    {
        for (auto p = task; p != nullptr; p->call(), p = p->next)
        {}
    }

    void add(basic_task &t) override
    {
        t.next = task;
        task   = &t;
    }

    void remove(basic_task &t) override
    {
        for (auto p = task; p != nullptr; p = p->next)
        {
            if (p->next == &t)
            {
                p->next = t.next;
                return;
            }
        }
    }

    static TimedTasks_imp *GetInstance()
    {
        return static_cast<TimedTasks_imp *>(TimedTasks::GetInstance());
    }
};
//------------------------------------------------------------------------------
void basic_task::unref_safe(basic_task *b)
{
    b->f = &emptyTask;
}
//------------------------------------------------------------------------------
basic_task::~basic_task()
{
    TimedTasks::GetInstance()->remove(*this);
}
//------------------------------------------------------------------------------
TimedTasks *TimedTasks::GetInstance()
{
    static TimedTasks_imp timp[1];
    return timp;
}
//------------------------------------------------------------------------------

} // namespace bsp
//------------------------------------------------------------------------------
extern "C" void SysTick_Handler(void)
{
    return bsp::TimedTasks_imp::GetInstance()->call();
}
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------


