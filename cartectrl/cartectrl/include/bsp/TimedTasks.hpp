/**
  ******************************************************************************
  * @file    TimedTasks.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-27
  * @brief   Do something in a timely fashion
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef TIMEDTASKS_HPP
#define TIMEDTASKS_HPP
//------------------------------------------------------------------------------
#include "miscutils.h"
//------------------------------------------------------------------------------

namespace bsp
{

class basic_task
{
    friend class TimedTasks_imp;

    funcref<void()> *f;
    basic_task *next = nullptr;

    void call() const { return (*f)(); }
protected:
    static void unref_safe(basic_task *);
public:
    basic_task(funcref<void()> *f) : f{f} {}
    ~basic_task();
};

template <typename F>
 class Task : public funcref_imp<F, void()>, public basic_task
{
public:
    Task(F &&f) : funcref_imp<F, void()>(std::move(f)), basic_task(this) {}
    ~Task() { unref_safe(this); }
};

template <typename F>
 Task<F> make_Task(F &&f)
{
    return {std::move(f)};
}

class TimedTasks
{
protected:
    TimedTasks() {}
public:
    static TimedTasks *GetInstance();

    virtual void add   (basic_task &) = 0;
    virtual void remove(basic_task &) = 0;
};

class CPUclockdiff
{
public:
    static unsigned val();

    unsigned cpudiff() const { return (cnt - val()); }

    enum { cycle2us = 84 };

    void upd() { cnt = val(); }

    CPUclockdiff() { upd(); }
private:
    unsigned cnt = val();
};


} // namespace bsp

//------------------------------------------------------------------------------
#endif // TIMEDTASK_HPP
