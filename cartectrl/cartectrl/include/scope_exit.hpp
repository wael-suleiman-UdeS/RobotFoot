/**
  ******************************************************************************
  * @file    scope_exit.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-21
  * @brief   Run some code at scope exit.
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef SCOPE_EXIT_HPP
#define SCOPE_EXIT_HPP
//------------------------------------------------------------------------------

// Utility object.
template <typename Func>
 struct scope_exit_obj : Func
{
    scope_exit_obj(Func &&f) : Func(std::move(f)) {}
    ~scope_exit_obj() { (*this)(); }
};

/**
 *  Pass any function object (function pointer, functor, lamda) and it will
 *  call it when the object get destroyed. Keep the returned object in a
 *  variable!
 *
 * Example:
 * @code
 * auto se = scope_exit([&] { (do something...) });
 * @endcode
 */

template <typename Func>
 scope_exit_obj<Func> scope_exit(Func &&f)
{
    return {std::move(f)};
}

//------------------------------------------------------------------------------
#endif // SCOPE_EXIT_HPP
