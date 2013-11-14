/**
  ******************************************************************************
  * @file    sound.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V0.1
  * @date    2013-08-31
  * @brief   Sound control
  ******************************************************************************
  */

#ifndef SOUND_HPP
#define SOUND_HPP
//------------------------------------------------------------------------------
#include <cstdint>
#include <utility>
//------------------------------------------------------------------------------

namespace snd
{

/**
 *  Base class. Users inherit from this class and implement fillAudioData
 *  to do something useful when there is something to play.
 *
 *  Note that you should not instantiate your derived class directly, as it will
 *  not be able to do any sound on its own. You should use the @c snd template
 *  class below.
 */
class SoundStream
{
    template <typename> friend class Sound;
    friend class SoundManager;
protected:
    SoundStream()           {}
    virtual ~SoundStream()  {}
    /**
     * @brief Virtual method that is called to generate sound
     * @param p Pointer to a buffer of size @c s
     * @param s Size of the buffer pointed by @c p.
     * @note  Do @e NOT block in this method, or do something complicated, it
     *        will make the entire system slow and unresponsive.
     */
    virtual bool fillAudioData(int16_t *const p, std::size_t s) = 0;
public:
    /**
     * @brief Does what you expect from its name.
     */
    void play()     { status_ = playing; }
    /**
     * @brief Does what you expect from its name, too.
     */
    void stop()     { status_ = stopped; }
    /**
     * @brief Get the volume
     * @note  See the overload below for details regarding how volume is
     *        interpreted.
     */
    auto volume() const -> std::uint16_t { return volume_; }
    /**
     * @brief Set the volume.
     * @param vol Volume to set.
     *        @c 0xFFFF → ~0dB 
     *        @c 0x0000 → -Inf dB
     *        Thus, it is like a 16-bit unsigned fractional number that goes
     *        from 0 to almost 1.
     */
    void volume(std::uint16_t vol)       { volume_ = vol;  }

    enum state : bool { stopped, playing };

    /**
     * @brief Does what you expect from its name.
     */
    state status() const    { return status_; }
    /**
     * @brief Does what you expect from its name.
     */
    bool  isStopped() const { return status_ == state::stopped; }
    /**
     * @brief Does what you expect from its name.
     */
    bool  isPlaying() const { return status_ == state::playing; }
private:

    void imp_init();
    void imp_deinit();

    mutable SoundStream *next = nullptr;


    state        status_      = stopped;
    std::int16_t volume_      = 0xC800;
};

/**
 *  Utility class. This class automatically register your sound object and
 *  unregister it when it is destroyed.
 *
 *  Example:
 *  @code
 *      using namespace snd; // Not strictly necessary
 *
 *      class mySound : public SoundStream
 *      {
 *          // Define the fillAudioData method.
 *          // ...
 *      };
 *
 *      ...
 *      {
 *          Sound<MySound> mysound( <em>MySound ctor args</em> );
 *          mysound.play();
 *          // You can stop it yourself; when going out of scope, it will stop by
 *          // itself.
 *      }
 *  @endcode
 */
template <typename Base>
 class Sound final : public Base
{
public:
    template <typename... Params>
     Sound(Params&&... params)
    : Base(std::forward<Params>(params)...)
    {
        this->imp_init();
    }
    ~Sound()
    {
        this->stop();
        this->imp_deinit();
    }
};


} // namespace snd
//------------------------------------------------------------------------------
#endif // SOUND_HPP

