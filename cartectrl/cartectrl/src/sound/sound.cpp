/**
  ******************************************************************************
  * @file    sound.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-09-02
  * @brief   Sound routines
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include <sound/sound.hpp>
//------------------------------------------------------------------------------
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_dac.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_tim.h>
//------------------------------------------------------------------------------


using std::uint16_t;
using std::int16_t;
using std::uint8_t;
using std::int8_t;
using std::size_t;



enum {
    sound_buffer_size = 1024,
    half_sound_buffer = sound_buffer_size / 2
};

namespace snd
{

class SoundManager
{
    friend SoundStream;
public:
    SoundManager();
    void processSound(uint16_t *const buf, size_t siz);
private:
    SoundStream *head = nullptr;
};

static SoundManager sndman;

} // namespace snd

static uint16_t sample_buffer[sound_buffer_size];
static int16_t  sound_process_buffer[half_sound_buffer];

static uint16_t *const lobuf = sample_buffer;
static uint16_t *const hibuf = sample_buffer + half_sound_buffer;


/**
  * @brief ISR that take care of generating sound
  *
  */
extern "C" void DMA1_Stream5_IRQHandler(void)
{
    uint16_t *buf;
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        // update upper buffer
        buf = hibuf;
    }
    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF5))
    {
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF5);
        // update lower buffer
        buf = lobuf;
    }

    std::fill_n(buf, half_sound_buffer, 0);

    snd::sndman.processSound(buf, half_sound_buffer);

    // turn unsigned samples (0--FFFF) into signed (-7FFF--7FFF) samples
    for (size_t i=half_sound_buffer; i--;)
    {
        *buf++ ^= 0x8000;
    }
}


namespace snd
{

#define SIGN_EXT8(x)        ((int)(int8_t)(x))
#define SIGN_EXT16(x)       ((int)(int16_t)(x))
#define APPLY_VOLUME(x, g)  ((int)((SIGN_EXT16(x) * (g)) >> 16))

/**
  * @brief Iterate over the sound objects to produce sound
  * @param buf Where to put sample data
  */
void SoundManager::processSound(uint16_t *const buf, size_t siz)
{
    for (auto snd = head; snd; snd = snd->next)
    {
        if (!snd->isPlaying())
            continue;

        auto status = snd->fillAudioData(sound_process_buffer,
                                         half_sound_buffer);
        if (!status)
        {
            snd->stop();
        }
        else for (auto i = 0u;
                  i < half_sound_buffer && i < siz;
                  ++i)
        {
            buf[i] += (uint16_t) APPLY_VOLUME(sound_process_buffer[i],
                                              snd->volume());
        }
    }
}

/**
  * @brief  Initialise the sound system
  * @return 0 (Success)
  * @note   An overview of the inner working of sound generation:
  *
  *         @par
  *         To produce some sound, one has to periodically feed some data to a
  *         digital to analog converter (DAC). The MCU embeds two of them.
  *         To do this, we use a DMA peripheral (Direct Memory Access).
  *         A programmable DMA like in this MCU can transfer some memory from
  *         on place to another without further CPU intervention. Below, we
  *         program a DMA to read from a buffer and write to the DAC. The DMA
  *         transfer is triggered by the DAC, which is itself triggered by a
  *         timer (TIM6), which ticks to a constant sample rate (48kHz). The
  *         DMA is configured to loop over the buffer and issues an interrupt
  *         when the lower half and the upper half of the buffer has been
  *         transfered (roughly speaking). The application just has to fill the
  *         appropriate part of the sound buffer with new sound data.
  */
SoundManager::SoundManager()
{
    // Enable clocks on needed peripherals
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN, ENABLE);

    // Configure the DAC pin.
    GPIO_InitTypeDef gpinit[1] = {{0}};
    gpinit->GPIO_Pin   = GPIO_Pin_4;
    gpinit->GPIO_Mode  = GPIO_Mode_AIN;
    gpinit->GPIO_Speed = GPIO_Speed_50MHz;
    gpinit->GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, gpinit);

    // Configure the DAC.
    DAC_DeInit();
    DAC_InitTypeDef dac[1];
    dac->DAC_Trigger = DAC_Trigger_T6_TRGO;
    dac->DAC_WaveGeneration = DAC_WaveGeneration_None;
    dac->DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
    dac->DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, dac);
    DAC_Cmd(DAC_Channel_1, ENABLE);

    // Configure the DMA.
    DMA_DeInit(DMA1_Stream5);
    DMA_InitTypeDef dma[1];
    dma->DMA_Channel = DMA_Channel_7;
    dma->DMA_PeripheralBaseAddr = (uint32_t) &DAC->DHR12L1;
    dma->DMA_Memory0BaseAddr = (uint32_t) sample_buffer;
    dma->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma->DMA_BufferSize = sound_buffer_size;
    dma->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma->DMA_Mode = DMA_Mode_Circular;
    dma->DMA_Priority = DMA_Priority_High;
    dma->DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma->DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dma->DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, dma);

    // Configure DMA interrupts
    NVIC_InitTypeDef nvic[1];
    nvic->NVIC_IRQChannel = DMA1_Stream5_IRQn;
    nvic->NVIC_IRQChannelPreemptionPriority = 7;
    nvic->NVIC_IRQChannelSubPriority = 7;
    nvic->NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(nvic);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);

    // Enable DMA on DAC.
    DAC_DMACmd(DAC_Channel_1, ENABLE);


    // Configure Timer 6
    TIM_DeInit(TIM6);
    TIM_TimeBaseInitTypeDef    tim[1];
    TIM_TimeBaseStructInit(tim);
    tim->TIM_Period = 1749; // TODO: put a var here
    tim->TIM_Prescaler = 0;
    tim->TIM_ClockDivision = 0;
    tim->TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, tim);

    TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

    // Enable Timer. Sound (albeit silent) is produced at this instant.
    TIM_Cmd(TIM6, ENABLE);
}
/**
  * @brief Adds an SoundStream object to the SoundManager's queue
  * @param s SoundStream object to be added in the queue
  * @note  Is not thread safe, but is ISR-safe
  */
void SoundStream::imp_init()
{
    next = sndman.head;
    sndman.head = this;
}
/**
  * @brief Removes an SoundStream object to the SoundManager's queue
  * @note  Is not thread safe, but is ISR-safe
  */
void SoundStream::imp_deinit()
{
    auto p = sndman.head;
    while (p && p->next != this) p = p->next;
    if (p) p->next = this->next;
}
//------------------------------------------------------------------------------


} // namespace snd

