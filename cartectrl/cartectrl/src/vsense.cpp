/**
  ******************************************************************************
  * @file    vsense.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 1.0
  * @date    2013-11-06
  * @brief   Allow sensing of battery voltage
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "vsense.hpp"
//------------------------------------------------------------------------------
#include "bsp/GPIO.hpp"
#include "stm32f4xx_adc.h"
#include <stm32f4xx_gpio.h>
#include <numeric>
#include <iterator>
#include <LEDs.hpp>
#include <sound/sound.hpp>
//------------------------------------------------------------------------------


/**
 *  Sound object that just generate a fixed 4kHz triangular tone each .5 sec.
 */
class PanicSound : public snd::SoundStream
{
    uint16_t key;
    uint8_t  phase1;
    uint8_t  phase2;
    uint16_t freqk;

    virtual bool fillAudioData(int16_t *buf, std::size_t len) override
    {
        enum { freqp = 16 };

        for (;len--;)
        {
            key     += freqk;
            phase1  += freqp;
            phase2  += freqp;

            int16_t out = 0;

            if (key & 0x8000)
            {
                out = (phase2 ^ -!(phase1 & 0x80)) << 9;
            }

            *buf++ = out;
        }

        LED1.set(key & 0x8000);

        return true;
    }
public:
    void alarm(bool type)
    {
        freqk = type ? 3u : 20u;
        reset();
        play();
    }

    void reset()
    {
        key    = 0;
        phase1 = 0;
        phase2 = 0x40;
    }
};


namespace bsp
{

enum { adc_filter_size = 32 };

static uint16_t adc_filter[adc_filter_size];

using std::accumulate;
using std::distance;
using std::begin;
using std::end;

/**
 *  Implementation of VSense.
 */
class VSense_imp : public VSense
{
    float voltage;
public:
    virtual float getVoltage() const override
    {
        return voltage;
    }
    /**
     *  @brief Update the voltage and check if voltage range is exceeded.
     *  @param voltage New voltage value
     *  @note  There is a 0.2 Volts hysteresis to prevent useless switching
     *   of the alarm.
     */
    void updAlarm(float voltage)
    {
        const bool oldalarm = alarm;
        this->voltage = voltage;

        bool undervoltage = false;

        if (voltage < low_voltage || voltage > hi_voltage)
        {
            undervoltage = voltage < low_voltage;
            alarm = true;
        }
        else if ((voltage > (low_voltage + .2f))
                                            && (voltage < (hi_voltage - .2f)))
        {
            alarm = false;
        }
        //else nothing

        if (oldalarm != alarm)
        {
            if (alarm)
            {
                panic.alarm(undervoltage);
            }
            else
            {
                panic.stop();
                LED1.reset();
            }
        }
    }

    snd::Sound<PanicSound> panic;
};


static VSense_imp vsense_imp;
VSense &vsense = vsense_imp;

/**
 *  @brief Initialize the ADC, DMA, etc.
 */
VSense::VSense()
{
    // Enable GPIO pin
    GPIO_Pin<bsp::gpio::a, 0>(pinfunction::analog,
                              outtype::pushpull,
                              pullresistor::none,
                              iospeed::_2MHz);

    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    // Enable DMA
    DMA_DeInit(DMA2_Stream0);
    DMA_InitTypeDef dma[1];
    dma->DMA_Channel            = DMA_Channel_0;
    dma->DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    dma->DMA_Memory0BaseAddr    = (uint32_t) adc_filter;
    dma->DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma->DMA_BufferSize         = adc_filter_size;
    dma->DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma->DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma->DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma->DMA_Mode               = DMA_Mode_Circular;
    dma->DMA_Priority           = DMA_Priority_VeryHigh;
    dma->DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma->DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    dma->DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma->DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream0, dma);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    ADC_InitTypeDef ini[1] = {{0}};
    ADC_StructInit(ini);

    ini->ADC_Resolution = ADC_Resolution_12b;
    ini->ADC_ScanConvMode = DISABLE;
    ini->ADC_ContinuousConvMode = ENABLE;
    ini->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ini->ADC_DataAlign = ADC_DataAlign_Right;
    ini->ADC_NbrOfConversion = 1;

    ADC_CommonInitTypeDef inic[1] = {{0}};
    ADC_CommonStructInit(inic);

    inic->ADC_Mode = ADC_Mode_Independent;
    inic->ADC_Prescaler = ADC_Prescaler_Div8;
    inic->ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    inic->ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;

    ADC_DeInit();

    ADC_CommonInit(inic);
    ADC_Init(ADC1, ini);

    ADC_ContinuousModeCmd(ADC1, ENABLE);

    // Configure DMA interrupts
    NVIC_InitTypeDef nvic[1];
    nvic->NVIC_IRQChannel = DMA2_Stream0_IRQn;
    nvic->NVIC_IRQChannelPreemptionPriority = 10;
    nvic->NVIC_IRQChannelSubPriority = 7;
    nvic->NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(nvic);

    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_SoftwareStartConv(ADC1);
}
/**
 *  @brief IRQ handler for the DMA Stream 2 transfer complete interrupt.
 */
extern "C" void DMA2_Stream0_IRQHandler()
{
    static const float MCU_Voltage = 3.41f;

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        auto v = accumulate(begin(adc_filter), end(adc_filter), 0u);
        v /= distance(begin(adc_filter), end(adc_filter));

        vsense_imp.updAlarm(static_cast<float>(v) * (MCU_Voltage * 3.f/4095.f));
    }
}
//------------------------------------------------------------------------------


} // namespace bsp
