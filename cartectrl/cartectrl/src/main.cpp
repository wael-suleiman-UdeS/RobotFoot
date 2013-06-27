
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_flash.h>
#include <stm32f4xx_rcc.h>
#include "initClock.h"
#include "herkulex.h"
#include "usb_com.h"
#include "Tools.h"
#include "stm32f4xx_conf.h"

#include <stm32f4xx_dac.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_tim.h>
#include <cstdlib>

#include "CortexM4.h"



uint16_t buffer[1024];

uint16_t *const lobuf = buffer;
uint16_t *const hibuf = buffer + 512;

static uint32_t phase;

#define NUMEL(x) (sizeof(x)/sizeof(*x))

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

    for (int i = 0; i < 512; ++i)
    {
        int out = phase >> 15;

        if (out & 0x10000)
        {
            out ^= 0x1FFFF;
        }
        *buf++ = (out * 0xAFFF) >> 16;
        phase +=  23409859;
    }
}


int init_DAC()
{
    GPIO_InitTypeDef gpinit[1] = {{0}};
    gpinit->GPIO_Pin   = GPIO_Pin_4;
    gpinit->GPIO_Mode  = GPIO_Mode_AIN;
    gpinit->GPIO_Speed = GPIO_Speed_50MHz;
    gpinit->GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, gpinit);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);



    DAC_DeInit();
    DAC_InitTypeDef dac[1];
    dac->DAC_Trigger = DAC_Trigger_T6_TRGO;
    dac->DAC_WaveGeneration = DAC_WaveGeneration_None;
    dac->DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
    dac->DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, dac);

    DAC_Cmd(DAC_Channel_1, ENABLE);

    DMA_DeInit(DMA1_Stream5);
    DMA_InitTypeDef dma[1];
    dma->DMA_Channel = DMA_Channel_7;
    dma->DMA_PeripheralBaseAddr = (uint32_t) &DAC->DHR12L1;
    dma->DMA_Memory0BaseAddr = (uint32_t) buffer;
    dma->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma->DMA_BufferSize = NUMEL(buffer);
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

    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);

    NVIC_InitTypeDef nvic[1];
    nvic->NVIC_IRQChannel = DMA1_Stream5_IRQn;
    nvic->NVIC_IRQChannelPreemptionPriority = 7;
    nvic->NVIC_IRQChannelSubPriority = 7;
    nvic->NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(nvic);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    DAC_DMACmd(DAC_Channel_1, ENABLE);

    TIM_DeInit(TIM6);

    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 1749;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

    TIM_Cmd(TIM6, ENABLE);


    return 0;
}

/* This funcion shows how to initialize
 * the GPIO pins on GPIOD and how to configure
 * them as inputs and outputs
 */

void init_GPIO(void)
{

    /* This TypeDef is a structure defined in the
     * ST's library and it contains all the properties
     * the corresponding peripheral has, such as output mode,
     * pullup / pulldown resistors etc.
     *
     * These structures are defined for every peripheral so
     * every peripheral has it's own TypeDef. The good news is
     * they always work the same so once you've got a hang
     * of it you can initialize any peripheral.
     *
     * The properties of the periperals can be found in the corresponding
     * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
     */
    GPIO_InitTypeDef GPIO_InitStruct;

    /* This enables the peripheral clock to the GPIOD IO module
     * Every peripheral's clock has to be enabled
     *
     * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
     * datasheet contain the information which peripheral clock has to be used.
     *
     * It is also mentioned at the beginning of the peripheral library's
     * source file, e.g. stm32f4xx_gpio.c
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* In this block of instructions all the properties
     * of the peripheral, the GPIO port in this case,
     * are filled with actual information and then
     * given to the Init function which takes care of
     * the low level stuff (setting the correct bits in the
     * peripheral's control register)
     *
     *
     * The LEDs on the STM324F Discovery are connected to the
     * pins PD12 thru PD15
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14; // we want to configure all LED GPIO pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
    GPIO_Init(GPIOE, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

    /* This enables the peripheral clock to
     * the GPIOA IO module
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Here the GPIOA module is initialized.
     * We want to use PA0 as an input because
     * the USER button on the board is connected
     * between this pin and VCC.
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		  // we want to configure PA0
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   // this enables the pulldown resistor --> we want to detect a high level
    GPIO_Init(GPIOC, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}

int main(void)
{

    // initialize the GPIO pins we need
    initClock();
    init_GPIO();
    init_DAC();
    usb::init();
    Herkulex hercules;
    Tools::Delay(Tools::DELAY_AROUND_1S);

    hercules.setTorque(DEFAULT_ID, TORQUE_ON);

    unsigned debounce = 10000, oldb=0;

    for(;;)
    {
        char p[4];
        uint32_t r = usb::read(p);
        for (int i=0; i < r; ++i) p[i] += 2;
        usb::write(p, r);
        if (r) GPIOE->ODR += 0x4000;
        if (!debounce--)
        {
            const unsigned b = ~GPIOC->IDR;
            const unsigned p = (b ^ oldb) & b;
            if (p & 2)
            {
                GPIOE->ODR ^= 0x8000;
                hercules.positionControl(DEFAULT_ID, 512 + 300, 40, 0);
            }
            else if (p & 4)
            {
                GPIOE->ODR ^= 0x4000;
                hercules.positionControl(DEFAULT_ID, 512, 40, 0);
            }
            else if (p & 8)
            {
                GPIOE->ODR -= 0x4000;
                hercules.positionControl(DEFAULT_ID, 512 - 300, 40, 0);
            }
            oldb = b;
            debounce = 10000;
        }
    }

}
