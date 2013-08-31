
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_flash.h>
#include <stm32f4xx_rcc.h>
#include "initClock.h"
#include "herkulex.h"
#include "usb_com.h"
#include "Tools.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"
#include "CortexM4.h"

#include <stm32f4xx_dac.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_tim.h>
#include <cstdlib>

#include <stm32f4xx_spi.h>

#include "CortexM4.h"

#include <cstring>

// DAC part {

uint16_t buffer[1024];

static uint16_t *const lobuf = buffer;
static uint16_t *const hibuf = buffer + 512;

#define NUMEL(x)            (sizeof(x)/sizeof(*x))
#define SIGN_EXT8(x)        ((int)(int8_t)(x))
#define SIGN_EXT16(x)       ((int)(int16_t)(x))
#define APPLY_VOLUME(x, g)  ((SIGN_EXT16(x) * (g)) >> 16)

static void (*DAC_data_handler)(int16_t *ptr, size_t size) = 0;

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
    if (DAC_data_handler)
        DAC_data_handler((int16_t *)buf, 512);
    else
        std::memset(buf, 0, 512*2);
    for (size_t i=512; i--;)
    {
        *buf++ ^= 0x8000;
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
// } ADC part {
// TEST!
static void panic_sound(int16_t *buf, size_t size)
{
    static uint16_t key = 0;
    static uint8_t  phase1 = 0, phase2 = 0x40;


    enum { freqp = 16, freqk = 3};

    for (;size--;)
    {
        key     += freqk;
        phase1  += freqp;
        phase2  += freqp;

        int16_t out = 0;

        if (key & 0x8000)
        {

            out = APPLY_VOLUME(
                                (phase2 ^ -!(phase1 & 0x80)) << 9,
                                0xBFFF);

        }
        *buf++ = out;
    }
}

extern "C" void ADC_IRQHandler()
{
    // Assume interrupt comes from analog watchdog
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    ADC_ITConfig(ADC1, ADC_IT_AWD, DISABLE);    // No longer need this interrupt
    // Do something...
    DAC_data_handler = panic_sound;
}

void init_ADC(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOAEN, ENABLE);
    GPIO_InitTypeDef gpini[1] = {{0}};
    gpini->GPIO_Pin = GPIO_Pin_0;
    gpini->GPIO_Mode = GPIO_Mode_AIN;
    gpini->GPIO_Speed = GPIO_Speed_2MHz;
    gpini->GPIO_OType = GPIO_OType_PP;
    gpini->GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, gpini);

    ADC_InitTypeDef ini[1] = {{0}};
    ADC_StructInit(ini);
    // must be 30MHz max

    ini->ADC_Resolution = ADC_Resolution_12b;
    ini->ADC_ScanConvMode = DISABLE;
    ini->ADC_ContinuousConvMode = ENABLE;
    ini->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ini->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_TRGO; // Don't care
    ini->ADC_DataAlign = ADC_DataAlign_Left;
    ini->ADC_NbrOfConversion = 1;

    ADC_CommonInitTypeDef inic[1] = {{0}};
    ADC_CommonStructInit(inic);
    inic->ADC_Mode = ADC_Mode_Independent;
    inic->ADC_Prescaler = ADC_Prescaler_Div8;
    inic->ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    inic->ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles; // don't care

    ADC_DeInit();
    ADC_CommonInit(inic);
    ADC_Init(ADC1, ini);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
    ADC_ContinuousModeCmd(ADC1, ENABLE);
    ADC_EOCOnEachRegularChannelCmd(ADC1, DISABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);

}

void enable_ADC_watchdog(uint16_t low, uint16_t high)
{
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_0);
    ADC_AnalogWatchdogThresholdsConfig(ADC1, high, low);
    ADC_ClearFlag(ADC1, ADC_FLAG_AWD);
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// } GYRO/ACC {
void init_GYACC(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIODEN, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_SPI3EN, ENABLE);

    GPIO_InitTypeDef gpio[1];
    gpio->GPIO_Pin  = 0x0F; // 0, 1, 2, 3
    gpio->GPIO_Mode = GPIO_Mode_IN;
    gpio->GPIO_Speed= GPIO_Speed_50MHz;
    gpio->GPIO_OType= GPIO_OType_OD;
    gpio->GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, gpio);

    GPIOD->ODR |= 0xC0;
    gpio->GPIO_Pin  = 0xF0; // 4, 5, 6, 7 (5 is useless although)
    gpio->GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOD, gpio);

    gpio->GPIO_Pin  = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    gpio->GPIO_Mode = GPIO_Mode_AF;
    gpio->GPIO_Speed= GPIO_Speed_50MHz;
    gpio->GPIO_OType= GPIO_OType_PP;
    gpio->GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, gpio);

   	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); //
   	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

    SPI_InitTypeDef spi[1];
    SPI_StructInit(spi);
    spi->SPI_Direction  = SPI_Direction_2Lines_FullDuplex;
    spi->SPI_Mode       = SPI_Mode_Master;
    spi->SPI_DataSize   = SPI_DataSize_16b;
    spi->SPI_CPOL       = SPI_CPOL_High;
    spi->SPI_CPHA       = SPI_CPHA_2Edge;
    spi->SPI_NSS        = SPI_NSS_Soft;
    spi->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi->SPI_FirstBit   = SPI_FirstBit_MSB;
    SPI_Init(SPI3, spi);
    SPI_Cmd(SPI3, ENABLE);
}

uint16_t GYACC_txrx(bool which, uint16_t data)
{
    const uint32_t pin = which ? 0x80 : 0x40;
    GPIOD->ODR &= ~pin;
    SPI3->DR = data;
    while(SPI_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY));
    GPIOD->ODR |=  pin;
    return SPI3->DR;
}
enum { USE_GYRO, USE_ACC };
// }

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
#define DEFAULT_ID 2
int main(void)
{
    bool isOn = false;
    // initialize the GPIO pins we need
    initClock();
    init_GPIO();

    init_ADC();

    init_DAC();
    init_GYACC();
    usb::init();

    Herkulex hercules;
    hercules.reset(0xFE); // TEST


    GPIOE->ODR |=  0xC000;
    Tools::Delay(Tools::DELAY_AROUND_1S / 2);
    GPIOE->ODR &= ~0xC000;
    enable_ADC_watchdog(2760, 3870);    // ~6.9V -- ~9.5V (V33 = 3.41V)



    unsigned debounce = 10000, oldb=0;
    unsigned g = GYACC_txrx(USE_GYRO, 0x8F00);
    unsigned a = GYACC_txrx(USE_ACC,  0xA000);

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
            if (!isOn && (p & 0xE))
            {
                isOn = false;
                hercules.setTorque(DEFAULT_ID, TORQUE_ON);
            }
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
