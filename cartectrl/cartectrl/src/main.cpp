
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
    ini->ADC_ContinuousConvMode = ENABLE; //???
    ini->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ini->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_TRGO; // Don't care
    ini->ADC_DataAlign = ADC_DataAlign_Left;
    ini->ADC_NbrOfConversion = 1; //???

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

// } UART 2: {
void init_UART2()
{
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization


	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = 115200;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);
}
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

int main(void)
{

    // initialize the GPIO pins we need
    initClock();
    init_GPIO();

    init_ADC();

    init_DAC();

    usb::init();
    init_UART2();
    Herkulex hercules;

    GPIOE->ODR |=  0xC000;
    Tools::Delay(Tools::DELAY_AROUND_1S / 2);
    GPIOE->ODR &= ~0xC000;
    enable_ADC_watchdog(2760, 3870);    // ~6.9V -- ~9.5V (V33 = 3.41V)

    hercules.setTorque(DEFAULT_ID, TORQUE_ON);

    unsigned debounce = 10000, oldb=0;

    for(;;)
    {
        char p[4];
        uint32_t r = usb::read(p);
        for (int i=0; i < r; ++i) p[i] += 2;
        usb::write(p, r);
        if (r) GPIOE->ODR += 0x4000;
        if (USART2->SR & 0x20)
        {
            USART2->DR += 1;
            GPIOE->ODR += 0x4000;
        }
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
