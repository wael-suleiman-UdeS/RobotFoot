
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

extern "C" void ADC_IRQHandler()
{
    // Assume interrupt comes from analog watchdog
    ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    ADC_ITConfig(ADC1, ADC_IT_AWD, DISABLE);    // No longer need this interrupt
    // Do something...
    GPIOE->ODR = 0x8000;    //TEST
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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
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
    usb::init();
    Herkulex hercules;

    GPIOE->ODR |=  0xC000;
    Tools::Delay(Tools::DELAY_AROUND_1S / 2);
    GPIOE->ODR &= ~0xC000;
    enable_ADC_watchdog(2860, 3870);    // ~7.1V -- ~9.5V (V33 = 3.41V)

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
