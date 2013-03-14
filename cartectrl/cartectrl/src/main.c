#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"

USB_OTG_CORE_HANDLE  USB_OTG_dev __attribute__ ((aligned (4)));

void Delay(volatile uint32_t nCount)
{
    while(nCount--)
    {
    }
}

#define DELAYELEM 2000000L

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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

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
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
    GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

    /* This enables the peripheral clock to
     * the GPIOA IO module
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Here the GPIOA module is initialized.
     * We want to use PA0 as an input because
     * the USER button on the board is connected
     * between this pin and VCC.
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
    GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}

void initClock(void)
{
    /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/
    /* At this stage the HSI is already enabled */

    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH_SetLatency(FLASH_Latency_5);
    FLASH_PrefetchBufferCmd(ENABLE);

    FLASH_InstructionCacheCmd(ENABLE);
    FLASH_DataCacheCmd(ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1ENR_PWREN, DISABLE);
    PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    RCC_WaitForHSEStartUp(); // NOTE: return value discarded!
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
    RCC_PLLCmd(ENABLE);
    RCC_HCLKConfig(RCC_HCLK_Div1);   // APB1 = 42MHz
    RCC_PCLK1Config(RCC_HCLK_Div4);   // APB1 = 42MHz
    RCC_PCLK2Config(RCC_HCLK_Div2);   // APB2 = 84MHz

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }


}

int main(void)
{
    int len;
    uint8_t buf[255];
    // initialize the GPIO pins we need
    initClock();
    init_GPIO();

    GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
    Delay(10000000L);		 // wait a short period of time
    GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15
    Delay(10000000L);

    USBD_Init(&USB_OTG_dev,
              USB_OTG_FS_CORE_ID,
              &USR_desc,
              &USBD_CDC_cb,
              &USR_cb);

    int i = 0;
    for (;;)
    {
        if (i++ > 100000)
        {
            GPIOD->ODR ^= 0xF000;
            i = 0;
        }
        len = VCP_get_string(buf);
        if(len)
            VCP_send_str(buf);
    }
}

void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
