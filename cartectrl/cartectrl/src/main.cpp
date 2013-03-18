#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_flash.h>
#include <stm32f4xx_rcc.h>

#include "herkulex.h"
#include "Tools.h"

#define DELAYELEM 2000000L

/* This funcion shows how to initialize
 * the GPIO pins on GPIOD and how to configure
 * them as inputs and outputs
 */
void init_GPIO(void){

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
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/
  /* At this stage the HSI is already enabled */

  /* Enable Prefetch Buffer and set Flash Latency */

  //FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
  FLASH_SetLatency(FLASH_Latency_5);
  FLASH_PrefetchBufferCmd(ENABLE);

  FLASH_InstructionCacheCmd(ENABLE);

  #if 0
  /* HCLK = SYSCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

  /* PCLK = HCLK */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

  /* PLL configuration = (HSI/2) * 12 = ~48 MHz */
  RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);
  #endif
  RCC->CFGR = 0x00009400;

  RCC->PLLCFGR = 0x04005410;
  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;


  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }

  /* Select PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }

}

int main(void){

  // initialize the GPIO pins we need
  initClock();
  init_GPIO();

  /* This flashed the LEDs on the board once
   * Two registers are used to set the pins (pin level is VCC)
   * or to reset the pins (pin level is GND)
   *
   * BSRR stands for bit set/reset register
   * it is seperated into a high and a low word (each of 16 bit size)
   *
   * A logical 1 in BSRRL will set the pin and a logical 1 in BSRRH will
   * reset the pin. A logical 0 in either register has no effect
   */
/*
  GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
  Tools::Delay(10000000L);		 // wait a short period of time
  GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15

  while (1){

		GPIOD->BSRRL = 0x1000; // this sets LED1 (green)
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRL = 0x2000; // this sets LED2 (orange)
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRL = 0x4000; // this sets LED3 (red)
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRL = 0x8000; // this sets LED4
		Tools::Delay(DELAYELEM);

		GPIOD->BSRRH = 0x1000; // this resets LED1
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRH = 0x2000; // this resets LED2
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRH = 0x4000; // this resets LED3
		Tools::Delay(DELAYELEM);
		GPIOD->BSRRH = 0x8000; // this resets LED4 (blue)
		Tools::Delay(DELAYELEM);
	}
*/

  while (1){

      Herkulex servomotor = Herkulex();

      servomotor.clear(0xFD);

      servomotor.setTorque(0xFD, TORQUE_ON);
      uint16_t position(0);
      while(1)
      {
          position = servomotor.getPos(0xFD);
          servomotor.positionControl(0xFD, 1000, 60, 0x00);
          Tools::Delay(90000000);

          position = servomotor.getPos(0xFD);
          servomotor.positionControl(0xFD, 23, 60, 0x00);
          Tools::Delay(90000000);

      }

     //Delay(9000000);
     //UARTUtility::USART_puts(USART1, "Hello World!!");
  }
}
