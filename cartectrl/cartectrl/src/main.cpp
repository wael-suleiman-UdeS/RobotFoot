#include "LEDs.hpp"
#include "vsense.hpp"
#include "Tools.h"
#include "USBProtocol.hpp"
#include "herkulex.h"
#include "UARTs.hpp"
//------------------------------------------------------------------------------


#include <stm32f4xx_spi.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
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
    gpio->GPIO_Pin  = 0xF0; // 4, 5, 6, 7 (5 is useless though)
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

int main(void)
{
    Herkulex    herk    = {&UART3i};
    HerkulexMan herkman = {&herk};
    USBProtocol app     = {&herkman};

    herk.send_reboot(BROADCAST_ID);

    bsp::vsense.setLimits(7.2f, 9.5f);
    // wait
    Tools::Delay(Tools::DELAY_AROUND_1S/10);

    // Application loop
    app.loop();
}
//------------------------------------------------------------------------------

