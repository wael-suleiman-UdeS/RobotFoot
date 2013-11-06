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


#include "usb/usb_com.hpp"
#include <cstring>
#include <sstream>
#include <iomanip>

void usbsend(const char *s)
{
    auto e = std::strlen(s);
    const char *const endp = s + e;

    while(s != endp)
    {
        auto r = usb::write(s,endp-s);
        if (r > 0) s+= r;
    }
}

void usbread(char *s, unsigned len)
{
    char *const endp = s + len;

    while(s != endp)
    {
        auto r = usb::read(s,endp-s);
        if (r > 0) s+= r;
    }
}
void usbreadlf(char *s, unsigned len)
{
    char *const endp = s + len;

    while(s != endp)
    {
        auto r = usb::read(s,1);
        if (r > 0)
        {
            if (*s == '\r') { *s = 0; break; }
            else { usb::write(s, 1); ++s; }
        }
    }
}

int GYACC_read(bool b, unsigned axes)
{
    if (axes > 2u) axes = 2u;

    const uint16_t base = 0xA800 + (axes << 9);

    int out = (GYACC_txrx(b, base) & 0xFF);
    Tools::Delay(1);
    out |= ((GYACC_txrx(b, base+0x10) & 0xFF) << 8);
    if (out & 0x8000) out |= 0xFFFF0000;
    return out;
}

extern "C" int _open() { return -1; }
//------------------------------------------------------------------------------

int main(void)
{
    init_GYACC();
    usb::init();

    bsp::vsense.setLimits(7.2f, 9.5f);
    char bbb[128] = {0};

    GYACC_txrx(USE_ACC,  0x2037);
    GYACC_txrx(USE_GYRO, 0x200F);

    for (;;)
    {
        usbsend("(GYRO/ACC  READ ADDR HEX)> ");
        usbreadlf(bbb, sizeof bbb);
        usbsend("\r\n");
        if (bbb[std::strspn(bbb," 0123456789ABCDEFabcdef")])
        {
            usbsend("GARBAGE DETECTED...\r\n");
        }
        else if (bbb[sizeof bbb - 1])
        {
            usbsend("TOO LONG LINE...\r\n");
        }
        else
        {
            int g = 0, d = 0;
            std::stringstream ss(bbb);
            ss >> std::hex >> g >> d;
            if (ss)
            {
                d = ((d & 0x2F) << 8) | 0x8000;
                const int r = GYACC_txrx(g, d);
                int c = r;
                bbb[0]='0';
                bbb[1]='x';
                for (int i=4; i--;)
                {
                    bbb[2+i] = "0123456789ABCDEF"[c&0xF];
                    c >>= 4;
                }
                bbb[6]=0;

                usbsend("RESULT: ");
                usbsend(bbb);
                usbsend("\r\n");
            }
            else
            {
                usbsend("EXTRACTION ERROR.\r\n");
            }
        }
    }

//    unsigned g = GYACC_txrx(USE_GYRO, 0x8F00);
//    unsigned a = GYACC_txrx(USE_ACC,  0xA000);
}
