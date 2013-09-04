#include "EmergencyStop.h"

#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#include "herkulex.h"

EmergencyStop::EmergencyStop()
{
}

EmergencyStop::~EmergencyStop()
{
}

void EmergencyStop::StartEmergencyButton()
{
    initBtnPollingTimer();
}

/*
 *  This funcion initializes the timer that polls the state of the buttons
 */
void EmergencyStop::initBtnPollingTimer()
{
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5EN, ENABLE);
    TIM_DeInit(TIM5);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 1680000;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM5, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

unsigned buttonsStates;
extern "C" void ButtonTimer(void)
{
    enum { PB_MASK = 0x000E };
    buttonsStates = ~(GPIOC->IDR & PB_MASK);

    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    if(buttonsStates & 0x0008)
    {
        Herkulex::GetInstance()->setTorque(BROADCAST_ID, TORQUE_FREE);
    }
}
