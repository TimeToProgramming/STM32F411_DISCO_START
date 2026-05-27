#pragma once
#include "stm32f411xe.h"

/*******************************************
 *           Tryby timera                  *
 *******************************************/
#define TIM_MODE_CONTINUOUS  0x01U
#define TIM_MODE_ONE_SHOT    0x02U
#define TIM_MODE_PWM         0x04U

#define TIM_PWM_MODE1        1U
#define TIM_PWM_MODE2        2U

#define TIM_POLARITY_HIGH    1U
#define TIM_POLARITY_LOW     0U

/*******************************************
 *         Deklaracje funkcji              *
 *******************************************/
void TIM_EnableClock(TIM_TypeDef *TIMx);
void TIM_Reset(TIM_TypeDef *TIMx);
void TIM_Config(TIM_TypeDef *TIMx, uint16_t Mode,
                uint32_t PSC, uint32_t ARR);
void TIM_Start(TIM_TypeDef *TIMx);
void TIM_Stop(TIM_TypeDef *TIMx);
void TIM_EnableIRQ(TIM_TypeDef *TIMx);
void TIM_DisableIRQ(TIM_TypeDef *TIMx);
void TIM_ClearUpdateFlag(TIM_TypeDef *TIMx);
void TIM_ConfigPWM(TIM_TypeDef *TIMx, uint8_t Channel,
                   uint32_t PSC, uint32_t ARR, uint32_t CCR,
                   uint8_t Polarity, uint8_t PWMMode);
void TIM_SetCCR(TIM_TypeDef *TIMx, uint8_t Channel, uint32_t CCR);
void TIM_ConfigChannel(TIM_TypeDef *TIMx, uint8_t Channel,
                       uint32_t CCR, uint8_t Polarity, uint8_t PWMMode);