// exti.h
#pragma once
#include "stm32f411xe.h"

#define EXTI_TRIGGER_RISING   0x01U
#define EXTI_TRIGGER_FALLING  0x02U
#define EXTI_TRIGGER_BOTH     0x03U

void EXTI_Config(GPIO_TypeDef *Port, uint8_t Pin, uint32_t Trigger);
void EXTI_Enable(uint8_t Pin);
void EXTI_Disable(uint8_t Pin);
void EXTI_ClearFlag(uint8_t Pin);