#pragma once
#include "stm32f411xe.h"

/*******************************************
 *              Tryby GPIO                 *
 *******************************************/
#define GPIO_MODE_INPUT      0x0U
#define GPIO_MODE_OUTPUT     0x1U
#define GPIO_MODE_AF         0x2U
#define GPIO_MODE_ANALOG     0x3U

#define GPIO_PULL_NONE       0x0U
#define GPIO_PULL_UP         0x1U
#define GPIO_PULL_DOWN       0x2U

#define GPIO_OTYPE_PP        0x0U   // Push-pull
#define GPIO_OTYPE_OD        0x1U   // Open-drain

#define GPIO_SPEED_LOW       0x0U
#define GPIO_SPEED_MEDIUM    0x1U
#define GPIO_SPEED_HIGH      0x2U
#define GPIO_SPEED_VERY_HIGH 0x3U

#define GPIO_PIN_SET         1U
#define GPIO_PIN_RESET       0U

/*******************************************
 *         Deklaracje funkcji              *
 *******************************************/
void GPIO_EnableClock(GPIO_TypeDef *Port);
void GPIO_ConfigPin(GPIO_TypeDef *Port, uint32_t Pin,
                    uint32_t Mode, uint32_t Pull, uint32_t OType);
void GPIO_SetSpeed(GPIO_TypeDef *Port, uint32_t Pin, uint32_t Speed);
void GPIO_SetAF(GPIO_TypeDef *Port, uint32_t Pin, uint8_t AF);
void GPIO_WritePin(GPIO_TypeDef *Port, uint32_t Pin, uint8_t State);
void GPIO_TogglePin(GPIO_TypeDef *Port, uint32_t Pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *Port, uint32_t Pin);