#include "gpio.h"

void GPIO_EnableClock(GPIO_TypeDef *Port) {
    if      (Port == GPIOA) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; }
    else if (Port == GPIOB) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; }
    else if (Port == GPIOC) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; }
    else if (Port == GPIOD) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; }
    else if (Port == GPIOE) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOEEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; }
    else if (Port == GPIOH) { if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOHEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN; }
    // F411 ma: GPIOA-E oraz GPIOH — brak F,G
    (void)(RCC->AHB1ENR); // wymuszenie zapisu przed użyciem peryferiów
}

void GPIO_ConfigPin(GPIO_TypeDef *Port, uint32_t Pin,
                    uint32_t Mode, uint32_t Pull, uint32_t OType) {
    uint8_t shift2 = Pin * 2;

    Port->MODER  &= ~(3U << shift2);
    Port->MODER  |=  (Mode << shift2);

    Port->PUPDR  &= ~(3U << shift2);
    Port->PUPDR  |=  (Pull << shift2);

    if (Mode == GPIO_MODE_OUTPUT || Mode == GPIO_MODE_AF) {
        if (OType == GPIO_OTYPE_OD)
            Port->OTYPER |=  (1U << Pin);
        else
            Port->OTYPER &= ~(1U << Pin);
    }
}

void GPIO_SetSpeed(GPIO_TypeDef *Port, uint32_t Pin, uint32_t Speed) {
    Port->OSPEEDR &= ~(3U << (Pin * 2));
    Port->OSPEEDR |=  (Speed << (Pin * 2));
}

void GPIO_SetAF(GPIO_TypeDef *Port, uint32_t Pin, uint8_t AF) {
    if (Pin < 8) {
        Port->AFR[0] &= ~(0xFU << (Pin * 4));
        Port->AFR[0] |=  ((uint32_t)AF << (Pin * 4));
    } else {
        Port->AFR[1] &= ~(0xFU << ((Pin - 8) * 4));
        Port->AFR[1] |=  ((uint32_t)AF << ((Pin - 8) * 4));
    }
}

void GPIO_WritePin(GPIO_TypeDef *Port, uint32_t Pin, uint8_t State) {
    if (State == GPIO_PIN_SET)
        Port->BSRR = (1U << Pin);           // set
    else
        Port->BSRR = (1U << (Pin + 16));    // reset przez górną połówkę BSRR
    // F411 nie ma BRR — używamy BSRR[31:16]
}

void GPIO_TogglePin(GPIO_TypeDef *Port, uint32_t Pin) {
    Port->ODR ^= (1U << Pin);
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *Port, uint32_t Pin) {
    return (uint8_t)((Port->IDR >> Pin) & 0x1U);
}