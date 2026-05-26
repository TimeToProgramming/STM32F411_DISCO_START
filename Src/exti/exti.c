// exti.c
#include "exti.h"

void EXTI_Config(GPIO_TypeDef *Port, uint8_t Pin, uint32_t Trigger) {
    // F411: mapowanie EXTI przez SYSCFG — wymaga zegara SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    uint8_t reg = Pin / 4;
    uint8_t pos = Pin % 4;

    uint8_t port_num = (Port == GPIOA) ? 0x0 :
                       (Port == GPIOB) ? 0x1 :
                       (Port == GPIOC) ? 0x2 :
                       (Port == GPIOD) ? 0x3 :
                       (Port == GPIOE) ? 0x4 :
                       (Port == GPIOH) ? 0x7 : 0x0;

    SYSCFG->EXTICR[reg] &= ~(0xFU << (pos * 4));
    SYSCFG->EXTICR[reg] |=  (port_num << (pos * 4));

    if (Trigger & EXTI_TRIGGER_RISING)  EXTI->RTSR |= (1U << Pin);
    else                                EXTI->RTSR &= ~(1U << Pin);

    if (Trigger & EXTI_TRIGGER_FALLING) EXTI->FTSR |= (1U << Pin);
    else                                EXTI->FTSR &= ~(1U << Pin);

    EXTI_Enable(Pin);
}

void EXTI_Enable(uint8_t Pin) {
    EXTI->IMR |= (1U << Pin);

    // F411 ma osobne IRQ dla każdego pinu 0-4, 
    // potem grupowe 5-9 i 10-15
    if      (Pin == 0)  NVIC_EnableIRQ(EXTI0_IRQn);
    else if (Pin == 1)  NVIC_EnableIRQ(EXTI1_IRQn);
    else if (Pin == 2)  NVIC_EnableIRQ(EXTI2_IRQn);
    else if (Pin == 3)  NVIC_EnableIRQ(EXTI3_IRQn);
    else if (Pin == 4)  NVIC_EnableIRQ(EXTI4_IRQn);
    else if (Pin <= 9)  NVIC_EnableIRQ(EXTI9_5_IRQn);
    else if (Pin <= 15) NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI_Disable(uint8_t Pin) {
    EXTI->IMR &= ~(1U << Pin);
}

void EXTI_ClearFlag(uint8_t Pin) {
    // F411: jeden rejestr PR (nie RPR1/FPR1 jak G030)
    EXTI->PR |= (1U << Pin);
}