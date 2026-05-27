#include "timer.h"

/*
 * F411 timery i ich magistrale:
 *   APB1 (max 50 MHz): TIM2, TIM3, TIM4, TIM5
 *   APB2 (max 100 MHz): TIM1, TIM9, TIM10, TIM11
 */

void TIM_EnableClock(TIM_TypeDef *TIMx) {
    if      (TIMx == TIM1)  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else if (TIMx == TIM2)  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (TIMx == TIM3)  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (TIMx == TIM4)  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (TIMx == TIM5)  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    else if (TIMx == TIM9)  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    else if (TIMx == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    else if (TIMx == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
}

void TIM_Reset(TIM_TypeDef *TIMx) {
    if      (TIMx == TIM1)  { RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;  RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;  }
    else if (TIMx == TIM2)  { RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;  }
    else if (TIMx == TIM3)  { RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;  }
    else if (TIMx == TIM4)  { RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM4RST;  }
    else if (TIMx == TIM5)  { RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST;  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM5RST;  }
    else if (TIMx == TIM9)  { RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST;  RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST;  }
    else if (TIMx == TIM10) { RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST; RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM10RST; }
    else if (TIMx == TIM11) { RCC->APB2RSTR |= RCC_APB2RSTR_TIM11RST; RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM11RST; }
}

void TIM_Config(TIM_TypeDef *TIMx, uint16_t Mode,
                uint32_t PSC, uint32_t ARR) {
    TIM_EnableClock(TIMx);
    TIM_Reset(TIMx);

    if (Mode == TIM_MODE_ONE_SHOT)
        TIMx->CR1 |= TIM_CR1_OPM;
    else
        TIMx->CR1 &= ~TIM_CR1_OPM;

    TIMx->PSC = PSC;
    TIMx->ARR = ARR;
    TIMx->CNT = 0;
    TIMx->EGR = TIM_EGR_UG;
    TIMx->SR &= ~TIM_SR_UIF;
}

void TIM_Start(TIM_TypeDef *TIMx) {
    TIMx->DIER |= TIM_DIER_UIE;
    TIMx->CR1  |= TIM_CR1_CEN;
}

void TIM_Stop(TIM_TypeDef *TIMx) {
    TIMx->CR1  &= ~TIM_CR1_CEN;
    TIMx->DIER &= ~TIM_DIER_UIE;
}

void TIM_EnableIRQ(TIM_TypeDef *TIMx) {
    TIMx->DIER |= TIM_DIER_UIE;
    if      (TIMx == TIM1)  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    else if (TIMx == TIM2)  NVIC_EnableIRQ(TIM2_IRQn);
    else if (TIMx == TIM3)  NVIC_EnableIRQ(TIM3_IRQn);
    else if (TIMx == TIM4)  NVIC_EnableIRQ(TIM4_IRQn);
    else if (TIMx == TIM5)  NVIC_EnableIRQ(TIM5_IRQn);
    else if (TIMx == TIM9)  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    else if (TIMx == TIM10) NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    else if (TIMx == TIM11) NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void TIM_DisableIRQ(TIM_TypeDef *TIMx) {
    TIMx->DIER &= ~TIM_DIER_UIE;
    if      (TIMx == TIM2)  NVIC_DisableIRQ(TIM2_IRQn);
    else if (TIMx == TIM3)  NVIC_DisableIRQ(TIM3_IRQn);
    else if (TIMx == TIM4)  NVIC_DisableIRQ(TIM4_IRQn);
    else if (TIMx == TIM5)  NVIC_DisableIRQ(TIM5_IRQn);
}

void TIM_ClearUpdateFlag(TIM_TypeDef *TIMx) {
    TIMx->SR &= ~TIM_SR_UIF;
}

void TIM_ConfigPWM(TIM_TypeDef *TIMx, uint8_t Channel,
                   uint32_t PSC, uint32_t ARR, uint32_t CCR,
                   uint8_t Polarity, uint8_t PWMMode) {
    TIM_Config(TIMx, TIM_MODE_PWM, PSC, ARR);
    TIMx->CR1 |= TIM_CR1_ARPE;

    uint32_t oc_mode = (PWMMode == TIM_PWM_MODE1)
        ? (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
        : (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2);

    switch (Channel) {
        case 1:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
            TIMx->CCMR1 |= oc_mode | TIM_CCMR1_OC1PE;
            TIMx->CCER  &= ~TIM_CCER_CC1P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC1P;
            TIMx->CCER  |= TIM_CCER_CC1E;
            TIMx->CCR1   = CCR;
            break;
        case 2:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
            TIMx->CCMR1 |= (oc_mode << 8) | TIM_CCMR1_OC2PE;
            TIMx->CCER  &= ~TIM_CCER_CC2P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC2P;
            TIMx->CCER  |= TIM_CCER_CC2E;
            TIMx->CCR2   = CCR;
            break;
        case 3:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
            TIMx->CCMR2 |= oc_mode | TIM_CCMR2_OC3PE;
            TIMx->CCER  &= ~TIM_CCER_CC3P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC3P;
            TIMx->CCER  |= TIM_CCER_CC3E;
            TIMx->CCR3   = CCR;
            break;
        case 4:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
            TIMx->CCMR2 |= (oc_mode << 8) | TIM_CCMR2_OC4PE;
            TIMx->CCER  &= ~TIM_CCER_CC4P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC4P;
            TIMx->CCER  |= TIM_CCER_CC4E;
            TIMx->CCR4   = CCR;
            break;
        default: return;
    }

    // TIM1 jest timerem zaawansowanym — wymaga MOE żeby wyjście działało
    if (TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;

    TIMx->EGR |= TIM_EGR_UG;
    TIMx->CR1 |= TIM_CR1_CEN;
}

void TIM_SetCCR(TIM_TypeDef *TIMx, uint8_t Channel, uint32_t CCR) {
    switch (Channel) {
        case 1:
            TIMx->CCR1 = CCR;
            if (CCR == 0) TIMx->CCER &= ~TIM_CCER_CC1E;
            else          TIMx->CCER |=  TIM_CCER_CC1E;
            break;
        case 2:
            TIMx->CCR2 = CCR;
            if (CCR == 0) TIMx->CCER &= ~TIM_CCER_CC2E;
            else          TIMx->CCER |=  TIM_CCER_CC2E;
            break;
        case 3:
            TIMx->CCR3 = CCR;
            if (CCR == 0) TIMx->CCER &= ~TIM_CCER_CC3E;
            else          TIMx->CCER |=  TIM_CCER_CC3E;
            break;
        case 4:
            TIMx->CCR4 = CCR;
            if (CCR == 0) TIMx->CCER &= ~TIM_CCER_CC4E;
            else          TIMx->CCER |=  TIM_CCER_CC4E;
            break;
        default: return;
    }
}

void TIM_ConfigChannel(TIM_TypeDef *TIMx, uint8_t Channel,
                       uint32_t CCR, uint8_t Polarity, uint8_t PWMMode) {
    uint32_t oc_mode = (PWMMode == TIM_PWM_MODE1)
        ? (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
        : (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2);

    switch (Channel) {
        case 1:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;
            TIMx->CCMR1 |= oc_mode | TIM_CCMR1_OC1PE;
            TIMx->CCER  &= ~TIM_CCER_CC1P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC1P;
            TIMx->CCER  |= TIM_CCER_CC1E;
            TIMx->CCR1   = CCR;
            break;
        case 2:
            TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;
            TIMx->CCMR1 |= (oc_mode << 8) | TIM_CCMR1_OC2PE;
            TIMx->CCER  &= ~TIM_CCER_CC2P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC2P;
            TIMx->CCER  |= TIM_CCER_CC2E;
            TIMx->CCR2   = CCR;
            break;
        case 3:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;
            TIMx->CCMR2 |= oc_mode | TIM_CCMR2_OC3PE;
            TIMx->CCER  &= ~TIM_CCER_CC3P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC3P;
            TIMx->CCER  |= TIM_CCER_CC3E;
            TIMx->CCR3   = CCR;
            break;
        case 4:
            TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;
            TIMx->CCMR2 |= (oc_mode << 8) | TIM_CCMR2_OC4PE;
            TIMx->CCER  &= ~TIM_CCER_CC4P;
            if (Polarity == TIM_POLARITY_LOW) TIMx->CCER |= TIM_CCER_CC4P;
            TIMx->CCER  |= TIM_CCER_CC4E;
            TIMx->CCR4   = CCR;
            break;
        default: return;
    }

    // Wymuś przeładowanie rejestrów — zapobiega tleniu przy CCR=0
    TIMx->EGR |= TIM_EGR_UG;
}