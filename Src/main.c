#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.h"
#include "clock/clock.h"
#include "timer/timer.h"

#define ARR_VAL     999     // okres PWM
#define PWM_STEP    10      // krok rozjaśniania
#define STEP_DELAY  5       // ms między krokami

static void vPWMTask(void *pv) {
    (void)pv;

    // Konfiguracja pinów PD12-PD15 jako AF2 (TIM4)
    GPIO_EnableClock(GPIOD);
    GPIO_ConfigPin(GPIOD, 12, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, 13, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, 14, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, 15, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_SetAF(GPIOD, 12, 2);
    GPIO_SetAF(GPIOD, 13, 2);
    GPIO_SetAF(GPIOD, 14, 2);
    GPIO_SetAF(GPIOD, 15, 2);
    GPIO_SetSpeed(GPIOD, 12, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(GPIOD, 13, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(GPIOD, 14, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(GPIOD, 15, GPIO_SPEED_HIGH);

    // Inicjalizacja TIM4
    TIM_Config(TIM4, TIM_MODE_PWM, 99, 999);
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR |= TIM_EGR_UG;

    // Konfiguracja kanałów
    TIM_ConfigChannel(TIM4, 1, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 2, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 3, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 4, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);

    // Włącz timer
    TIM4->CR1 |= TIM_CR1_CEN;

    for (;;) {
        // 3 okrążenia
        for (int lap = 0; lap < 3; lap++) {

            // CH1 rozjaśnia
            for (int ccr = 0; ccr <= 999; ccr += 10)
                { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 1, 999);

            // CH2 rozjaśnia
            for (int ccr = 0; ccr <= 999; ccr += 10)
                { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 2, 999);

            // CH3 rozjaśnia
            for (int ccr = 0; ccr <= 999; ccr += 10)
                { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 3, 999);

            // CH4 rozjaśnia
            for (int ccr = 0; ccr <= 999; ccr += 10)
                { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 4, 999);

            // CH1 gaśnie
            for (int ccr = 999; ccr >= 0; ccr -= 10)
                { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 1, 0);

            // CH2 gaśnie
            for (int ccr = 999; ccr >= 0; ccr -= 10)
                { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 2, 0);

            // CH3 gaśnie
            for (int ccr = 999; ccr >= 0; ccr -= 10)
                { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 3, 0);

            // CH4 gaśnie
            for (int ccr = 999; ccr >= 0; ccr -= 10)
                { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(5)); }
            TIM_SetCCR(TIM4, 4, 0);
        }

        // 3x mrugnięcie wszystkich
        for (int i = 0; i < 3; i++) {
            for (int ccr = 0; ccr <= 999; ccr += 50) {
                TIM_SetCCR(TIM4, 1, ccr);
                TIM_SetCCR(TIM4, 2, ccr);
                TIM_SetCCR(TIM4, 3, ccr);
                TIM_SetCCR(TIM4, 4, ccr);
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            TIM_SetCCR(TIM4, 1, 999);
            TIM_SetCCR(TIM4, 2, 999);
            TIM_SetCCR(TIM4, 3, 999);
            TIM_SetCCR(TIM4, 4, 999);

            for (int ccr = 999; ccr >= 0; ccr -= 50) {
                TIM_SetCCR(TIM4, 1, ccr);
                TIM_SetCCR(TIM4, 2, ccr);
                TIM_SetCCR(TIM4, 3, ccr);
                TIM_SetCCR(TIM4, 4, ccr);
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            TIM_SetCCR(TIM4, 1, 0);
            TIM_SetCCR(TIM4, 2, 0);
            TIM_SetCCR(TIM4, 3, 0);
            TIM_SetCCR(TIM4, 4, 0);

            vTaskDelay(pdMS_TO_TICKS(150));
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

int main(void) {
    SystemClock_Config();
    xTaskCreate(vPWMTask, "PWM", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) {}
}