#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.h"
#include "clock/clock.h"

#define LED_GREEN   12  // PD12
#define LED_ORANGE  13  // PD13
#define LED_RED     14  // PD14
#define LED_BLUE    15  // PD15

static void vSequenceTask(void *pv) {
    (void)pv;

    // Inicjalizacja GPIO
    GPIO_EnableClock(GPIOD);
    GPIO_ConfigPin(GPIOD, LED_GREEN,  GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, LED_ORANGE, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, LED_RED,    GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(GPIOD, LED_BLUE,   GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_OTYPE_PP);

    // Wszystkie zgaszone na start
    GPIO_WritePin(GPIOD, LED_GREEN,  GPIO_PIN_RESET);
    GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_RESET);
    GPIO_WritePin(GPIOD, LED_RED,    GPIO_PIN_RESET);
    GPIO_WritePin(GPIOD, LED_BLUE,   GPIO_PIN_RESET);

    for (;;) {
        // 3 okrążenia
        for (int lap = 0; lap < 3; lap++) {
            // Zapalanie po kolei
            GPIO_WritePin(GPIOD, LED_GREEN,  GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_RED,    GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_BLUE,   GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(80));

            // Gaszenie po kolei w tym samym tempie
            GPIO_WritePin(GPIOD, LED_GREEN,  GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_RED,    GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(80));
            GPIO_WritePin(GPIOD, LED_BLUE,   GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(80));
        }

        // 3x mrugnięcie wszystkich jednocześnie
        for (int i = 0; i < 3; i++) {
            GPIO_WritePin(GPIOD, LED_GREEN,  GPIO_PIN_SET);
            GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_SET);
            GPIO_WritePin(GPIOD, LED_RED,    GPIO_PIN_SET);
            GPIO_WritePin(GPIOD, LED_BLUE,   GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(120));

            GPIO_WritePin(GPIOD, LED_GREEN,  GPIO_PIN_RESET);
            GPIO_WritePin(GPIOD, LED_ORANGE, GPIO_PIN_RESET);
            GPIO_WritePin(GPIOD, LED_RED,    GPIO_PIN_RESET);
            GPIO_WritePin(GPIOD, LED_BLUE,   GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(120));
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int main(void) {
    SystemClock_Config();

    xTaskCreate(vSequenceTask, "SEQ", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) {}
}