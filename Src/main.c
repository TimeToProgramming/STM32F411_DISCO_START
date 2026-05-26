#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"

static void vLedTask(void *pv) {
    (void)pv;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (1 << (12 * 2));

    for (;;) {
        GPIOD->ODR ^= (1 << 12);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void) {
    xTaskCreate(vLedTask, "LED", 128, NULL, 1, NULL);
    vTaskStartScheduler();
    for (;;) {}
}
