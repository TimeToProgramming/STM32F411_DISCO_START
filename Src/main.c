#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.h"
#include "clock/clock.h"
#include "timer/timer.h"
#include "uart/uart.h"

/* ================================================ */
/*              KONFIGURACJA PWM                    */
/* ================================================ */
#define ARR_VAL     999   /* Okres PWM — razem z PSC=99 daje 1kHz przy 100MHz */
#define PWM_STEP    10    /* Krok zmiany CCR — im mniejszy tym płynniejsze ściemnianie */
#define STEP_DELAY  5     /* Czas między krokami w ms — reguluje szybkość animacji */

/* ================================================ */
/*         GLOBALNY HANDLE UART                     */
/* ================================================ */
/*
 * Globalny — dostępny z każdego taska przez extern.
 * Inicjalizowany w main() przed startem schedulera.
 * Mutex wewnątrz chroni przed równoczesnym dostępem.
 */
UART_Handle_t huart2;

/* ================================================ */
/*              TASK PWM — animacja LED             */
/* ================================================ */
/*
 * Steruje czterema diodami przez TIM4 CH1-CH4 (PD12-PD15).
 * Każda dioda rozjaśnia się i gaśnie po kolei w pętli.
 * vTaskDelay oddaje procesor innym taskom podczas czekania —
 * nie blokuje jak HAL_Delay.
 */
static void vPWMTask(void *pv) {
    (void)pv;

    /* Konfiguracja pinów PD12-PD15 jako AF2 = TIM4 CH1-CH4 */
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

    /*
     * Inicjalizacja TIM4:
     * PSC=99 → f_timer = APB1×2 / (PSC+1) = 100MHz/100 = 1MHz
     * ARR=999 → f_PWM = 1MHz / 1000 = 1kHz
     * ARPE — preload ARR, zmiana ARR wchodzi po przepełnieniu
     * EGR_UG — wymuś natychmiastowe załadowanie PSC i ARR
     */
    TIM_Config(TIM4, TIM_MODE_PWM, 99, ARR_VAL);
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR |= TIM_EGR_UG;

    /*
     * Konfiguracja kanałów CH1-CH4 z CCR=0 (zgaszone).
     * PWM Mode 1, polaryzacja HIGH:
     * pin=HIGH gdy CNT < CCR → jasność rośnie wraz z CCR
     */
    TIM_ConfigChannel(TIM4, 1, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 2, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 3, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 4, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);

    /* Start timera — od tej chwili generuje PWM */
    TIM4->CR1 |= TIM_CR1_CEN;

    for (;;) {
        /* CH1 (zielona PD12) — rozjaśnianie od 0 do ARR */
        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 1, ARR_VAL); /* Gwarancja pełnej jasności */

        /* CH2 (pomarańczowa PD13) — rozjaśnianie */
        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 2, ARR_VAL);

        /* CH3 (czerwona PD14) — rozjaśnianie */
        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 3, ARR_VAL);

        /* CH4 (niebieska PD15) — rozjaśnianie */
        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 4, ARR_VAL);

        /* CH1 — gaszenie od ARR do 0 */
        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 1, 0); /* Gwarancja pełnego zgaszenia */

        /* CH2 — gaszenie */
        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 2, 0);

        /* CH3 — gaszenie */
        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 3, 0);

        /* CH4 — gaszenie */
        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 4, 0);
    }
}

/* ================================================ */
/*          TASK DEBUG — logowanie przez UART       */
/* ================================================ */
/*
 * Osobny task odpowiedzialny wyłącznie za komunikację UART.
 * Inne taski mogą też wywołać UART_Printf(&huart2, ...) —
 * mutex w bibliotece zadba o bezpieczeństwo.
 * Priorytet taki sam jak PWM — scheduler przydziela czas równo.
 */
static void vDebugTask(void *pv) {
    (void)pv;

    /*
     * Pierwsze wywołanie UART_Printf po starcie schedulera.
     * Mutex działa dopiero gdy scheduler jest uruchomiony —
     * dlatego Printf jest tu a nie w main().
     */
    UART_Printf(&huart2, "=== System start ===\r\n");
    UART_Printf(&huart2, "Clock: %lu Hz\r\n", SystemCoreClock);
    UART_Printf(&huart2, "RTOS tick: %lu Hz\r\n", configTICK_RATE_HZ);

    uint32_t tick = 0;
    for (;;) {
        /* Co 500ms wyślij licznik — potwierdzenie że system żyje */
        UART_Printf(&huart2, "tick: %lu\r\n", tick++);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ================================================ */
/*                      MAIN                        */
/* ================================================ */
int main(void) {
    /*
     * Konfiguracja zegara — pierwsze co robimy po resecie.
     * Bez tego wszystkie peryferia taktowane z HSI 16MHz.
     * Po tej funkcji SYSCLK = 100MHz z HSE 8MHz przez PLL.
     */
    SystemClock_Config();

    /*
     * Konfiguracja USART2 na PA2 (TX) i PA3 (RX).
     * USART2 siedzi na APB1 = 50MHz — stąd ClockFreq.
     * DMA1 Stream6 kanał 4 = USART2_TX (z tabeli DMA w RM).
     * DMA1 Stream5 kanał 4 = USART2_RX (z tabeli DMA w RM).
     */
    const UART_Config_t uart2_cfg = {
        .Instance     = USART2,
        .BaudRate     = 115200,
        .ClockFreq    = 50000000U,
        .TxPort       = GPIOA, .TxPin = 2, .TxAF = 7,
        .RxPort       = GPIOA, .RxPin = 3, .RxAF = 7,
        .DmaTxStream  = DMA1_Stream6,
        .DmaTxChannel = 4,
        .DmaTxIRQn    = DMA1_Stream6_IRQn,
        .DmaRxStream  = DMA1_Stream5,
        .DmaRxChannel = 4,
        .NvicPriority = configLIBRARY_MAX_SYSCALL_IRQ_PRIORITY
    };

    UART_Init(&huart2, &uart2_cfg);

    /*
     * Tworzenie tasków — tylko rejestracja, nie uruchamiają się jeszcze.
     * Stack 256 słów = 1KB RAM na task.
     * Priorytet 1 — najniższy użytkownika (0 = Idle task FreeRTOS).
     */
    xTaskCreate(vPWMTask,   "PWM",   256, NULL, 1, NULL);
    xTaskCreate(vDebugTask, "DEBUG", 256, NULL, 1, NULL);

    /*
     * Start schedulera — od tej chwili FreeRTOS przejmuje kontrolę.
     * Ta funkcja nigdy nie wraca.
     * Jeśli wróci (błąd pamięci) — wpadamy w nieskończoną pętlę.
     */
    vTaskStartScheduler();
    for (;;) {}
}