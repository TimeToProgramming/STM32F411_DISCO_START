#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio/gpio.h"
#include "clock/clock.h"
#include "timer/timer.h"
#include "uart/uart.h"
#include "spi/spi.h"
#include "l3gd20/l3gd20.h"
#include "filters/filters.h"

/* ================================================ */
/*              KONFIGURACJA PWM                    */
/* ================================================ */
#define ARR_VAL     999   /* Okres PWM — PSC=99, ARR=999 → 1kHz przy APB1×2=100MHz */
#define PWM_STEP    10    /* Krok CCR — im mniejszy tym płynniejsze ściemnianie */
#define STEP_DELAY  5     /* Czas między krokami [ms] — reguluje szybkość animacji */

/* ================================================ */
/*         GLOBALNE HANDLERY PERYFERÓW              */
/* ================================================ */
/*
 * Globalne — dostępne z każdego taska przez extern.
 * Inicjalizowane w main() przed startem schedulera.
 */
UART_Handle_t   huart2;  /* USART2: PA2=TX, PA3=RX, 115200 baud */
SPI_Handle_t    hspi1;   /* SPI1:   PA5=SCK, PA6=MISO, PA7=MOSI */
L3GD20_Handle_t hgyro;   /* Żyroskop L3GD20: CS=PE3 */

/* ================================================ */
/*              TASK PWM — animacja LED             */
/* ================================================ */
/*
 * Steruje czterema diodami przez TIM4 CH1-CH4 (PD12-PD15).
 * Każda dioda rozjaśnia się i gaśnie po kolei w nieskończonej pętli.
 * vTaskDelay oddaje procesor innym taskom podczas czekania —
 * zero busy-waiting w przeciwieństwie do HAL_Delay.
 */
static void vPWMTask(void *pv) {
    (void)pv;

    /* Konfiguracja PD12-PD15 jako AF2 = TIM4 CH1-CH4 */
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
     * APB1=50MHz → TIM4 clock = APB1×2 = 100MHz
     * PSC=99  → f_timer = 100MHz/100 = 1MHz
     * ARR=999 → f_PWM  = 1MHz/1000  = 1kHz
     * ARPE — preload ARR, zmiana ARR wchodzi synchronicznie po przepełnieniu
     * EGR_UG — wymuś natychmiastowe załadowanie PSC i ARR do rejestrów shadow
     */
    TIM_Config(TIM4, TIM_MODE_PWM, 99, ARR_VAL);
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR |= TIM_EGR_UG;

    /*
     * CH1-CH4 startują z CCR=0 (zgaszone).
     * PWM Mode 1, polaryzacja HIGH:
     * pin=HIGH gdy CNT < CCR → jasność proporcjonalna do CCR/ARR
     */
    TIM_ConfigChannel(TIM4, 1, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 2, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 3, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 4, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);

    /* Start timera — od tej chwili generuje PWM */
    TIM4->CR1 |= TIM_CR1_CEN;

    for (;;) {
        /* CH1 (zielona PD12) — rozjaśnianie 0→ARR */
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

        /* CH1 — gaszenie ARR→0 */
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
/*          TASK GYRO — odczyt żyroskopu            */
/* ================================================ */
/*
 * Inicjalizuje L3GD20 i co 100ms odczytuje dane.
 * Wypisuje X/Y/Z przez UART — mutex w bibliotece UART
 * zadba o bezpieczeństwo przy równoległym dostępie.
 * Priorytet 2 — wyższy niż PWM żeby odczyt czujnika
 * nie był opóźniany przez animację LED.
 */
static void vGyroTask(void *pv) {
    (void)pv;

    /*
     * Inicjalizacja żyroskopu.
     * CS = PE3, zakres ±250°/s
     */
    if (!L3GD20_Init(&hgyro, &hspi1, GPIOE, 3, L3GD20_SCALE_250DPS)) {
        UART_Printf(&huart2, "L3GD20 init FAILED!\r\n");
        vTaskDelete(NULL);
    }

    UART_Printf(&huart2, "L3GD20 OK\r\n");

    /*
     * Deklaracja zmiennych — dane i offsety kalibracji.
     * offset wypełni Gyro_Calibrate, potem odejmujemy go
     * od każdego pomiaru przez Gyro_ApplyOffset.
     */
    L3GD20_Data_t data;
    Gyro_Offset_t offset;

    /*
     * Kalibracja offsetu — płytka musi stać nieruchomo!
     * 500 próbek × 1ms = ~0.5s
     * Wynik: offset.x/y/z = średnia z próbek = błąd zera czujnika
     */
    UART_Printf(&huart2, "Calibrating — keep still!\r\n");
    Gyro_Calibrate(&hgyro, &offset, 500);
    UART_Printf(&huart2, "Offsets: X=%.3f Y=%.3f Z=%.3f\r\n",
                offset.x, offset.y, offset.z);

    /*
     * Inicjalizacja filtrów LPF dla każdej osi.
     * alpha=0.15 — umiarkowane wygładzanie
     * Im mniejsze alpha tym gładszy sygnał ale większe opóźnienie
     */
    LPF_t lpf_x, lpf_y, lpf_z;
    LPF_Init(&lpf_x, 0.15f);
    LPF_Init(&lpf_y, 0.15f);
    LPF_Init(&lpf_z, 0.15f);

    for (;;) {
        if (L3GD20_ReadDPS(&hgyro, &data)) {

            /* Odejmij offset kalibracji */
            Gyro_ApplyOffset(&data, &offset);

            /* Przefiltruj każdą oś przez LPF */
            data.x = LPF_Update(&lpf_x, data.x);
            data.y = LPF_Update(&lpf_y, data.y);
            data.z = LPF_Update(&lpf_z, data.z);

            UART_Printf(&huart2,
                        "X: %6.2f  Y: %6.2f  Z: %6.2f [deg/s]\r\n",
                        data.x, data.y, data.z);
        }
        vTaskDelay(pdMS_TO_TICKS(10));  /* 100Hz */
    }
}

/* ================================================ */
/*                      MAIN                        */
/* ================================================ */
int main(void) {
    /*
     * Zegar — pierwsze co robimy po resecie.
     * HSE 8MHz → PLL → SYSCLK 100MHz.
     * Bez tego peryferia taktowane z HSI 16MHz.
     */
    SystemClock_Config();

    /*
     * UART2: PA2=TX, PA3=RX, 115200 baud
     * APB1=50MHz → ClockFreq=50000000
     * DMA1 Stream6 Ch4 = USART2_TX
     * DMA1 Stream5 Ch4 = USART2_RX
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
     * SPI1: PA5=SCK, PA6=MISO, PA7=MOSI, AF5
     * APB2=100MHz, BaudDiv=3 → 6.25MHz (L3GD20 max 10MHz)
     * Mode 0 (CPOL=0, CPHA=0) — testujemy wg datasheet Fig.12
     * "driven on falling, captured on rising edge"
     * DMA2 Stream3 Ch3 = SPI1_TX
     * DMA2 Stream0 Ch3 = SPI1_RX
     */
    const SPI_Config_t spi1_cfg = {
        .Instance     = SPI1,
        .ClockFreq    = 100000000U,
        .SckPort      = GPIOA, .SckPin  = 5, .SckAF  = 5,
        .MisoPort     = GPIOA, .MisoPin = 6, .MisoAF = 5,
        .MosiPort     = GPIOA, .MosiPin = 7, .MosiAF = 5,
        .DmaTxStream  = DMA2_Stream3,
        .DmaTxChannel = 3,
        .DmaTxIRQn    = DMA2_Stream3_IRQn,
        .DmaRxStream  = DMA2_Stream0,
        .DmaRxChannel = 3,
        .DmaRxIRQn    = DMA2_Stream0_IRQn,
        .NvicPriority = configLIBRARY_MAX_SYSCALL_IRQ_PRIORITY,
        .CPOL         = 0,   /* SCK idle LOW */
        .CPHA         = 0,   /* próbkuj na rosnącym zboczu */
        .BaudDiv      = 3    /* /16 = 6.25MHz */
    };
    SPI_Init(&hspi1, &spi1_cfg);

    /*
     * Tworzenie tasków — rejestracja, nie uruchamiają się jeszcze.
     * Stack 256 słów = 1KB RAM dla PWM.
     * Stack 512 słów = 2KB RAM dla GYRO (float, printf).
     * Priorytet GYRO=2 > PWM=1 — czujnik ważniejszy niż animacja.
     */
    xTaskCreate(vPWMTask,  "PWM",  256, NULL, 1, NULL);
    xTaskCreate(vGyroTask, "GYRO", 512, NULL, 2, NULL);

    /*
     * Start schedulera — FreeRTOS przejmuje kontrolę.
     * Nigdy nie wraca. Jeśli wróci (brak pamięci) —
     * wpadamy w nieskończoną pętlę zamiast wykonywać losowy kod.
     */
    vTaskStartScheduler();
    for (;;) {}
}