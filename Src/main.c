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
#include "event_groups.h"
#include <math.h>
#include "i2c/i2c.h"


#define GYRO_MAX_DPS  50.0f


// Flaga — bit 0 = kalibracja skończona
EventGroupHandle_t xSystemEvents;
#define FLAG_CALIB_DONE  (1 << 0)

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
I2C_Handle_t hi2c1;


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
     * PSC=99 → 1MHz, ARR=999 → 1kHz PWM
     */
    TIM_Config(TIM4, TIM_MODE_PWM, 99, ARR_VAL);
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->EGR |= TIM_EGR_UG;
    TIM_ConfigChannel(TIM4, 1, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 2, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 3, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM_ConfigChannel(TIM4, 4, 0, TIM_POLARITY_HIGH, TIM_PWM_MODE1);
    TIM4->CR1 |= TIM_CR1_CEN;

    /*
     * Animacja podczas kalibracji — diody kręcą się w kółko.
     * xEventGroupGetBits sprawdza flagę bez blokowania taska.
     * Gdy task Gyro skończy kalibrację i ustawi flagę — wychodzimy.
     */
    while (!(xEventGroupGetBits(xSystemEvents) & FLAG_CALIB_DONE)) {

        /* Rozjaśnianie po kolei CH1→CH2→CH3→CH4 */
        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 1, ARR_VAL);

        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 2, ARR_VAL);

        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 3, ARR_VAL);

        for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
            { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 4, ARR_VAL);

        /* Gaszenie po kolei CH1→CH2→CH3→CH4 */
        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 1, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 1, 0);

        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 2, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 2, 0);

        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 3, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 3, 0);

        for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
            { TIM_SetCCR(TIM4, 4, ccr); vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }
        TIM_SetCCR(TIM4, 4, 0);
    }

    /*
    * Kalibracja skończona — wszystkie diody zaświecają i gasną
    * jako sygnał "gotowy do pracy"
    */
    for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP)
        { TIM_SetCCR(TIM4, 1, ccr);
        TIM_SetCCR(TIM4, 2, ccr);
        TIM_SetCCR(TIM4, 3, ccr);
        TIM_SetCCR(TIM4, 4, ccr);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }

    for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP)
        { TIM_SetCCR(TIM4, 1, ccr);
        TIM_SetCCR(TIM4, 2, ccr);
        TIM_SetCCR(TIM4, 3, ccr);
        TIM_SetCCR(TIM4, 4, ccr);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY)); }

    TIM_SetCCR(TIM4, 1, 0);
    TIM_SetCCR(TIM4, 2, 0);
    TIM_SetCCR(TIM4, 3, 0);
    TIM_SetCCR(TIM4, 4, 0);

    vTaskDelete(NULL);
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

    if (!L3GD20_Init(&hgyro, &hspi1, GPIOE, 3, L3GD20_SCALE_250DPS)) {
        UART_Printf(&huart2, "L3GD20 init FAILED!\r\n");
        vTaskDelete(NULL);
    }
    UART_Printf(&huart2, "L3GD20 OK\r\n");

    L3GD20_Data_t data;
    Gyro_Offset_t offset;

    UART_Printf(&huart2, "Calibrating — keep still!\r\n");
    Gyro_Calibrate(&hgyro, &offset, 500);
    UART_Printf(&huart2, "Offsets: X=%.3f Y=%.3f Z=%.3f\r\n",
                offset.x, offset.y, offset.z);

    LPF_t lpf_x, lpf_y, lpf_z;
    LPF_Init(&lpf_x, 0.15f);
    LPF_Init(&lpf_y, 0.15f);
    LPF_Init(&lpf_z, 0.15f);

    /* Kalibracja skończona — obudź task PWM */
    xEventGroupSetBits(xSystemEvents, FLAG_CALIB_DONE);

    for (;;) {
        if (L3GD20_ReadDPS(&hgyro, &data)) {

            /* Odejmij offset i przefiltruj */
            Gyro_ApplyOffset(&data, &offset);
            data.x = LPF_Update(&lpf_x, data.x);
            data.y = LPF_Update(&lpf_y, data.y);
            data.z = LPF_Update(&lpf_z, data.z);

            /* ---- Wizualizacja na LEDach PWM ---- */

            /*
             * Oś X → CH1 (zielona) i CH2 (pomarańczowa)
             * Jasność proporcjonalna do prędkości kątowej
             * MAX_DPS = pełna jasność
             */
            uint32_t ccr_x = (uint32_t)(fabsf(data.x) / GYRO_MAX_DPS * 999.0f);
            if (ccr_x > 999) ccr_x = 999;

            if (data.x > 0.0f) {
                TIM_SetCCR(TIM4, 2, ccr_x);  /* pomarańczowa */
                TIM_SetCCR(TIM4, 1, 0);       /* zgaś zieloną */
            } else {
                TIM_SetCCR(TIM4, 1, ccr_x);  /* zielona */
                TIM_SetCCR(TIM4, 2, 0);       /* zgaś pomarańczową */
            }

            /*
             * Oś Y → CH3 (czerwona) i CH4 (niebieska)
             */
            uint32_t ccr_y = (uint32_t)(fabsf(data.y) / GYRO_MAX_DPS * 999.0f);
            if (ccr_y > 999) ccr_y = 999;

            if (data.y > 0.0f) {
                TIM_SetCCR(TIM4, 4, ccr_y);  /* niebieska */
                TIM_SetCCR(TIM4, 3, 0);       /* zgaś czerwoną */
            } else {
                TIM_SetCCR(TIM4, 3, ccr_y);  /* czerwona */
                TIM_SetCCR(TIM4, 4, 0);       /* zgaś niebieską */
            }

            UART_Printf(&huart2,
                        "X: %6.2f  Y: %6.2f  Z: %6.2f [deg/s]\r\n",
                        data.x, data.y, data.z);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
    xSystemEvents = xEventGroupCreate();

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

    const I2C_Config_t i2c1_cfg = {
        .Instance      = I2C1,
        .ClockSpeed    = 400000U,        // 400kHz — fast mode
        .ApbClockFreq  = 50000000U,      // APB1 = 50MHz
        .SclPort       = GPIOB, .SclPin = 6, .SclAF = 4,
        .SdaPort       = GPIOB, .SdaPin = 9, .SdaAF = 4,
        .DmaTxStream   = DMA1_Stream1,   // I2C1 TX = Stream1 Ch0
        .DmaTxChannel  = 0,
        .DmaTxIRQn     = DMA1_Stream1_IRQn,
        .DmaRxStream   = DMA1_Stream0,   // I2C1 RX = Stream0 Ch1
        .DmaRxChannel  = 1,
        .DmaRxIRQn     = DMA1_Stream0_IRQn,
        .EventIRQn     = I2C1_EV_IRQn,
        .ErrorIRQn     = I2C1_ER_IRQn,
        .NvicPriority  = configLIBRARY_MAX_SYSCALL_IRQ_PRIORITY,
        .MaxRetry      = 3
    };

    I2C_Init(&hi2c1, &i2c1_cfg);

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