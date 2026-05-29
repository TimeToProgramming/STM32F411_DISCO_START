#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "gpio/gpio.h"
#include "clock/clock.h"
#include "timer/timer.h"
#include "uart/uart.h"
#include "spi/spi.h"
#include "i2c/i2c.h"
#include "l3gd20/l3gd20.h"
#include "lsm303/lsm303.h"
#include "filters/filters.h"
#include "attitude/attitude.h"
#include <math.h>

/* ================================================ */
/*              KONFIGURACJA PWM                    */
/* ================================================ */
#define ARR_VAL     999   /* Okres PWM — PSC=99, ARR=999 → 1kHz przy APB1×2=100MHz */
#define PWM_STEP    10    /* Krok CCR — im mniejszy tym płynniejsze ściemnianie */
#define STEP_DELAY  5     /* Czas między krokami [ms] */

/* ================================================ */
/*              FLAGI EVENTGROUP                    */
/* ================================================ */
EventGroupHandle_t xSystemEvents;
#define FLAG_CALIB_DONE  (1 << 0)

/* ================================================ */
/*         GLOBALNE HANDLERY PERYFERÓW              */
/* ================================================ */
UART_Handle_t    huart2;
SPI_Handle_t     hspi1;
I2C_Handle_t     hi2c1;
L3GD20_Handle_t  hgyro;
LSM303_Handle_t  haccel;
AttitudeFilter_t attitude_filter;

/* ================================================ */
/*      HELPER — kąt [°] na wartość CCR PWM         */
/* ================================================ */
/*
 * Przelicza bezwzględny kąt przechyłu na jasność diody.
 * 0°                      → CCR = 0        (zgaszona)
 * ATTITUDE_MAX_ANGLE_DEG  → CCR = ARR_VAL  (pełna jasność)
 * Liniowo między 0 a max, klampowane do ARR_VAL.
 */
static uint32_t angle_to_ccr(float angle_deg) {
    if (angle_deg <= 0.0f) return 0U;
    float ratio = angle_deg / ATTITUDE_MAX_ANGLE_DEG;
    if (ratio > 1.0f) ratio = 1.0f;
    return (uint32_t)(ratio * (float)ARR_VAL);
}

/* ================================================ */
/*              TASK PWM — animacja LED             */
/* ================================================ */
/*
 * Konfiguruje TIM4 CH1-CH4 (PD12-PD15) i animuje diody
 * podczas kalibracji żyroskopu. Po ustawieniu FLAG_CALIB_DONE
 * przez vAttitudeTask — task kończy animację i usuwa się.
 * Od tej chwili LED steruje vAttitudeTask.
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

    /* TIM4: PSC=99 → 1MHz, ARR=999 → 1kHz PWM */
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
     * xEventGroupGetBits nie blokuje — sprawdza stan bez czekania.
     */
    while (!(xEventGroupGetBits(xSystemEvents) & FLAG_CALIB_DONE)) {

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

    /* Kalibracja skończona — wszystkie diody razem w górę i w dół */
    for (int ccr = 0; ccr <= ARR_VAL; ccr += PWM_STEP) {
        TIM_SetCCR(TIM4, 1, ccr);
        TIM_SetCCR(TIM4, 2, ccr);
        TIM_SetCCR(TIM4, 3, ccr);
        TIM_SetCCR(TIM4, 4, ccr);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
    }
    for (int ccr = ARR_VAL; ccr >= 0; ccr -= PWM_STEP) {
        TIM_SetCCR(TIM4, 1, ccr);
        TIM_SetCCR(TIM4, 2, ccr);
        TIM_SetCCR(TIM4, 3, ccr);
        TIM_SetCCR(TIM4, 4, ccr);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
    }

    TIM_SetCCR(TIM4, 1, 0);
    TIM_SetCCR(TIM4, 2, 0);
    TIM_SetCCR(TIM4, 3, 0);
    TIM_SetCCR(TIM4, 4, 0);

    /* Task skończył — od teraz LED steruje vAttitudeTask */
    vTaskDelete(NULL);
}

/* ================================================ */
/*          TASK ATTITUDE — czujniki + LED          */
/* ================================================ */
/*
 * Zastępuje poprzednie vGyroTask i vAccelTask.
 *
 * Dlaczego jeden task zamiast dwóch:
 *   Filtr komplementarny potrzebuje obu czujników w tej samej
 *   iteracji. Osobne taski wymagałyby synchronizacji między sobą
 *   (mutex/queue) — zbędna komplikacja.
 *
 * Mapowanie LED na przechył:
 *
 *         LD4 zielona  PD12  CH1 — przód (pitch < 0)
 *
 *  LD6 niebieska  PD15  CH4        LD3 pomarańczowa PD13 CH2
 *  lewo (roll < 0)                 prawo (roll > 0)
 *
 *         LD5 czerwona PD14  CH3 — tył  (pitch > 0)
 *
 * Jasność: 0° = zgaszona, ATTITUDE_MAX_ANGLE_DEG = pełna jasność.
 * Zawsze świeci tylko jedna dioda na każdej osi — przeciwna zgaszona.
 */
static void vAttitudeTask(void *pv) {
    (void)pv;

    /* --- Inicjalizacja żyroskopu --- */
    if (!L3GD20_Init(&hgyro, &hspi1, GPIOE, 3, L3GD20_SCALE_250DPS)) {
        UART_Printf(&huart2, "L3GD20 init FAILED!\r\n");
        vTaskDelete(NULL);
    }
    UART_Printf(&huart2, "L3GD20 OK\r\n");

    /* --- Inicjalizacja akcelerometru --- */
    if (!LSM303_Init(&haccel, &hi2c1, LSM303_SCALE_2G)) {
        UART_Printf(&huart2, "LSM303 init FAILED!\r\n");
        vTaskDelete(NULL);
    }
    UART_Printf(&huart2, "LSM303 OK\r\n");

    /* --- Kalibracja żyroskopu (płytka musi stać nieruchomo) --- */
    Gyro_Offset_t offset;
    UART_Printf(&huart2, "Calibrating gyro — keep still!\r\n");
    Gyro_Calibrate(&hgyro, &offset, 500);
    UART_Printf(&huart2, "Offsets: X=%.3f Y=%.3f Z=%.3f\r\n",
                offset.x, offset.y, offset.z);

    /* --- Inicjalizacja filtra komplementarnego --- */
    Attitude_Init(&attitude_filter);

    /* --- Filtry LPF na akcelerometr ---
     *
     * alpha=0.5: kompromis między tłumieniem spike'ów a opóźnieniem fazowym.
     * Filtr komplementarny i tak tłumi akcelerometr przez 2% wagę (τ≈5s),
     * więc agresywny LPF (alpha=0.15, τ≈57ms) był zbędny i szkodliwy.
     */
    LPF_t lpf_ax, lpf_ay, lpf_az;
    LPF_Init(&lpf_ax, 0.5f);
    LPF_Init(&lpf_ay, 0.5f);
    LPF_Init(&lpf_az, 0.5f);

    /* --- Powiadom task PWM że kalibracja skończona --- */
    xEventGroupSetBits(xSystemEvents, FLAG_CALIB_DONE);

    /* --- Pętla główna 100Hz --- */
    for (;;) {

        /* 1. Żyroskop */
        L3GD20_Data_t gyro;
        bool gyro_ok = L3GD20_ReadDPS(&hgyro, &gyro);

        /* 2. Akcelerometr */
        LSM303_Data_t accel;
        bool accel_ok = LSM303_ReadG(&haccel, &accel);

        if (!gyro_ok || !accel_ok) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 3. Odejmij offset kalibracji żyroskopu */
        Gyro_ApplyOffset(&gyro, &offset);

        /* 4. Filtr LPF na akcelerometr */
        accel.x = LPF_Update(&lpf_ax, accel.x);
        accel.y = LPF_Update(&lpf_ay, accel.y);
        accel.z = LPF_Update(&lpf_az, accel.z);

        /* 5. Filtr komplementarny → kąty roll i pitch */
        Attitude_t att;
        Attitude_Update(&attitude_filter,
                        accel.x, accel.y, accel.z,
                        gyro.x,  gyro.y,
                        &att);

        /* ---- Sterowanie LED wg schematu Discovery ----
         *
         * Orientacja użytkownika (USB/Blue=przód, D w kompasie):
         *
         *          U = Orange PD13 CH2  (tył)
         *
         * L = Green PD12 CH1       R = Red PD14 CH3
         *    (lewo)                        (prawo)
         *
         *          D = Blue  PD15 CH4  (przód/USB)
         *
         * Konwencja: dioda wskazuje kierunek przechyłu
         * (jak wskaźnik samolotowy — nos idzie w górę → górna pomarańczowa).
         *
         * Pitch (przód/tył):
         *   pitch < 0 → USB/nos idzie w górę → LD3 pomarańczowa (U, góra)
         *   pitch > 0 → tył idzie w górę     → LD6 niebieska   (D, dół/USB)
         *
         * Roll (lewo/prawo):
         *   roll < 0 → lewy bok w górze  → LD5 czerwona (R, prawo) — niski bok
         *   roll > 0 → prawy bok w górze → LD4 zielona  (L, lewo)  — niski bok
         */

        /* Pitch → pomarańczowa (U) lub niebieska (D) */
        uint32_t ccr_pitch = angle_to_ccr(fabsf(att.pitch));
        if (att.pitch < 0.0f) {
            TIM_SetCCR(TIM4, 2, ccr_pitch); /* nos w górze: LD3 pomarańczowa (U) */
            TIM_SetCCR(TIM4, 4, 0);
        } else {
            TIM_SetCCR(TIM4, 4, ccr_pitch); /* tył w górze: LD6 niebieska (D) */
            TIM_SetCCR(TIM4, 2, 0);
        }

        /* Roll → czerwona (R) lub zielona (L) */
        uint32_t ccr_roll = angle_to_ccr(fabsf(att.roll));
        if (att.roll < 0.0f) {
            TIM_SetCCR(TIM4, 3, ccr_roll);  /* lewy bok w górze: LD5 czerwona (R) */
            TIM_SetCCR(TIM4, 1, 0);
        } else {
            TIM_SetCCR(TIM4, 1, ccr_roll);  /* prawy bok w górze: LD4 zielona (L) */
            TIM_SetCCR(TIM4, 3, 0);
        }

        /* 6. Log UART */
        UART_Printf(&huart2,
            "Roll: %6.1f  Pitch: %6.1f [deg]  "
            "GX: %5.2f  GY: %5.2f [dps]  "
            "AX: %5.2f  AY: %5.2f  AZ: %5.2f [g]\r\n",
            att.roll, att.pitch,
            gyro.x,  gyro.y,
            accel.x, accel.y, accel.z);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ================================================ */
/*                      MAIN                        */
/* ================================================ */
int main(void) {

    /* Zegar — pierwsze co robimy po resecie */
    SystemClock_Config();

    xSystemEvents = xEventGroupCreate();

    /* ---- UART2: PA2=TX, PA3=RX, 115200 baud ---- */
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

    /* ---- SPI1: PA5=SCK, PA6=MISO, PA7=MOSI ---- */
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
        .CPOL         = 0,
        .CPHA         = 0,
        .BaudDiv      = 3
    };
    SPI_Init(&hspi1, &spi1_cfg);

    /* ---- I2C1: PB6=SCL, PB9=SDA, 400kHz ---- */
    const I2C_Config_t i2c1_cfg = {
        .Instance      = I2C1,
        .ClockSpeed    = 400000U,
        .ApbClockFreq  = 50000000U,
        .SclPort       = GPIOB, .SclPin = 6, .SclAF = 4,
        .SdaPort       = GPIOB, .SdaPin = 9, .SdaAF = 4,
        .DmaTxStream   = DMA1_Stream1,
        .DmaTxChannel  = 0,
        .DmaTxIRQn     = DMA1_Stream1_IRQn,
        .DmaRxStream   = DMA1_Stream0,
        .DmaRxChannel  = 1,
        .DmaRxIRQn     = DMA1_Stream0_IRQn,
        .EventIRQn     = I2C1_EV_IRQn,
        .ErrorIRQn     = I2C1_ER_IRQn,
        .NvicPriority  = configLIBRARY_MAX_SYSCALL_IRQ_PRIORITY,
        .MaxRetry      = 3
    };
    I2C_Init(&hi2c1, &i2c1_cfg);

    /* ---- Taski ---- */
    xTaskCreate(vPWMTask,      "PWM", 256, NULL, 1, NULL);
    xTaskCreate(vAttitudeTask, "ATT", 768, NULL, 2, NULL);

    /* Start schedulera — nigdy nie wraca */
    vTaskStartScheduler();
    for (;;) {}
}