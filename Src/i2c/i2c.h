#pragma once
/**
 * @file    i2c.h
 * @brief   Sterownik I2C dla STM32F411 
 *          Architektura: maszyna stanów + DMA + semafor FreeRTOS
 *
 * Mechanizmy:
 *   - Przerwania I2C (ITEVTEN/ITERREN) — obsługują sekwencję
 *     START/ADDR/BTF przez maszynę stanów
 *   - DMA — transfer danych bez obciążania CPU
 *   - Semafor FreeRTOS — task śpi podczas całej transakcji
 *   - Bus Recovery — automatyczne wznowienie po błędzie
 *   - Retry — automatyczne ponowienie transakcji
 *
 * I2C na F411 — starszy model rejestrów (CR1/CR2/SR1/SR2/DR/CCR/TRISE)
 * Różni się od G030 który ma rejestr TIMINGR.
 *
 * Piny na Discovery dla LSM303DLHC:
 *   PB6 — SCL (AF4)
 *   PB9 — SDA (AF4)
 */

#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================ */
/*              MASZYNA STANÓW                      */
/* ================================================ */

/**
 * @brief Stany transakcji I2C.
 *
 * Przepływ dla zapisu:
 *   IDLE → START → ADDR_W → TX_DMA → IDLE
 *
 * Przepływ dla odczytu:
 *   IDLE → START → ADDR_W → REG → RESTART → ADDR_R → RX_DMA → IDLE
 *
 * Błąd w dowolnym stanie → ERROR → (retry lub BusRecovery) → IDLE
 */
typedef enum {
    I2C_STATE_IDLE,      /*!< Brak aktywnej transakcji */
    I2C_STATE_START,     /*!< START wygenerowany, czekamy na SB */
    I2C_STATE_ADDR_W,    /*!< Adres+W wysłany, czekamy na ADDR */
    I2C_STATE_REG,       /*!< Adres rejestru wysłany, czekamy na BTF */
    I2C_STATE_RESTART,   /*!< Repeated START, czekamy na SB */
    I2C_STATE_ADDR_R,    /*!< Adres+R wysłany, czekamy na ADDR */
    I2C_STATE_TX_DMA,    /*!< DMA TX aktywne */
    I2C_STATE_RX_DMA,    /*!< DMA RX aktywne */
    I2C_STATE_ERROR      /*!< Błąd — retry lub Bus Recovery */
} I2C_State_t;

/* ================================================ */
/*              STRUKTURY                           */
/* ================================================ */

/**
 * @brief Konfiguracja przekazywana do I2C_Init.
 *        Wypełnij przed wywołaniem init.
 */
typedef struct {
    I2C_TypeDef        *Instance;       /*!< I2C1, I2C2, I2C3 */
    uint32_t            ClockSpeed;     /*!< 100000=100kHz, 400000=400kHz */
    uint32_t            ApbClockFreq;   /*!< Częstotliwość APB1 w Hz */

    GPIO_TypeDef       *SclPort;        /*!< Port SCL — używany też do Bus Recovery */
    uint32_t            SclPin;         /*!< Pin SCL */
    uint8_t             SclAF;          /*!< Alternate Function SCL (AF4 dla I2C1) */

    GPIO_TypeDef       *SdaPort;        /*!< Port SDA — używany też do Bus Recovery */
    uint32_t            SdaPin;         /*!< Pin SDA */
    uint8_t             SdaAF;          /*!< Alternate Function SDA (AF4 dla I2C1) */

    DMA_Stream_TypeDef *DmaTxStream;    /*!< Stream DMA TX (I2C1: DMA1_Stream1) */
    uint8_t             DmaTxChannel;   /*!< Kanał DMA TX (I2C1: Ch0) */
    IRQn_Type           DmaTxIRQn;      /*!< IRQ DMA TX */

    DMA_Stream_TypeDef *DmaRxStream;    /*!< Stream DMA RX (I2C1: DMA1_Stream0) */
    uint8_t             DmaRxChannel;   /*!< Kanał DMA RX (I2C1: Ch1) */
    IRQn_Type           DmaRxIRQn;      /*!< IRQ DMA RX */

    IRQn_Type           EventIRQn;      /*!< IRQ zdarzeń I2C (I2C1_EV_IRQn) */
    IRQn_Type           ErrorIRQn;      /*!< IRQ błędów I2C (I2C1_ER_IRQn) */

    uint8_t             NvicPriority;   /*!< Priorytet przerwań */
    uint8_t             MaxRetry;       /*!< Max liczba prób przed Bus Recovery */
} I2C_Config_t;

/**
 * @brief Handle — trwały stan sterownika.
 *        Nie modyfikuj pól bezpośrednio — używaj API.
 */
typedef struct {
    I2C_Config_t         Config;

    SemaphoreHandle_t    Done;           /*!< Semafor — koniec transakcji (TX lub RX) */
    volatile I2C_State_t State;          /*!< Aktualny stan maszyny stanów */
    volatile bool        Busy;           /*!< Czy transakcja trwa */
    volatile bool        IsRead;         /*!< Czy to odczyt (true) czy zapis (false) */
    volatile bool        Error;          /*!< Czy ostatnia transakcja zakończyła się błędem */

    uint8_t              DevAddr;        /*!< Adres 7-bitowy urządzenia slave */
    uint8_t              RegAddr;        /*!< Adres rejestru (tylko dla odczytu) */
    uint8_t             *Buffer;         /*!< Wskaźnik na bufor danych */
    uint16_t             Length;         /*!< Liczba bajtów do transferu */
    volatile uint8_t     RetryCount;     /*!< Aktualny licznik prób */
} I2C_Handle_t;

/* ================================================ */
/*                  PUBLICZNE API                   */
/* ================================================ */

/**
 * @brief  Inicjalizuje I2C z podaną konfiguracją.
 *         Tworzy semafor, konfiguruje GPIO, rejestry I2C, DMA i NVIC.
 */
void I2C_Init(I2C_Handle_t *hi2c, const I2C_Config_t *cfg);

/**
 * @brief  Zapisuje dane do urządzenia slave przez DMA.
 *         Task zasypia do końca transakcji.
 *         buf[0] = adres rejestru, buf[1..] = dane.
 * @return true jeśli OK, false jeśli błąd lub timeout
 */
bool I2C_Write(I2C_Handle_t *hi2c, uint8_t addr,
               uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/**
 * @brief  Odczytuje dane z urządzenia slave przez DMA.
 *         Task zasypia do końca transakcji.
 * @return true jeśli OK, false jeśli błąd lub timeout
 */
bool I2C_Read(I2C_Handle_t *hi2c, uint8_t addr, uint8_t reg,
              uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/**
 * @brief  Wywoływane z I2C_EV_IRQHandler — maszyna stanów.
 *         Nie używaj bezpośrednio.
 */
void I2C_IRQHandler_CB(I2C_Handle_t *hi2c);

/**
 * @brief  Wywoływane z DMA TX IRQHandler.
 *         Nie używaj bezpośrednio.
 */
void I2C_DMA_TX_IRQHandler_CB(I2C_Handle_t *hi2c);

/**
 * @brief  Wywoływane z DMA RX IRQHandler.
 *         Nie używaj bezpośrednio.
 */
void I2C_DMA_RX_IRQHandler_CB(I2C_Handle_t *hi2c);