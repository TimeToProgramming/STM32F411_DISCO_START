#pragma once
/**
 * @file    spi.h
 * @brief   Sterownik SPI dla STM32F411 — DMA TX+RX
 *
 * Podejście nieblokujące przez DMA + semafor binarny FreeRTOS:
 *   - Task wywołuje SPI_Transfer() i zasypia na semaforze
 *   - DMA wysyła i odbiera jednocześnie (full-duplex)
 *   - Przerwanie TC budzi task
 *   - Task kontynuuje przetwarzanie odebranych danych
 *
 * Dodatkowo SPI_Transfer_Blocking() do diagnostyki —
 * używaj tylko podczas debugowania, blokuje task.
 *
 * CS obsługiwany ręcznie przez warstwę czujnika.
 * Pełna konfiguracja przez SPI_Config_t.
 */

#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    SPI_TypeDef         *Instance;
    uint32_t             ClockFreq;
    GPIO_TypeDef        *SckPort;
    uint32_t             SckPin;
    uint8_t              SckAF;
    GPIO_TypeDef        *MisoPort;
    uint32_t             MisoPin;
    uint8_t              MisoAF;
    GPIO_TypeDef        *MosiPort;
    uint32_t             MosiPin;
    uint8_t              MosiAF;
    DMA_Stream_TypeDef  *DmaTxStream;
    uint8_t              DmaTxChannel;
    IRQn_Type            DmaTxIRQn;
    DMA_Stream_TypeDef  *DmaRxStream;
    uint8_t              DmaRxChannel;
    IRQn_Type            DmaRxIRQn;
    uint8_t              NvicPriority;
    /*
     * Tryb SPI:
     * Mode 0: CPOL=0 CPHA=0 — SCK idle LOW,  próbkuj na rosnącym zboczu
     * Mode 1: CPOL=0 CPHA=1 — SCK idle LOW,  próbkuj na opadającym zboczu
     * Mode 2: CPOL=1 CPHA=0 — SCK idle HIGH, próbkuj na opadającym zboczu
     * Mode 3: CPOL=1 CPHA=1 — SCK idle HIGH, próbkuj na rosnącym zboczu
     * L3GD20 wg datasheet Fig.12: driven on falling, captured on rising → Mode 0
     */
    uint8_t              CPOL;
    uint8_t              CPHA;
    /*
     * Prescaler zegara:
     * 0=/2, 1=/4, 2=/8, 3=/16, 4=/32, 5=/64, 6=/128, 7=/256
     * L3GD20 max 10MHz → APB2=100MHz, BaudDiv=3 → 6.25MHz
     */
    uint8_t              BaudDiv;
} SPI_Config_t;

typedef struct {
    SPI_Config_t         Config;
    SemaphoreHandle_t    TransferDone;
    volatile bool        Busy;
} SPI_Handle_t;

void SPI_Init(SPI_Handle_t *hspi, const SPI_Config_t *cfg);

bool SPI_Transfer(SPI_Handle_t  *hspi,
                  const uint8_t *tx_buf,
                  uint8_t       *rx_buf,
                  uint16_t       len,
                  uint32_t       timeout_ms);

/**
 * @brief  Transfer blokujący — TYLKO do diagnostyki.
 *         Polling bez DMA, bez semafora.
 *         Nie używaj w produkcji.
 */
bool SPI_Transfer_Blocking(SPI_Handle_t  *hspi,
                           const uint8_t *tx_buf,
                           uint8_t       *rx_buf,
                           uint16_t       len);

void SPI_DMA_RX_IRQHandler_CB(SPI_Handle_t *hspi);
void SPI_DMA_TX_IRQHandler_CB(SPI_Handle_t *hspi);