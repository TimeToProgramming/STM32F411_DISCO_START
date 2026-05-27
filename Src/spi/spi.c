/**
 * @file    spi.c
 * @brief   Implementacja sterownika SPI — DMA TX+RX + semafor FreeRTOS
 *
 * Przepływ transferu:
 *
 *  Task wywołuje SPI_Transfer(tx, rx, len, timeout)
 *       │
 *       ▼
 *  Konfiguruj DMA RX (SPI→pamięć) — NAJPIERW RX
 *  Konfiguruj DMA TX (pamięć→SPI)
 *       │
 *       ▼
 *  Włącz DMA RX, potem DMA TX — full-duplex start
 *       │
 *       ▼
 *  xSemaphoreTake — task zasypia, oddaje procesor innym taskom
 *       │
 *  ... DMA TX wysyła bajty, DMA RX odbiera równolegle ...
 *       │
 *       ▼
 *  Przerwanie TC (DMA RX) — odbiór zakończony
 *       │
 *       ▼
 *  xSemaphoreGiveFromISR — budzi task natychmiast
 *       │
 *       ▼
 *  Task kontynuuje — rx_buf zawiera odebrane dane
 */

#include "spi.h"
#include "../gpio/gpio.h"

/* ================================================ */
/*                  INICJALIZACJA                   */
/* ================================================ */

void SPI_Init(SPI_Handle_t *hspi, const SPI_Config_t *cfg) {

    /* Zapisz konfigurację i zresetuj stan */
    hspi->Config = *cfg;
    hspi->Busy   = false;

    /*
     * Utwórz semafor binarny.
     * Różnica od mutexa:
     *   Mutex — ma właściciela, ten sam task bierze i oddaje
     *   Semafor binarny — brak właściciela, przerwanie może oddać
     *                     semafor którego nie brało
     * Tu właśnie tego potrzebujemy: task bierze, przerwanie oddaje.
     */
    hspi->TransferDone = xSemaphoreCreateBinary();
    configASSERT(hspi->TransferDone != NULL);

    /* 1. GPIO — SCK, MISO, MOSI jako Alternate Function push-pull */
    GPIO_EnableClock(cfg->SckPort);
    GPIO_EnableClock(cfg->MisoPort);
    GPIO_EnableClock(cfg->MosiPort);

    GPIO_ConfigPin(cfg->SckPort,  cfg->SckPin,
                   GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(cfg->MisoPort, cfg->MisoPin,
                   GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(cfg->MosiPort, cfg->MosiPin,
                   GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);

    GPIO_SetAF(cfg->SckPort,  cfg->SckPin,  cfg->SckAF);
    GPIO_SetAF(cfg->MisoPort, cfg->MisoPin, cfg->MisoAF);
    GPIO_SetAF(cfg->MosiPort, cfg->MosiPin, cfg->MosiAF);

    GPIO_SetSpeed(cfg->SckPort,  cfg->SckPin,  GPIO_SPEED_HIGH);
    GPIO_SetSpeed(cfg->MisoPort, cfg->MisoPin, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(cfg->MosiPort, cfg->MosiPin, GPIO_SPEED_HIGH);

    /* 2. Zegar SPI — zależy od magistrali (APB1 lub APB2) */
    if      (cfg->Instance == SPI1) RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    else if (cfg->Instance == SPI2) RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    else if (cfg->Instance == SPI3) RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    else if (cfg->Instance == SPI4) RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    else if (cfg->Instance == SPI5) RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

    /* 3. Reset SPI — czyści wszystkie rejestry do wartości domyślnych */
    if (cfg->Instance == SPI1) {
        RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    } else if (cfg->Instance == SPI2) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
    } else if (cfg->Instance == SPI3) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_SPI3RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;
    } else if (cfg->Instance == SPI4) {
        RCC->APB2RSTR |=  RCC_APB2RSTR_SPI4RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
    } else if (cfg->Instance == SPI5) {
        RCC->APB2RSTR |=  RCC_APB2RSTR_SPI5RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI5RST;
    }

    /*
     * 4. Konfiguracja SPI CR1:
     *
     * MSTR         — tryb master, my generujemy SCK
     * BR[2:0]      — prescaler zegara z konfiguracji (BaudDiv)
     * CPOL         — polaryzacja SCK z konfiguracji
     * CPHA         — faza próbkowania z konfiguracji
     * SSM          — software slave management (CS obsługujemy przez GPIO)
     * SSI          — internal slave select, wymagane gdy SSM=1 i jesteśmy masterem
     * DFF=0        — ramka 8-bitowa (domyślnie)
     */
    cfg->Instance->CR1 = SPI_CR1_MSTR
                       | ((uint32_t)cfg->BaudDiv << SPI_CR1_BR_Pos)
                       | (cfg->CPOL ? SPI_CR1_CPOL : 0U)
                       | (cfg->CPHA ? SPI_CR1_CPHA : 0U)
                       | SPI_CR1_SSM
                       | SPI_CR1_SSI;

    /*
     * 5. Konfiguracja SPI CR2:
     * TXDMAEN — SPI żąda od DMA nowego bajtu gdy TX pusty
     * RXDMAEN — SPI przekazuje do DMA każdy odebrany bajt
     */
    cfg->Instance->CR2 = SPI_CR2_TXDMAEN
                       | SPI_CR2_RXDMAEN;

    /* 6. Włącz SPI — od tej chwili sprzęt jest gotowy */
    cfg->Instance->CR1 |= SPI_CR1_SPE;

    /*
     * 7. Zegar DMA.
     * SPI1 używa DMA2, SPI2/SPI3 używają DMA1.
     * Włączamy oba na wszelki wypadek — podwójne włączenie nie szkodzi.
     */
    if (cfg->Instance == SPI1 ||
        cfg->Instance == SPI4 ||
        cfg->Instance == SPI5) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    } else {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }

    /* 8. NVIC — tylko DMA RX generuje przerwanie TC które nas interesuje */
    NVIC_SetPriority(cfg->DmaRxIRQn, cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->DmaRxIRQn);
}

/* ================================================ */
/*                  TRANSFER SPI                    */
/* ================================================ */

bool SPI_Transfer(SPI_Handle_t  *hspi,
                  const uint8_t *tx_buf,
                  uint8_t       *rx_buf,
                  uint16_t       len,
                  uint32_t       timeout_ms) {

    DMA_Stream_TypeDef *dma_tx = hspi->Config.DmaTxStream;
    DMA_Stream_TypeDef *dma_rx = hspi->Config.DmaRxStream;

    /* Oznacz jako zajęte */
    hspi->Busy = true;

    /*
     * Konfiguracja DMA RX — NAJPIERW RX.
     * SPI w trybie full-duplex zaczyna odbierać natychmiast
     * gdy TX wysyła pierwszy bajt. DMA RX musi być gotowe
     * zanim TX ruszy, inaczej pierwszy bajt zostanie utracony.
     */
    dma_rx->CR = 0;
    while (dma_rx->CR & DMA_SxCR_EN);  // czekaj na wyłączenie

    /*
     * Wyczyść flagi statusu DMA RX.
     * SPI1 RX = DMA2 Stream0 → flagi w LIFCR (strumienie 0-3).
     * Stare flagi z poprzedniego transferu mogłyby natychmiast
     * wyzwolić przerwanie po starcie nowego.
     */
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0
                | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0
                | DMA_LIFCR_CFEIF0;

    dma_rx->PAR  = (uint32_t)&hspi->Config.Instance->DR; // źródło: rejestr danych SPI
    dma_rx->M0AR = (uint32_t)rx_buf;                      // cel: bufor w pamięci
    dma_rx->NDTR = len;                                    // liczba bajtów
    dma_rx->CR   = ((uint32_t)hspi->Config.DmaRxChannel << DMA_SxCR_CHSEL_Pos)
                 | DMA_SxCR_MINC              // inkrementacja adresu rx_buf
                 | (0U << DMA_SxCR_DIR_Pos)  // kierunek: peryferium → pamięć
                 | DMA_SxCR_TCIE;            // przerwanie po Transfer Complete

    /*
     * Konfiguracja DMA TX — wysyła z tx_buf do SPI DR.
     * SPI1 TX = DMA2 Stream3 → flagi w LIFCR.
     */
    dma_tx->CR = 0;
    while (dma_tx->CR & DMA_SxCR_EN);

    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3
                | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3
                | DMA_LIFCR_CFEIF3;

    dma_tx->PAR  = (uint32_t)&hspi->Config.Instance->DR;
    dma_tx->M0AR = (uint32_t)tx_buf;
    dma_tx->NDTR = len;
    dma_tx->CR   = ((uint32_t)hspi->Config.DmaTxChannel << DMA_SxCR_CHSEL_Pos)
                 | DMA_SxCR_MINC              // inkrementacja adresu tx_buf
                 | (1U << DMA_SxCR_DIR_Pos); // kierunek: pamięć → peryferium
                                              // brak TCIE — TX nie generuje przerwania

    /*
     * Start obu DMA — RX przed TX.
     * Od tej chwili SPI wysyła i odbiera bez udziału CPU.
     */
    dma_rx->CR |= DMA_SxCR_EN;
    dma_tx->CR |= DMA_SxCR_EN;

    /*
     * Task zasypia na semaforze.
     * Przerwanie TC (DMA RX) obudzi go gdy wszystkie bajty zostaną odebrane.
     * Podczas snu procesor wykonuje inne taski — zero busy-waiting.
     */
    if (xSemaphoreTake(hspi->TransferDone,
                       pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        /* Timeout — transfer nie zakończył się w czasie */
        hspi->Busy = false;
        return false;
    }

    hspi->Busy = false;
    return true;
}

/* ================================================ */
/*          PRZERWANIE DMA RX — Transfer Complete   */
/* ================================================ */

void SPI_DMA_RX_IRQHandler_CB(SPI_Handle_t *hspi) {
    /*
     * Sprawdzamy flagę TC dla Stream0 (SPI1 RX) w rejestrze LISR.
     * LISR zawiera flagi dla streamów 0-3.
     * TCIF0 = bit 5 w LISR.
     */
    if (DMA2->LISR & DMA_LISR_TCIF0) {

        /* Wyczyść flagę TC — obowiązkowe, inaczej przerwanie się zapętli */
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;

        /* Wyłącz oba streamy — transfer zakończony */
        hspi->Config.DmaTxStream->CR &= ~DMA_SxCR_EN;
        hspi->Config.DmaRxStream->CR &= ~DMA_SxCR_EN;

        /*
         * Obudź task przez semafor.
         *
         * xHigherPriorityTaskWoken — FreeRTOS sprawdza czy task
         * czekający na semafor ma wyższy priorytet niż aktualnie
         * wykonywany task. Jeśli tak — portYIELD_FROM_ISR
         * natychmiast przełącza kontekst zamiast czekać do
         * następnego ticka schedulera.
         * Efekt: minimalna latencja między TC a wznowieniem taska.
         */
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(hspi->TransferDone, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void SPI_DMA_TX_IRQHandler_CB(SPI_Handle_t *hspi) {
    /*
     * TX Transfer Complete — na razie nieużywane.
     * RX TC decyduje o końcu transferu full-duplex
     * bo RX kończy się zawsze po TX (SPI odbiera podczas wysyłania).
     */
    (void)hspi;
}


/**
 * @brief  Transfer blokujący — tylko do diagnostyki.
 *         Wysyła i odbiera przez polling bez DMA.
 *         Nie używaj w produkcji — blokuje task.
 */
bool SPI_Transfer_Blocking(SPI_Handle_t  *hspi,
                           const uint8_t *tx_buf,
                           uint8_t       *rx_buf,
                           uint16_t       len) {
    SPI_TypeDef *spi = hspi->Config.Instance;

    for (uint16_t i = 0; i < len; i++) {
        /* Czekaj aż bufor TX pusty */
        while (!(spi->SR & SPI_SR_TXE));
        spi->DR = tx_buf[i];

        /* Czekaj na odebrany bajt */
        while (!(spi->SR & SPI_SR_RXNE));
        rx_buf[i] = (uint8_t)spi->DR;
    }

    /* Czekaj aż SPI skończy ostatni bajt */
    while (spi->SR & SPI_SR_BSY);

    return true;
}
