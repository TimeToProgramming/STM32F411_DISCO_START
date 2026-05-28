/**
 * @file    i2c.c
 * @brief   Implementacja sterownika I2C — maszyna stanów + DMA + FreeRTOS
 *
 * I2C na F411 używa starszego modelu rejestrów:
 *   CR1, CR2  — kontrola i przerwania
 *   CCR       — prędkość zegara
 *   TRISE     — czas narastania sygnału
 *   SR1, SR2  — flagi statusu i błędów
 *   DR        — rejestr danych
 *
 * Sekwencja zapisu:
 *   START(SB) → ADDR+W(ADDR) → dane przez DMA(TC) → STOP
 *
 * Sekwencja odczytu:
 *   START(SB) → ADDR+W(ADDR) → REG(BTF) →
 *   RESTART(SB) → ADDR+R(ADDR) → dane przez DMA(TC) → STOP
 *
 * Błąd w dowolnym miejscu → retry → Bus Recovery jeśli wyczerpano próby
 */

#include "i2c.h"
#include "../gpio/gpio.h"

/* ================================================ */
/*              BUS RECOVERY                        */
/* ================================================ */

/**
 * @brief  Przywraca magistralę I2C po błędzie (bus hang).
 *
 * Problem: slave utknął w trakcie transferu i trzyma SDA=LOW.
 * Rozwiązanie wg standardu I2C (UM10204 sekcja 3.1.16):
 *   1. Przełącz piny na GPIO
 *   2. Generuj do 9 taktów SCL — slave dokończy bajt i zwolni SDA
 *   3. Wyślij ręczny STOP
 *   4. Przywróć piny do AF i zresetuj peryferium
 *
 * 9 taktów = 8 bitów danych + 1 bit ACK — najgorszy przypadek.
 */
static void I2C_BusRecovery(I2C_Handle_t *hi2c) {
    GPIO_TypeDef *scl_port = hi2c->Config.SclPort;
    GPIO_TypeDef *sda_port = hi2c->Config.SdaPort;
    uint32_t      scl_pin  = hi2c->Config.SclPin;
    uint32_t      sda_pin  = hi2c->Config.SdaPin;

    /* 1. Wyłącz peryferium */
    hi2c->Config.Instance->CR1 &= ~I2C_CR1_PE;

    /* 2. Przełącz na GPIO output open-drain z pull-up */
    GPIO_ConfigPin(scl_port, scl_pin,
                   GPIO_MODE_OUTPUT, GPIO_PULL_UP, GPIO_OTYPE_OD);
    GPIO_ConfigPin(sda_port, sda_pin,
                   GPIO_MODE_OUTPUT, GPIO_PULL_UP, GPIO_OTYPE_OD);

    GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);

    /* 3. Generuj do 9 taktów SCL */
    for (int i = 0; i < 9; i++) {
        GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 50; d++);  /* ~5µs przy 100MHz */
        GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        for (volatile int d = 0; d < 50; d++);

        if (GPIO_ReadPin(sda_port, sda_pin)) {
            break;  /* Slave zwolnił SDA */
        }
    }

    /* 4. Ręczny STOP: SCL HIGH, SDA LOW→HIGH */
    GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 50; d++);
    GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++);
    GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 50; d++);

    /* 5. Przywróć AF */
    GPIO_ConfigPin(scl_port, scl_pin,
                   GPIO_MODE_AF, GPIO_PULL_UP, GPIO_OTYPE_OD);
    GPIO_ConfigPin(sda_port, sda_pin,
                   GPIO_MODE_AF, GPIO_PULL_UP, GPIO_OTYPE_OD);
    GPIO_SetAF(scl_port, scl_pin, hi2c->Config.SclAF);
    GPIO_SetAF(sda_port, sda_pin, hi2c->Config.SdaAF);

    /* 6. Reset peryferium przez RCC */
    if (hi2c->Config.Instance == I2C1) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (hi2c->Config.Instance == I2C2) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    } else if (hi2c->Config.Instance == I2C3) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C3RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
    }

    /* 7. Przywróć konfigurację rejestrów */
    hi2c->Config.Instance->CR2 =
        (hi2c->Config.ApbClockFreq / 1000000U);

    uint32_t ccr;
    if (hi2c->Config.ClockSpeed <= 100000U) {
        ccr = hi2c->Config.ApbClockFreq / (2U * hi2c->Config.ClockSpeed);
        hi2c->Config.Instance->CCR   = ccr;
        hi2c->Config.Instance->TRISE =
            (hi2c->Config.ApbClockFreq / 1000000U) + 1U;
    } else {
        ccr = hi2c->Config.ApbClockFreq / (3U * hi2c->Config.ClockSpeed);
        hi2c->Config.Instance->CCR   = I2C_CCR_FS | ccr;
        hi2c->Config.Instance->TRISE =
            ((hi2c->Config.ApbClockFreq / 1000000U) * 3U / 10U) + 1U;
    }

    /* 8. Włącz z powrotem */
    hi2c->Config.Instance->CR1 |= I2C_CR1_PE;

    /* Reset stanu */
    hi2c->State      = I2C_STATE_IDLE;
    hi2c->RetryCount = 0;
    hi2c->Error      = false;
}

/* ================================================ */
/*                  INICJALIZACJA                   */
/* ================================================ */

void I2C_Init(I2C_Handle_t *hi2c, const I2C_Config_t *cfg) {
    hi2c->Config     = *cfg;
    hi2c->Busy       = false;
    hi2c->State      = I2C_STATE_IDLE;
    hi2c->Error      = false;
    hi2c->RetryCount = 0;

    hi2c->Done = xSemaphoreCreateBinary();
    configASSERT(hi2c->Done != NULL);

    /* 1. GPIO — open-drain z pull-up */
    GPIO_EnableClock(cfg->SclPort);
    GPIO_EnableClock(cfg->SdaPort);
    GPIO_ConfigPin(cfg->SclPort, cfg->SclPin,
                   GPIO_MODE_AF, GPIO_PULL_UP, GPIO_OTYPE_OD);
    GPIO_ConfigPin(cfg->SdaPort, cfg->SdaPin,
                   GPIO_MODE_AF, GPIO_PULL_UP, GPIO_OTYPE_OD);
    GPIO_SetAF(cfg->SclPort, cfg->SclPin, cfg->SclAF);
    GPIO_SetAF(cfg->SdaPort, cfg->SdaPin, cfg->SdaAF);
    GPIO_SetSpeed(cfg->SclPort, cfg->SclPin, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(cfg->SdaPort, cfg->SdaPin, GPIO_SPEED_HIGH);

    /* 2. Zegar I2C */
    if      (cfg->Instance == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    else if (cfg->Instance == I2C2) RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    else if (cfg->Instance == I2C3) RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    /* 3. Reset */
    if (cfg->Instance == I2C1) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (cfg->Instance == I2C2) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    } else if (cfg->Instance == I2C3) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C3RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
    }

    /* 4. CR2 — częstotliwość APB1 w MHz */
    cfg->Instance->CR2 = (cfg->ApbClockFreq / 1000000U);

    /* 5. CCR — dzielnik zegara */
    uint32_t ccr;
    if (cfg->ClockSpeed <= 100000U) {
        ccr = cfg->ApbClockFreq / (2U * cfg->ClockSpeed);
        cfg->Instance->CCR = ccr;
    } else {
        ccr = cfg->ApbClockFreq / (3U * cfg->ClockSpeed);
        cfg->Instance->CCR = I2C_CCR_FS | ccr;
    }

    /* 6. TRISE — czas narastania */
    if (cfg->ClockSpeed <= 100000U) {
        cfg->Instance->TRISE = (cfg->ApbClockFreq / 1000000U) + 1U;
    } else {
        cfg->Instance->TRISE =
            ((cfg->ApbClockFreq / 1000000U) * 3U / 10U) + 1U;
    }

    /* 7. Włącz I2C */
    cfg->Instance->CR1 |= I2C_CR1_PE;

    /* 8. DMA1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* 9. NVIC — przerwania I2C i DMA */
    NVIC_SetPriority(cfg->EventIRQn,  cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->EventIRQn);
    NVIC_SetPriority(cfg->ErrorIRQn,  cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->ErrorIRQn);
    NVIC_SetPriority(cfg->DmaTxIRQn,  cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->DmaTxIRQn);
    NVIC_SetPriority(cfg->DmaRxIRQn,  cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->DmaRxIRQn);
}

/* ================================================ */
/*              MASZYNA STANÓW — PRZERWANIE I2C     */
/* ================================================ */

void I2C_IRQHandler_CB(I2C_Handle_t *hi2c) {
    I2C_TypeDef *i2c = hi2c->Config.Instance;
    uint32_t sr1 = i2c->SR1;

    /*
     * Obsługa błędów — najwyższy priorytet.
     * BERR = błąd magistrali
     * ARLO = arbitration lost
     * AF   = NACK (slave nie odpowiada)
     * OVR  = overrun
     */
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO |
               I2C_SR1_AF   | I2C_SR1_OVR)) {

        i2c->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO |
                      I2C_SR1_AF   | I2C_SR1_OVR);
        i2c->CR1 |= I2C_CR1_STOP;

        hi2c->Config.DmaTxStream->CR &= ~DMA_SxCR_EN;
        hi2c->Config.DmaRxStream->CR &= ~DMA_SxCR_EN;
        i2c->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);

        if (hi2c->RetryCount < hi2c->Config.MaxRetry) {
            /* Ponów transakcję */
            hi2c->RetryCount++;
            hi2c->State = I2C_STATE_START;
            i2c->CR1 |= I2C_CR1_START;
        } else {
            /* Wyczerpano retry — Bus Recovery */
            I2C_BusRecovery(hi2c);
            hi2c->Error = true;
            hi2c->Busy  = false;

            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(hi2c->Done, &woken);
            portYIELD_FROM_ISR(woken);
        }
        return;
    }

    switch (hi2c->State) {

        case I2C_STATE_START:
        case I2C_STATE_RESTART:
            /*
             * SB — Start Bit wygenerowany.
             * Wyślij adres urządzenia.
             */
            if (sr1 & I2C_SR1_SB) {
                if (!hi2c->IsRead || hi2c->State == I2C_STATE_START) {
                    /* Zapis lub pierwsza faza odczytu — adres+W */
                    i2c->DR = (uint8_t)((hi2c->DevAddr << 1) & ~0x01U);
                    hi2c->State = I2C_STATE_ADDR_W;
                } else {
                    /* Druga faza odczytu po RESTART — adres+R */
                    i2c->DR = (uint8_t)((hi2c->DevAddr << 1) | 0x01U);
                    hi2c->State = I2C_STATE_ADDR_R;
                }
            }
            break;

        case I2C_STATE_ADDR_W:
            /*
             * ADDR — adres+W potwierdzony (ACK).
             * Kasuj ADDR przez odczyt SR1+SR2.
             */
            if (sr1 & I2C_SR1_ADDR) {
                (void)i2c->SR1;
                (void)i2c->SR2;

                if (hi2c->IsRead) {
                    /* Odczyt — wyślij adres rejestru */
                    i2c->DR = hi2c->RegAddr;
                    hi2c->State = I2C_STATE_REG;
                } else {
                    /* Zapis — włącz DMA TX */
                    i2c->CR1 |= I2C_CR1_ACK;
                    i2c->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
                    hi2c->Config.DmaTxStream->CR |= DMA_SxCR_EN;
                    hi2c->State = I2C_STATE_TX_DMA;
                }
            }
            break;

        case I2C_STATE_REG:
            /*
             * BTF — adres rejestru wysłany i potwierdzony.
             * Generuj Repeated START dla odczytu.
             */
            if (sr1 & I2C_SR1_BTF) {
                i2c->CR1 |= I2C_CR1_START;
                hi2c->State = I2C_STATE_RESTART;
            }
            break;

        case I2C_STATE_ADDR_R:
            /*
             * ADDR — adres+R potwierdzony.
             * Włącz ACK i DMA RX.
             * LAST — DMA wyczyści ACK przed ostatnim bajtem → NACK → STOP.
             */
            if (sr1 & I2C_SR1_ADDR) {
                i2c->CR1 |= I2C_CR1_ACK;
                (void)i2c->SR1;
                (void)i2c->SR2;

                i2c->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
                hi2c->Config.DmaRxStream->CR |= DMA_SxCR_EN;
                hi2c->State = I2C_STATE_RX_DMA;
            }
            break;

        default:
            break;
    }
}

/* ================================================ */
/*          PRZERWANIA DMA                          */
/* ================================================ */

void I2C_DMA_TX_IRQHandler_CB(I2C_Handle_t *hi2c) {
    /*
     * TC dla DMA TX — wszystkie bajty wysłane.
     * Generujemy STOP i budzimy task.
     */
    if (DMA1->LISR & DMA_LISR_TCIF1) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        hi2c->Config.DmaTxStream->CR &= ~DMA_SxCR_EN;
        hi2c->Config.Instance->CR2   &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        hi2c->Config.Instance->CR1   |= I2C_CR1_STOP;

        hi2c->State = I2C_STATE_IDLE;
        hi2c->Busy  = false;

        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(hi2c->Done, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void I2C_DMA_RX_IRQHandler_CB(I2C_Handle_t *hi2c) {
    /*
     * TC dla DMA RX — wszystkie bajty odebrane.
     * DMA automatycznie wygenerował NACK (bit LAST).
     * Generujemy STOP i budzimy task.
     */
    if (DMA1->LISR & DMA_LISR_TCIF0) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        hi2c->Config.DmaRxStream->CR &= ~DMA_SxCR_EN;
        hi2c->Config.Instance->CR1   &= ~I2C_CR1_ACK;
        hi2c->Config.Instance->CR2   &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        hi2c->Config.Instance->CR1   |= I2C_CR1_STOP;

        hi2c->State = I2C_STATE_IDLE;
        hi2c->Busy  = false;

        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(hi2c->Done, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ================================================ */
/*              PUBLICZNE API                       */
/* ================================================ */

bool I2C_Write(I2C_Handle_t *hi2c, uint8_t addr,
               uint8_t *buf, uint16_t len, uint32_t timeout_ms) {

    if (hi2c->Busy) return false;

    /* Konfiguracja DMA TX */
    DMA_Stream_TypeDef *dma = hi2c->Config.DmaTxStream;
    dma->CR = 0;
    while (dma->CR & DMA_SxCR_EN);

    DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1
                | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1
                | DMA_LIFCR_CFEIF1;

    dma->PAR  = (uint32_t)&hi2c->Config.Instance->DR;
    dma->M0AR = (uint32_t)buf;
    dma->NDTR = len;
    dma->CR   = ((uint32_t)hi2c->Config.DmaTxChannel << DMA_SxCR_CHSEL_Pos)
              | DMA_SxCR_MINC
              | (1U << DMA_SxCR_DIR_Pos)
              | DMA_SxCR_TCIE;

    /* Parametry transakcji */
    hi2c->DevAddr    = addr;
    hi2c->Buffer     = buf;
    hi2c->Length     = len;
    hi2c->IsRead     = false;
    hi2c->Error      = false;
    hi2c->RetryCount = 0;
    hi2c->Busy       = true;
    hi2c->State      = I2C_STATE_START;

    /* Włącz przerwania I2C i wyślij START */
    hi2c->Config.Instance->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    hi2c->Config.Instance->CR1 |= I2C_CR1_START;

    /* Task zasypia */
    if (xSemaphoreTake(hi2c->Done,
                       pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        hi2c->Busy = false;
        return false;
    }

    /* Wyłącz przerwania I2C */
    hi2c->Config.Instance->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    return !hi2c->Error;
}

bool I2C_Read(I2C_Handle_t *hi2c, uint8_t addr, uint8_t reg,
              uint8_t *buf, uint16_t len, uint32_t timeout_ms) {

    if (hi2c->Busy) return false;

    /* Konfiguracja DMA RX */
    DMA_Stream_TypeDef *dma_rx = hi2c->Config.DmaRxStream;
    dma_rx->CR = 0;
    while (dma_rx->CR & DMA_SxCR_EN);

    DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0
                | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0
                | DMA_LIFCR_CFEIF0;

    dma_rx->PAR  = (uint32_t)&hi2c->Config.Instance->DR;
    dma_rx->M0AR = (uint32_t)buf;
    dma_rx->NDTR = len;
    dma_rx->CR   = ((uint32_t)hi2c->Config.DmaRxChannel << DMA_SxCR_CHSEL_Pos)
                 | DMA_SxCR_MINC
                 | (0U << DMA_SxCR_DIR_Pos)
                 | DMA_SxCR_TCIE;

    /* Parametry transakcji */
    hi2c->DevAddr    = addr;
    hi2c->RegAddr    = reg;
    hi2c->Buffer     = buf;
    hi2c->Length     = len;
    hi2c->IsRead     = true;
    hi2c->Error      = false;
    hi2c->RetryCount = 0;
    hi2c->Busy       = true;
    hi2c->State      = I2C_STATE_START;

    /* Włącz przerwania I2C i wyślij START */
    hi2c->Config.Instance->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    hi2c->Config.Instance->CR1 |= I2C_CR1_START;

    /* Task zasypia */
    if (xSemaphoreTake(hi2c->Done,
                       pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        hi2c->Busy = false;
        return false;
    }

    /* Wyłącz przerwania I2C */
    hi2c->Config.Instance->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    return !hi2c->Error;
}