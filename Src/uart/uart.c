/**
 * @file    uart.c
 * @brief   Implementacja sterownika USART — DMA TX + DMA RX
 */

#include "uart.h"
#include "../gpio/gpio.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ================================================ */
/*          POMOCNICZE — START TRANSFERU DMA TX     */
/* ================================================ */

/**
 * @brief  Startuje transfer DMA z BufB.
 *         Wywoływane gdy:
 *         1) UART_Printf wykryje że DMA jest wolne
 *         2) Przerwanie TC wykryje że w BufA są nowe dane
 *
 *         Schemat:
 *         BufA (dane od taska) → kopiuj do BufB → start DMA z BufB
 *         BufA czyść → task może pisać od nowa
 */
static void dma_tx_start(UART_Handle_t *huart) {
    DMA_Stream_TypeDef *dma = huart->Config.DmaTxStream;

    /*
     * Kopiuj BufA do BufB — DMA będzie wysyłać z BufB.
     * Dzięki temu task może natychmiast pisać do BufA
     * podczas gdy DMA wysyła BufB. To właśnie sens podwójnego bufora.
     */
    memcpy(huart->TxBufB, huart->TxBufA, huart->TxLenA);
    uint16_t len = huart->TxLenA;

    /* Wyczyść BufA i jego licznik — task może pisać od nowa */
    huart->TxLenA = 0;

    /* Oznacz DMA jako zajęte — flaga sprawdzana w UART_Printf */
    huart->TxDmaBusy = true;

    /* Wyłącz stream przed rekonfiguracją — wymóg hardware */
    dma->CR &= ~DMA_SxCR_EN;
    while (dma->CR & DMA_SxCR_EN);

    /*
     * Wyczyść flagi statusu DMA w rejestrze IFCR.
     * Jeśli tego nie zrobisz — stare flagi mogą natychmiast
     * wyzwolić przerwanie po starcie, zanim cokolwiek wyślesz.
     * Stream6 → bity w HIFCR (strumienie 4-7)
     */
    DMA1->HIFCR = DMA_HIFCR_CTCIF6
                | DMA_HIFCR_CHTIF6
                | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CDMEIF6
                | DMA_HIFCR_CFEIF6;

    /* Adres źródła (pamięć) i cel (rejestr DR UARTa) */
    dma->M0AR = (uint32_t)huart->TxBufB;
    dma->PAR  = (uint32_t)&huart->Config.Instance->DR;
    dma->NDTR = len;

    /*
     * Konfiguracja streamu:
     * - CHSEL: numer kanału DMA (przypisanie sprzętowe)
     * - MINC:  inkrementacja adresu pamięci (kolejne bajty bufora)
     * - DIR:   kierunek pamięć → peryferium
     * - TCIE:  przerwanie po Transfer Complete
     */
    dma->CR = ((uint32_t)huart->Config.DmaTxChannel << DMA_SxCR_CHSEL_Pos)
            | DMA_SxCR_MINC
            | (1U << DMA_SxCR_DIR_Pos)
            | DMA_SxCR_TCIE;

    /* Włącz DMA TX w USART */
    huart->Config.Instance->CR3 |= USART_CR3_DMAT;

    /* Start — od tej chwili DMA wysyła bez udziału CPU */
    dma->CR |= DMA_SxCR_EN;
}

/* ================================================ */
/*                  INICJALIZACJA                   */
/* ================================================ */

void UART_Init(UART_Handle_t *huart, const UART_Config_t *cfg) {

    /* Zapisz konfigurację i zresetuj stan */
    huart->Config    = *cfg;
    huart->TxLenA    = 0;
    huart->TxDmaBusy = false;
    huart->RxTail    = 0;
    memset(huart->TxBufA, 0, sizeof(huart->TxBufA));
    memset(huart->TxBufB, 0, sizeof(huart->TxBufB));

    /*
     * Utwórz mutex TX.
     * Mutex gwarantuje że tylko jeden task na raz
     * modyfikuje TxBufA i TxLenA.
     * Musi być utworzony przed pierwszym wywołaniem UART_Printf.
     */
    huart->TxMutex = xSemaphoreCreateMutex();
    configASSERT(huart->TxMutex != NULL);

    /* 1. GPIO — piny TX i RX w trybie Alternate Function */
    GPIO_EnableClock(cfg->TxPort);
    GPIO_EnableClock(cfg->RxPort);
    GPIO_ConfigPin(cfg->TxPort, cfg->TxPin, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_ConfigPin(cfg->RxPort, cfg->RxPin, GPIO_MODE_AF, GPIO_PULL_UP,   GPIO_OTYPE_PP);
    GPIO_SetAF(cfg->TxPort, cfg->TxPin, cfg->TxAF);
    GPIO_SetAF(cfg->RxPort, cfg->RxPin, cfg->RxAF);
    GPIO_SetSpeed(cfg->TxPort, cfg->TxPin, GPIO_SPEED_HIGH);
    GPIO_SetSpeed(cfg->RxPort, cfg->RxPin, GPIO_SPEED_HIGH);

    /* 2. Zegar USART — zależy od instancji i magistrali */
    if      (cfg->Instance == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    else if (cfg->Instance == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    else if (cfg->Instance == USART6) RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    /* 3. Reset USART — czyści wszystkie rejestry do wartości domyślnych */
    if (cfg->Instance == USART1) {
        RCC->APB2RSTR |=  RCC_APB2RSTR_USART1RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    } else if (cfg->Instance == USART2) {
        RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
    } else if (cfg->Instance == USART6) {
        RCC->APB2RSTR |=  RCC_APB2RSTR_USART6RST;
        RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
    }

    /*
     * 4. Baudrate — wzór z RM:
     *    BRR = f_CLK / baudrate
     *    Dodajemy baudrate/2 przed dzieleniem — zaokrąglenie do najbliższej liczby
     *    zamiast obcinania. Np. 50MHz/115200 = 434.027 → bez zaokrąglenia = 434,
     *    z zaokrągleniem = 434. Przy 9600: 50MHz/9600 = 5208.3 → 5208 bez, 5208 z.
     *    Błąd baudrate <0.1% przy poprawnym zaokrągleniu.
     */
    cfg->Instance->BRR = (cfg->ClockFreq + cfg->BaudRate / 2U) / cfg->BaudRate;

    /*
     * 5. Konfiguracja USART:
     *    TE  — włącz nadajnik
     *    RE  — włącz odbiornik
     *    UE  — włącz USART (musi być ostatni)
     */
    cfg->Instance->CR1 = USART_CR1_TE
                       | USART_CR1_RE
                       | USART_CR1_UE;

    /*
     * 6. Włącz DMA dla RX w rejestrze CR3.
     *    DMAR — USART przekazuje odebrane bajty do DMA zamiast
     *    generować przerwanie RXNE przy każdym bajcie.
     */
    cfg->Instance->CR3 |= USART_CR3_DMAR;

    /* 7. Zegar DMA1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /*
     * 8. DMA RX — tryb circular:
     *    Stream odbiera bajty z DR UARTa i wpisuje do RxBuf[].
     *    CIRC — gdy dojdzie do końca bufora, wraca na początek.
     *    NDTR jest automatycznie przeładowywany przez hardware.
     */
    DMA_Stream_TypeDef *dma_rx = cfg->DmaRxStream;
    dma_rx->CR = 0;
    while (dma_rx->CR & DMA_SxCR_EN);

    dma_rx->PAR  = (uint32_t)&cfg->Instance->DR;
    dma_rx->M0AR = (uint32_t)huart->RxBuf;
    dma_rx->NDTR = UART_RX_BUF_SIZE;
    dma_rx->CR   = ((uint32_t)cfg->DmaRxChannel << DMA_SxCR_CHSEL_Pos)
                 | DMA_SxCR_MINC
                 | DMA_SxCR_CIRC;
    dma_rx->CR  |= DMA_SxCR_EN;

    /*
     * 9. NVIC dla DMA TX:
     *    Tylko DMA TX potrzebuje przerwania (TC po wysłaniu bloku).
     *    DMA RX działa w pełni autonomicznie — zero przerwań.
     */
    NVIC_SetPriority(cfg->DmaTxIRQn, cfg->NvicPriority);
    NVIC_EnableIRQ(cfg->DmaTxIRQn);
}

/* ================================================ */
/*                  WYSYŁANIE TX                    */
/* ================================================ */

void UART_Printf(UART_Handle_t *huart, const char *fmt, ...) {
    /*
     * Krok 1: Formatowanie stringa na lokalnym stosie taska.
     * vsnprintf nigdy nie przekroczy sizeof(local_buf) —
     * bezpieczne nawet przy złym formacie.
     */
    char local_buf[UART_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(local_buf, sizeof(local_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > (int)sizeof(local_buf) - 1) len = sizeof(local_buf) - 1;

    /*
     * Krok 2: Weź mutex.
     * Tylko jeden task na raz może modyfikować TxBufA.
     * Czekamy max UART_TX_TIMEOUT_MS — jeśli inny task
     * trzyma mutex zbyt długo, porzucamy wysyłanie.
     * Lepiej stracić jeden log niż zawiesić system.
     */
    if (xSemaphoreTake(huart->TxMutex,
        pdMS_TO_TICKS(UART_TX_TIMEOUT_MS)) != pdTRUE) {
        return;  // timeout — porzuć, nie blokuj systemu
    }

    /*
     * Krok 3: Sprawdź czy dane zmieszczą się w BufA.
     * Jeśli nie — porzuć (można by poczekać, ale
     * przy logowaniu drona lepiej stracić jeden pomiar).
     */
    if ((huart->TxLenA + (uint16_t)len) <= UART_TX_BUF_SIZE) {
        memcpy(&huart->TxBufA[huart->TxLenA], local_buf, len);
        huart->TxLenA += (uint16_t)len;
    }

    /*
     * Krok 4: Jeśli DMA wolne i mamy dane — startuj transfer.
     * Jeśli DMA zajęte — dane czekają w BufA.
     * Przerwanie TC samo odpali kolejny transfer gdy skończy.
     */
    if (!huart->TxDmaBusy && huart->TxLenA > 0) {
        dma_tx_start(huart);
    }

    /* Krok 5: Oddaj mutex — inny task może pisać */
    xSemaphoreGive(huart->TxMutex);
}

/* ================================================ */
/*                  ODBIERANIE RX                   */
/* ================================================ */

uint16_t UART_RxAvailable(UART_Handle_t *huart) {
    /*
     * DMA dekrementuje NDTR przy każdym odebranym bajcie.
     * Aktualna pozycja zapisu:
     *   head = rozmiar_bufora - pozostało_do_końca
     *
     * Przykład: bufor 128B, NDTR=100 → head=28
     * Oznacza: DMA zapisało 28 bajtów, zaczyna od indeksu 0.
     */
    uint16_t head = UART_RX_BUF_SIZE
                  - (uint16_t)huart->Config.DmaRxStream->NDTR;

    /* Obsługa zawinięcia bufora circular */
    if (head >= huart->RxTail)
        return head - huart->RxTail;
    else
        return UART_RX_BUF_SIZE - huart->RxTail + head;
}

uint8_t UART_RxRead(UART_Handle_t *huart) {
    uint8_t byte = huart->RxBuf[huart->RxTail];
    huart->RxTail = (huart->RxTail + 1U) % UART_RX_BUF_SIZE;
    return byte;
}

void UART_RxFlush(UART_Handle_t *huart) {
    /* Przesuń RxTail do aktualnej pozycji DMA — ignoruj wszystko co przyszło */
    huart->RxTail = UART_RX_BUF_SIZE
                  - (uint16_t)huart->Config.DmaRxStream->NDTR;
}

/* ================================================ */
/*            PRZERWANIE DMA TX — Transfer Complete */
/* ================================================ */

void UART_DMA_TX_IRQHandler_CB(UART_Handle_t *huart) {
    DMA_Stream_TypeDef *dma = huart->Config.DmaTxStream;

    /*
     * Sprawdź flagę TC (Transfer Complete) w rejestrze HISR.
     * Stream6 → bity w HISR[21:16].
     * Tylko TC nas interesuje — błędy DMA pomijamy na razie.
     */
    if (DMA1->HISR & DMA_HISR_TCIF6) {

        /* Wyczyść flagę TC — obowiązkowe, inaczej przerwanie się zapętli */
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        /* Wyłącz stream i DMA TX w USART */
        dma->CR &= ~DMA_SxCR_EN;
        huart->Config.Instance->CR3 &= ~USART_CR3_DMAT;

        /*
         * Sprawdź czy task zdążył wpisać nowe dane do BufA
         * podczas gdy DMA wysyłało BufB.
         * Jeśli tak — od razu startuj kolejny transfer.
         * Jeśli nie — oznacz DMA jako wolne i czekaj.
         */
        if (huart->TxLenA > 0) {
            dma_tx_start(huart);
        } else {
            huart->TxDmaBusy = false;
        }
    }
}