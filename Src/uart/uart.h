#pragma once
/**
 * @file    uart.h
 * @brief   Sterownik USART dla STM32F411
 *
 *
 * TX — podwójny bufor + DMA:
 *   - Task pisze do BufA przez mutex (bezpieczne wielowątkowo)
 *   - Gdy BufA gotowy — zamiana z BufB, DMA wysyła BufB
 *   - Po TC (Transfer Complete) — sprawdź czy jest coś w nowym BufA
 *   - CPU nie jest przerywany co bajt — jedno przerwanie TC na cały blok
 *
 * RX — DMA circular:
 *   - DMA sam wpisuje odebrane bajty do RxBuf[] bez udziału CPU
 *   - Odczyt przez porównanie pozycji DMA z RxTail
 *   - Zero przerwań przy odbiorze pojedynczych bajtów
 *
 * Pełna konfiguracja przez UART_Config_t —
 * brak hardkodowanych pinów, instancji, prędkości.
 */

#include "stm32f411xe.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================ */
/*              KONFIGURACJA BUFORÓW                */
/* ================================================ */

/** Rozmiar jednego bufora TX (A i B mają taki sam rozmiar) */
#define UART_TX_BUF_SIZE    256

/** Rozmiar bufora RX dla DMA circular */
#define UART_RX_BUF_SIZE    128

/** Maksymalny czas oczekiwania taska na wolny bufor TX [ms] */
#define UART_TX_TIMEOUT_MS  10

/* ================================================ */
/*              STRUKTURY                           */
/* ================================================ */

/**
 * @brief Konfiguracja przekazywana do UART_Init.
 *        Wypełnij przed wywołaniem init, potem nie modyfikuj.
 */
typedef struct {
    USART_TypeDef       *Instance;      /*!< USART1, USART2, USART6 */
    uint32_t             BaudRate;      /*!< np. 115200 */
    uint32_t             ClockFreq;     /*!< częstotliwość APB w Hz */

    GPIO_TypeDef        *TxPort;        /*!< np. GPIOA */
    uint32_t             TxPin;         /*!< np. 2 */
    uint8_t              TxAF;          /*!< np. 7 */

    GPIO_TypeDef        *RxPort;
    uint32_t             RxPin;
    uint8_t              RxAF;

    DMA_Stream_TypeDef  *DmaTxStream;   /*!< np. DMA1_Stream6 */
    uint8_t              DmaTxChannel;  /*!< np. 4 */
    IRQn_Type            DmaTxIRQn;     /*!< np. DMA1_Stream6_IRQn */

    DMA_Stream_TypeDef  *DmaRxStream;   /*!< np. DMA1_Stream5 */
    uint8_t              DmaRxChannel;  /*!< np. 4 */

    uint8_t              NvicPriority;  /*!< priorytet przerwań */
} UART_Config_t;

/**
 * @brief Handle — trwały stan sterownika.
 *        Jeden handle na każdy używany UART.
 *        Nie modyfikuj pól bezpośrednio — używaj API.
 */
typedef struct {
    UART_Config_t        Config;

    /*
     * TX — podwójny bufor:
     * BufA — task pisze tutaj (chroniony mutexem)
     * BufB — DMA wysyła stąd
     * Po TC następuje zamiana: A←→B
     */
    uint8_t              TxBufA[UART_TX_BUF_SIZE];  /*!< bufor zapisu */
    uint8_t              TxBufB[UART_TX_BUF_SIZE];  /*!< bufor DMA    */
    volatile uint16_t    TxLenA;        /*!< ile bajtów w BufA         */
    volatile bool        TxDmaBusy;     /*!< czy DMA aktualnie wysyła  */

    SemaphoreHandle_t    TxMutex;       /*!< mutex dla wielowątkowego TX */

    /*
     * RX — DMA circular:
     * DMA wpisuje do RxBuf[] bez udziału CPU
     * RxTail śledzi gdzie skończyliśmy czytać
     */
    uint8_t              RxBuf[UART_RX_BUF_SIZE];
    volatile uint16_t    RxTail;
} UART_Handle_t;

/* ================================================ */
/*                  PUBLICZNE API                   */
/* ================================================ */

/**
 * @brief  Inicjalizuje UART z podaną konfiguracją.
 *         Wywołaj raz przed vTaskStartScheduler lub z taska.
 * @param  huart  wskaźnik na handle
 * @param  cfg    wskaźnik na wypełnioną strukturę konfiguracji
 */
void     UART_Init(UART_Handle_t *huart, const UART_Config_t *cfg);

/**
 * @brief  Wysyła sformatowany string przez UART (nieblokująco).
 *         Czeka max UART_TX_TIMEOUT_MS na wolny bufor.
 *         Bezpieczne do wywołania z wielu tasków jednocześnie.
 */
void     UART_Printf(UART_Handle_t *huart, const char *fmt, ...);

/**
 * @brief  Zwraca liczbę dostępnych bajtów w buforze RX.
 */
uint16_t UART_RxAvailable(UART_Handle_t *huart);

/**
 * @brief  Odczytuje jeden bajt z bufora RX.
 *         Sprawdź UART_RxAvailable() przed wywołaniem.
 */
uint8_t  UART_RxRead(UART_Handle_t *huart);

/**
 * @brief  Czyści bufor RX — przesuwa RxTail do aktualnej pozycji DMA.
 */
void     UART_RxFlush(UART_Handle_t *huart);

/**
 * @brief  Wywoływane z DMA TX IRQHandler — nie używaj bezpośrednio.
 */
void     UART_DMA_TX_IRQHandler_CB(UART_Handle_t *huart);