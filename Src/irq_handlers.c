/**
 * @file    irq_handlers.c
 * @brief   Handlery przerwań sprzętowych.
 *
 * Zasada: tylko przekazanie wywołania do biblioteki.
 * Zero logiki aplikacji w tym pliku.
 *
 * UART2 TX: DMA1 Stream6 → TC budzi task UART
 * SPI1  RX: DMA2 Stream0 → TC budzi task czujnika
 * SPI1  TX: DMA2 Stream3 → nieużywane (RX decyduje o końcu)
 */

#include "uart/uart.h"
#include "spi/spi.h"

extern UART_Handle_t huart2;
extern SPI_Handle_t  hspi1;

/*
 * DMA1 Stream6 — Transfer Complete dla USART2 TX.
 * Wywołuje CB który sprawdza czy są kolejne dane do wysłania.
 */
void DMA1_Stream6_IRQHandler(void) {
    UART_DMA_TX_IRQHandler_CB(&huart2);
}

/*
 * DMA2 Stream0 — Transfer Complete dla SPI1 RX.
 * Budzi task który czeka na dane z czujnika.
 */
void DMA2_Stream0_IRQHandler(void) {
    SPI_DMA_RX_IRQHandler_CB(&hspi1);
}

/*
 * DMA2 Stream3 — Transfer Complete dla SPI1 TX.
 * Na razie nieużywane — RX TC decyduje o końcu full-duplex transferu.
 */
void DMA2_Stream3_IRQHandler(void) {
    SPI_DMA_TX_IRQHandler_CB(&hspi1);
}