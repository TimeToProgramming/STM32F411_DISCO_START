/**
 * @file    irq_handlers.c
 * @brief   Handlery przerwań sprzętowych.
 *          Tylko przekazują wywołanie do biblioteki —
 *          zero logiki aplikacji tutaj.
 */

#include "uart/uart.h"

extern UART_Handle_t huart2;

/*
 * DMA1 Stream6 — Transfer Complete dla USART2 TX.
 * Wywołuje CB który sprawdza czy są nowe dane do wysłania.
 */
void DMA1_Stream6_IRQHandler(void) {
    UART_DMA_TX_IRQHandler_CB(&huart2);
}