/**
 * @file    irq_handlers.c
 * @brief   Handlery przerwań sprzętowych.
 *
 * Zasada: tylko przekazanie wywołania do biblioteki.
 * Zero logiki aplikacji w tym pliku.
 *
 * UART2 TX:  DMA1 Stream6 Ch4 → TC budzi task UART
 * SPI1  RX:  DMA2 Stream0 Ch3 → TC budzi task czujnika
 * SPI1  TX:  DMA2 Stream3 Ch3 → nieużywane
 * I2C1  EV:  I2C1_EV_IRQn    → maszyna stanów I2C
 * I2C1  ER:  I2C1_ER_IRQn    → obsługa błędów I2C
 * I2C1  TX:  DMA1 Stream1 Ch0 → TC budzi task I2C
 * I2C1  RX:  DMA1 Stream0 Ch1 → TC budzi task I2C
 */

#include "uart/uart.h"
#include "spi/spi.h"
#include "i2c/i2c.h"

extern UART_Handle_t huart2;
extern SPI_Handle_t  hspi1;
extern I2C_Handle_t  hi2c1;

/* UART2 TX */
void DMA1_Stream6_IRQHandler(void) {
    UART_DMA_TX_IRQHandler_CB(&huart2);
}

/* SPI1 RX */
void DMA2_Stream0_IRQHandler(void) {
    SPI_DMA_RX_IRQHandler_CB(&hspi1);
}

/* SPI1 TX */
void DMA2_Stream3_IRQHandler(void) {
    SPI_DMA_TX_IRQHandler_CB(&hspi1);
}

/* I2C1 — zdarzenia (SB, ADDR, BTF) i błędy (BERR, ARLO, AF) */
void I2C1_EV_IRQHandler(void) {
    I2C_IRQHandler_CB(&hi2c1);
}

void I2C1_ER_IRQHandler(void) {
    I2C_IRQHandler_CB(&hi2c1);
}

/* I2C1 TX DMA */
void DMA1_Stream1_IRQHandler(void) {
    I2C_DMA_TX_IRQHandler_CB(&hi2c1);
}

/* I2C1 RX DMA */
void DMA1_Stream0_IRQHandler(void) {
    I2C_DMA_RX_IRQHandler_CB(&hi2c1);
}