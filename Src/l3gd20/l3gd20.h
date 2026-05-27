#pragma once
/**
 * @file    l3gd20.h
 * @brief   Sterownik żyroskopu L3GD20 dla STM32F411E-DISCO
 *
 * L3GD20 — żyroskop 3-osiowy MEMS
 * Interfejs: SPI1, Mode 0 (CPOL=0, CPHA=0) wg datasheet Fig.12
 * Piny na Discovery:
 *   PA5 — SCK
 *   PA6 — MISO (SDO)
 *   PA7 — MOSI (SDI)
 *   PE3 — CS
 *
 * Format ramki SPI wg datasheet sekcja 5.2:
 *   bit7 = RW  (1=read,  0=write)
 *   bit6 = MS  (1=auto-increment adresu w multi-byte)
 *   bit5-0 = adres rejestru AD(5:0)
 */

#include "stm32f411xe.h"
#include "../spi/spi.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================ */
/*              ADRESY REJESTRÓW                    */
/* ================================================ */
#define L3GD20_REG_WHO_AM_I     0x0F  /*!< ID chipu — odczyt: 0xD4/0xD7 */
#define L3GD20_REG_CTRL_REG1    0x20  /*!< ODR, BW, PD, Zen, Yen, Xen */
#define L3GD20_REG_CTRL_REG2    0x21  /*!< Filtr HP */
#define L3GD20_REG_CTRL_REG3    0x22  /*!< Konfiguracja przerwań */
#define L3GD20_REG_CTRL_REG4    0x23  /*!< Zakres, BDU, BLE, SIM */
#define L3GD20_REG_CTRL_REG5    0x24  /*!< FIFO, HP filter */
#define L3GD20_REG_OUT_X_L      0x28  /*!< Oś X — dolny bajt */
#define L3GD20_REG_OUT_X_H      0x29  /*!< Oś X — górny bajt */
#define L3GD20_REG_OUT_Y_L      0x2A  /*!< Oś Y — dolny bajt */
#define L3GD20_REG_OUT_Y_H      0x2B  /*!< Oś Y — górny bajt */
#define L3GD20_REG_OUT_Z_L      0x2C  /*!< Oś Z — dolny bajt */
#define L3GD20_REG_OUT_Z_H      0x2D  /*!< Oś Z — górny bajt */

/* ================================================ */
/*              BITY PROTOKOŁU SPI                  */
/* ================================================ */
/*
 * Bit 7 bajtu adresu — READ/WRITE:
 * 0 = zapis do rejestru
 * 1 = odczyt z rejestru
 */
#define L3GD20_READ_BIT         0x80U

/*
 * Bit 6 bajtu adresu — auto-increment:
 * 0 = adres stały (single read/write)
 * 1 = adres inkrementowany po każdym bajcie (multi read/write)
 */
#define L3GD20_AUTO_INC_BIT     0x40U

/* ================================================ */
/*              WHO_AM_I — wartości                 */
/* ================================================ */
#define L3GD20_WHO_AM_I_VAL     0xD4U  /*!< Standardowa wersja L3GD20 */
#define L3GD20H_WHO_AM_I_VAL    0xD7U  /*!< Wersja L3GD20H */
#define L3GD20T_WHO_AM_I_VAL    0xD3U  /*!< Starsza wersja — Discovery F411 */

/* ================================================ */
/*              CTRL_REG1 — bity                    */
/* ================================================ */
/*
 * PD=1 → Normal mode (PD=0 → Power Down)
 * Zen/Yen/Xen=1 → dana oś włączona
 * DR[1:0]+BW[1:0] → ODR i bandwidth (domyślnie 00 00 = 95Hz)
 */
#define L3GD20_CTRL1_PD         (1U << 3)
#define L3GD20_CTRL1_ZEN        (1U << 2)
#define L3GD20_CTRL1_YEN        (1U << 1)
#define L3GD20_CTRL1_XEN        (1U << 0)

/* ================================================ */
/*              CTRL_REG4 — zakresy                 */
/* ================================================ */
/*
 * FS[1:0] w bitach [5:4] CTRL_REG4:
 * 00 = ±250  dps, czułość 8.75  mdps/digit
 * 01 = ±500  dps, czułość 17.5  mdps/digit
 * 10 = ±2000 dps, czułość 70.0  mdps/digit
 */
#define L3GD20_SCALE_250DPS     0x00U
#define L3GD20_SCALE_500DPS     0x10U
#define L3GD20_SCALE_2000DPS    0x20U

/* ================================================ */
/*              STRUKTURY                           */
/* ================================================ */

/**
 * @brief Surowe dane z ADC czujnika — przed przeliczeniem.
 *        Wartości 16-bitowe ze znakiem (two's complement).
 */
typedef struct {
    int16_t x;  /*!< Surowa wartość osi X */
    int16_t y;  /*!< Surowa wartość osi Y */
    int16_t z;  /*!< Surowa wartość osi Z */
} L3GD20_RawData_t;

/**
 * @brief Dane przeliczone na stopnie na sekundę [°/s].
 */
typedef struct {
    float x;  /*!< Prędkość kątowa osi X [°/s] */
    float y;  /*!< Prędkość kątowa osi Y [°/s] */
    float z;  /*!< Prędkość kątowa osi Z [°/s] */
} L3GD20_Data_t;

/**
 * @brief Handle czujnika — trwały stan sterownika.
 */
typedef struct {
    SPI_Handle_t    *hspi;          /*!< Wskaźnik na zainicjalizowany handle SPI */
    GPIO_TypeDef    *CsPort;        /*!< Port pinu CS */
    uint32_t         CsPin;         /*!< Numer pinu CS */
    float            Sensitivity;   /*!< Czułość [°/s na digit] — zależy od zakresu */
    uint8_t          Scale;         /*!< Aktualny zakres (L3GD20_SCALE_*) */
} L3GD20_Handle_t;

/* ================================================ */
/*                  PUBLICZNE API                   */
/* ================================================ */

/**
 * @brief  Inicjalizuje żyroskop.
 *         Konfiguruje CS, sprawdza WHO_AM_I,
 *         ustawia ODR 95Hz i wybrany zakres.
 * @param  hgyro    handle czujnika
 * @param  hspi     handle SPI (już zainicjalizowany)
 * @param  cs_port  port CS (np. GPIOE)
 * @param  cs_pin   pin CS (np. 3)
 * @param  scale    zakres (L3GD20_SCALE_250DPS itp.)
 * @return true jeśli WHO_AM_I poprawny i konfiguracja OK
 */
bool L3GD20_Init(L3GD20_Handle_t *hgyro,
                 SPI_Handle_t    *hspi,
                 GPIO_TypeDef    *cs_port,
                 uint32_t         cs_pin,
                 uint8_t          scale);

/**
 * @brief  Odczytuje surowe dane 16-bit z czujnika.
 * @param  hgyro  handle czujnika
 * @param  raw    wskaźnik na strukturę wynikową
 * @return true jeśli transfer SPI zakończony poprawnie
 */
bool L3GD20_ReadRaw(L3GD20_Handle_t *hgyro, L3GD20_RawData_t *raw);

/**
 * @brief  Odczytuje dane przeliczone na °/s.
 *         Wywołuje L3GD20_ReadRaw i mnoży przez czułość.
 * @param  hgyro  handle czujnika
 * @param  data   wskaźnik na strukturę wynikową
 * @return true jeśli odczyt OK
 */
bool L3GD20_ReadDPS(L3GD20_Handle_t *hgyro, L3GD20_Data_t *data);