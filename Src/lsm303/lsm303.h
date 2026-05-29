#pragma once
/**
 * @file    lsm303.h
 * @brief   Sterownik akcelerometru LSM303DLHC dla STM32F411E-DISCO
 *
 * LSM303DLHC — akcelerometr 3-osiowy + magnetometr
 * Interfejs: I2C1, PB6=SCL, PB9=SDA
 * Adresy:
 *   Akcelerometr: 0x19
 *   Magnetometr:  0x1E (na razie nie używamy)
 *
 * Czułość przy ±2g: 1mg/digit = 0.001g/digit
 */

#include "stm32f411xe.h"
#include "../i2c/i2c.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================ */
/*              ADRESY I2C                          */
/* ================================================ */
#define LSM303_ACCEL_ADDR        0x19U  /*!< Adres 7-bitowy akcelerometru */
#define LSM303_MAG_ADDR          0x1EU  /*!< Adres 7-bitowy magnetometru */

/* ================================================ */
/*              REJESTRY AKCELEROMETRU              */
/* ================================================ */
#define LSM303_CTRL_REG1_A       0x20U  /*!< ODR, LPen, osie XYZ */
#define LSM303_CTRL_REG4_A       0x23U  /*!< Zakres, high-res, BDU */
#define LSM303_OUT_X_L_A         0x28U  /*!< Dane X — dolny bajt */
#define LSM303_OUT_X_H_A         0x29U  /*!< Dane X — górny bajt */
#define LSM303_OUT_Y_L_A         0x2AU  /*!< Dane Y — dolny bajt */
#define LSM303_OUT_Y_H_A         0x2BU  /*!< Dane Y — górny bajt */
#define LSM303_OUT_Z_L_A         0x2CU  /*!< Dane Z — dolny bajt */
#define LSM303_OUT_Z_H_A         0x2DU  /*!< Dane Z — górny bajt */

/*
 * Bit 7 adresu rejestru przy odczycie wielu bajtów przez I2C.
 * LSM303 wymaga ustawienia MSB=1 żeby auto-inkrementował adres.
 * Inaczej czytasz ciągle ten sam rejestr.
 */
#define LSM303_AUTO_INC          0x80U

/* ================================================ */
/*              KONFIGURACJA — ZAKRESY              */
/* ================================================ */
/*
 * FS[1:0] w CTRL_REG4_A bits[5:4]:
 * 00 = ±2g,  czułość 1mg/digit    = 0.001 g/digit
 * 01 = ±4g,  czułość 2mg/digit    = 0.002 g/digit
 * 10 = ±8g,  czułość 4mg/digit    = 0.004 g/digit
 * 11 = ±16g, czułość 12mg/digit   = 0.012 g/digit
 */
#define LSM303_SCALE_2G          0x00U
#define LSM303_SCALE_4G          0x10U
#define LSM303_SCALE_8G          0x20U
#define LSM303_SCALE_16G         0x30U

/* ================================================ */
/*              STRUKTURY                           */
/* ================================================ */

/**
 * @brief Surowe dane z akcelerometru (przed przeliczeniem).
 *        16-bitowe ze znakiem, left-justified (dane w bitach [15:4]).
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} LSM303_RawData_t;

/**
 * @brief Dane przeliczone na g (przyspieszenie ziemskie).
 *        W spoczynku na poziomej powierzchni: Z≈1.0g, X≈0, Y≈0
 */
typedef struct {
    float x;  /*!< Przyspieszenie osi X [g] */
    float y;  /*!< Przyspieszenie osi Y [g] */
    float z;  /*!< Przyspieszenie osi Z [g] */
} LSM303_Data_t;

/**
 * @brief Handle czujnika.
 */
typedef struct {
    I2C_Handle_t  *hi2c;         /*!< Wskaźnik na handle I2C */
    float          Sensitivity;  /*!< Czułość [g/digit] */
    uint8_t        Scale;        /*!< Aktualny zakres */
} LSM303_Handle_t;

/* ================================================ */
/*                  PUBLICZNE API                   */
/* ================================================ */

/**
 * @brief  Inicjalizuje akcelerometr.
 * @param  haccel  handle czujnika
 * @param  hi2c    handle I2C (już zainicjalizowany)
 * @param  scale   zakres (LSM303_SCALE_2G itp.)
 * @return true jeśli OK
 */
bool LSM303_Init(LSM303_Handle_t *haccel,
                 I2C_Handle_t    *hi2c,
                 uint8_t          scale);

/**
 * @brief  Odczytuje surowe dane z akcelerometru.
 */
bool LSM303_ReadRaw(LSM303_Handle_t *haccel, LSM303_RawData_t *raw);

/**
 * @brief  Odczytuje dane przeliczone na g.
 */
bool LSM303_ReadG(LSM303_Handle_t *haccel, LSM303_Data_t *data);