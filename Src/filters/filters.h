#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../l3gd20/l3gd20.h"

/* ================================================ */
/*          FILTR DOLNOPRZEPUSTOWY (LPF)            */
/* ================================================ */

/**
 * @brief Stan filtra EMA (Exponential Moving Average).
 *        Jeden filtr na jedną oś — tworzysz 3 instancje dla X/Y/Z.
 *
 * Wzór: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 *
 * alpha: 0.0 .. 1.0
 *   → blisko 0 = silne wygładzanie, duże opóźnienie
 *   → blisko 1 = słabe wygładzanie, małe opóźnienie
 */
typedef struct {
    float alpha;    /*!< Współczynnik filtra 0..1 */
    float value;    /*!< Aktualny wynik (stan filtra) */
    bool  init;     /*!< Czy filtr był już zasilony pierwszą próbką */
} LPF_t;

/* ================================================ */
/*          KALIBRACJA OFFSETU ŻYROSKOPU            */
/* ================================================ */

/**
 * @brief Wyniki kalibracji — offsety do odjęcia od każdego pomiaru.
 *        Mierzone raz przy starcie gdy płytka stoi nieruchomo.
 */
typedef struct {
    float x;    /*!< Offset osi X [°/s] */
    float y;    /*!< Offset osi Y [°/s] */
    float z;    /*!< Offset osi Z [°/s] */
} Gyro_Offset_t;

/* ================================================ */
/*              PUBLICZNE API                       */
/* ================================================ */

/**
 * @brief  Inicjalizuje filtr — zeruje stan, ustawia alpha.
 * @param  f      wskaźnik na filtr
 * @param  alpha  współczynnik 0..1
 */
void LPF_Init(LPF_t *f, float alpha);

/**
 * @brief  Podaje nową próbkę do filtra, zwraca przefiltrowaną wartość.
 * @param  f    wskaźnik na filtr
 * @param  val  nowa próbka
 * @return przefiltrowana wartość
 */
float LPF_Update(LPF_t *f, float val);

/**
 * @brief  Resetuje filtr do stanu początkowego.
 *         Użyj gdy chcesz zacząć filtrowanie od nowa.
 * @param  f  wskaźnik na filtr
 */
void LPF_Reset(LPF_t *f);

/**
 * @brief  Kalibruje offset żyroskopu.
 *         Zbiera n_samples próbek przy nieruchomej płytce
 *         i oblicza średnią — to jest offset do odjęcia.
 *         Płytka MUSI stać nieruchomo podczas kalibracji!
 *
 * @param  hgyro      handle czujnika
 * @param  offset     wskaźnik na strukturę wynikową
 * @param  n_samples  liczba próbek (500 = ~0.5s)
 */
void Gyro_Calibrate(L3GD20_Handle_t *hgyro,
                    Gyro_Offset_t   *offset,
                    uint16_t         n_samples);

/**
 * @brief  Aplikuje offset do odczytu — odejmuje kalibrację.
 *         Wywołuj po każdym L3GD20_ReadDPS.
 *
 * @param  data    wskaźnik na odczytane dane
 * @param  offset  wskaźnik na wyniki kalibracji
 */
void Gyro_ApplyOffset(L3GD20_Data_t *data, const Gyro_Offset_t *offset);