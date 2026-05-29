#pragma once
/**
 * @file    attitude.h
 * @brief   Estymacja kąta przechyłu — filtr komplementarny
 *
 * Łączy żyroskop (szybki, bez dryfu krótkoterminowego) z akcelerometrem
 * (wolny, ale nie dryfuje długoterminowo).
 *
 * Wzór:
 *   angle = alpha * (angle + gyro_dps * dt) + (1 - alpha) * accel_angle
 *
 * alpha = 0.98 oznacza:
 *   98% zaufania do żyroskopu (dobry na dynamiczne ruchy)
 *    2% korekcji z akcelerometru (zapobiega dryfowi)
 *
 * Układ osi na STM32F411E-DISCO (płytka leży USB w górę):
 *   Roll  (przechył lewo/prawo) — oś X akcelerometru, oś X żyroskopu
 *   Pitch (przechył przód/tył)  — oś Y akcelerometru, oś Y żyroskopu
 *
 * Jednostki:
 *   kąty w stopniach [°]
 *   dt    w sekundach [s]
 *   gyro  w stopniach na sekundę [°/s]
 *   accel w jednostkach g [g]
 */

#include <stdint.h>
#include <stdbool.h>

/* ================================================ */
/*              KONFIGURACJA                        */
/* ================================================ */

/**
 * Współczynnik filtra komplementarnego.
 * 0.98 = standardowa wartość dla dronów przy 100Hz.
 * Większy → mniej dryfu żyroskopu korygowane, wolniejsza odpowiedź na stały przechył.
 * Mniejszy → szybsza korekcja, ale więcej szumów z akcelerometru.
 */
#define ATTITUDE_ALPHA          0.98f

/**
 * Okres próbkowania w sekundach.
 * Musi zgadzać się z vTaskDelay w tasku (100ms = 0.1s).
 * UWAGA: przy zmianie częstotliwości taska zmień też tę wartość.
 */
#define ATTITUDE_DT             0.1f

/**
 * Maksymalny kąt przy którym LED świeci z pełną jasnością.
 * Powyżej tej wartości CCR = ARR (100% jasności).
 * Poniżej — jasność liniowo proporcjonalna do kąta.
 */
#define ATTITUDE_MAX_ANGLE_DEG  45.0f

/* ================================================ */
/*              STRUKTURY                           */
/* ================================================ */

/**
 * @brief Szacowane kąty przechyłu platformy.
 *        Roll  = przechył lewo/prawo  (obrót wokół osi X)
 *        Pitch = przechył przód/tył   (obrót wokół osi Y)
 */
typedef struct {
    float roll;   /*!< Przechył lewo/prawo [°], + = prawo,  - = lewo  */
    float pitch;  /*!< Przechył przód/tył  [°], + = przód, - = tył   */
} Attitude_t;

/**
 * @brief Stan filtra komplementarnego.
 *        Jedna instancja na całą aplikację.
 */
typedef struct {
    float roll;   /*!< Aktualny szacowany kąt roll  [°] */
    float pitch;  /*!< Aktualny szacowany kąt pitch [°] */
    bool  init;   /*!< Czy filtr był już zasilony pierwszą próbką */
} AttitudeFilter_t;

/* ================================================ */
/*              PUBLICZNE API                       */
/* ================================================ */

/**
 * @brief  Inicjalizuje filtr — zeruje stan.
 * @param  f  wskaźnik na filtr
 */
void Attitude_Init(AttitudeFilter_t *f);

/**
 * @brief  Aktualizuje filtr nową próbką i zwraca szacowane kąty.
 *
 * Wywołuj co ATTITUDE_DT sekund (co tick taska).
 *
 * @param  f          wskaźnik na filtr
 * @param  ax         przyspieszenie X [g]
 * @param  ay         przyspieszenie Y [g]
 * @param  az         przyspieszenie Z [g]
 * @param  gyro_x     prędkość kątowa X [°/s] — po odjęciu offsetu kalibracji
 * @param  gyro_y     prędkość kątowa Y [°/s] — po odjęciu offsetu kalibracji
 * @param  out        wskaźnik na wynikowe kąty
 */
void Attitude_Update(AttitudeFilter_t *f,
                     float ax, float ay, float az,
                     float gyro_x, float gyro_y,
                     Attitude_t *out);