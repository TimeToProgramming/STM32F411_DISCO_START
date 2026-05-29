/**
 * @file    lsm303.c
 * @brief   Implementacja sterownika akcelerometru LSM303DLHC
 *
 * Komunikacja przez I2C — brak CS (I2C adresuje urządzenia przez adres 7-bitowy).
 * Odczyt wielu rejestrów przez auto-increment (bit7=1 w adresie rejestru).
 *
 * W spoczynku na poziomej powierzchni:
 *   X ≈ 0g, Y ≈ 0g, Z ≈ 1g (grawitacja)
 *
 * Zmiany względem v1:
 *   [FIX] LSM303_ReadRaw: poprawiony shift przy składaniu int16_t —
 *         rzutowanie na uint16_t przed >> 4 zapobiega implementation-defined
 *         behaviour przy ujemnych wartościach (np. oś Z po obróceniu płytki).
 *   [FIX] LSM303_ReadRaw: __DSB() po I2C_Read — bariera synchronizacji danych
 *         po transferze DMA; zapewnia że CPU czyta aktualne wartości z bufora.
 *   [FIX] LSM303_Init/ReadRaw/ReadG: null-check wskaźników na wejściu.
 *   [FIX] LSM303_Init: timeout 50ms zamiast 10ms — init może trafić na bus
 *         recovery (9 cykli SCL + reset), który trwa dłużej niż 10ms.
 */

#include "lsm303.h"

/* ================================================ */
/*                  INICJALIZACJA                   */
/* ================================================ */

bool LSM303_Init(LSM303_Handle_t *haccel,
                 I2C_Handle_t    *hi2c,
                 uint8_t          scale) {

    /* [FIX] Null-check — przy debugowaniu nowego kodu częsty błąd */
    if (!haccel || !hi2c) return false;

    /* Zapisz konfigurację */
    haccel->hi2c  = hi2c;
    haccel->Scale = scale;

    /*
     * Czułość zależy od zakresu.
     * LSM303 jest left-justified — dane w bitach [15:4].
     * Żeby dostać właściwą wartość dzielimy przez 16 (przesunięcie >> 4).
     * Czułości z datasheet (przy high-res mode, 12-bit):
     *   ±2g  → 1mg/digit   = 0.001g
     *   ±4g  → 2mg/digit   = 0.002g
     *   ±8g  → 4mg/digit   = 0.004g
     *   ±16g → 12mg/digit  = 0.012g  (wartość typowa, duży spread w DS)
     */
    switch (scale) {
        case LSM303_SCALE_2G:  haccel->Sensitivity = 0.001f;  break;
        case LSM303_SCALE_4G:  haccel->Sensitivity = 0.002f;  break;
        case LSM303_SCALE_8G:  haccel->Sensitivity = 0.004f;  break;
        case LSM303_SCALE_16G: haccel->Sensitivity = 0.012f;  break;
        default:               haccel->Sensitivity = 0.001f;  break;
    }

    /*
     * Konfiguracja CTRL_REG1_A (0x20):
     *
     * Wysyłamy przez I2C: [adres_rejestru][wartość]
     * buf[0] = adres rejestru = 0x20
     * buf[1] = wartość = 0x57
     *
     * 0x57 = 0101 0111:
     *   bits[7:4] = 0101 → ODR 100Hz (nowe dane co 10ms)
     *   bit[3]    = 0    → normal power mode
     *   bit[2]    = 1    → oś Z włączona
     *   bit[1]    = 1    → oś Y włączona
     *   bit[0]    = 1    → oś X włączona
     *
     * [FIX] timeout 50ms: init może trafić na Bus Recovery (9×SCL + reset I2C)
     *       który przy 100kHz zajmuje ~100us × 9 + narzut FreeRTOS > 10ms.
     */
    uint8_t ctrl1[2] = { LSM303_CTRL_REG1_A, 0x57U };
    if (!I2C_Write(haccel->hi2c, LSM303_ACCEL_ADDR,
                   ctrl1, 2, 50)) {
        return false;
    }

    /*
     * Konfiguracja CTRL_REG4_A (0x23):
     *
     * buf[0] = adres rejestru = 0x23
     * buf[1] = zakres | high-res
     *
     * Bit[3] = 1 → high-resolution mode (12-bit zamiast 10-bit)
     * Bits[5:4] = zakres z parametru scale
     *
     * Przykład dla ±2g + high-res:
     *   0x00 | 0x08 = 0x08 = 0000 1000
     */
    uint8_t ctrl4[2] = { LSM303_CTRL_REG4_A, (uint8_t)(scale | 0x08U) };
    if (!I2C_Write(haccel->hi2c, LSM303_ACCEL_ADDR,
                   ctrl4, 2, 50)) {
        return false;
    }

    return true;
}

/* ================================================ */
/*                  ODCZYT DANYCH                   */
/* ================================================ */

bool LSM303_ReadRaw(LSM303_Handle_t *haccel, LSM303_RawData_t *raw) {

    /* [FIX] Null-check */
    if (!haccel || !raw) return false;

    /*
     * Odczyt 6 bajtów od OUT_X_L_A (0x28).
     * Bit7=1 w adresie rejestru = auto-increment.
     * Bez tego I2C czytałby ciągle ten sam rejestr 0x28.
     *
     * Kolejność bajtów:
     *   buf[0] = OUT_X_L (0x28) — X dolny
     *   buf[1] = OUT_X_H (0x29) — X górny
     *   buf[2] = OUT_Y_L (0x2A) — Y dolny
     *   buf[3] = OUT_Y_H (0x2B) — Y górny
     *   buf[4] = OUT_Z_L (0x2C) — Z dolny
     *   buf[5] = OUT_Z_H (0x2D) — Z górny
     *
     * LSM303 jest left-justified — dane w bitach [15:4].
     * Przesuwamy >> 4 żeby dostać właściwą wartość 12-bitową.
     *
     * [FIX] aligned(4): bufor na stosie musi być wyrównany do 4 bajtów
     *       żeby DMA mógł pisać bez problemów z wyrównaniem na Cortex-M4.
     */
    uint8_t buf[6] __attribute__((aligned(4)));

    if (!I2C_Read(haccel->hi2c, LSM303_ACCEL_ADDR,
                  LSM303_OUT_X_L_A | LSM303_AUTO_INC,
                  buf, 6, 10)) {
        return false;
    }

    /*
     * [FIX] __DSB() — Data Synchronization Barrier po transferze DMA.
     *
     * DMA pisze do bufora bezpośrednio w pamięci (omijając pipeline CPU).
     * Na Cortex-M4 bez D-cache zwykle działa bez bariery, ale __DSB()
     * gwarantuje że wszystkie zapisy DMA są widoczne przed czytaniem buf[].
     * Koszt: ~1 cykl — warto mieć dla pewności.
     */
    __DSB();

    /*
     * Składamy 16-bitową wartość z dwóch bajtów (little-endian).
     *
     * [FIX] Poprawna kolejność rzutowań:
     *   BŁĄD v1:  (int16_t)((buf[1] << 8) | buf[0]) >> 4
     *             buf[1] jest uint8_t, (buf[1] << 8) to int — jeśli bit7=1,
     *             to po rzutowaniu na int16_t i przesunięciu >> 4 zachowanie
     *             jest implementation-defined (C99 §6.5.7).
     *
     *   FIX v2:  (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4
     *             Najpierw składamy jako uint16_t (bez rozszerzenia znaku),
     *             potem rzutujemy na int16_t (zachowuje bit znaku w bit15),
     *             potem >> 4 — teraz to arithmetic shift na int16_t, poprawny.
     *
     * Przykład: Z = -1g po obróceniu płytki
     *   buf[5]=0x00, buf[4]=0xF0  → raw bez fix = 0, z fix = -16 → -16×0.001 = -0.016g  ← ŹLE
     *   buf[5]=0xFF, buf[4]=0x00  → raw = 0xFF00 → int16 = -256 >> 4 = -16 → -0.016g    ← OK
     */
    raw->x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4;
    raw->y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]) >> 4;
    raw->z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]) >> 4;

    return true;
}

bool LSM303_ReadG(LSM303_Handle_t *haccel, LSM303_Data_t *data) {

    /* [FIX] Null-check */
    if (!haccel || !data) return false;

    /*
     * Przeliczenie surowych danych na g:
     *   wartość[g] = surowa × czułość
     *
     * Przykład przy ±2g:
     *   surowa = 1000 → 1000 × 0.001 = 1.0g
     *
     * W spoczynku na poziomej powierzchni:
     *   X ≈ 0g, Y ≈ 0g, Z ≈ 1g (oś Z mierzy grawitację)
     */
    LSM303_RawData_t raw;
    if (!LSM303_ReadRaw(haccel, &raw)) return false;

    data->x = (float)raw.x * haccel->Sensitivity;
    data->y = (float)raw.y * haccel->Sensitivity;
    data->z = (float)raw.z * haccel->Sensitivity;

    return true;
}