/**
 * @file    l3gd20.c
 * @brief   Implementacja sterownika żyroskopu L3GD20
 *
 * Komunikacja SPI wg datasheet sekcja 5.2 (Figure 12):
 *   - Mode 0 (CPOL=0, CPHA=0)
 *   - driven on falling edge, captured on rising edge
 *
 * Format bajtu adresu:
 *   bit7 = RW  (1=read,  0=write)
 *   bit6 = MS  (1=auto-increment)
 *   bit5-0 = adres rejestru AD(5:0)
 *
 * CS obsługiwany ręcznie:
 *   cs_low() → SPI_Transfer() → cs_high()
 */

#include "l3gd20.h"
#include "../gpio/gpio.h"

/* ================================================ */
/*          POMOCNICZE — CS i zapis/odczyt          */
/* ================================================ */

/**
 * @brief CS low — aktywuj czujnik, rozpocznij transfer.
 *        Tylko jeden czujnik na raz może mieć CS=LOW.
 */
static inline void cs_low(L3GD20_Handle_t *hgyro) {
    GPIO_WritePin(hgyro->CsPort, hgyro->CsPin, GPIO_PIN_RESET);
}

/**
 * @brief CS high — zakończ transfer, dezaktywuj czujnik.
 */
static inline void cs_high(L3GD20_Handle_t *hgyro) {
    GPIO_WritePin(hgyro->CsPort, hgyro->CsPin, GPIO_PIN_SET);
}

/**
 * @brief  Zapisuje jeden bajt do rejestru L3GD20.
 *
 * Ramka TX: [0|0|AD5..AD0] [DATA]
 *   bit7=0 → zapis, bit6=0 → single write
 *
 * @param  hgyro  handle czujnika
 * @param  reg    adres rejestru
 * @param  data   wartość do zapisania
 */
static void write_reg(L3GD20_Handle_t *hgyro, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg & ~L3GD20_READ_BIT, data };
    uint8_t rx[2] = { 0, 0 };

    cs_low(hgyro);
    SPI_Transfer(hgyro->hspi, tx, rx, 2, 10);
    cs_high(hgyro);
}

/**
 * @brief  Odczytuje jeden bajt z rejestru L3GD20.
 *
 * Ramka TX: [1|0|AD5..AD0] [0x00 dummy]
 *   bit7=1 → odczyt, bit6=0 → single read
 * Odpowiedź czujnika ląduje w rx[1].
 *
 * @param  hgyro  handle czujnika
 * @param  reg    adres rejestru
 * @return odczytana wartość
 */
static uint8_t read_reg(L3GD20_Handle_t *hgyro, uint8_t reg) {
    uint8_t tx[2] = { reg | L3GD20_READ_BIT, 0x00 };
    uint8_t rx[2] = { 0, 0 };

    cs_low(hgyro);
    SPI_Transfer(hgyro->hspi, tx, rx, 2, 10);
    cs_high(hgyro);

    return rx[1];
}

/**
 * @brief  Odczytuje N bajtów z kolejnych rejestrów (auto-increment).
 *
 * Ramka TX: [1|1|AD5..AD0] [0x00]×N
 *   bit7=1 → odczyt, bit6=1 → auto-increment adresu
 * Dane lądują w rx[1..N], rx[0] ignorowany.
 *
 * @param  hgyro  handle czujnika
 * @param  reg    adres pierwszego rejestru
 * @param  buf    bufor wynikowy
 * @param  len    liczba bajtów do odczytania
 */
static void read_regs(L3GD20_Handle_t *hgyro,
                      uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tx[7] = { 0 };
    uint8_t rx[7] = { 0 };

    /* Adres z READ i AUTO_INC — reszta TX to dummy 0x00 */
    tx[0] = reg | L3GD20_READ_BIT | L3GD20_AUTO_INC_BIT;

    cs_low(hgyro);
    SPI_Transfer(hgyro->hspi, tx, rx, len + 1, 10);
    cs_high(hgyro);

    /* rx[0] = odpowiedź podczas wysyłania adresu — ignoruj */
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = rx[i + 1];
    }
}

/* ================================================ */
/*                  INICJALIZACJA                   */
/* ================================================ */

bool L3GD20_Init(L3GD20_Handle_t *hgyro,
                 SPI_Handle_t    *hspi,
                 GPIO_TypeDef    *cs_port,
                 uint32_t         cs_pin,
                 uint8_t          scale) {

    hgyro->hspi   = hspi;
    hgyro->CsPort = cs_port;
    hgyro->CsPin  = cs_pin;
    hgyro->Scale  = scale;

    /*
     * Czułość wg datasheet Table 4:
     *   ±250dps  → 8.75  mdps/digit
     *   ±500dps  → 17.5  mdps/digit
     *   ±2000dps → 70.0  mdps/digit
     */
    switch (scale) {
        case L3GD20_SCALE_250DPS:  hgyro->Sensitivity = 0.00875f; break;
        case L3GD20_SCALE_500DPS:  hgyro->Sensitivity = 0.01750f; break;
        case L3GD20_SCALE_2000DPS: hgyro->Sensitivity = 0.07000f; break;
        default:                   hgyro->Sensitivity = 0.00875f; break;
    }

    /*
     * CS jako output push-pull, domyślnie HIGH.
     * Chwilowe LOW wymusza tryb SPI
     * (CS=0 → SPI mode, CS=1 → I2C mode wg datasheet Table 2).
     */
    GPIO_EnableClock(cs_port);
    GPIO_ConfigPin(cs_port, cs_pin,
                   GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_OTYPE_PP);
    GPIO_SetSpeed(cs_port, cs_pin, GPIO_SPEED_HIGH);

    GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(10));

    /*
     * Weryfikacja WHO_AM_I.
     * Poprawne: 0xD4 (standard), 0xD7 (L3GD20H), 0xD3 (starsza wersja).
     */
    uint8_t who = read_reg(hgyro, L3GD20_REG_WHO_AM_I);
    if (who != L3GD20_WHO_AM_I_VAL  &&
        who != L3GD20H_WHO_AM_I_VAL &&
        who != L3GD20T_WHO_AM_I_VAL) {
        return false;
    }

    /*
     * CTRL_REG1: Normal mode, ODR 95Hz, wszystkie osie aktywne.
     * PD=1 → wyjście z power-down (domyślnie PD=0 po resecie!)
     * Zen=Yen=Xen=1 → X, Y, Z włączone
     */
    write_reg(hgyro, L3GD20_REG_CTRL_REG1,
              L3GD20_CTRL1_PD  |
              L3GD20_CTRL1_ZEN |
              L3GD20_CTRL1_YEN |
              L3GD20_CTRL1_XEN);

    /*
     * CTRL_REG4: zakres pomiarowy.
     * FS[1:0] w bitach [5:4]:
     *   0x00 = ±250dps, 0x10 = ±500dps, 0x20 = ±2000dps
     * BDU=0 → continuous update
     * SIM=0 → 4-wire SPI
     */
    write_reg(hgyro, L3GD20_REG_CTRL_REG4, scale);

    return true;
}

/* ================================================ */
/*                  ODCZYT DANYCH                   */
/* ================================================ */

bool L3GD20_ReadRaw(L3GD20_Handle_t *hgyro, L3GD20_RawData_t *raw) {
    /*
     * Odczyt 6 bajtów od OUT_X_L (0x28) przez auto-increment:
     *   buf[0]=OUT_X_L, buf[1]=OUT_X_H → raw.x = (H<<8)|L
     *   buf[2]=OUT_Y_L, buf[3]=OUT_Y_H → raw.y = (H<<8)|L
     *   buf[4]=OUT_Z_L, buf[5]=OUT_Z_H → raw.z = (H<<8)|L
     *
     * Little-endian (BLE=0 w CTRL_REG4).
     * int16_t — two's complement, zakres ±32768.
     */
    uint8_t buf[6];
    read_regs(hgyro, L3GD20_REG_OUT_X_L, buf, 6);

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

bool L3GD20_ReadDPS(L3GD20_Handle_t *hgyro, L3GD20_Data_t *data) {
    /*
     * Przeliczenie: wartość[°/s] = surowa × czułość
     *
     * Przykład ±250dps:
     *   surowa=1000 → 1000×0.00875 = 8.75°/s
     *   surowa=-500 → -500×0.00875 = -4.375°/s
     *
     * W spoczynku X i Y ~0°/s, Z ma offset ~4-5°/s
     * (zero-rate level czujnika — kalibrujemy przed lotem).
     */
    L3GD20_RawData_t raw;
    if (!L3GD20_ReadRaw(hgyro, &raw)) return false;

    data->x = (float)raw.x * hgyro->Sensitivity;
    data->y = (float)raw.y * hgyro->Sensitivity;
    data->z = (float)raw.z * hgyro->Sensitivity;

    return true;
}