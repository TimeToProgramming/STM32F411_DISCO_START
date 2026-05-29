#include "attitude.h"

#define RAD_TO_DEG  57.2957795131f
#define EPS         1e-6f

void Attitude_Init(AttitudeFilter_t *f) {
    f->roll  = 0.0f;
    f->pitch = 0.0f;
    f->init  = false;
}

/**
 * @brief  Przybliżenie atan2 w stopniach — nie wymaga libm.
 *         Błąd < 0.5° w zakresie [-180°, 180°].
 *         Używa tylko operacji FPU Cortex-M4.
 */
static float atan2_deg(float y, float x) {
    float ax = x < 0.0f ? -x : x;
    float ay = y < 0.0f ? -y : y;
    float a  = (ax < ay) ? ax / ay : ay / ax;
    float s  = a * a;

    float r = ((-0.0464964749f * s + 0.15931422f) * s
               - 0.32762241f) * s * a + a;

    if (ay > ax) r = 1.5707963f - r;
    if (x  < 0)  r = 3.1415927f - r;
    if (y  < 0)  r = -r;

    return r * RAD_TO_DEG;
}

void Attitude_Update(AttitudeFilter_t *f,
                     float ax, float ay, float az,
                     float gyro_x, float gyro_y,
                     Attitude_t *out) {
    /*
     * Kąt z akcelerometru — poprawione osie wg danych z Discovery:
     *
     * Pomiar: gdy użytkownik podnosi USB (pitch), zmienia się AX.
     *         Gdy przechyla lewo/prawo (roll), zmienia się AY.
     *
     * Dlatego:
     *   pitch_accel używa AX  (nie -AY jak było wcześniej)
     *   roll_accel  używa AY  (nie AX jak było wcześniej)
     *
     * Znaki dobrane tak żeby:
     *   USB w górze → AX ujemne → pitch_accel ujemne (spójne z GY<0)
     *   lewo w górze → AY ujemne → roll_accel ujemne (spójne z GX<0)
     */
    float pitch_accel = atan2_deg(ax,   az);   /* AX dla pitch */
    float roll_accel  = atan2_deg(ay,   az);   /* AY dla roll  */

    /* Pierwsza próbka — inicjuj od razu poprawnym kątem */
    if (!f->init) {
        f->roll  = roll_accel;
        f->pitch = pitch_accel;
        f->init  = true;
    }

    /*
     * Filtr komplementarny:
     *   angle = ALPHA * (angle + gyro * dt) + (1-ALPHA) * angle_accel
     *
     * ALPHA = 0.98:
     *   98% żyroskop — szybki, precyzyjny krótkoterminowo
     *    2% akcelerometr — korekcja dryfu długookresowego
     *
     * Efektywna stała czasowa akcelerometru:
     *   τ = ALPHA/(1-ALPHA) * dt = 0.98/0.02 * 0.1s = 4.9s
     * Oznacza: małe drgania i szumy są tłumione przez ~5 sekund uśredniania.
     */
    f->roll  = ATTITUDE_ALPHA * (f->roll  + gyro_x * ATTITUDE_DT)
             + (1.0f - ATTITUDE_ALPHA) * roll_accel;

    f->pitch = ATTITUDE_ALPHA * (f->pitch + gyro_y * ATTITUDE_DT)
             + (1.0f - ATTITUDE_ALPHA) * pitch_accel;

    out->roll  = f->roll;
    out->pitch = f->pitch;
}