#include "filters.h"

void LPF_Init(LPF_t *f, float alpha) {
    f->alpha = alpha;
    f->value = 0.0f;
    f->init  = false;
}

float LPF_Update(LPF_t *f, float val) {
    if (!f->init) {
        f->value = val;
        f->init  = true;
    } else {
        f->value = f->alpha * val + (1.0f - f->alpha) * f->value;
    }
    return f->value;
}

void LPF_Reset(LPF_t *f) {
    f->value = 0.0f;
    f->init  = false;
}


void Gyro_Calibrate(L3GD20_Handle_t *hgyro,
                    Gyro_Offset_t   *offset,
                    uint16_t         n_samples) {
    float suma_x = 0.0f;  // poprawka 1
    float suma_y = 0.0f;
    float suma_z = 0.0f;

    L3GD20_Data_t data;   // poprawka 2 — tu lądują odczyty

    for (int n = 0; n < n_samples; n++) {  // poprawka 3
        L3GD20_ReadDPS(hgyro, &data);      // poprawka 2
        suma_x += data.x;
        suma_y += data.y;
        suma_z += data.z;
        vTaskDelay(pdMS_TO_TICKS(1));      // poprawka 4
    }

    offset->x = suma_x / n_samples;
    offset->y = suma_y / n_samples;
    offset->z = suma_z / n_samples;
}

void Gyro_ApplyOffset(L3GD20_Data_t *data, const Gyro_Offset_t *offset) {
    data->x -= offset->x;
    data->y -= offset->y;
    data->z -= offset->z;
}