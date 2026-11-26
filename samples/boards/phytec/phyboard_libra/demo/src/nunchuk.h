#ifndef NUNCHUK_H
#define NUNCHUK_H

#define INPUT_KEY_Z_NUNCHUK 309
#define INPUT_KEY_C_NUNCHUK 306

#include <stdint.h>
#include <stdbool.h>

struct nunchuk_data_t {
    uint8_t joystick_x;
    uint8_t joystick_y;
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    bool btn_c;
    bool btn_z;
};

struct nunchuk_data_t read_nunchuk(void);

#endif /* NUNCHUK_H */