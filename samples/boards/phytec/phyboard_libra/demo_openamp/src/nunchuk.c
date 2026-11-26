#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_nunchuk.h>

#include "nunchuk.h"

static struct nunchuk_data_t nunchuk_state = {130,130,512,512,512,false,false};

static void nunchuk_input_cb(struct input_event *evt, void *user_data)
{
    switch (evt->type) {
    case INPUT_EV_ABS:
        if (evt->code == INPUT_ABS_X) {
            nunchuk_state.joystick_x = evt->value;
        } else if (evt->code == INPUT_ABS_Y) {
            nunchuk_state.joystick_y = evt->value;
        }
        break;

    case INPUT_EV_KEY:
        if (evt->code == INPUT_KEY_Z_NUNCHUK) {
            nunchuk_state.btn_z = evt->value;   
        } else if (evt->code == INPUT_KEY_C_NUNCHUK) {
            nunchuk_state.btn_c = evt->value;
        }
        break;

    case INPUT_EV_DEVICE:
        if (evt->code == INPUT_NUNCHUK_ACCEL_X) {
            nunchuk_state.accel_x = evt->value;
        } else if (evt->code == INPUT_NUNCHUK_ACCEL_Y) {
            nunchuk_state.accel_y = evt->value;
        } else if (evt->code == INPUT_NUNCHUK_ACCEL_Z) {
            nunchuk_state.accel_z = evt->value;
        }
        break;

    default:
        break;
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(nunchuk0)), nunchuk_input_cb, NULL);

struct nunchuk_data_t read_nunchuk(void)
{
    return nunchuk_state;
}