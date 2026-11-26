#include <zephyr/kernel.h>
#include "nunchuk.h"
#include "counter_hour.h"

#define ID_WAKE 2

int cubes_screen(struct nunchuk_data_t actual_nunchuk);
int init_screen_cubes(void);