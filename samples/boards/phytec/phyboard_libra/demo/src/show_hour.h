#include <zephyr/kernel.h>
#include <lvgl.h>
#include "counter_hour.h"

#define BTN_COLOR 0x0000FF 

void init_info_show_hour(lv_obj_t *screen,lv_obj_t *page_init,bool *initialized);
void update_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label);
void set_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label);
void reset_show_hour(void);