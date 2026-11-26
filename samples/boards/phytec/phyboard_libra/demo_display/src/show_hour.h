#include <zephyr/kernel.h>
#include <lvgl.h>

#define BTN_COLOR 0x0000FF 

enum mounth_t {
    JANUARY,
    FEBRUARY,
    MARCH,
    APRIL,
    MAY,
    JUNE,
    JULY,
    AUGUST,
    SEPTEMBER,
    OCTOBER,
    NOVEMBER,
    DECEMBER
};

struct state_show_hour_t {
    int day;
    enum mounth_t mounth;
    int year;
    int hour;
    int minute;
};

void init_info_show_hour(lv_obj_t *screen,lv_obj_t *page_init,bool *initialized);
void update_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label);
void set_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label);
void reset_show_hour(void);