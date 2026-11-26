#include "show_hour.h"

static lv_obj_t *page_init_show_hour;

static lv_obj_t * roller_day ;
static lv_obj_t * roller_month ;
static lv_obj_t * roller_year;
static lv_obj_t * roller_hour;
static lv_obj_t * roller_minute;

static struct counter_clock_t clock_data; 

static bool check_init=false;

static void set_day_roller_options(lv_obj_t *roller, int nb_days);
static void event_cb(lv_event_t * e);

static void event_cb(lv_event_t *e){
    
    int month = lv_roller_get_selected(roller_month) + 1;
    int year = lv_roller_get_selected(roller_year) + 1950;
    int old_day = lv_roller_get_selected(roller_day) + 1;
    int nb_days = days_in_month(month,year);

    if(old_day > nb_days){
        old_day = nb_days;
    }
    set_day_roller_options(roller_day,nb_days);
    lv_roller_set_selected(roller_day,old_day - 1,false);
}

static void set_day_roller_options(lv_obj_t *roller, int nb_days){
    char buf[128];
    size_t offset = 0;

    for (int i = 1; i <= nb_days; i++) {
        offset += snprintf(buf + offset, sizeof(buf) - offset,
                            "%d%s", i, (i < nb_days) ? "\n" : "");
    }

    lv_roller_set_options(roller, buf, LV_ROLLER_MODE_NORMAL);
}

static void init_date_hour_cb(lv_event_t *e){

    bool *initialized = lv_event_get_user_data(e);

    clock_data.day=lv_roller_get_selected(roller_day)+1;
    clock_data.month=lv_roller_get_selected(roller_month);
    clock_data.year=lv_roller_get_selected(roller_year)+1950;
    clock_data.hour=lv_roller_get_selected(roller_hour);
    clock_data.minutes=lv_roller_get_selected(roller_minute);
    setClock(clock_data);
    *initialized=true;

    lv_obj_set_hidden(page_init_show_hour, true);
}

void init_info_show_hour(lv_obj_t *screen,lv_obj_t *current_page,bool *initialized){

    if(!check_init){
        lv_obj_t * scr = screen;

        page_init_show_hour = lv_obj_create(scr);
        lv_obj_set_size(page_init_show_hour, lv_pct(100), lv_pct(100));
        lv_obj_center(page_init_show_hour);
        lv_obj_set_style_bg_opa(page_init_show_hour, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_scrollable(page_init_show_hour, false);

        roller_day = lv_roller_create(page_init_show_hour);

        char buf_day[128];
        size_t offset_day = 0;
        for (int i = 1; i <= 31; i++) {
            offset_day += snprintf(buf_day + offset_day, sizeof(buf_day) - offset_day,
                                "%d%s", i, (i < 31) ? "\n" : "");
        }
        lv_roller_set_options(roller_day, buf_day, LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(roller_day, 3);
        lv_obj_align(roller_day,LV_ALIGN_CENTER,-80,-50);

        roller_month = lv_roller_create(page_init_show_hour);
        lv_obj_set_width(roller_month, 100);
        lv_roller_set_options(roller_month, "JANUARY\nFEBRUARY\nMARCH\nAPRIL\nMAY\nJUNE\nJULY\nAUGUST\nSEPTEMBER\nOCTOBER\nNOVEMBER\nDECEMBER", LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(roller_month, 3);
        lv_obj_add_event_cb(roller_month, event_cb, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_align(roller_month,LV_ALIGN_CENTER,-5,-50);


        roller_year = lv_roller_create(page_init_show_hour);
        char buf_year[600];
        size_t offset_year = 0;
        for (int i = 1950; i <= 2050; i++) {
            offset_year += snprintf(buf_year + offset_year, sizeof(buf_year) - offset_year,
                                "%d%s", i, (i < 2050) ? "\n" : "");
        }
        lv_roller_set_options(roller_year, buf_year, LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(roller_year, 3);
        lv_obj_add_event_cb(roller_year, event_cb, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_align(roller_year,LV_ALIGN_CENTER,80,-50);
        lv_roller_set_selected(roller_year, 75, false);


        roller_hour = lv_roller_create(page_init_show_hour);
        char buf_hour[72];
        size_t offset_hour = 0;
        for (int i = 0; i <= 23; i++) {
            offset_hour += snprintf(buf_hour + offset_hour, sizeof(buf_hour) - offset_hour,
                                "%d%s", i, (i < 23) ? "\n" : "");
        }
        lv_roller_set_options(roller_hour, buf_hour, LV_ROLLER_MODE_INFINITE);
        lv_roller_set_visible_row_count(roller_hour, 3);
        lv_obj_align(roller_hour,LV_ALIGN_CENTER,-30,50);

        roller_minute = lv_roller_create(page_init_show_hour);
        char buf_minute[240];
        size_t offset_minute = 0;
        for (int i = 0; i <= 59; i++) {
            offset_minute += snprintf(buf_minute + offset_minute, sizeof(buf_minute) - offset_minute,
                                "%d%s", i, (i < 59) ? "\n" : "");
        }
        lv_roller_set_options(roller_minute, buf_minute, LV_ROLLER_MODE_INFINITE);
        lv_roller_set_visible_row_count(roller_minute, 3);
        lv_obj_align(roller_minute,LV_ALIGN_CENTER,30,50);


        lv_obj_t *label_sep = lv_label_create(page_init_show_hour);
        lv_obj_t *btn_init = lv_btn_create(page_init_show_hour);

        lv_obj_set_style_text_font(label_sep,&lv_font_montserrat_18,LV_PART_MAIN);
        lv_label_set_text(label_sep,":");
        lv_obj_align(label_sep,LV_ALIGN_CENTER,0,50);

        lv_obj_t *label_btn = lv_label_create(btn_init);
        lv_label_set_text(label_btn,"Enter");
        lv_obj_center(label_btn);

        lv_obj_align(btn_init,LV_ALIGN_CENTER,0,120);
        lv_obj_set_size(btn_init,150,30);
        lv_obj_set_style_bg_color(btn_init,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
        lv_obj_add_event_cb(btn_init,init_date_hour_cb,LV_EVENT_CLICKED,initialized);


        lv_obj_t *label_title = lv_label_create(page_init_show_hour);
        lv_obj_set_style_text_font(label_title,&lv_font_montserrat_20,LV_PART_MAIN);
        lv_label_set_text(label_title,"SELECT HOUR DAY");
        lv_obj_align(label_title,LV_ALIGN_CENTER,0,-125);

        check_init = true;
    }
    lv_obj_set_hidden(page_init_show_hour, false);
    lv_obj_set_scrollable(page_init_show_hour, false);
    lv_obj_set_hidden(current_page, true);
}

void update_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label){
    clock_data = getClock();
    set_show_hour(hour_label,date_label);
}

void set_show_hour(lv_obj_t *hour_label,lv_obj_t *date_label){
    static const char *month_names[] = {
        "JANUARY",
        "FEBRUARY",
        "MARCH",
        "APRIL",
        "MAY",
        "JUNE",
        "JULY",
        "AUGUST",
        "SEPTEMBER",
        "OCTOBER",
        "NOVEMBER",
        "DECEMBER"
    };

    lv_label_set_text_fmt(hour_label,"%02d:%02d:%02d",clock_data.hour,clock_data.minutes,clock_data.seconds);
    lv_label_set_text_fmt(date_label,"%d %s %d",clock_data.day,month_names[clock_data.month],clock_data.year);
}

void reset_show_hour(void){
    check_init = false;
    page_init_show_hour = NULL;
    roller_day = NULL;
    roller_month = NULL;
    roller_year = NULL;
    roller_hour = NULL;
    roller_minute = NULL;
}