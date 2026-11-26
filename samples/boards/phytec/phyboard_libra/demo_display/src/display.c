#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/led.h>

#include "display.h"
#include "nunchuk_2.h"
#include "show_hour.h"

static void nunchuk_pointer_read(lv_indev_t *indev, lv_indev_data_t *data);
static void start_btn_cb(lv_event_t *e);
static void scroll_left_cb(lv_event_t *e);
static void scroll_right_cb(lv_event_t *e);
static void hue_canvas_cb(lv_event_t *e);
static void edit_btn_cb(lv_event_t *e);
static void reboot_btn_cb(lv_event_t *e);

static const struct device *display_dev;
static const struct device *rgb;
static struct display_capabilities caps;
static struct cursor_position_t cursor_pos = {0,0,false,INIT_CURSOR_COLOR};
static struct state_button_t state_btn = {0,0};
static struct state_button_t previous_state_btn = {0,0};
static enum state_t state = INIT;
static enum state_t previous_state = SHOW_HOUR;

static lv_obj_t *hue_canvas;
static lv_color_t canvas_buf[BAR_W * BAR_H];

static lv_obj_t *screen;
static lv_obj_t *cursor;
static lv_indev_t *nunchuk_indev;
static lv_obj_t *left;
static lv_obj_t *right;

static lv_obj_t *page_init;
static lv_obj_t *page_hour;
static lv_obj_t *label_hour1;
static lv_obj_t *label_hour2;
static lv_obj_t *page_rgb;
static lv_obj_t *page_ctrl_a35;
static lv_obj_t *page_reboot;
static lv_obj_t *current_page;

int init_display(){
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        LOG_ERR("Device %s not found. Aborting sample.", display_dev->name);
        return 1;
    }

    display_get_capabilities(display_dev, &caps);
    init_i2c();
    return 0;
}

int init_i2c(){
    rgb = DEVICE_DT_GET(DT_ALIAS(rgb_led0));

    if (!device_is_ready(rgb)) {
        printk("PCA9633 not ready\n");
        return 1;
    }

    return 0;
}

int init_screen(){
    int check_display = init_display();

    if(check_display){
        return 1;
    }

    screen = lv_scr_act();
    lv_obj_clean(screen);
    reset_show_hour();

/// ----------- init cursor and nunchuk ------------
  
    static bool input_created = false;
    if(!input_created){
        cursor = lv_obj_create(screen);
        lv_obj_set_size(cursor,CURSOR_SIZE,CURSOR_SIZE);
        lv_obj_set_style_bg_color(cursor,lv_color_hex(cursor_pos.color),LV_PART_MAIN);
        lv_obj_set_style_translate_x(cursor,-CURSOR_SIZE/2,LV_PART_MAIN);
        lv_obj_set_style_translate_y(cursor,-CURSOR_SIZE/2,LV_PART_MAIN);
        lv_obj_clear_flag(cursor,LV_OBJ_FLAG_CLICKABLE);

        nunchuk_indev = lv_indev_create();
        lv_indev_set_type(nunchuk_indev,LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(nunchuk_indev,nunchuk_pointer_read);
        lv_indev_set_cursor(nunchuk_indev,cursor);

        input_created = true;
    }

/// ----------- init of page_init ------------

    page_init = lv_obj_create(screen);
    lv_obj_set_size(page_init, lv_pct(100), lv_pct(100));
    lv_obj_center(page_init);
    lv_obj_set_style_bg_opa(page_init, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(page_init, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label_init1 = lv_label_create(page_init);
    lv_obj_t *label_init2 = lv_label_create(page_init);
    lv_obj_t *btn_init = lv_btn_create(page_init);

    lv_obj_set_style_text_font(label_init1,&lv_font_montserrat_18,LV_PART_MAIN);
    lv_label_set_text(label_init1,"PHYTEC FRANCE");
    lv_obj_align(label_init1,LV_ALIGN_CENTER,0,-50);

    lv_obj_set_style_text_font(label_init2,&lv_font_montserrat_14,LV_PART_MAIN);
    lv_label_set_text(label_init2,"with ZEPHYR");
    lv_obj_align(label_init2,LV_ALIGN_CENTER,0,-25);

    lv_obj_t *label_btn = lv_label_create(btn_init);
    lv_label_set_text(label_btn,"Start");
    lv_obj_center(label_btn);

    lv_obj_align(btn_init,LV_ALIGN_CENTER,BTN_START_X,BTN_START_Y);
    lv_obj_set_size(btn_init,BTN_START_LENGTH,BTN_START_WIDTH);
    lv_obj_set_style_bg_color(btn_init,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
    lv_obj_add_event_cb(btn_init,start_btn_cb,LV_EVENT_CLICKED,NULL);

/// ----------- init of page_hour ------------

    page_hour = lv_obj_create(screen);
    lv_obj_set_size(page_hour, lv_pct(100), lv_pct(100));
    lv_obj_center(page_hour);
    lv_obj_set_style_bg_opa(page_hour, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(page_hour, LV_OBJ_FLAG_SCROLLABLE);

    label_hour1 = lv_label_create(page_hour);
    label_hour2 = lv_label_create(page_hour);

    lv_obj_set_style_text_font(label_hour1,&lv_font_montserrat_20,LV_PART_MAIN);
    lv_obj_set_style_text_font(label_hour2,&lv_font_montserrat_14,LV_PART_MAIN);

    lv_label_set_text(label_hour1,"HOUR");
    lv_label_set_text(label_hour2,"DAY");

    lv_obj_align(label_hour1,LV_ALIGN_CENTER,0,-75);
    lv_obj_align(label_hour2,LV_ALIGN_CENTER,0,-25);

    lv_obj_t *btn_edit = lv_btn_create(page_hour);

    lv_obj_t *label_btn_edit = lv_label_create(btn_edit);
    lv_label_set_text(label_btn_edit,"EDIT");
    lv_obj_center(label_btn_edit);

    lv_obj_align(btn_edit,LV_ALIGN_CENTER,BTN_START_X,BTN_START_Y);
    lv_obj_set_size(btn_edit,BTN_START_LENGTH,BTN_START_WIDTH);
    lv_obj_set_style_bg_color(btn_edit,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
    lv_obj_add_event_cb(btn_edit,edit_btn_cb,LV_EVENT_CLICKED,NULL);

/// ----------- init of page_rgb ------------

    page_rgb = lv_obj_create(screen);
    lv_obj_set_size(page_rgb, lv_pct(100), lv_pct(100));
    lv_obj_center(page_rgb);
    lv_obj_set_style_bg_opa(page_rgb, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(page_rgb, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label_rgb1 = lv_label_create(page_rgb);
    lv_obj_t *label_rgb2 = lv_label_create(page_rgb);

    lv_label_set_text(label_rgb1,"RGB LED I2C");
    lv_label_set_text(label_rgb2,"PICK-COLOR");

    lv_obj_align(label_rgb1,LV_ALIGN_CENTER,0,-50);
    lv_obj_align(label_rgb2,LV_ALIGN_CENTER,0,50);

    hue_canvas = lv_canvas_create(page_rgb);
    lv_canvas_set_buffer(hue_canvas,canvas_buf,BAR_W,BAR_H,LV_COLOR_FORMAT_RGB565);
    lv_obj_align(hue_canvas,LV_ALIGN_CENTER,0,0);
    lv_obj_add_flag(hue_canvas,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hue_canvas,hue_canvas_cb,LV_EVENT_PRESSING,NULL);

    for(int x = 0; x < BAR_W; x++){
        uint16_t pos = (uint16_t)((uint32_t)x * 1536 / BAR_W);

        uint8_t r,g,b;
        hue_to_rgb(pos,&r,&g,&b);

        lv_color_t c = lv_color_make(r,g,b);

        for(int y = 0; y < BAR_H; y++){
            lv_canvas_set_px(hue_canvas,x,y,c,LV_OPA_COVER);
        }
    }

/// ----------- init of page_ctrl_a35 ------------

    page_ctrl_a35 = lv_obj_create(screen);
    lv_obj_set_size(page_ctrl_a35, lv_pct(100), lv_pct(100));
    lv_obj_center(page_ctrl_a35);
    lv_obj_set_style_bg_opa(page_ctrl_a35, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(page_ctrl_a35, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label_ctrl_a35_1 = lv_label_create(page_ctrl_a35);
    lv_obj_t *label_ctrl_a35_2 = lv_label_create(page_ctrl_a35);

    lv_label_set_text(label_ctrl_a35_1,"A35");
    lv_label_set_text(label_ctrl_a35_2,"BOOT");

    lv_obj_align(label_ctrl_a35_1,LV_ALIGN_CENTER,0,-50);
    lv_obj_align(label_ctrl_a35_2,LV_ALIGN_CENTER,0,-25);

/// ----------- init of page_reboot ------------

    page_reboot = lv_obj_create(screen);
    lv_obj_set_size(page_reboot, lv_pct(100), lv_pct(100));
    lv_obj_center(page_reboot);
    lv_obj_set_style_bg_opa(page_reboot, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(page_reboot, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label_ctrl_reboot1 = lv_label_create(page_reboot);
    lv_obj_t *label_ctrl_reboot2 = lv_label_create(page_reboot);
    lv_obj_t *btn_reboot = lv_btn_create(page_reboot);


    lv_label_set_text(label_ctrl_reboot1,"M33");
    lv_label_set_text(label_ctrl_reboot2,"REBOOT");

    lv_obj_align(label_ctrl_reboot1,LV_ALIGN_CENTER,0,-50);
    lv_obj_align(label_ctrl_reboot2,LV_ALIGN_CENTER,0,-25);

    lv_obj_t *label_btn_reboot = lv_label_create(btn_reboot);
    lv_label_set_text(label_btn_reboot,"REBOOT");
    lv_obj_center(label_btn_reboot);

    lv_obj_align(btn_reboot,LV_ALIGN_CENTER,BTN_START_X,BTN_START_Y);
    lv_obj_set_size(btn_reboot,BTN_START_LENGTH,BTN_START_WIDTH);
    lv_obj_set_style_bg_color(btn_reboot,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
    lv_obj_add_event_cb(btn_reboot,reboot_btn_cb,LV_EVENT_CLICKED,NULL);

/// ----------- init scroll ------------

    left = lv_btn_create(screen);
    lv_obj_t *label_left = lv_label_create(left);
    lv_label_set_text(label_left,"<");
    lv_obj_center(label_left);
    lv_obj_align(left,LV_ALIGN_CENTER,(-caps.x_resolution+BTN_SCROLL_SIZE+BTN_SCROLL_BORDER_SIZE)/2,(caps.y_resolution-BTN_SCROLL_SIZE-BTN_SCROLL_BORDER_SIZE)/2);
    lv_obj_set_size(left,BTN_SCROLL_SIZE,BTN_SCROLL_SIZE);
    lv_obj_set_style_bg_color(left,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
    lv_obj_add_event_cb(left,scroll_left_cb,LV_EVENT_CLICKED,NULL);

    right = lv_btn_create(screen);
    lv_obj_t *label_right = lv_label_create(right);
    lv_label_set_text(label_right,">");
    lv_obj_center(label_right);
    lv_obj_align(right,LV_ALIGN_CENTER,(caps.x_resolution-BTN_SCROLL_SIZE-BTN_SCROLL_BORDER_SIZE)/2,(caps.y_resolution-BTN_SCROLL_SIZE-BTN_SCROLL_BORDER_SIZE)/2);
    lv_obj_set_size(right,BTN_SCROLL_SIZE,BTN_SCROLL_SIZE);
    lv_obj_set_style_bg_color(right,lv_color_hex(BTN_COLOR),LV_PART_MAIN);
    lv_obj_add_event_cb(right,scroll_right_cb,LV_EVENT_CLICKED,NULL);

/// ----------- hiding all widgets ------------

    lv_obj_add_flag(page_hour, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(page_rgb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(page_ctrl_a35, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(page_reboot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(left, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(right, LV_OBJ_FLAG_HIDDEN);


    current_page=page_init;

    lv_obj_move_foreground(cursor);

/// ----------- end of init ------------

    display_clear(display_dev);
    lv_timer_handler();
    display_blanking_off(display_dev);

    return 0;
}

static void set_active_page(lv_obj_t *new_page){
    int64_t t0 = k_uptime_get();

    if (current_page) {
        lv_obj_add_flag(current_page, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(new_page, LV_OBJ_FLAG_HIDDEN);

    current_page = new_page;

    if (new_page == page_init) {
        lv_obj_add_flag(left, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(right, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(left, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(right, LV_OBJ_FLAG_HIDDEN);
    }


    lv_obj_move_foreground(cursor);
    lv_timer_handler();
    printk("time_handler %lld ms\n", k_uptime_get() - t0);

}

void set_i2c_rgb_led(int screen_x){

    lv_area_t area;
    lv_obj_get_coords(hue_canvas,&area);

    int x = screen_x - area.x1;

    if(x < 0){
        x = 0;
    }

    if(x >= BAR_W){
        x = BAR_W - 1;
    }

    uint16_t pos_x = (uint16_t)(((uint32_t)x * 1536) / BAR_W);

    uint8_t r,g,b;
    hue_to_rgb(pos_x,&r,&g,&b);

    led_set_brightness(rgb,RED_LED,r/2.55);
    led_set_brightness(rgb,GREEN_LED,g/2.55);
    led_set_brightness(rgb,BLUE_LED,b/2.55);

}

void fill_hue_bar(uint16_t *buf,int width,int height){
    for(int x = 0; x < width; x++){
        uint16_t pos = (uint16_t)((uint32_t)x * 1536 / width);

        uint8_t r,g,b;
        hue_to_rgb(pos,&r,&g,&b);

        uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);

        for(int y = 0; y < height; y++){
            buf[y*width+x] = color565;
        }
    }
}

void hue_to_rgb(uint16_t pos,uint8_t *r,uint8_t *g,uint8_t *b){
    uint16_t segment = pos / 256;
    uint8_t step = pos % 256;

    switch(segment){
        case 0: *r = 255;     *g = step;      *b = 0;        break;
        case 1: *r = 255-step;*g = 255;       *b = 0;        break;
        case 2: *r = 0;       *g = 255;       *b = step;     break;
        case 3: *r = 0;       *g = 255-step;  *b = 255;      break;
        case 4: *r = step;    *g = 0;         *b = 255;      break;
        case 5: *r = 255;     *g = 0;         *b = 255-step; break;
    }
}

void update_cursor_position(struct cursor_position_t *actual_pos){

    struct nunchuk_data_t actual_nunchuk = read_nunchuk();

    if(actual_pos->gyro){
        if(!(MIN_DEADZONE_ACCELEROMETER<actual_nunchuk.accel_x && actual_nunchuk.accel_x<=MAX_DEADZONE_ACCELEROMETER)){
            actual_pos->x += (actual_nunchuk.accel_x-ZEROING_POS_ACC)/RESCALING_ACCELEROMETER;
        }

        if(!(MIN_DEADZONE_ACCELEROMETER<actual_nunchuk.accel_y && actual_nunchuk.accel_y<=MAX_DEADZONE_ACCELEROMETER)){
            actual_pos->y -= (actual_nunchuk.accel_y-ZEROING_POS_ACC)/RESCALING_ACCELEROMETER;
        }
    }else{
        if(!(MIN_DEADZONE_JOYSTICK<actual_nunchuk.joystick_x && actual_nunchuk.joystick_x<=MAX_DEADZONE_JOYSTICK)){
            actual_pos->x += (actual_nunchuk.joystick_x-ZEROING_POS)/RESCALING_JOYSTICK;
        }

        if(!(MIN_DEADZONE_JOYSTICK<actual_nunchuk.joystick_y && actual_nunchuk.joystick_y<=MAX_DEADZONE_JOYSTICK)){
            actual_pos->y -= (actual_nunchuk.joystick_y-ZEROING_POS)/RESCALING_JOYSTICK;
        }
    }

    if(actual_pos->x < -caps.x_resolution/2){
        actual_pos->x = -caps.x_resolution/2;
    }else if(actual_pos->x >= caps.x_resolution/2){
        actual_pos->x = caps.x_resolution/2-1;
    }

    if(actual_pos->y < -caps.y_resolution/2){
        actual_pos->y = -caps.y_resolution/2;
    }else if(actual_pos->y >= caps.y_resolution/2){
        actual_pos->y = caps.y_resolution/2-1;
    }

    state_btn = (struct state_button_t){
        .c = actual_nunchuk.btn_c,
        .z = actual_nunchuk.btn_z
    };

    if(state_btn.c && !previous_state_btn.c){
        actual_pos->gyro = !actual_pos->gyro;
        actual_pos->color = actual_pos->color ^ SWITCH_CURSOR_COLOR;
        lv_obj_set_style_bg_color(cursor, lv_color_hex(actual_pos->color), LV_PART_MAIN);
    }
    previous_state_btn = state_btn;
}

static void nunchuk_pointer_read(lv_indev_t *indev,lv_indev_data_t *data){
    data->point.x = cursor_pos.x + caps.x_resolution/2;
    data->point.y = cursor_pos.y + caps.y_resolution/2;
    data->state = state_btn.z ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

bool checkEntry(){
    if(state==previous_state){
        return false;
    }else{
        return true;
    }
}

int display(){
    static int64_t time_now;
    static int64_t time_start;
    static bool entry = true;
    static bool time_defined = false;
    static bool time_started = false;
    while(1){
        time_now = k_uptime_get();
        entry = checkEntry();
        previous_state = state;

        switch(state){
            case INIT:
                if(entry){
                    if(init_screen()){
                        return 1;
                    }
                    set_active_page(page_init);
                }
            break;

            case SHOW_HOUR_INIT:
                if(entry){
                    printk("hehe");
                    time_defined=false;
                    time_started=false;
                    lv_obj_add_flag(left, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(right, LV_OBJ_FLAG_HIDDEN);
                    init_info_show_hour(screen,current_page,&time_defined);
                }
                if(time_defined){
                    state=SHOW_HOUR;
                }
            break;

            case SHOW_HOUR:
                if(entry){
                    set_active_page(page_hour);
                }
            break;

            case RGB_LED:
                if(entry){
                    set_active_page(page_rgb);
                }
            break;

            case CTRL_A35:
                if(entry){
                    set_active_page(page_ctrl_a35);
                }
            break;

            case REBOOT_M33:
                if(entry){
                    set_active_page(page_reboot);
                }
            break;

            default:
            break;
            
        }

        if(time_defined && !time_started){
            time_start = time_now;
            time_started = true;
            set_show_hour(label_hour1,label_hour2);
        }

        if(time_defined && time_now-time_start>=60000){
            update_show_hour(label_hour1,label_hour2);
            time_start = time_now;
        }
        update_cursor_position(&cursor_pos);
        lv_timer_handler();        

        k_msleep(5);
    }
}

static void start_btn_cb(lv_event_t *e){
    state = SHOW_HOUR_INIT;
}

static void reboot_btn_cb(lv_event_t *e){
    state = INIT;
}

static void edit_btn_cb(lv_event_t *e){
    state = SHOW_HOUR_INIT;
}

static void scroll_left_cb(lv_event_t *e){
    switch(state){
        case SHOW_HOUR: state = REBOOT_M33; break;
        case RGB_LED:   state = SHOW_HOUR;  break;
        case CTRL_A35:  state = RGB_LED;    break;
        case REBOOT_M33:state = CTRL_A35;   break;
        default: break;
    }
}

static void scroll_right_cb(lv_event_t *e){
    switch(state){
        case SHOW_HOUR: state = RGB_LED;    break;
        case RGB_LED:   state = CTRL_A35;   break;
        case CTRL_A35:  state = REBOOT_M33; break;
        case REBOOT_M33:state = SHOW_HOUR;  break;
        default: break;
    }
}

static void hue_canvas_cb(lv_event_t *e){
    lv_point_t p;
    lv_indev_t *indev = lv_indev_active();

    if(indev == NULL){
        return;
    }

    lv_indev_get_point(indev,&p);
    set_i2c_rgb_led(p.x);
}
