#include <zephyr/kernel.h>

#include <lvgl.h>

#define MIN_DEADZONE_JOYSTICK 110
#define MAX_DEADZONE_JOYSTICK 140

#define MIN_DEADZONE_ACCELEROMETER 448
#define MAX_DEADZONE_ACCELEROMETER 576

#define ZEROING_POS 130
#define ZEROING_POS_ACC 512
#define RESCALING_JOYSTICK 24
#define RESCALING_ACCELEROMETER 32

#define CURSOR_SIZE 10
#define BTN_START_X 0
#define BTN_START_Y 25
#define BTN_START_WIDTH 50
#define BTN_START_LENGTH 100
#define BTN_SCROLL_SIZE 50
#define BTN_SCROLL_BORDER_SIZE 5

#define BTN_COLOR 0x0000FF 
#define INIT_CURSOR_COLOR 0xFF0010
#define BTN_PRESSED_COLOR 0xBA0000
#define SWITCH_CURSOR_COLOR 0xFFFF00

#define BAR_W 180
#define BAR_H 60

#define RED_LED   0
#define GREEN_LED 1
#define BLUE_LED  2

struct cursor_position_t {
    int x;
    int y;
    bool gyro;
    uint32_t color;
};

struct state_button_t {
    bool c;
    bool z;
};

enum state_t {
    INIT,
    SHOW_HOUR,
    SHOW_HOUR_INIT,
    RGB_LED,
    CTRL_A35,
    REBOOT_M33
};

int display(void);
int init_screen(void);
int init_i2c(void);
bool checkEntry(void);

void update_cursor_position(struct cursor_position_t *actual_pos);
void hue_to_rgb(uint16_t pos, uint8_t *r, uint8_t *g, uint8_t *b);