#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include "cubes.h"

#define SCALE        3
#define FONT_W       5
#define FONT_H       7
#define GW           (FONT_W * SCALE)
#define GH           (FONT_H * SCALE)
#define GAP          SCALE
#define NB_CHARS     8

#define COLOR_BG     0xFFFF
#define COLOR_FG     0x0000

#define RED_LED   0
#define GREEN_LED 1
#define BLUE_LED  2
#define PIN_NODE  DT_NODELABEL(gpioa)

#define GLYPH_COLON  10
#define GLYPH_NONE   0xFF

#define CUBE_W 20
#define CUBE_H 20

static const uint8_t font5x7[11][FONT_H] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, /* 0 */
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, /* 1 */
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, /* 2 */
    {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}, /* 3 */
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, /* 4 */
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}, /* 5 */
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, /* 6 */
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, /* 7 */
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, /* 8 */
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}, /* 9 */
    {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}, /* : */
};
static struct counter_clock_t c;
static uint8_t txt[NB_CHARS];

static const struct device *display_dev;
static const struct device *rgb;
static const struct device *gpio_wake_up;
static struct display_capabilities caps;

static uint16_t glyph_buf[GW * GH];

static uint16_t x0;
static uint16_t y0;
static uint8_t  shown[NB_CHARS];

static int64_t time_rgb = 0;
static int r = 100, g = 0, b = 0;
static int z_state = 0;
static int z_previous = 0;

static void change_color_led_rgb(int *r, int *g, int *b);
static void update_clock(void);

enum joy_state { 
    JOY_CENTER, 
    JOY_LEFT, 
    JOY_RIGHT 
};

static enum joy_state now = JOY_CENTER;
static enum joy_state last = JOY_CENTER;
static uint16_t cube_color = sys_cpu_to_be16(0xFFFF);
static uint16_t buf_color[CUBE_W * CUBE_H];
static uint16_t cube_x;
static uint16_t cube_y;
static bool reset_cube = false;
static int64_t time_start;
static int64_t time_now;
static struct display_buffer_descriptor desc = {
    .buf_size = sizeof(buf_color),
    .width    = CUBE_W,
    .height   = CUBE_H,
    .pitch    = CUBE_W,
}; 

static void draw_glyph(uint8_t idx, uint16_t px, uint16_t py)
{
    const uint16_t fg = sys_cpu_to_be16(COLOR_FG);
    const uint16_t bg = sys_cpu_to_be16(COLOR_BG);
    const struct display_buffer_descriptor d = {
        .buf_size = sizeof(glyph_buf),
        .width    = GW,
        .height   = GH,
        .pitch    = GW,
    };

    for (int y = 0; y < GH; y++) {
        uint8_t row = font5x7[idx][y / SCALE];
        for (int x = 0; x < GW; x++) {
            bool on = row & (1 << (FONT_W - 1 - (x / SCALE)));
            glyph_buf[y * GW + x] = on ? fg : bg;
        }
    }
    display_write(display_dev, px, py, &d, glyph_buf);
}

static void clear_screen(void)
{
    const uint16_t bg = sys_cpu_to_be16(COLOR_BG);

    for (int i = 0; i < GW * GH; i++) {
        glyph_buf[i] = bg;
    }

    for (uint16_t by = 0; by < caps.y_resolution; by += GH) {
        for (uint16_t bx = 0; bx < caps.x_resolution; bx += GW) {
            uint16_t w = MIN(GW, caps.x_resolution - bx);
            uint16_t h = MIN(GH, caps.y_resolution - by);
            struct display_buffer_descriptor d = {
                .buf_size = w * h * sizeof(uint16_t),
                .width    = w,
                .height   = h,
                .pitch    = w,
            };
            display_write(display_dev, bx, by, &d, glyph_buf);
        }
    }
}

int init_screen_cubes(void)
{
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        printk("Display non pret\n");
        return -1;
    }

    rgb = DEVICE_DT_GET(DT_ALIAS(rgb_led0));
    if (!device_is_ready(rgb)) {
        printk("PCA9633 not ready\n");
        return -1;
    }

    gpio_wake_up = DEVICE_DT_GET(PIN_NODE);
    if (!device_is_ready(gpio_wake_up)) {
        printk("GPIOA pas prêt\n");
        return -1;
    }

    gpio_pin_configure(gpio_wake_up, 3, GPIO_OUTPUT_LOW);
    display_get_capabilities(display_dev, &caps);
    clear_screen();

    const uint16_t total_w = NB_CHARS * GW + (NB_CHARS - 1) * GAP;
    x0 = (caps.x_resolution - total_w) / 2;
    y0 = (caps.y_resolution - GH) / 4;

    update_clock();

    for (int i = 0; i < NB_CHARS; i++) {
        draw_glyph(txt[i], x0 + i * (GW + GAP), y0);
        shown[i] = txt[i];
    }
    cube_x = (caps.x_resolution - CUBE_W) / 2;
    cube_y = (caps.y_resolution - CUBE_H) / 2;
    cube_color = sys_cpu_to_be16(0x001F);
    display_write(display_dev,cube_x+50,cube_y,&desc,buf_color);
    display_write(display_dev,cube_x-50,cube_y,&desc,buf_color);
    display_blanking_off(display_dev);
    time_rgb = k_uptime_get();
    time_start = k_uptime_get();
    time_now = k_uptime_get();

    
    return 0;
}

int cubes_screen(struct nunchuk_data_t actual_nunchuk)
{
    z_state = actual_nunchuk.btn_z;

    if ((z_state == 1) && (z_state != z_previous)) {
        gpio_pin_set(gpio_wake_up, 3, 1);
        k_msleep(1000);
        gpio_pin_set(gpio_wake_up, 3, 0);
        return ID_WAKE;
    }

    if (actual_nunchuk.joystick_x < 110) {
        now = JOY_LEFT;
    } else if (actual_nunchuk.joystick_x >= 140) {
        now = JOY_RIGHT;
    } else {
        now = JOY_CENTER;
    }

    if (now != last) {
            if (now == JOY_LEFT) {
                cube_color = sys_cpu_to_be16(0xF800);
                reset_cube=true;
                for (int i = 0; i < CUBE_W * CUBE_H; i++) {
                    buf_color[i] = cube_color;
	            }
                display_write(display_dev,cube_x-50,cube_y,&desc,buf_color);
                time_start = k_uptime_get();

            } else if (now == JOY_RIGHT) {
                cube_color = sys_cpu_to_be16(0xF800);
                reset_cube=true;
                for (int i = 0; i < CUBE_W * CUBE_H; i++) {
                    buf_color[i] = cube_color;
	            }
                display_write(display_dev,cube_x+50,cube_y,&desc,buf_color);
                time_start = k_uptime_get();
                
            }
        }

        if(reset_cube && ((time_now-time_start)>=5 && now != last)) {
            reset_cube=false;
            cube_color = sys_cpu_to_be16(0x001F);

            for (int i = 0; i < CUBE_W * CUBE_H; i++) {
                buf_color[i] = cube_color;
            }

            display_write(display_dev,cube_x+50,cube_y,&desc,buf_color);
            display_write(display_dev,cube_x-50,cube_y,&desc,buf_color);
            
        }

    update_clock();

    for (int i = 0; i < NB_CHARS; i++) {
        if (txt[i] != shown[i]) {
            draw_glyph(txt[i], x0 + i * (GW + GAP), y0);
            shown[i] = txt[i];
        }
    }

    int64_t now = k_uptime_get();
    if (now - time_rgb > 5) {
        change_color_led_rgb(&r, &g, &b);
        led_set_brightness(rgb, RED_LED, r);
        led_set_brightness(rgb, GREEN_LED, g);
        led_set_brightness(rgb, BLUE_LED, b);
        time_rgb = now;
    }
    last = now;
    time_now = k_uptime_get();
    z_previous = z_state;
    k_msleep(10);
    return 0;
}

static void change_color_led_rgb(int *r, int *g, int *b)
{
    if ((*r == 100) && (*g < 100) && (*b == 0)) {
        (*g)++;
    } else if ((*r > 0) && (*g == 100) && (*b == 0)) {
        (*r)--;
    } else if ((*r == 0) && (*g == 100) && (*b < 100)) {
        (*b)++;
    } else if ((*r == 0) && (*g > 0) && (*b == 100)) {
        (*g)--;
    } else if ((*r < 100) && (*g == 0) && (*b == 100)) {
        (*r)++;
    } else if ((*r == 100) && (*g == 0) && (*b > 0)) {
        (*b)--;
    }
}

static void update_clock(){
    c = getClock();
    txt[0] = c.hour / 10;
    txt[1] = c.hour % 10;
    txt[2] = GLYPH_COLON;
    txt[3] = c.minutes / 10;
    txt[4] = c.minutes % 10;
    txt[5] = GLYPH_COLON;
    txt[6] = c.seconds / 10;
    txt[7] = c.seconds % 10;
}