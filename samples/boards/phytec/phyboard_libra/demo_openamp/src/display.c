#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>

#include "display.h"
#include "nunchuk.h"
#include "rpmsg_platform.h"

static const struct device *display_dev;
static const struct device *rgb;

#define W 20
#define H 20

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define PIN_NODE DT_NODELABEL(gpioa)

static void change_color_led_rgb(int *r,int *g,int *b);

int init()
{
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        printk("Display non pret\n");
        return 1;
    }

    rgb = DEVICE_DT_GET(DT_ALIAS(rgb_led0));

    if (!device_is_ready(rgb)) {
		printf("PCA9633 not ready\n");
		return -1;
	}

    return 0;
}

int init_screen()
{

    static uint16_t buf_color[W * H];
    struct nunchuk_data_t actual_nunchuk;
    int ret = 0;
    enum joy_state { JOY_CENTER, JOY_LEFT, JOY_RIGHT };
    enum joy_state last = JOY_CENTER;
    bool reset_cube = false;

    if(init()){
        printk("---------- ERROR INIT SCREEN ----------\n");
        return 1;
    }

    static const struct device *gpio_wake_up;
    gpio_wake_up = DEVICE_DT_GET(PIN_NODE);

	if (!device_is_ready(gpio_wake_up)) {
		printk("GPIOA pas prêt\n");
	}
	int reaea = gpio_pin_configure(gpio_wake_up, 3, GPIO_OUTPUT_LOW);
	printk("configure PA3: %d\n", reaea);    

    struct display_capabilities caps;

    display_get_capabilities(display_dev,&caps);

	printk("Ecran %ux%u, format=0x%x\n",
	       caps.x_resolution, caps.y_resolution,
	       caps.current_pixel_format);
    
    /// WHITE : Background_color
    uint16_t color = sys_cpu_to_be16(0xFFFF);

    for (int i = 0; i < W * H; i++) {
		buf_color[i] = color;
	}

	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(buf_color),
		.width    = W,
		.height   = H,
		.pitch    = W,
	}; 

	uint16_t x = (caps.x_resolution - W) / 2;
	uint16_t y = (caps.y_resolution - H) / 2;

	for(int bx = 0; bx < caps.x_resolution;bx+=W){
        for(int by = 0; by < caps.y_resolution;by+=H){
            display_write(display_dev,bx,by,&desc,buf_color);
        }
    }

    color = sys_cpu_to_be16(0x001F);

	for (int i = 0; i < W * H; i++) {
		buf_color[i] = color;
	}

    ret = display_write(display_dev,x+50,y,&desc,buf_color);
    ret = display_write(display_dev,x-50,y,&desc,buf_color);

    display_blanking_off(display_dev);

    int64_t time_start = k_uptime_get();
    int64_t time_now = k_uptime_get();
    int64_t time_rgb = k_uptime_get();

    int r = 100;
    int g = 0;
    int b = 0;
    int z_state= 0;
    int z_previous =0;

    bool awake = true;
    enum joy_state now = JOY_CENTER;
    while(1){

        actual_nunchuk = read_nunchuk();
        z_state = actual_nunchuk.btn_z;

        if((z_state==1) && (z_state != z_previous)){
            if (awake){
                rpmsg_tty_send("SLEEP\n", 6);
                awake = false;
            }else{
        		gpio_pin_set(gpio_wake_up, 3, 1);
        		k_msleep(1000);
        		gpio_pin_set(gpio_wake_up, 3, 0);
                awake=true;
            }
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
                int r = rpmsg_tty_send("LEFT\n", 5);
                if (r < 0) {
                    printk("rpmsg_tty_send failed: %d\n", r);
                }

                color = sys_cpu_to_be16(0xF800);
                reset_cube=true;
                for (int i = 0; i < W * H; i++) {
                    buf_color[i] = color;
	            }

                ret = display_write(display_dev,x-50,y,&desc,buf_color);
                time_start = k_uptime_get();

            } else if (now == JOY_RIGHT) {
                rpmsg_tty_send("RIGHT\n", 6);

                color = sys_cpu_to_be16(0xF800);
                reset_cube=true;
                for (int i = 0; i < W * H; i++) {
                    buf_color[i] = color;
	            }
                ret = display_write(display_dev,x+50,y,&desc,buf_color);
                time_start = k_uptime_get();
                
            }
        }

        if(reset_cube && ((time_now-time_start)>=5 && now != last)) {
            reset_cube=false;
                color = sys_cpu_to_be16(0x001F);

            for (int i = 0; i < W * H; i++) {
                buf_color[i] = color;
            }

            ret = display_write(display_dev,x+50,y,&desc,buf_color);
            ret = display_write(display_dev,x-50,y,&desc,buf_color);
            
        }

        if (time_now-time_rgb>5) {
            change_color_led_rgb(&r,&g,&b);
            led_set_brightness(rgb, RED_LED, r);
            led_set_brightness(rgb, GREEN_LED, g);
            led_set_brightness(rgb, BLUE_LED, b);

            time_rgb = k_uptime_get();
        }
        last = now;
        time_now = k_uptime_get();  
        z_previous=z_state;
        k_msleep(10);
    }

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

