#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>

#include <string.h>

#include "display.h"

static const struct device *display_dev;

#define W 150
#define H 215
#define RED_RGB565 0xF800
#define BLUE_RGB565 0x001F

int init_display()
{
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        printk("Display non pret\n");
        return 1;
    }

    return 0;
}

int init_screen()
{
    static uint16_t buf_red[W * H];
    static uint16_t buf_blue[W * H];
    
    if(init_display()){
        return 1;
    }

    struct display_capabilities caps;

    display_get_capabilities(display_dev,&caps);

	printk("Ecran %ux%u, format=0x%x\n",
	       caps.x_resolution, caps.y_resolution,
	       caps.current_pixel_format);

	uint16_t red = sys_cpu_to_be16(RED_RGB565);
    uint16_t blue = sys_cpu_to_be16(BLUE_RGB565);


	for (int i = 0; i < W * H; i++) {
		buf_red[i] = red;
	}

	for (int i = 0; i < W * H; i++) {
		buf_blue[i] = blue;
	}

	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(buf_red),
		.width    = W,
		.height   = H,
		.pitch    = W,
	};

	uint16_t x = (caps.x_resolution - W) / 2;
	uint16_t y = (caps.y_resolution - H) / 2;

	display_blanking_off(display_dev);
    int ret ;
    int64_t start;
    int64_t end;

    while(1){
        printk("ROUGE\n");

        start = k_uptime_get();
        ret = display_write(display_dev,x,y,&desc,buf_red);
        end = k_uptime_get();

        printk("write ret=%d time=%lld ms\n",ret,end-start);

        k_msleep(2000);

        printk("BLEU\n");

        start = k_uptime_get();
        ret = display_write(display_dev,x,y,&desc,buf_blue);
        end = k_uptime_get();

        printk("write ret=%d time=%lld ms\n",ret,end-start);

        k_msleep(2000);
    }

    return 0;
}
