#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

int main(void){
    static const struct device *rgb = DEVICE_DT_GET(DT_ALIAS(rgb_led0));

    if (!device_is_ready(rgb)) {
		printf("PCA9633 not ready\n");
		return -1;
	}

    while(1){
        printf("ROUGE\n");

        led_set_brightness(rgb, RED_LED, 100);
        led_set_brightness(rgb, GREEN_LED, 0);
        led_set_brightness(rgb, BLUE_LED, 0);

        k_msleep(2000);

        printf("VERT\n");
        
        led_set_brightness(rgb, RED_LED, 0);
        led_set_brightness(rgb, GREEN_LED, 100);
        led_set_brightness(rgb, BLUE_LED, 0);

        k_msleep(2000);

        printf("BLEU\n");
        
        led_set_brightness(rgb, RED_LED, 0);
        led_set_brightness(rgb, GREEN_LED, 0);
        led_set_brightness(rgb, BLUE_LED, 100);

        k_msleep(2000);
    }

    return 0;
}
