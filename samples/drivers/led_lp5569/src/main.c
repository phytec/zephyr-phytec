/*
 * Copyright (c) 2021 Phytec Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <drivers/led.h>
#include <sys/util.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <devicetree.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

#define LED_DEV_NAME DT_LABEL(DT_INST(0, ti_lp5569))
#define NUM_LEDS		8
#define MAX_BRIGHTNESS	254

#define DELAY_TIME		K_MSEC(4)
#define LED_ON_TIME		K_MSEC(2000)
#define LED_OFF_TIME	K_MSEC(2000)

void main(void)
{
	int i;
	const struct device *led_dev;
	uint8_t brightness;
	
	led_dev = device_get_binding(LED_DEV_NAME);
	if (led_dev) {
		LOG_INF("Found LED device %s", LED_DEV_NAME);
	} else {
		LOG_ERR("LED device %s not found", LED_DEV_NAME);
		return;
	}

	/*
	 * Display a continuous pattern that turns on 8 LEDs on one by
	 * one until it reaches the end and turns off LEDs in same order.
	 */
	LOG_INF("Displaying the pattern");

	while (1) {
		/* Turn on LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			if (led_on(led_dev, i) != 0) {
				return;
			}
		}
		k_sleep(LED_ON_TIME);

		/* Turn off LEDs one by one */
		for (i = 0; i < NUM_LEDS; i++) {
			if (led_off(led_dev, i) != 0) {
				return;
			}
		}
		k_sleep(LED_OFF_TIME);
		
		/* Dim up all LEDs */
		for (brightness = 0; brightness < MAX_BRIGHTNESS; brightness++) {
			for (i = 0; i < NUM_LEDS; i++) {
				if (led_set_brightness(led_dev, i, brightness) != 0) {
					return;
				}
			}
			k_sleep(DELAY_TIME);
		}

		/* Dim down all LEDs */
		for (brightness = 0; brightness <= MAX_BRIGHTNESS; brightness++) {
			for (i = 0; i < NUM_LEDS; i++) {
				if (led_set_brightness(led_dev, i, MAX_BRIGHTNESS - brightness) != 0) {
					return;
				}
			}
			k_sleep(DELAY_TIME);
		}

		k_sleep(LED_OFF_TIME);
	}
}
