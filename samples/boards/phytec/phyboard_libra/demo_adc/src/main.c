/*
 * Copyright (C) 2025 PHYTEC Messtechnik GmbH
 * Authors: Hong-Ly Ky <h.ky@phytec.fr>
 *          Christophe Parant <c.parant@phytec.fr>
 *          Nicolas Raimundo <n.raimundo@phytec.fr>
 */

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>

#define SLEEP_TIME_MS 1000

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define PIN_NODE DT_NODELABEL(gpioi)

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct device *gpioi_dev;
static const struct device *rgb = DEVICE_DT_GET(DT_ALIAS(rgb_led0));

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

int init_output(void)
{
	gpioi_dev = DEVICE_DT_GET(PIN_NODE);

	if (!device_is_ready(gpioi_dev)) {
		return -1;
	}

	return gpio_pin_configure(gpioi_dev, 3, GPIO_OUTPUT_INACTIVE);
}

int i2c_setup_rgb(void)
{
	if (!device_is_ready(rgb)) {
		printf("PCA9633 not ready\n");
		return -1;
	}

	return 0;
}

int adc_setup(void)
{
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		int err;

		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n",
			       adc_channels[i].dev->name);
			return -1;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return err;
		}
	}

	return 0;
}

void set_pin(bool on)
{
	gpio_pin_set(gpioi_dev, 3, on ? 1 : 0);
}

int32_t adc_read_channel(void)
{
	int err;
	int32_t result = -1;

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		uint16_t buf;
		int32_t val_mv;

		struct adc_sequence sequence = {
			.buffer = &buf,
			.buffer_size = sizeof(buf),
		};

		printk("- %s, channel %d: ",
		       adc_channels[i].dev->name,
		       adc_channels[i].channel_id);

		err = adc_sequence_init_dt(&adc_channels[i], &sequence);
		if (err < 0) {
			printk("Could not initialize sequence (%d)\n", err);
			continue;
		}

		err = adc_read_dt(&adc_channels[i], &sequence);
		if (err < 0) {
			printk("Could not read (%d)\n", err);
			continue;
		}

		if (adc_channels[i].channel_cfg.differential) {
			val_mv = (int32_t)((int16_t)buf);
		} else {
			val_mv = (int32_t)buf;
		}

		err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
		if (err < 0) {
			printk(" (value in mV not available)\n");
		} else {
			printk(" Value : %" PRId32 "\n", val_mv);
			result = val_mv;
		}
	}

	return result;
}

int button_debounce(int *current_state, int *previous_state)
{
	return *current_state != *previous_state;
}

bool setup(void){
	
	int setup_rgb_check = 0;
	int setup_adc_check = 0;
	int setup_output_check = 0;

	setup_rgb_check = i2c_setup_rgb();
	setup_adc_check = adc_setup();
	setup_output_check = init_output();

	bool ok_rgb = (setup_rgb_check == 0);
	bool ok_adc = (setup_adc_check == 0);
	bool ok_output = (setup_output_check == 0);

	if (!(ok_rgb && ok_adc && ok_output)) {
		printf("Setup failed : \n"
				"i2c rgb check : %d \n"
				"adc check : %d \n"
				"output check : %d \n"
				"Next attempt in 1 second...\n",
				setup_rgb_check,
				setup_adc_check,
				setup_output_check);
	}

	return ok_rgb && ok_adc && ok_output;
}



int main(void)
{
	//Init all instances
	bool setup_check = false;
	int current_state = 0;
	int previous_state = 0;
	int state_rgb = 1;
	bool pin_state = false;
	int32_t adc_reading = -1;
	int64_t time_start = k_uptime_get();
	int64_t time_now = time_start;
	// Setup
	while(1){
		printk("I'm still alive\n");
		k_msleep(2000);
	}

	while (!setup_check) {
		setup_check = setup();
		k_msleep(SLEEP_TIME_MS);
	}

	// Main loop
	while (true) {
		adc_reading = adc_read_channel();

		if (adc_reading != -1) {
			if (adc_reading < 900) {
				current_state = 0;
			} else {
				current_state = 1;
			}
		} else {
			printk("Error value for adc\n");
		}

		if (button_debounce(&current_state, &previous_state)) {
			state_rgb++;

			if (state_rgb == 4) {
				state_rgb = 1;
			}

			if (state_rgb == 1) {
				led_set_brightness(rgb, RED_LED, 100);
				led_set_brightness(rgb, GREEN_LED, 0);
				led_set_brightness(rgb, BLUE_LED, 0);
			} else if (state_rgb == 2) {
				led_set_brightness(rgb, RED_LED, 0);
				led_set_brightness(rgb, GREEN_LED, 100);
				led_set_brightness(rgb, BLUE_LED, 0);
			} else {
				led_set_brightness(rgb, RED_LED, 0);
				led_set_brightness(rgb, GREEN_LED, 0);
				led_set_brightness(rgb, BLUE_LED, 100);
			}
		}

		previous_state = current_state;
		time_now = k_uptime_get();

		if (time_now - time_start > 3000) {
			time_start = time_now;
			pin_state = !pin_state;
			set_pin(pin_state);
		}
	}

	return 0;
}