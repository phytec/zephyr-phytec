/*
 * Copyright (c) 2021 Phytec Messtechnik GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_lp5569

/**
 * @file
 * @brief LP5569 LED driver
 *
 * Limitations:
 * - Blink period and brightness value are controlled by two sets of PSCx/PWMx
 *   registers. This driver partitions the available LEDs into two groups as
 *   0 to 7 and 8 to 15 and assigns PSC0/PWM0 to LEDs from 0 to 7 and PSC1/PWM1
 *   to LEDs from 8 to 15. So, it is not possible to set unique blink period
 *   and brightness value for LEDs in a group, changing either of these
 *   values for a LED will affect other LEDs also.
 */

#include <drivers/i2c.h>
#include <drivers/led.h>
#include <sys/util.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <devicetree.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lp5569, LOG_LEVEL);

#include "led_context.h"
	
#define LP5569_NUM_LEDS					9
#define LP5569_DEFAULT_LED_CURRENT		255
#define LP5569_DEFAULT_DUTY_CYCLE_ON	255
#define LP5569_LED_OFFSET_MAX			8

/* LP5569 Registers */
#define LP5569_CONFIG                0x00 /* Configuration Register */
#define LP5569_CHIP_EN              (1 << 6) /* LP5569 enable */
#define LP5569_LED0_PWM              0x16 /* LED0 PWM Duty Cycle */
#define LP5569_LED1_PWM              0x17 /* LED1 PWM Duty Cycle */
#define LP5569_LED2_PWM              0x18 /* LED2 PWM Duty Cycle */
#define LP5569_LED3_PWM              0x19 /* LED3 PWM Duty Cycle */
#define LP5569_LED4_PWM              0x1A /* LED4 PWM Duty Cycle */
#define LP5569_LED5_PWM              0x1B /* LED5 PWM Duty Cycle */
#define LP5569_LED6_PWM              0x1C /* LED6 PWM Duty Cycle */
#define LP5569_LED7_PWM              0x1D /* LED7 PWM Duty Cycle */
#define LP5569_LED8_PWM              0x1E /* LED8 PWM Duty Cycle */
#define LP5569_LED0_CURRENT         0x22 /* LED0 Current Control */
#define LP5569_LED1_CURRENT         0x23 /* LED1 Current Control */
#define LP5569_LED2_CURRENT         0x24 /* LED2 Current Control */
#define LP5569_LED3_CURRENT         0x25 /* LED3 Current Control */
#define LP5569_LED4_CURRENT         0x26 /* LED4 Current Control */
#define LP5569_LED5_CURRENT         0x27 /* LED5 Current Control */
#define LP5569_LED6_CURRENT         0x28 /* LED6 Current Control */
#define LP5569_LED7_CURRENT         0x29 /* LED7 Current Control */
#define LP5569_LED8_CURRENT         0x2A /* LED8 Current Control */

#define LP5569_MISC					0x2F /* Miscellaneous Register */
#define LP5569_POWERSAVE_EN			(1 << 5) /* en for auto powersave mode */

struct lp5569_cfg {
	char *i2c_dev_name;
	uint16_t i2c_addr;
};

struct lp5569_data {
	const struct device *i2c;
	struct led_data dev_data;
};

static int lp5569_led_set_brightness(const struct device *dev, uint32_t led,
				     uint8_t value)
{
	const struct lp5569_cfg *config = dev->config;
	struct lp5569_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	
	if (led > LP5569_LED_OFFSET_MAX) {
		return -EINVAL;
	}

	if (value < dev_data->min_brightness ||
			value > dev_data->max_brightness) {
		return -EINVAL;
	}
	
	if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
			       LP5569_LED0_PWM + led, value)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}

	return 0;
}

static inline int lp5569_led_on(const struct device *dev, uint32_t led)
{
	const struct lp5569_cfg *config = dev->config;
	struct lp5569_data *data = dev->data;

	if (led > LP5569_LED_OFFSET_MAX) {
		return -EINVAL;
	}

	/* Set LED state to ON */
	if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
			       LP5569_LED0_PWM + led, LP5569_DEFAULT_DUTY_CYCLE_ON)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}

	return 0;
}

static inline int lp5569_led_off(const struct device *dev, uint32_t led)
{
	const struct lp5569_cfg *config = dev->config;
	struct lp5569_data *data = dev->data;
	
	if (led > LP5569_LED_OFFSET_MAX) {
		return -EINVAL;
	}

	/* Set LED state to OFF */
	if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
			       LP5569_LED0_PWM + led, 0)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}
	
	return 0;
}

static int lp5569_led_init(const struct device *dev)
{
	const struct lp5569_cfg *config = dev->config;
	struct lp5569_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	int led_offs;
	
	data->i2c = device_get_binding(config->i2c_dev_name);
	if (data->i2c == NULL) {
		LOG_DBG("Failed to get I2C device");
		return -EINVAL;
	}

	/* Hardware specific limits */
	dev_data->min_period = 0U;
	dev_data->max_period = 1600U;
	dev_data->min_brightness = 0U;
	dev_data->max_brightness = 255U;
	
	if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
			       LP5569_CONFIG, LP5569_CHIP_EN)) {
		LOG_ERR("Enable LP5569 failed");
		return -EIO;
	}
	k_sleep(K_MSEC(1));

	/* Set default current for all leds once. LEDs will be controlled via PWM
	 * within this driver.*/
	for(led_offs=0; led_offs < LP5569_NUM_LEDS; led_offs++) {
		if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
				       LP5569_LED0_CURRENT + led_offs,
					   LP5569_DEFAULT_LED_CURRENT)) {
			LOG_ERR("LED reg update failed");
			return -EIO;
		}
		if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
				       LP5569_LED0_PWM + led_offs, 0)) {
			LOG_ERR("LED reg update failed");
			return -EIO;
		}
	}

	if (i2c_reg_write_byte(data->i2c, config->i2c_addr,
			       LP5569_MISC, LP5569_POWERSAVE_EN)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}

	return 0;
}

static const struct led_driver_api lp5569_led_api = {
	.set_brightness = lp5569_led_set_brightness,
	.on = lp5569_led_on,
	.off = lp5569_led_off,
};

#define LED_LP5569_DEVICE(id)						\
	static const struct lp5569_cfg led_lp5569_##id##_cfg = {\
		.i2c_dev_name = DT_INST_BUS_LABEL(id),			\
		.i2c_addr     = DT_INST_REG_ADDR(id),			\
	};								\
									\
	static struct lp5569_data led_lp5569_##id##_data;	\
									\
	DEVICE_DT_INST_DEFINE(id,					\
			    &lp5569_led_init,				\
			    device_pm_control_nop,			\
			    &led_lp5569_##id##_data,			\
			    &led_lp5569_##id##_cfg, POST_KERNEL,	\
			    CONFIG_LED_INIT_PRIORITY,		\
			    &lp5569_led_api);

DT_INST_FOREACH_STATUS_OKAY(LED_LP5569_DEVICE)
