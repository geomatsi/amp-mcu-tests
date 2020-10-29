/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <resource_table.h>

#define SLEEP_TIME_MS   100

#define LED_NODE DT_NODELABEL(red_led)

#define LED	DT_GPIO_LABEL(LED_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED_NODE, gpios)

#define __resource Z_GENERIC_SECTION(.resource_table)

struct fw_resource_table resource_table __resource = {
	.ver = 1,
	.num = 0,
};

void main(void)
{
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding(LED);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}
