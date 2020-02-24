/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <soc.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   (K_MSEC(1000))

#define RUN_LOCAL 0

#define SW0  DT_ALIAS(sw0)
#define LED0 DT_ALIAS(led0)
#define LED1 DT_ALIAS(led1)
#define LED2 DT_ALIAS(led2)

static int board_sam4l_wm400_cape_board_init(const struct device *dev)
{
#if (RUN_LOCAL == 1)
	const struct device *ctrl_btn;
	const struct device *ctrl_led1;
	bool led_is_on = true;
#endif
	const struct device *ctrl_led2;
	int ret;

	ARG_UNUSED(dev);

	ctrl_led2 = device_get_binding(DT_GPIO_LABEL(LED2, gpios));
	if (ctrl_led2 == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_led2,
				 DT_GPIO_PIN(LED2, gpios),
				 DT_GPIO_FLAGS(LED2, gpios) |
				 GPIO_OUTPUT_ACTIVE);

	if (ret < 0) {
		return 0;
	}

#if (RUN_LOCAL == 1)
	ctrl_led1 = device_get_binding(DT_GPIO_LABEL(LED0, gpios));
	if (ctrl_led1 == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_led1,
				 DT_GPIO_PIN(LED0, gpios),
				 DT_GPIO_FLAGS(LED0, gpios) |
				 GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return 0;
	}

	ctrl_btn = device_get_binding(DT_GPIO_LABEL(SW0, gpios));
	if (ctrl_btn == NULL) {
		return 0;
	}

	ret = gpio_pin_configure(ctrl_btn,
				 DT_GPIO_PIN(SW0, gpios),
				 DT_GPIO_FLAGS(SW0, gpios) |
				 GPIO_INPUT);

	if (ret < 0) {
		return 0;
	}
#endif

#if (RUN_LOCAL == 1)
	while (1) {
		ret = gpio_pin_get(ctrl_btn, DT_GPIO_PIN(SW0, gpios));
		gpio_pin_set(ctrl_led1, DT_GPIO_PIN(LED0, gpios), ret);

		gpio_pin_set(ctrl_led2, DT_GPIO_PIN(LED2, gpios),
			     (int) led_is_on);
		led_is_on = !led_is_on;

		k_sleep(SLEEP_TIME_MS);
	}
#endif
	return 0;
}

SYS_INIT(board_sam4l_wm400_cape_board_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
