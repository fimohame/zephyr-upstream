/*
 * Copyright (c) 2017, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_gpio_port

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <sl_gpio.h>
#include <sl_hal_gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#if GPIO_SILABS_COMMON_INIT_PRIORITY >= CONFIG_GPIO_INIT_PRIORITY
#error GPIO_SILABS_COMMON_INIT_PRIORITY must be less than \
	CONFIG_GPIO_INIT_PRIORITY.
#endif

#if DT_NODE_HAS_PROP(id, peripheral_id)
#define GET_SILABS_GPIO_INDEX(id) DT_INST_PROP(id, peripheral_id)
#else
#if defined(CONFIG_SOC_FAMILY_SILABS_S2)
#define SILABS_GPIO_PORT_ADDR_SPACE_SIZE sizeof(sl_gpio_port_t)
#endif
/* Assumption for calculating gpio index:
 * 1. Address space of the first GPIO port is the address space for GPIO port A
 */
#define GET_SILABS_GPIO_INDEX(id) (DT_INST_REG_ADDR(id) - DT_REG_ADDR(DT_NODELABEL(gpioa))) \
	/ SILABS_GPIO_PORT_ADDR_SPACE_SIZE
#endif /* DT_NODE_HAS_PROP(id, peripheral_id) */


#define NUMBER_OF_PORTS (SIZEOF_FIELD(GPIO_TypeDef, P) / \
			 SIZEOF_FIELD(GPIO_TypeDef, P[0]))

struct gpio_silabs_dev_common_config {
};

struct gpio_silabs_dev_common_data {
	/* a list of all ports */
	const struct device *ports[NUMBER_OF_PORTS];
	size_t count;
};

struct gpio_silabs_dev_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	sl_gpio_port_t gpio_index;
};

struct gpio_silabs_dev_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* mask of pins on which interrupt is enabled */
	uint32_t int_enabled_mask;
};

static inline void gpio_silabs_add_port(struct gpio_silabs_dev_common_data *data,
				       const struct device *dev)
{
	__ASSERT(dev, "No port device!");
	data->ports[data->count++] = dev;
}

static int gpio_silabs_dev_configure(const struct device *dev,
				gpio_pin_t pin,
				gpio_flags_t flags)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;
	sl_gpio_t gpio = { .port = gpio_index, .pin = pin };
	sl_gpio_mode_t mode;
	unsigned int out = 0U;

	if (flags & GPIO_OUTPUT) {
		/* Following modes enable both output and input */
		if (flags & GPIO_SINGLE_ENDED) {
			if (flags & GPIO_LINE_OPEN_DRAIN) {
				mode = SL_GPIO_MODE_WIRED_AND;
			} else {
				mode = SL_GPIO_MODE_WIRED_OR;
			}
		} else {
			mode = SL_GPIO_MODE_PUSH_PULL;
		}
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			out = 1U;
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			out = 0U;
		} else {
			bool pin_out;
			out = sl_gpio_get_pin_output(&gpio, &pin_out);
		}
	} else if (flags & GPIO_INPUT) {
		if (flags & GPIO_PULL_UP) {
			mode = SL_GPIO_MODE_INPUT_PULL;
			out = 1U; /* pull-up*/
		} else if (flags & GPIO_PULL_DOWN) {
			mode = SL_GPIO_MODE_INPUT_PULL;
			/* out = 0 means pull-down*/
		} else {
			mode = SL_GPIO_MODE_INPUT;
		}
	} else {
		/* Neither input nor output mode is selected */
		mode = SL_GPIO_MODE_DISABLED;
	}
	/* The flags contain options that require touching registers in the
	 * GPIO module and the corresponding PORT module.
	 *
	 * Start with the GPIO module and set up the pin direction register.
	 * 0 - pin is input, 1 - pin is output
	 */

	sl_gpio_set_pin_mode(&gpio, mode, out);

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_silabs_dev_get_config(const struct device *dev,
				 gpio_pin_t pin,
				 gpio_flags_t *out_flags)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;
	sl_gpio_t gpio = { .port = gpio_index, .pin = pin };
	sl_gpio_pin_config_t pin_config;
	bool out;
	gpio_flags_t flags = 0;

	sl_gpio_get_pin_config(&gpio, &pin_config);
	sl_gpio_get_pin_output(&gpio, &out);

	switch (pin_config.mode) {
	case SL_GPIO_MODE_WIRED_AND:
		flags = GPIO_OUTPUT | GPIO_OPEN_DRAIN;
		flags |= out ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW;
		break;
	case SL_GPIO_MODE_WIRED_OR:
		flags = GPIO_OUTPUT | GPIO_OPEN_SOURCE;
		flags |= out ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW;
		break;
	case SL_GPIO_MODE_PUSH_PULL:
		flags = GPIO_OUTPUT | GPIO_PUSH_PULL;
		flags |= out ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW;
		break;
	case SL_GPIO_MODE_INPUT_PULL:
		flags = GPIO_INPUT;
		flags |= out ? GPIO_PULL_UP : GPIO_PULL_DOWN;
		break;
	case SL_GPIO_MODE_INPUT:
		flags = GPIO_INPUT;
		break;
	case SL_GPIO_MODE_DISABLED:
		flags = GPIO_DISCONNECTED;
		break;
	default:
		break;
	}

	*out_flags = flags;

	return 0;
}
#endif

static int gpio_silabs_dev_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;

	sl_gpio_get_port_input(gpio_index, value);

	return 0;
}

static int gpio_silabs_dev_port_set_masked_raw(const struct device *dev,
					  uint32_t mask,
					  uint32_t value)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;

	sl_gpio_set_port(gpio_index, value & mask);

	return 0;
}

static int gpio_silabs_dev_port_set_bits_raw(const struct device *dev,
					uint32_t mask)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;

	sl_gpio_set_port(gpio_index, mask);

	return 0;
}

static int gpio_silabs_dev_port_clear_bits_raw(const struct device *dev,
					  uint32_t mask)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;

	sl_gpio_clear_port(gpio_index, mask);

	return 0;
}

static int gpio_silabs_dev_port_toggle_bits(const struct device *dev,
				       uint32_t mask)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	sl_gpio_port_t gpio_index = config->gpio_index;

	// for (uint8_t pin = 0; pin < 32; ++pin) {
	// 	if (mask & (1 << pin)) {
	// 		sl_gpio_t gpio = { .port = gpio_index, .pin = pin };
	// 		sl_gpio_toggle_pin(&gpio);
	// 	}
	// }
	sl_hal_gpio_toggle_port(gpio_index, mask);

	return 0;
}

static int gpio_silabs_dev_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	const struct gpio_silabs_dev_config *config = dev->config;
	struct gpio_silabs_dev_data *data = dev->data;
	sl_gpio_t gpio = {.port = config->gpio_index, .pin = pin};
	sl_gpio_interrupt_flag_t flag = SL_GPIO_INTERRUPT_RISING_FALLING_EDGE;

	/* Interrupt on static level is not supported by the hardware */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		sl_gpio_disable_interrupts(BIT(pin));
	} else {
		/* Interrupt line is already in use */
		if ((GPIO->IEN & BIT(pin)) != 0) {
			/* Check if the interrupt is already configured for this port */
			if (!(data->int_enabled_mask & BIT(pin))) {
				return -EBUSY;
			}
		}

		if (trig == GPIO_INT_TRIG_LOW) {
			flag = SL_GPIO_INTERRUPT_FALLING_EDGE;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			flag = SL_GPIO_INTERRUPT_RISING_EDGE;
		} /* default is GPIO_INT_TRIG_BOTH */

		int32_t int_no = (int32_t)pin;
		sl_gpio_configure_external_interrupt(&gpio, &int_no, flag, NULL, NULL);
	}

	WRITE_BIT(data->int_enabled_mask, pin, mode != GPIO_INT_DISABLE);

	return 0;
}

static int gpio_silabs_dev_manage_callback(const struct device *dev,
				      struct gpio_callback *callback, bool set)
{
	struct gpio_silabs_dev_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

/**
 * Handler for both odd and even pin interrupts
 */
static void gpio_silabs_common_isr(const struct device *dev)
{
	struct gpio_silabs_dev_common_data *data = dev->data;
	uint32_t enabled_int, int_status;
	const struct device *port_dev;
	struct gpio_silabs_dev_data *port_data;

	int_status = GPIO->IF;

	for (unsigned int i = 0; int_status && (i < data->count); i++) {
		port_dev = data->ports[i];
		port_data = port_dev->data;
		enabled_int = int_status & port_data->int_enabled_mask;
		if (enabled_int != 0) {
			int_status &= ~enabled_int;
#if defined(_SILICON_LABS_32B_SERIES_2)
			GPIO->IF_CLR = enabled_int;
#else
			GPIO->IFC = enabled_int;
#endif
			gpio_fire_callbacks(&port_data->callbacks, port_dev,
					    enabled_int);
		}
	}
}

static DEVICE_API(gpio, gpio_silabs_dev_driver_api) = {
	.pin_configure = gpio_silabs_dev_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_silabs_dev_get_config,
#endif
	.port_get_raw = gpio_silabs_dev_port_get_raw,
	.port_set_masked_raw = gpio_silabs_dev_port_set_masked_raw,
	.port_set_bits_raw = gpio_silabs_dev_port_set_bits_raw,
	.port_clear_bits_raw = gpio_silabs_dev_port_clear_bits_raw,
	.port_toggle_bits = gpio_silabs_dev_port_toggle_bits,
	.pin_interrupt_configure = gpio_silabs_dev_pin_interrupt_configure,
	.manage_callback = gpio_silabs_dev_manage_callback,
};

static DEVICE_API(gpio, gpio_silabs_dev_common_driver_api) = {
	.manage_callback = gpio_silabs_dev_manage_callback,
};

static int gpio_silabs_common_init(const struct device *dev);

static const struct gpio_silabs_dev_common_config gpio_silabs_dev_common_config = {
};

static struct gpio_silabs_dev_common_data gpio_silabs_dev_common_data;

DEVICE_DT_DEFINE(DT_INST(0, silabs_gpio),
		    gpio_silabs_common_init,
		    NULL,
		    &gpio_silabs_dev_common_data, &gpio_silabs_dev_common_config,
		    PRE_KERNEL_1, CONFIG_gpio_silabs_common_init_PRIORITY,
		    &gpio_silabs_dev_common_driver_api);

static int gpio_silabs_common_init(const struct device *dev)
{
#ifdef CONFIG_SOC_GECKO_DEV_INIT
	// CMU_ClockEnable(cmuClock_GPIO, true);
	sl_gpio_init();
#endif
	gpio_silabs_dev_common_data.count = 0;
	IRQ_CONNECT(GPIO_EVEN_IRQn,
		    DT_IRQ_BY_NAME(DT_INST(0, silabs_gpio), gpio_even, priority),
		    gpio_silabs_common_isr,
		    DEVICE_DT_GET(DT_INST(0, silabs_gpio)), 0);

	IRQ_CONNECT(GPIO_ODD_IRQn,
		    DT_IRQ_BY_NAME(DT_INST(0, silabs_gpio), gpio_odd, priority),
		    gpio_silabs_common_isr,
		    DEVICE_DT_GET(DT_INST(0, silabs_gpio)), 0);

	irq_enable(GPIO_EVEN_IRQn);
	irq_enable(GPIO_ODD_IRQn);

	return 0;
}

#define GPIO_PORT_INIT(idx) \
static int gpio_silabs_dev_port##idx##_init(const struct device *dev); \
\
static const struct gpio_silabs_dev_config gpio_silabs_dev_port##idx##_config = { \
	.common = { \
		.port_pin_mask = (gpio_port_pins_t)(-1), \
	}, \
	.gpio_index = GET_SILABS_GPIO_INDEX(idx), \
}; \
\
static struct gpio_silabs_dev_data gpio_silabs_dev_port##idx##_data; \
\
DEVICE_DT_INST_DEFINE(idx, \
		    gpio_silabs_dev_port##idx##_init, \
		    NULL, \
		    &gpio_silabs_dev_port##idx##_data, \
		    &gpio_silabs_dev_port##idx##_config, \
		    POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, \
		    &gpio_silabs_dev_driver_api); \
\
static int gpio_silabs_dev_port##idx##_init(const struct device *dev) \
{ \
	gpio_silabs_add_port(&gpio_silabs_dev_common_data, dev); \
	return 0; \
}

DT_INST_FOREACH_STATUS_OKAY(GPIO_PORT_INIT)

