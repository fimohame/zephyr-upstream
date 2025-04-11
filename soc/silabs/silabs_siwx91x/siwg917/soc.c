/*
 * Copyright (c) 2023 Antmicro
 * Copyright (c) 2024 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sw_isr_table.h>

#include "em_device.h"
#include "sl_si91x_power_manager.h"

void soc_early_init_hook(void)
{
	SystemInit();

if (IS_ENABLED(CONFIG_PM)) {
	sl_si91x_power_manager_init();
	sl_si91x_power_manager_set_clock_scaling(SL_SI91X_POWER_MANAGER_PERFORMANCE);
	sl_si91x_power_manager_add_ps_requirement(SL_SI91X_POWER_MANAGER_PS4);
	sl_si91x_power_manager_remove_ps_requirement(SL_SI91X_POWER_MANAGER_PS4);

}

}

/* SiWx917's bootloader requires IRQn 32 to hold payload's entry point address. */
extern void z_arm_reset(void);
Z_ISR_DECLARE_DIRECT(32, 0, z_arm_reset);
