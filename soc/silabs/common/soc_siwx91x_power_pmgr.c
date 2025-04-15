/*
 * Copyright (c) 2018, Piotr Mienkowski
 * Copyright (c) 2023, Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include "sl_si91x_power_manager.h"
#include "sli_si91x_clock_manager.h"
#include "sl_rsi_utility.h"
#include "sl_si91x_m4_ps.h"

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

void sli_si91x_m4_ta_wakeup_configurations(void);

/*
 * Power state map:
 * PM_STATE_RUNTIME_IDLE: SL_SI91X_POWER_MANAGER_STANDBY (PS4)
 * PM_STATE_SUSPEND_TO_IDLE: SL_SI91X_POWER_MANAGER_SLEEP (PS4 Sleep)
 */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	sl_power_state_t current_power_state = sl_si91x_power_manager_get_current_state();
	uint32_t target_power_state = 0;
	uint32_t frontend_switch_control = 0;

	/* FIXME: The kernel disables interrupts using BASEPRI.
	 * This prevents waking up from interrupts with priority not equal to 0.
	 * Workaround: Use BASEPRI or other methods to handle interrupts as recommended by ARM.
	 */

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		target_power_state = SL_SI91X_POWER_MANAGER_STANDBY;
		break;

	case PM_STATE_SUSPEND_TO_IDLE:
		target_power_state = SL_SI91X_POWER_MANAGER_SLEEP;
		break;

	default:
		/* Unsupported power state */
		printf("Error: Unsupported power state %d\r\n", state);
		irq_unlock(0);
		break;
	}

	/* Set PRIMASK */
	__disable_irq();
	/* Set BASEPRI to 0. */
	irq_unlock(0);

	if (!sl_si91x_power_manager_is_ok_to_sleep()) {
		// Device is not ready to sleep; perform necessary actions if required.
	} else {
		printf("Initial power state: %d\r\n", current_power_state);
		printf("SoC entering power state: %d\r\n", state);
		printf("Configuring power state %d\r\n", target_power_state);

		if (target_power_state == SL_SI91X_POWER_MANAGER_STANDBY) {
			printf("Entering Standby mode... \r\n");
			sl_si91x_power_manager_standby();
		} else {
			printf("Entering Sleep mode...\r\n");
			if (sli_si91x_config_clocks_to_mhz_rc() != 0) {
				printf("Error: Failed to configure clocks for sleep mode\r\n");
			} else {
#if SL_WIFI_COMPONENT_INCLUDED
				/* Check if SOC is in PS2 state. If so, skip writing to PLL
				 * registers. */
				if (!(M4_ULP_SLP_STATUS_REG & ULP_MODE_SWITCHED_NPSS)) {
					if (!sl_si91x_is_device_initialized()) {
						printf("Error: Device is not initialized\r\n");
					} else {
						sli_si91x_xtal_turn_off_request_from_m4_to_TA();
					}
				}
#endif
				if (sl_si91x_power_manager_sleep() != 0) {
					printf("Error: Failed to transition to sleep mode\r\n");
				} else {
					if (frontend_switch_control != 0) {
						sli_si91x_configure_wireless_frontend_controls(
							frontend_switch_control);
					}
					sl_si91x_host_clear_sleep_indicator();
					sli_si91x_m4_ta_wakeup_configurations();
				}
			}
		}

		printf("SoC leaving power state %d\r\n", target_power_state);
	}

	/* Clear PRIMASK */
	__enable_irq();
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}
