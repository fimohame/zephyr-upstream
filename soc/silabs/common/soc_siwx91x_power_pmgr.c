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

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/*
 * Power state map:
 * PM_STATE_RUNTIME_IDLE: SL_SI91X_POWER_MANAGER_STANDBY (PS4)
 * PM_STATE_SUSPEND_TO_IDLE: SL_SI91X_POWER_MANAGER_SLEEP (PS4 Sleep)
 */
bool is_interrupt_enabled(uint32_t irq_num) {
    return NVIC_GetEnableIRQ((IRQn_Type)irq_num);
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(substate_id);

    sl_power_state_t power_state = sl_si91x_power_manager_get_current_state();
    uint32_t power_state_1;

    /* FIXME: The kernel disables interrupts using BASEPRI. 
     * This prevents waking up from interrupts with priority not equal to 0.
     * Workaround: Use BASEPRI or other methods to handle interrupts as recommended by ARM.
     */

    switch (state) {
    case PM_STATE_RUNTIME_IDLE:
	//printf("Transitioning to Standby mode)\r\n");
        power_state_1 = SL_SI91X_POWER_MANAGER_STANDBY;
        break;
    case PM_STATE_SUSPEND_TO_IDLE:
	//printf("Transitioning to Sleep mode\r\n");
        power_state_1 = SL_SI91X_POWER_MANAGER_SLEEP;
        break;
    default:
	printf("Unsupported power state: %d\r\n", state);

        /* Clear BASEPRI before returning */
        irq_unlock(0);
        return;
    }

    /* Perform power state requirements */
    //printf("Configuring power state requirements for %d\r\n", power_state);
        // sl_si91x_power_manager_add_ps_requirement(power_state);
        // sl_si91x_power_manager_remove_ps_requirement(power_state);
	/* Set PRIMASK */
	__disable_irq();

        irq_unlock(0);

        // NVIC_SetPriority(22, 0);
		// NVIC_EnableIRQ(22);
		// if (is_interrupt_enabled(22)) {
		// 	// Interrupt is enabled
		// 	printf("SysRTC interrupt is enabled.\n");
		// } else {
		// 	// Interrupt is disabled
		// 	printf("SysRTC interrupt is disabled.\n");
		// }
        if (!sl_si91x_power_manager_is_ok_to_sleep())
		{
            //printf("Slnet init... \r\n");
		}else{
            printf("Initial power state: %d\r\n", power_state);
            printf("SoC entering power state: %d\r\n", state);
            printf("Configuring power state requirements for %d\r\n", power_state_1);
		
        if (power_state_1 == SL_SI91X_POWER_MANAGER_STANDBY) {
            printf("Entering Standby mode... \r\n");
            sl_si91x_power_manager_standby(); 
        } else {
            printf("Entering Sleep mode...\r\n");
            // printf("%u\n",*((volatile uint32_t*)MCU_FSM->MCU_FSM_SLEEP_CTRLS_AND_WAKEUP_MODE));
            RSI_PS_SetWkpSources(SYSRTC_BASED_WAKEUP);
            printf("%u\n",RSI_PS_GetWkpSources);
			sli_si91x_config_clocks_to_mhz_rc();
            sl_si91x_power_manager_sleep(); 
            
        }
        printf("SoC leaving power state %d\r\n", power_state_1);
        
	}
   	/* Clear PRIMASK */
	__enable_irq();
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);
    LOG_DBG("Exited low-power mode; performing post-ops...");
}