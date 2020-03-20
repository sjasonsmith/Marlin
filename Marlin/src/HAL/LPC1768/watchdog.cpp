/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifdef TARGET_LPC1768

#include "../../inc/MarlinConfig.h"

#if ENABLED(USE_WATCHDOG)

#include <chip.h>
#include "watchdog.h"

void watchdog_init() {
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    // We enable the watchdog timer, but only for the interrupt.

    // Configure WDT to only trigger an interrupt
    // Disable WDT interrupt (just in case, to avoid triggering it!)
    NVIC_DisableIRQ(WDT_IRQn);

    // We NEED memory barriers to ensure Interrupts are actually disabled!
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    __DSB();
    __ISB();

    // Configure WDT to only trigger an interrupt
    // Initialize WDT with the given parameters
    WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_INT_ONLY);

    // Configure and enable WDT interrupt.
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0); // Use highest priority, so we detect all kinds of lockups
    NVIC_EnableIRQ(WDT_IRQn);
  #else
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
    Chip_WWDT_Init(LPC_WWDT);
  #endif
  Chip_WWDT_SetTimeOut(LPC_WWDT, WDT_TIMEOUT);
  Chip_WWDT_Start(LPC_WWDT);
}

void HAL_watchdog_refresh() {
  Chip_WWDT_Feed(LPC_WWDT);
  #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
    TOGGLE(LED_PIN);  // heartbeat indicator
  #endif
}

// Timeout state
bool watchdog_timed_out() { return 0 != (Chip_WWDT_GetStatus(LPC_WWDT) & WWDT_WDMOD_WDTOF); }
void watchdog_clear_timeout_flag() { Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF); }

#endif // USE_WATCHDOG
#endif // TARGET_LPC1768
