/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfig.h"

// ------------------------
// Local defines
// ------------------------


// Default timer priorities. Override by specifying alternate priorities in the board pins file.
// The TONE timer is not present here, as it currently cannot be set programmatically. It is set
// by defining TIM_IRQ_PRIO in the variant.h or platformio.ini file, which adjusts the default
// priority for STM32 HardwareTimer objects.
#define SWSERIAL_TIMER_IRQ_PRIO_DEFAULT  1 // Requires tight bit timing to communicate reliably with TMC drivers
#define SERVO_TIMER_IRQ_PRIO_DEFAULT     1 // Requires tight PWM timing to control a BLTouch reliably
#define STEP_TIMER_IRQ_PRIO_DEFAULT      2
#define TEMP_TIMER_IRQ_PRIO_DEFAULT     14 // Low priority avoids interference with other hardware and timers

#ifndef STEP_TIMER_IRQ_PRIO
  #define STEP_TIMER_IRQ_PRIO STEP_TIMER_IRQ_PRIO_DEFAULT
#endif
#ifndef TEMP_TIMER_IRQ_PRIO
  #define TEMP_TIMER_IRQ_PRIO TEMP_TIMER_IRQ_PRIO_DEFAULT
#endif
#if HAS_TMC_SW_SERIAL
  #include <SoftwareSerial.h>
  #ifndef SWSERIAL_TIMER_IRQ_PRIO
    #define SWSERIAL_TIMER_IRQ_PRIO SWSERIAL_TIMER_IRQ_PRIO_DEFAULT
  #endif
#endif
#if HAS_SERVOS
  #include "Servo.h"
  #ifndef SERVO_TIMER_IRQ_PRIO
    #define SERVO_TIMER_IRQ_PRIO SERVO_TIMER_IRQ_PRIO_DEFAULT
  #endif
#endif
#if ENABLED(SPEAKER)
  // Ensure the default timer priority is somewhere between the STEP and TEMP priorities.
  // The STM32 framework defaults to interrupt 14 for all timers. This should be increased so that
  // timing-sensitive operations such as speaker output are not impacted by the long-running
  // temperature ISR. This must be defined in the platformio.ini file or the board's variant.h,
  // so that it will be consumed by framework code.
  #if !(TIM_IRQ_PRIO > STEP_TIMER_IRQ_PRIO && TIM_IRQ_PRIO < TEMP_TIMER_IRQ_PRIO)
    #error "Default timer interrupt priority is unspecified or set to a value which may degrade performance."
  #endif
#endif

#ifdef STM32F0xx
  #define MCU_STEP_TIMER 16
  #define MCU_TEMP_TIMER 17
#elif defined(STM32F1xx)
  #define MCU_STEP_TIMER  4
  #define MCU_TEMP_TIMER  2
#elif defined(STM32F401xC) || defined(STM32F401xE)
  #define MCU_STEP_TIMER  9
  #define MCU_TEMP_TIMER 10
#elif defined(STM32F4xx) || defined(STM32F7xx)
  #define MCU_STEP_TIMER  6           // STM32F401 has no TIM6, TIM7, or TIM8
  #define MCU_TEMP_TIMER 14           // TIM7 is consumed by Software Serial if used.
#endif

#ifndef HAL_TIMER_RATE
  #define HAL_TIMER_RATE GetStepperTimerClkFreq()
#endif

#ifndef STEP_TIMER
  #define STEP_TIMER MCU_STEP_TIMER
#endif
#ifndef TEMP_TIMER
  #define TEMP_TIMER MCU_TEMP_TIMER
#endif

#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)
#define TEMP_TIMER_DEV _TIMER_DEV(TEMP_TIMER)

#define __TIMER_IRQ_NAME(X) TIM##X##_IRQn
#define _TIMER_IRQ_NAME(X) __TIMER_IRQ_NAME(X)
#define STEP_TIMER_IRQ_NAME _TIMER_IRQ_NAME(STEP_TIMER)
#define TEMP_TIMER_IRQ_NAME _TIMER_IRQ_NAME(TEMP_TIMER)

// ------------------------
// Private Variables
// ------------------------

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { nullptr };

// ------------------------
// Public functions
// ------------------------

uint32_t GetStepperTimerClkFreq() {
  // Timer input clocks vary between devices, and in some cases between timers on the same device.
  // Retrieve at runtime to ensure device compatibility. Cache result to avoid repeated overhead.
  static uint32_t clkfreq = timer_instance[STEP_TIMER_NUM]->getTimerClkFreq();
  return clkfreq;
}

// frequency is in Hertz
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!HAL_timer_initialized(timer_num)) {
    switch (timer_num) {
      case STEP_TIMER_NUM: // STEPPER TIMER - use a 32bit timer if possible
        timer_instance[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        /* Set the prescaler to the final desired value.
         * This will change the effective ISR callback frequency but when
         * HAL_timer_start(timer_num=0) is called in the core for the first time
         * the real frequency isn't important as long as, after boot, the ISR
         * gets called with the correct prescaler and count register. So here
         * we set the prescaler to the correct, final value and ignore the frequency
         * asked. We will call back the ISR in 1 second to start at full speed.
         *
         * The proper fix, however, would be a correct initialization OR a
         * HAL_timer_change(const uint8_t timer_num, const uint32_t frequency)
         * which changes the prescaler when an IRQ frequency change is needed
         * (for example when steppers are turned on)
         */

        timer_instance[timer_num]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally
        timer_instance[timer_num]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
        break;
      case TEMP_TIMER_NUM: // TEMP TIMER - any available 16bit timer
        timer_instance[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        // The prescale factor is computed automatically for HERTZ_FORMAT
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
    }

    // Disable preload. Leaving it default-enabled can cause the timer to stop if it happens
    // to exit the ISR after the start time for the next interrupt has already passed.
    timer_instance[timer_num]->setPreloadEnable(false);

    HAL_timer_enable_interrupt(timer_num);

    // Start the timer.
    timer_instance[timer_num]->resume(); // First call to resume() MUST follow the attachInterrupt()

    // This is fixed in Arduino_Core_STM32 1.8.
    // These calls can be removed and replaced with
    // timer_instance[timer_num]->setInterruptPriority
    switch (timer_num) {
      case STEP_TIMER_NUM:
        timer_instance[timer_num]->setInterruptPriority(STEP_TIMER_IRQ_PRIO, 0);
        break;
      case TEMP_TIMER_NUM:
        timer_instance[timer_num]->setInterruptPriority(TEMP_TIMER_IRQ_PRIO, 0);
        break;
    }
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !timer_instance[timer_num]->hasInterrupt()) {
    switch (timer_num) {
      case STEP_TIMER_NUM:
        timer_instance[timer_num]->attachInterrupt(Step_Handler);
        break;
      case TEMP_TIMER_NUM:
        timer_instance[timer_num]->attachInterrupt(Temp_Handler);
        break;
    }
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num)) timer_instance[timer_num]->detachInterrupt();
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) && timer_instance[timer_num]->hasInterrupt();
}

void SetTimerInterruptPriorities() {
  TERN_(HAS_TMC_SW_SERIAL, SoftwareSerial::setInterruptPriority(SWSERIAL_TIMER_IRQ_PRIO, 0));
  TERN_(HAS_SERVOS, libServo::setInterruptPriority(SERVO_TIMER_IRQ_PRIO, 0));
}

// This is a terrible hack to replicate the behavior used in the framework's SoftwareSerial.cpp
// to choose a serial timer. It will select TIM7 on most boards used by Marlin, but this is more
// resiliant to new MCUs which may not have a TIM7. Best practice is to explicitly specify
// TIMER_SERIAL to avoid relying on framework selections which may not be predictable.
#if !defined(TIMER_SERIAL)
  #if defined (TIM18_BASE)
    #define TIMER_SERIAL TIM18
  #elif defined (TIM7_BASE)
    #define TIMER_SERIAL TIM7
  #elif defined (TIM6_BASE)
    #define TIMER_SERIAL TIM6
  #elif defined (TIM22_BASE)
    #define TIMER_SERIAL TIM22
  #elif defined (TIM21_BASE)
    #define TIMER_SERIAL TIM21
  #elif defined (TIM17_BASE)
    #define TIMER_SERIAL TIM17
  #elif defined (TIM16_BASE)
    #define TIMER_SERIAL TIM16
  #elif defined (TIM15_BASE)
    #define TIMER_SERIAL TIM15
  #elif defined (TIM14_BASE)
    #define TIMER_SERIAL TIM14
  #elif defined (TIM13_BASE)
    #define TIMER_SERIAL TIM13
  #elif defined (TIM11_BASE)
    #define TIMER_SERIAL TIM11
  #elif defined (TIM10_BASE)
    #define TIMER_SERIAL TIM10
  #elif defined (TIM12_BASE)
    #define TIMER_SERIAL TIM12
  #elif defined (TIM19_BASE)
    #define TIMER_SERIAL TIM19
  #elif defined (TIM9_BASE)
    #define TIMER_SERIAL TIM9
  #elif defined (TIM5_BASE)
    #define TIMER_SERIAL TIM5
  #elif defined (TIM4_BASE)
    #define TIMER_SERIAL TIM4
  #elif defined (TIM3_BASE)
    #define TIMER_SERIAL TIM3
  #elif defined (TIM2_BASE)
    #define TIMER_SERIAL TIM2
  #elif defined (TIM20_BASE)
    #define TIMER_SERIAL TIM20
  #elif defined (TIM8_BASE)
    #define TIMER_SERIAL TIM8
  #elif defined (TIM1_BASE)
    #define TIMER_SERIAL TIM1
  #else
    #error No suitable timer found for SoftwareSerial, define TIMER_SERIAL in variant.h
  #endif
#endif




#undef PinMap_PWM
#undef HAL_ADC_MODULE_ENABLED
#undef HAL_DAC_MODULE_ENABLED
#undef HAL_I2C_MODULE_ENABLED
#undef HAL_I2C_MODULE_ENABLED
#undef HAL_UART_MODULE_ENABLED
#undef HAL_SPI_MODULE_ENABLED
#undef HAL_CAN_MODULE_ENABLED
#undef HAL_QSPI_MODULE_ENABLED
#undef HAL_PCD_MODULE_ENABLED

typedef struct {
  PinName pin;
  uintptr_t peripheral;
  int function;
} HackyPinMap;

#define PinMap HackyPinMap
#undef TIM1
#define TIM1 TIM1_BASE
#undef TIM2
#define TIM2 TIM2_BASE
#undef TIM3
#define TIM3 TIM3_BASE
#undef TIM4
#define TIM4 TIM4_BASE
#undef TIM5
#define TIM5 TIM5_BASE
#undef TIM6
#define TIM6 TIM6_BASE
#undef TIM7
#define TIM7 TIM7_BASE
#undef TIM8
#define TIM8 TIM8_BASE
#undef TIM9
#define TIM9 TIM9_BASE
#undef TIM10
#define TIM10 TIM10_BASE
#undef TIM11
#define TIM11 TIM11_BASE
#undef TIM12
#define TIM12 TIM12_BASE
#undef TIM13
#define TIM13 TIM13_BASE
#undef TIM14
#define TIM14 TIM14_BASE
#undef TIM15
#define TIM15 TIM15_BASE
#undef TIM16
#define TIM16 TIM16_BASE
#undef TIM17
#define TIM17 TIM17_BASE
#undef TIM18
#define TIM18 TIM18_BASE
#undef TIM19
#define TIM19 TIM19_BASE
#undef TIM20
#define TIM20 TIM20_BASE
#undef STM_PIN_DATA_EXT
#define STM_PIN_DATA_EXT(...)

#undef WEAK
#define WEAK constexpr

namespace PinHack{
  #include <PeripheralPins.c>

  // constexpr auto timer = PinHack::PinMap_PWM[0].peripheral;

  // static constexpr uintptr_t get_timer_for_pin(PinName pin, const HackyPinMap* begin, const HackyPinMap* end) {
  //   if (begin == end) return 0;
  //   return pin == begin->pin ? begin->peripheral : get_timer_for_pin(pin, begin + 1, end);
  // }

  static constexpr uintptr_t get_timer_for_pin(PinName pin) {
    for (const auto& pinmap : PinHack::PinMap_PWM)
      if (pin == pinmap.pin)
        return pinmap.peripheral;
    return 0;
  }

  static constexpr uintptr_t get_timer_for_pin(int pin) {
    return get_timer_for_pin(PinName(pin));
  }

  // constexpr auto PA_Timer = get_timer_for_pin(PA_3, &PinHack::PinMap_PWM[0], &PinHack::PinMap_PWM[0] + COUNT(PinHack::PinMap_PWM));

  // static_assert(PA_Timer != 0, "Pin is not PWM-capable");
}


// Place all timers used into an array, then recursively check for duplicates during compilation.
// This does not currently account for timers used for PWM, such as for fans.
// Timers are actually pointers. Convert to integers to simplify constexpr logic.
static constexpr uintptr_t timers_in_use[] = {
  uintptr_t(TEMP_TIMER_DEV),  // Override in pins file
  uintptr_t(STEP_TIMER_DEV),  // Override in pins file
  #if HAS_TMC_SW_SERIAL
    uintptr_t(TIMER_SERIAL),  // Set in variant.h, or as a define in platformio.h if not present in variant.h
  #endif
  #if ENABLED(SPEAKER)
    uintptr_t(TIMER_TONE),    // Set in variant.h, or as a define in platformio.h if not present in variant.h
  #endif
  #if HAS_SERVOS
    uintptr_t(TIMER_SERVO),   // Set in variant.h, or as a define in platformio.h if not present in variant.h
  #endif
  };

// Fan Timers are handled separately. Conflicts between fans is fine, as many timers have multiple
// PWM output channels. No conflicts may exist with the timers in timers_in_use.
static constexpr uintptr_t fan_timers_in_use[] = {
  #if DISABLED(FAN_SOFT_PWM)
    #if HAS_FAN0
      PinHack::get_timer_for_pin(FAN0_PIN),
    #endif
    #if HAS_FAN1
      PinHack::get_timer_for_pin(FAN1_PIN),
    #endif
    #if HAS_FAN2
      PinHack::get_timer_for_pin(FAN2_PIN),
    #endif    
    #if HAS_FAN3
      PinHack::get_timer_for_pin(FAN3_PIN),
    #endif    
    #if HAS_FAN4
      PinHack::get_timer_for_pin(FAN4_PIN),
    #endif    
    #if HAS_FAN5
      PinHack::get_timer_for_pin(FAN5_PIN),
    #endif    
    #if HAS_FAN6
      PinHack::get_timer_for_pin(FAN6_PIN),
    #endif    
    #if HAS_FAN7
      PinHack::get_timer_for_pin(FAN7_PIN),
    #endif    
  #endif
};

static constexpr bool verify_no_duplicate_timers() {
  LOOP_L_N(i, COUNT(timers_in_use)) {
    LOOP_S_L_N(j, i + 1, COUNT(timers_in_use))
      if (timers_in_use[i] == timers_in_use[j]) return false;
    LOOP_L_N(j, COUNT(fan_timers_in_use))
      if (timers_in_use[i] == fan_timers_in_use[j]) return false;
  }
  return true;
}

// If this assertion fails at compile time, review the timers_in_use array. If default_envs is
// defined properly in platformio.ini, VS Code can evaluate the array when hovering over it,
// making it easy to identify the conflicting timers.
static_assert(verify_no_duplicate_timers(), "One or more timer conflict detected");

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
