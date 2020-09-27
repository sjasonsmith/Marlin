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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#ifndef STM32F446xx
  #error "Oops! Select an STM32F446 environment"
#endif

#define BOARD_INFO_NAME      "NUCLEO-F446"
#define DEFAULT_MACHINE_NAME "Prototype Board"

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  // Not yet supported on F7 hardware
  // #define FLASH_EEPROM_LEVELING
#endif

/**
 * Timer assignments
 * 
 * TIM1 -
 * TIM2 - Hardware PWM (Fan/Heater Pins)
 * TIM3 - Hardware PWM (Servo Pins)
 * TIM4 - STEP_TIMER (Marlin)
 * TIM5 -
 * TIM6 - TIMER_TONE (variant.h)
 * TIM7 - TIMER_SERVO (variant.h)
 * TIM9 - TIMER_SERIAL (platformio.ini)
 * TIM10 - For some reason trips Watchdog when used for SW Serial
 * TIM11 -
 * TIM12 -
 * TIM13 -
 * TIM14 - TEMP_TIMER (Marlin)
 * 
 */
#define STEP_TIMER 4
#define TEMP_TIMER 14


/**
 * These pin assignments are arbitrary and intending for testing purposes.
 * Assignments may not be ideal, and not every assignment has been tested.
 * Proceed at your own risk.
 *
 *                     _____                                    _____
 *      (X_STEP) PC10 | · · | PC11 (X_EN)          (X_MIN) PC9 | · · | PC8 (X_MAX)
 *       (X_DIR) PC12 | · · | PD2 (X_CS)           (Y_MIN) PB8 | · · | PC6 (Y_MAX)
 *                VDD | · · | E5V                  (Z_MIN) PB9 | · · | PC5
 *              BOOT0 | · · | GND                         AVDD | · · | U5V
 *                 NC | · · | NC                           GND | · · | NC
 *                 NC | · · | IOREF                  "led" PA5 | · · | PA12 (LCD_ENABLE)
 *      (Y_STEP) PA13 | · · | RESET                 (MISO) PA6 | · · | PA11 (LCD_RS)
 *       (Y_DIR) PA14 | · · | +3V3                  (MOSI) PA7 | · · | PB12 (LCD_D4)
 *        (Y_EN) PA15 | · · | +5V                   (SDSS) PB6 | · · | NC
 *                GND | · · | GND                    (FAN) PC7 | · · | GND
 *         (Y_CS) PB7 | · · | GND                 (Z_STEP) PA9 | · · | PB2 (Z_EN)
 *       (Z_MAX) PC13 | · · | VIN                  (Z_DIR) PA8 | · · | PB1 (Z_CS)
 *      (BEEPER) PC14 | · · | NC                 (E_STEP) PB10 | · · | PB15 (E_EN)
 *               PC15 | · · | PA0                  (E_DIR) PB4 | · · | PB14 (E_CS)
 *     (TEMP_BED) PH0 | · · | PA1             (SERVO0_PIN) PB5 | · · | PB13 (SERVO1_PIN)
 *       (TEMP_0) PH1 | · · | PA4                   (FAN1) PB3 | · · | AGND
 *               VBAT | · · | PB0            (HEATER_BED) PA10 | · · | PC4 (SCLK)
 *      (BTN_EN1) PC2 | · · | PC1 (BTN_ENC)     (HEATER_0) PA2 | · · | NC
 *      (BTN_EN2) PC3 | · · | PC0 (SD_DETECT)   (HEATER_1) PA3 | · · | NC
 *                     ￣CN7                                    ￣CN10
 */

#define X_MIN_PIN                           PC9
#define X_MAX_PIN                           PC8
#define Y_MIN_PIN                           PB8
#define Y_MAX_PIN                           PC6
#define Z_MIN_PIN                           PB9
#define Z_MAX_PIN                           PC13

//
// Steppers
//
#define X_STEP_PIN                          PC10
#define X_DIR_PIN                           PC12
#define X_ENABLE_PIN                        PC11
#define X_CS_PIN                            PD2

#define Y_STEP_PIN                          PA13
#define Y_DIR_PIN                           PA14
#define Y_ENABLE_PIN                        PA15
#define Y_CS_PIN                            PB7

#define Z_STEP_PIN                          PA9
#define Z_DIR_PIN                           PA8
#define Z_ENABLE_PIN                        PB2
#define Z_CS_PIN                            PB1

#define E0_STEP_PIN                         PB10
#define E0_DIR_PIN                          PB4
#define E0_ENABLE_PIN                       PB15
#define E0_CS_PIN                           PB14

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   X_CS_PIN
  #define X_SERIAL_RX_PIN                   X_CS_PIN

  #define Y_SERIAL_TX_PIN                   Y_CS_PIN
  #define Y_SERIAL_RX_PIN                   Y_CS_PIN

  #define Z_SERIAL_TX_PIN                   Z_CS_PIN
  #define Z_SERIAL_RX_PIN                   Z_CS_PIN

  #define E_SERIAL_TX_PIN                   E0_CS_PIN
  #define E_SERIAL_RX_PIN                   E0_CS_PIN      
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PH1
#define TEMP_BED_PIN                        PH0

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA2  // PWM Capable, TIM2_CH1
#define HEATER_BED_PIN                      PA10 // PWM Capable, TIM2_CH2

#ifndef FAN_PIN
  #define FAN_PIN                           PC7  // PWM Capable, TIM2_CH3
#endif
#define FAN1_PIN                            PB3  // PWM Capable, TIM2_CH4

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN               FAN1_PIN
#endif

//
// Servos
//
#define SERVO0_PIN                          PB5  // PWM Capable, TIM3_CH1
#define SERVO1_PIN                          PB13 // PWM Capable, TIM3_CH2

// SPI for external SD Card (Not entirely sure this will work)
#define SCK_PIN                             PC4
#define MISO_PIN                            PA6
#define MOSI_PIN                            PA7
#define SS_PIN                              PB6
#define SDSS                                PB6

#define LED_PIN                             PA5

//
// LCD / Controller
//
#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define BEEPER_PIN                        PC14  // LCD_BEEPER
  #define BTN_ENC                           PC1   // BTN_ENC
  #define SD_DETECT_PIN                     PC0
  #define LCD_PINS_RS                       PA11  // LCD_RS
  #define LCD_PINS_ENABLE                   PA12  // LCD_EN
  #define LCD_PINS_D4                       PB12  // LCD_D4
  // #define LCD_PINS_D5
  // #define LCD_PINS_D6
  // #define LCD_PINS_D7
  #define BTN_EN1                           PC2   // BTN_EN1
  #define BTN_EN2                           PC3   // BTN_EN2

  #define BOARD_ST7920_DELAY_1  DELAY_NS(125)
  #define BOARD_ST7920_DELAY_2  DELAY_NS(63)
  #define BOARD_ST7920_DELAY_3  DELAY_NS(780)
#endif
