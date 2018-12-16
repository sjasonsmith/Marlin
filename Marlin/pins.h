#ifndef PINS_H
#define PINS_H

#if MOTHERBOARD == 1  // 
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif

#define X_STEP_PIN         65
#define X_DIR_PIN          64
#define X_ENABLE_PIN       66
#define X_MIN_PIN          47
#define X_MAX_PIN          -1   //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define Y_STEP_PIN         23 // A6
#define Y_DIR_PIN          22 // A0
#define Y_ENABLE_PIN       24
#define Y_MIN_PIN          -1
#define Y_MAX_PIN          48   //15

#define Z_STEP_PIN         31 // A2
#define Z_DIR_PIN          32 // A6
#define Z_ENABLE_PIN       30 // A1
#define Z_MIN_PIN          49 //46
#define Z_MAX_PIN          -1

#define Z2_STEP_PIN         28
#define Z2_DIR_PIN          63
#define Z2_ENABLE_PIN       29

#define Z3_STEP_PIN         14
#define Z3_DIR_PIN          15
#define Z3_ENABLE_PIN       39

#define FILAMENT_E0         44 // 3RD pin in connector
#define FILAMENT_E1         42 // 3RD pin in connector
#define DOOR_PIN            45

#define E0_STEP_PIN         34
#define E0_DIR_PIN          35
#define E0_ENABLE_PIN       33

#define E1_STEP_PIN         37
#define E1_DIR_PIN          40
#define E1_ENABLE_PIN       36

#define LED_PIN            13
#define FAN_PIN            10
#define PS_ON_PIN          -1
#define KILL_PIN           -1
#define SOL1_PIN           16
#define SOL2_PIN           7

#define HEATER_0_PIN       9
#define HEATER_1_PIN       8 // 12 
#define HEATER_2_PIN       -1 // 13
#define TEMP_0_PIN         13 //D27   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_1_PIN         15 // 1
#define TEMP_2_PIN         -1 // 2
#define HEATER_BED_PIN     6 // 14/15
#define TEMP_BED_PIN       14 // 1,2 or I2C
/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27) 28 (29) (30) (31)  */

#endif

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN, TEMP_BED_PIN }
#endif
