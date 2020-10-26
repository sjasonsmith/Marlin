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

/**
 * feature/z_stepper_align.h
 */

#include "../inc/MarlinConfig.h"

constexpr xy_pos_t invalid_align_xy[NUM_Z_STEPPER_DRIVERS] = {{NAN, NAN}, {NAN, NAN}};

class ZStepperAlign {
  public:
    // Values are only stored if explicitly declared in header or set with M422.
    static xy_pos_t xy[NUM_Z_STEPPER_DRIVERS];
    static bool store_xy_pos;

    #if ENABLED(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)
      static xy_pos_t stepper_xy[NUM_Z_STEPPER_DRIVERS];
    #endif

    static void reset_to_default();
    static bool are_probe_points_valid();
};

extern ZStepperAlign z_stepper_align;
