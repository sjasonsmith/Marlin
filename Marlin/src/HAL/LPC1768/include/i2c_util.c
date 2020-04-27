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

/**
 * HAL_LPC1768/include/i2c_util.c
 */

#ifdef TARGET_LPC1768

#include "i2c_util.h"

#define U8G_I2C_OPT_FAST 16  // from u8g.h

#ifdef __cplusplus
  extern "C" {
#endif

void configure_i2c(const uint8_t clock_option) {
  /**
   * Init I2C pin connect
   */
  const uint32_t mode = 0;
  const uint8_t port = 0;
  #if I2C_MASTER_ID == 0
    const uint8_t func = 1;
    const uint8_t pin = 27;     // SDA0 / D57  AUX-1 ... SCL0 / D58  AUX-1
  #elif I2C_MASTER_ID == 1
    const uint8_t func = 3;
    const uint8_t pin = 0;      // SDA1 / D20 SCA ... SCL1 / D21 SCL
  #elif I2C_MASTER_ID == 2
    const uint8_t func = 2;
    const uint8_t pin = 10;     // SDA2 / D38  X_ENABLE_PIN ... SCL2 / D55  X_DIR_PIN
  #endif
  // Pins are immediately adjacent to each other
  Chip_IOCON_PinMux(LPC_IOCON, port, pin, mode, func);
  Chip_IOCON_PinMux(LPC_IOCON, port, pin + 1, mode, func);

  // Initialize I2C peripheral
  Chip_I2C_Init(I2C_MASTER_ID);
  Chip_I2C_SetClockRate(I2C_MASTER_ID, (clock_option & U8G_I2C_OPT_FAST) ? 400000: 100000);  // LCD data rates

  // Enable Master I2C operation
  // TODO: This might no longer be needed.
  //I2C_Cmd(I2CDEV_M, I2C_MASTER_MODE, ENABLE);
}

#ifdef __cplusplus
  }
#endif

#endif // TARGET_LPC1768
