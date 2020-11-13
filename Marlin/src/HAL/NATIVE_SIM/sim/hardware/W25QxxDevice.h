#pragma once

#include "../user_interface.h"

#include "SPISlavePeripheral.h"

/**
 * SPI Flash W25Qxx device
 **/
 #define SPI_FLASH_IMAGE "./spi_flash.bin"
 #ifndef SPI_FLASH_IMAGE
   #error "You need set SPI_FLASH_IMAGE to keep SPI FLASH data."
 #endif

class W25QxxDevice: public SPISlavePeripheral {
public:
  W25QxxDevice(pin_type clk, pin_type mosi, pin_type miso, pin_type cs, size_t flash_size) : SPISlavePeripheral(clk, mosi, miso, cs), flash_size(flash_size) {
    // read current data
    fp = fopen(SPI_FLASH_IMAGE, "rb+");
    data = new uint8_t[flash_size];
    memset(data, 0xFF, flash_size);
    fread(data, 1, flash_size, fp);
    fclose(fp);
  }
  virtual ~W25QxxDevice() {
  };

  size_t flash_size;

  void update() {};
  void ui_callback(UiWindow* window) {};

  void onByteReceived(uint8_t _byte) override;
  void onEndTransaction() override;
  void onRequestedDataReceived(uint8_t token, uint8_t* _data, size_t count) override;

  FILE *fp = nullptr;
  uint8_t *data;
  int32_t currentAddress = -1;
};
