#ifdef __PLAT_NATIVE_SIM__

#include <mutex>
#include <fstream>
#include <cmath>
#include <random>
#include "Gpio.h"

#include <GL/glew.h>
#include <GL/gl.h>

#include "XPT2046Device.h"
#include "../../tft/xpt2046.h"

void XPT2046Device::onByteReceived(uint8_t _byte) {
  SPISlavePeripheral::onByteReceived(_byte);
  switch (_byte) {
    //TODO: touch hold
    case XPT2046_Z1:
      if (dirty) {
        setResponse16(XPT2046_Z1_THRESHOLD); // respond that we have data to send
      }
      else {
        setResponse16(0);
      }
      break;

    case XPT2046_X:
      setResponse16(lastClickX);
      break;

    case XPT2046_Y:
      setResponse16(lastClickY);
      break;

    default:
      break;
  }
}


void XPT2046Device::ui_callback(UiWindow* window) {
  if (ImGui::IsWindowFocused() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    lastClickX = ImGui::GetIO().MousePos.x;
    lastClickY = ImGui::GetIO().MousePos.y;
    dirty = true;
    // printf("click x: %d, y: %d\n", lastClickX, lastClickY);
  }
}

#endif
