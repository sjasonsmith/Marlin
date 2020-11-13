#pragma once

#include <fstream>

#include "window.h"
#include "user_interface.h"

#include "hardware/Heater.h"
#include "hardware/ST7920Device.h"
#include "hardware/print_bed.h"

#include "visualisation.h"

#include <string>
#include <imgui.h>
#include <implot.h>
#include "user_interface.h"

struct GraphWindow : public UiWindow {
  bool hovered = false;
  bool focused = false;
  GLuint texture_id = 0;
  float aspect_ratio = 0.0f;

  template<class... Args>
  GraphWindow(std::string name, GLuint texture_id, float aratio, Args... args) : UiWindow(name, args...), texture_id{texture_id}, aspect_ratio(aratio) {}
  void show() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{2, 2});
    ImGui::Begin((char *)name.c_str(), nullptr);
    auto size = ImGui::GetContentRegionAvail();
    size.y = size.x / aspect_ratio;
    ImGui::Image((ImTextureID)(intptr_t)texture_id, size, ImVec2(0,0), ImVec2(1,1));
    hovered = ImGui::IsItemHovered();
    focused = ImGui::IsWindowFocused();
    ImGui::End();
    ImGui::PopStyleVar();
  }
};

#include "src/inc/MarlinConfig.h"

class Simulation {
public:

  Simulation() :  hotend(HEATER_0_PIN, TEMP_0_PIN),
                  bed_heater(HEATER_BED_PIN, TEMP_BED_PIN),
                  display(LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, KILL_PIN) {}

  void process_event(SDL_Event& e) {}

  void update() {
    hotend.update();
    bed_heater.update();
    display.update();
  }

  void ui_callback(UiWindow*) {

  }

  Heater hotend;
  Heater bed_heater;
  ST7920Device display;
  Visualisation vis;
};

class Application {
public:
  Application();
  ~Application();

  void update();
  void render();

  bool active = true;
  Window window;
  UserInterface user_interface;
  Simulation sim;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  std::ifstream input_file;
};