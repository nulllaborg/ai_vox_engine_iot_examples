#include "display.h"

#include <algorithm>
#include <map>

#include "esp_lvgl_port.h"
#include "font_awesome_symbols.h"

LV_FONT_DECLARE(font_awesome_30_1);
LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

Display::Display(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height, bool mirror_x, bool mirror_y)
    : width_(width), height_(height) {
  lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  port_cfg.task_priority = tskIDLE_PRIORITY;
  lvgl_port_init(&port_cfg);

  const lvgl_port_display_cfg_t display_cfg = {
      .io_handle = panel_io,
      .panel_handle = panel,
      .control_handle = nullptr,
      .buffer_size = static_cast<uint32_t>(width * height),
      .double_buffer = false,
      .trans_size = 0,
      .hres = static_cast<uint32_t>(width),
      .vres = static_cast<uint32_t>(height),
      .monochrome = true,
      .rotation =
          {
              .swap_xy = false,
              .mirror_x = mirror_x,
              .mirror_y = mirror_y,
          },
      .color_format = LV_COLOR_FORMAT_UNKNOWN,
      .flags =
          {
              .buff_dma = 1,
              .buff_spiram = 0,
              .sw_rotate = 0,
              .swap_bytes = 0,
              .full_refresh = 0,
              .direct_mode = 0,
          },
  };

  display_ = lvgl_port_add_disp(&display_cfg);
}

Display::~Display() {
  // TODO:
}

void Display::Start() {
  lvgl_port_lock(0);
  auto screen = lv_screen_active();
  lv_obj_set_style_text_font(screen, &font_puhui_14_1, 0);
  lv_obj_set_style_text_color(screen, lv_color_black(), 0);

  /* Container */
  container_ = lv_obj_create(screen);
  lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_all(container_, 0, 0);
  lv_obj_set_style_border_width(container_, 0, 0);
  lv_obj_set_style_pad_row(container_, 0, 0);

  /* Status bar */
  status_bar_ = lv_obj_create(container_);
  lv_obj_set_size(status_bar_, LV_HOR_RES, 16);
  lv_obj_set_style_border_width(status_bar_, 0, 0);
  lv_obj_set_style_pad_all(status_bar_, 0, 0);
  lv_obj_set_style_radius(status_bar_, 0, 0);

  /* Content */
  content_ = lv_obj_create(container_);
  lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_radius(content_, 0, 0);
  lv_obj_set_style_pad_all(content_, 0, 0);
  lv_obj_set_width(content_, LV_HOR_RES);
  lv_obj_set_flex_grow(content_, 1);
  lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_flex_main_place(content_, LV_FLEX_ALIGN_CENTER, 0);

  // 创建左侧固定宽度的容器
  content_left_ = lv_obj_create(content_);
  lv_obj_set_size(content_left_, 32, LV_SIZE_CONTENT);  // 固定宽度32像素
  lv_obj_set_style_pad_all(content_left_, 0, 0);
  lv_obj_set_style_border_width(content_left_, 0, 0);

  emotion_label_ = lv_label_create(content_left_);
  lv_obj_set_style_text_font(emotion_label_, &font_awesome_30_1, 0);
  lv_label_set_text(emotion_label_, FONT_AWESOME_AI_CHIP);
  lv_obj_center(emotion_label_);
  lv_obj_set_style_pad_top(emotion_label_, 8, 0);

  content_right_ = lv_obj_create(content_);
  lv_obj_set_size(content_right_, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(content_right_, 0, 0);
  lv_obj_set_style_border_width(content_right_, 0, 0);
  lv_obj_set_flex_grow(content_right_, 1);
  lv_obj_add_flag(content_right_, LV_OBJ_FLAG_HIDDEN);

  chat_message_label_ = lv_label_create(content_right_);
  lv_label_set_text(chat_message_label_, "");
  lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_set_width(chat_message_label_, width_ - 32);
  lv_obj_set_style_pad_top(chat_message_label_, 14, 0);

  // 延迟一定的时间后开始滚动字幕
  static lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_delay(&a, 1000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_obj_set_style_anim(chat_message_label_, &a, LV_PART_MAIN);
  lv_obj_set_style_anim_duration(chat_message_label_, lv_anim_speed_clamped(60, 300, 60000), LV_PART_MAIN);

  /* Status bar */
  lv_obj_set_flex_flow(status_bar_, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_all(status_bar_, 0, 0);
  lv_obj_set_style_border_width(status_bar_, 0, 0);
  lv_obj_set_style_pad_column(status_bar_, 0, 0);

  network_label_ = lv_label_create(status_bar_);
  lv_label_set_text(network_label_, "");
  lv_obj_set_style_text_font(network_label_, &font_awesome_14_1, 0);

  notification_label_ = lv_label_create(status_bar_);
  lv_obj_set_flex_grow(notification_label_, 1);
  lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text(notification_label_, "");
  lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

  status_label_ = lv_label_create(status_bar_);
  lv_obj_set_flex_grow(status_label_, 1);
  lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);

  mute_label_ = lv_label_create(status_bar_);
  lv_label_set_text(mute_label_, "");
  lv_obj_set_style_text_font(mute_label_, &font_awesome_14_1, 0);

  lvgl_port_unlock();
}

void Display::SetChatMessage(std::string content) {
  std::replace(content.begin(), content.end(), '\n', ' ');

  if (content.empty()) {
    lvgl_port_lock(0);
    lv_obj_add_flag(content_right_, LV_OBJ_FLAG_HIDDEN);
    lvgl_port_unlock();
  } else {
    lvgl_port_lock(0);
    lv_label_set_text(chat_message_label_, content.c_str());
    lv_obj_clear_flag(content_right_, LV_OBJ_FLAG_HIDDEN);
    lvgl_port_unlock();
  }
}

void Display::ShowStatus(const char* status) {
  lvgl_port_lock(0);
  lv_label_set_text(status_label_, status);
  lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);
  lvgl_port_unlock();
}

void Display::SetEmotion(const std::string& emotion) {
  std::map<std::string, const char*> emotion_map = {
      {"neutral", FONT_AWESOME_EMOJI_NEUTRAL},     {"happy", FONT_AWESOME_EMOJI_HAPPY},     {"laughing", FONT_AWESOME_EMOJI_LAUGHING},
      {"funny", FONT_AWESOME_EMOJI_FUNNY},         {"sad", FONT_AWESOME_EMOJI_SAD},         {"angry", FONT_AWESOME_EMOJI_ANGRY},
      {"crying", FONT_AWESOME_EMOJI_CRYING},       {"loving", FONT_AWESOME_EMOJI_LOVING},   {"embarrassed", FONT_AWESOME_EMOJI_EMBARRASSED},
      {"surprised", FONT_AWESOME_EMOJI_SURPRISED}, {"shocked", FONT_AWESOME_EMOJI_SHOCKED}, {"thinking", FONT_AWESOME_EMOJI_THINKING},
      {"winking", FONT_AWESOME_EMOJI_WINKING},     {"cool", FONT_AWESOME_EMOJI_COOL},       {"relaxed", FONT_AWESOME_EMOJI_RELAXED},
      {"delicious", FONT_AWESOME_EMOJI_DELICIOUS}, {"kissy", FONT_AWESOME_EMOJI_KISSY},     {"confident", FONT_AWESOME_EMOJI_CONFIDENT},
      {"sleepy", FONT_AWESOME_EMOJI_SLEEPY},       {"sleepy", FONT_AWESOME_EMOJI_SLEEPY},   {"silly", FONT_AWESOME_EMOJI_SILLY},
      {"confused", FONT_AWESOME_EMOJI_CONFUSED},
  };

  auto it = emotion_map.find(emotion);
  if (it != emotion_map.end()) {
    lvgl_port_lock(0);
    lv_label_set_text(emotion_label_, it->second);
    lvgl_port_unlock();
  } else {
    lvgl_port_lock(0);
    lv_label_set_text(emotion_label_, FONT_AWESOME_EMOJI_NEUTRAL);
    lvgl_port_unlock();
  }
}
