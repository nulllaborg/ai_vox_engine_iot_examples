#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include "ai_vox_engine.h"
#include "ai_vox_observer.h"
#include "audio_input_device_sph0645.h"
#include "display.h"
#include "i2s_std_audio_output_device.h"

#ifndef ARDUINO_ESP32S3_DEV
#error "This example only supports ESP32S3-Dev board."
#endif

#ifndef CONFIG_SPIRAM_MODE_OCT
#error "This example requires PSRAM to OPI PSRAM. Please enable it in Arduino IDE."
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "ssid"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "password"
#endif

namespace {

// WS2812B LedPin Definition
constexpr gpio_num_t kLedPin = GPIO_NUM_46;  // led PIn

// Microphone Pin Definition (SPI Communication)
constexpr gpio_num_t kMicPinBclk = GPIO_NUM_5;
constexpr gpio_num_t kMicPinWs = GPIO_NUM_2;
constexpr gpio_num_t kMicPinDin = GPIO_NUM_4;

// // Speaker Pin Definition (I2S Communication)
constexpr gpio_num_t kSpeakerPinBclk = GPIO_NUM_13;
constexpr gpio_num_t kSpeakerPinWs = GPIO_NUM_14;
constexpr gpio_num_t kSpeakerPinDout = GPIO_NUM_1;

constexpr gpio_num_t kTriggerPin = GPIO_NUM_0;

// Definition of display screen pins (SPI communication)
constexpr gpio_num_t kDisplayBacklightPin = GPIO_NUM_11;
constexpr gpio_num_t kDisplayMosiPin = GPIO_NUM_17;
constexpr gpio_num_t kDisplayClkPin = GPIO_NUM_16;
constexpr gpio_num_t kDisplayDcPin = GPIO_NUM_12;
constexpr gpio_num_t kDisplayRstPin = GPIO_NUM_21;
constexpr gpio_num_t kDisplayCsPin = GPIO_NUM_15;

// // Display screen parameter configuration
constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

const uint32_t kLedNum = 12;  // Led nums

std::shared_ptr<ai_vox::iot::Entity> g_ws2812b_iot_entity;
auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

struct RGBColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  const char* color_name;
};

// Preset color library
const RGBColor color_library[] = {{255, 0, 0, "红色"},
                                  {0, 255, 0, "绿色"},
                                  {0, 0, 255, "蓝色"},
                                  {255, 255, 0, "黄色"},
                                  {255, 0, 255, "紫色"},
                                  {0, 255, 255, "青色"},
                                  {255, 255, 255, "白色"},
                                  {0, 0, 0, "黑色"},
                                  {128, 128, 128, "灰色"},
                                  {128, 0, 0, "深红色"},
                                  {0, 128, 0, "深绿色"},
                                  {0, 0, 128, "深蓝色"},
                                  {255, 165, 0, "橙色"},
                                  {255, 192, 203, "粉色"},
                                  {240, 230, 140, " khaki色"},
                                  {70, 130, 180, "钢蓝色"}};

Adafruit_NeoPixel g_strip(kLedNum, kLedPin, NEO_GRB + NEO_KHZ800);

void InitDisplay() {
  pinMode(kDisplayBacklightPin, OUTPUT);
  analogWrite(kDisplayBacklightPin, 255);

  spi_bus_config_t buscfg{
      .mosi_io_num = kDisplayMosiPin,
      .miso_io_num = GPIO_NUM_NC,
      .sclk_io_num = kDisplayClkPin,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .data_io_default_level = false,
      .max_transfer_sz = kDisplayWidth * kDisplayHeight * sizeof(uint16_t),
      .flags = 0,
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
      .intr_flags = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  esp_lcd_panel_handle_t panel = nullptr;
  // 液晶屏控制IO初始化
  ESP_LOGD(TAG, "Install panel IO");
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kDisplayCsPin;
  io_config.dc_gpio_num = kDisplayDcPin;
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  // 初始化液晶屏驱动芯片
  ESP_LOGD(TAG, "Install LCD driver");
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = kDisplayRstPin;
  panel_config.rgb_ele_order = kDisplayRgbElementOrder;
  panel_config.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

  esp_lcd_panel_reset(panel);

  esp_lcd_panel_init(panel);
  esp_lcd_panel_invert_color(panel, kDisplayInvertColor);
  esp_lcd_panel_swap_xy(panel, kDisplaySwapXY);
  esp_lcd_panel_mirror(panel, kDisplayMirrorX, kDisplayMirrorY);

  g_display = std::make_unique<Display>(
      panel_io, panel, kDisplayWidth, kDisplayHeight, 0, 0, kDisplayMirrorX, kDisplayMirrorY, kDisplaySwapXY);
  g_display->Start();
}

void InitIot() {
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();

  // WS2812B
  // 1. Define the properties of the WS2812B RGB LED ring entity
  std::vector<ai_vox::iot::Property> ws2812b_properties({
      {
          "brightness",                    // property name
          "亮度",                          // property description
          ai_vox::iot::ValueType::kNumber  // property type
      },
      // add more properties as needed
  });

  for (uint32_t i = 1; i <= kLedNum; ++i) {
    std::string prop_name = "color" + std::to_string(i);
    std::string prop_desc = std::to_string(i) + "号灯颜色";

    ws2812b_properties.push_back({prop_name, prop_desc, ai_vox::iot::ValueType::kString});
  }

  // 2.Define the functions for the WS2812B RGB LED ring entity
  std::vector<ai_vox::iot::Function> ws2812b_functions({
      {"SetIndexColor",    // function name
       "设置指定LED颜色",  // function description
       {
           {
               "index",                          // parameter name
               "LED索引(0-总数-1)",              // parameter description
               ai_vox::iot::ValueType::kNumber,  // parameter type
               true                              // parameter required
           },
           {"red", "红色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"green", "绿色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"blue", "蓝色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           // add more parameters as needed
       }},
      {"SetRangeIndexsColor",
       "设置连续LED范围颜色",
       {
           {"start", "起始LED索引(0-总数-1)", ai_vox::iot::ValueType::kNumber, true},
           {"end", "结束LED索引(0-总数-1)", ai_vox::iot::ValueType::kNumber, true},
           {"red", "红色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"green", "绿色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"blue", "蓝色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           // add more parameters as needed
       }},
      {"SetAllIndexsColor",
       "设置所有LED颜色",
       {
           {"red", "红色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"green", "绿色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           {"blue", "蓝色值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           // add more parameters as needed
       }},
      {"SetBrightness",
       "设置亮度",
       {
           {"brightness", "亮度值(0-255)", ai_vox::iot::ValueType::kNumber, true},
           // add more parameters as needed
       }},
      {"Clear", "清除所有LED", {}},
      // add more functions as needed
  });

  // 3.Create the WS2812B RGB LED ring entity
  g_ws2812b_iot_entity = std::make_shared<ai_vox::iot::Entity>("WS2812B",                      // name
                                                               "RGB灯环",                      // description
                                                               std::move(ws2812b_properties),  // properties
                                                               std::move(ws2812b_functions)    // functions
  );

  // 4.Initialize the WS2812B RGB LED ring entity with default values
  g_ws2812b_iot_entity->UpdateState("brightness", 128);
  // 初始化每个LED颜色
  for (uint32_t i = 1; i <= kLedNum; ++i) {
    std::string prop_name = "color" + std::to_string(i);
    g_ws2812b_iot_entity->UpdateState(prop_name, "无");
  }

  // 5.Register the WS2812B RGB LED ring entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_ws2812b_iot_entity);
}

float ColorDistance(uint8_t first_color_r,
                    uint8_t first_color_g,
                    uint8_t first_color_b,
                    uint8_t second_color_r,
                    uint8_t second_color_g,
                    uint8_t second_color_b) {
  int dr = first_color_r - second_color_r;
  int dg = first_color_g - second_color_g;
  int db = first_color_b - second_color_b;
  return sqrt(dr * dr + dg * dg + db * db);
}

// Dynamically resolve color names
std::string GetColorName(uint32_t color) {
  uint8_t first_color_r = (color >> 16) & 0xFF;
  uint8_t first_color_g = (color >> 8) & 0xFF;
  uint8_t first_color_b = color & 0xFF;

  float min_distance = 9999;
  const char* closest_color = "未知颜色";

  for (size_t i = 0; i < sizeof(color_library) / sizeof(color_library[0]); i++) {
    const RGBColor& ref_color = color_library[i];
    float distance = ColorDistance(first_color_r, first_color_g, first_color_b, ref_color.r, ref_color.g, ref_color.b);

    if (distance < min_distance) {
      min_distance = distance;
      closest_color = ref_color.color_name;
    }
  }
  return std::string(closest_color);
}

// ws2812b init funtion
void Ws2812bInit() {
  g_strip.begin();
  g_strip.setBrightness(128);  // Set the default brightness to medium
  g_strip.show();              // Turn off all LEDs during initialization
}

// Implementation of brightness control function
void SetBrightness(uint8_t brightness) {
  g_strip.setBrightness(brightness);
  g_strip.show();
}

// Implementation of Region Filling Function - RGB version
void FillRange(uint16_t startIndex, uint16_t endIndex, uint8_t red, uint8_t green, uint8_t blue) {
  // Ensure that the starting and ending Indexs are within the valid range
  if (startIndex >= kLedNum || endIndex >= kLedNum || startIndex > endIndex) {
    return;
  }

  uint32_t color = g_strip.Color(red, green, blue);

  // Implementation of Region Filling Function
  for (uint16_t i = startIndex; i <= endIndex; i++) {
    g_strip.setPixelColor(i, color);
  }
}

#ifdef PRINT_HEAP_INFO_INTERVAL
void PrintMemInfo() {
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    printf("SPIRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_INTERNAL) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    printf("IRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_DEFAULT) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    printf("DRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }
}
#endif
}  // namespace

void setup() {
  Serial.begin(115200);

  Ws2812bInit();

  InitDisplay();
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
    g_display->SetChatMessage(Display::Role::kSystem, "No SPIRAM available, please check your board.");
    while (true) {
      printf("No SPIRAM available, please check your board.\n");
      delay(1000);
    }
  }

  g_display->ShowStatus("Wifi connecting...");

  WiFi.useStaticBuffers(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    printf("Connecting to WiFi, ssid: %s, password: %s\n", WIFI_SSID, WIFI_PASSWORD);
  }

  printf("WiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
  g_display->ShowStatus("Wifi connected");

  InitIot();

  auto audio_input_device = std::make_shared<AudioInputDeviceSph0645>(kMicPinBclk, kMicPinWs, kMicPinDin);
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetTrigger(kTriggerPin);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  ai_vox_engine.Start(audio_input_device, g_audio_output_device);
  g_display->ShowStatus("AI Vox Engine starting...");
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  const auto events = g_observer->PopEvents();
  for (auto& event : events) {
    if (auto activation_event = std::get_if<ai_vox::Observer::ActivationEvent>(&event)) {
      printf("activation code: %s, message: %s\n", activation_event->code.c_str(), activation_event->message.c_str());
      g_display->ShowStatus("激活设备");
      g_display->SetChatMessage(Display::Role::kSystem, activation_event->message);
    } else if (auto state_changed_event = std::get_if<ai_vox::Observer::StateChangedEvent>(&event)) {
      switch (state_changed_event->new_state) {
        case ai_vox::ChatState::kIdle: {
          printf("Idle\n");
          break;
        }
        case ai_vox::ChatState::kIniting: {
          printf("Initing...\n");
          g_display->ShowStatus("初始化");
          break;
        }
        case ai_vox::ChatState::kStandby: {
          printf("Standby\n");
          g_display->ShowStatus("待命");
          break;
        }
        case ai_vox::ChatState::kConnecting: {
          printf("Connecting...\n");
          g_display->ShowStatus("连接中...");
          break;
        }
        case ai_vox::ChatState::kListening: {
          printf("Listening...\n");
          g_display->ShowStatus("聆听中");
          break;
        }
        case ai_vox::ChatState::kSpeaking: {
          printf("Speaking...\n");
          g_display->ShowStatus("说话中");
          break;
        }
        default: {
          break;
        }
      }
    } else if (auto emotion_event = std::get_if<ai_vox::Observer::EmotionEvent>(&event)) {
      printf("emotion: %s\n", emotion_event->emotion.c_str());
      g_display->SetEmotion(emotion_event->emotion);
    } else if (auto chat_message_event = std::get_if<ai_vox::Observer::ChatMessageEvent>(&event)) {
      switch (chat_message_event->role) {
        case ai_vox::ChatRole::kAssistant: {
          printf("role: assistant, content: %s\n", chat_message_event->content.c_str());
          g_display->SetChatMessage(Display::Role::kAssistant, chat_message_event->content);
          break;
        }
        case ai_vox::ChatRole::kUser: {
          printf("role: user, content: %s\n", chat_message_event->content.c_str());
          g_display->SetChatMessage(Display::Role::kUser, chat_message_event->content);
          break;
        }
      }
    } else if (auto iot_message_event = std::get_if<ai_vox::Observer::IotMessageEvent>(&event)) {
      printf("IOT message: %s, function: %s\n", iot_message_event->name.c_str(), iot_message_event->function.c_str());
      for (const auto& [key, value] : iot_message_event->parameters) {
        if (std::get_if<bool>(&value)) {
          printf("key: %s, value: %s\n", key.c_str(), std::get<bool>(value) ? "true" : "false");
        } else if (std::get_if<std::string>(&value)) {
          printf("key: %s, value: %s\n", key.c_str(), std::get<std::string>(value).c_str());
        } else if (std::get_if<int64_t>(&value)) {
          printf("key: %s, value: %lld\n", key.c_str(), std::get<int64_t>(value));
        }
      }

      if (iot_message_event->name == "WS2812B") {
        if (iot_message_event->function == "SetIndexColor") {  // Specify the color of a certain light
          int64_t index = 0;
          int64_t red = 0;
          int64_t green = 0;
          int64_t blue = 0;
          bool has_index = false;
          bool has_red = false;
          bool has_green = false;
          bool has_blue = false;

          if (const auto it = iot_message_event->parameters.find("index"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              index = std::get<int64_t>(it->second);
              has_index = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);
              has_red = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);
              has_green = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);
              has_blue = true;
            }
          }

          if (has_index && has_red && has_green && has_blue) {
            printf("Set LED %lld to color RGB(%lld, %lld, %lld)\n", index, red, green, blue);

            // Send operation instructions to WS2812B
            if ((index - 1) < kLedNum) {
              char prop_name[8];
              snprintf(prop_name, sizeof(prop_name), "color%lld", index + 1);
              std::string color_name = GetColorName(g_strip.Color(red, green, blue));

              g_ws2812b_iot_entity->UpdateState(prop_name, color_name);
              g_strip.setPixelColor(index, g_strip.Color(red, green, blue));
            }
            g_strip.show();
          }
        } else if (iot_message_event->function == "SetRangeIndexsColor") {  // Set the color from one light to another
          int64_t start = 0;
          int64_t end = 0;
          int64_t red = 0;
          int64_t green = 0;
          int64_t blue = 0;
          bool has_start = false;
          bool has_end = false;
          bool has_red = false;
          bool has_green = false;
          bool has_blue = false;
          if (const auto it = iot_message_event->parameters.find("start"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              start = std::get<int64_t>(it->second);
              has_start = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("end"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              end = std::get<int64_t>(it->second);
              has_end = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);
              has_red = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);
              has_green = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);
              has_blue = true;
            }
          }

          if (has_start && has_end && has_red && has_green && has_blue) {
            // ensure   start <= end
            if (start > end) {
              std::swap(start, end);
            }

            printf("Set LEDs from %lld to %lld to color RGB(%lld, %lld, %lld)\n", start, end, red, green, blue);

            // Send operation instructions to WS2812B
            FillRange(start, end, red, green, blue);
            std::string color_name = GetColorName(g_strip.Color(red, green, blue));
            for (uint32_t i = start + 1; i <= end + 1; ++i) {
              std::string prop_name = "color" + std::to_string(i);
              g_ws2812b_iot_entity->UpdateState(prop_name, color_name);
            }
            g_strip.show();
          }
        } else if (iot_message_event->function == "SetAllIndexsColor") {  // Unified setting of all lights
          int64_t red = 0;
          int64_t green = 0;
          int64_t blue = 0;
          bool has_red = false;
          bool has_green = false;
          bool has_blue = false;

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);
              has_red = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);
              has_green = true;
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);
              has_blue = true;
            }
          }

          if (has_red && has_green && has_blue) {
            printf("Set all LEDs to color RGB(%lld, %lld, %lld)\n", red, green, blue);

            // Send operation instructions to WS2812B
            uint32_t color = g_strip.Color(red, green, blue);
            g_strip.fill(color);
            std::string color_name = GetColorName(g_strip.Color(red, green, blue));
            for (uint32_t i = 1; i <= kLedNum; ++i) {
              std::string prop_name = "color" + std::to_string(i);
              g_ws2812b_iot_entity->UpdateState(prop_name, color_name);
            }
            g_strip.show();
          }
        } else if (iot_message_event->function == "SetBrightness") {  // Set Brightness
          if (const auto it = iot_message_event->parameters.find("brightness"); it != iot_message_event->parameters.end()) {
            auto brightness = it->second;
            if (std::get_if<int64_t>(&brightness)) {
              printf("Set LED brightness: %lld\n", std::get<int64_t>(brightness));

              // Send operation instructions to WS2812B
              //   g_strip.setBrightness((uint8_t)brightness);
              // 假设brightness总是存储整数
              if (auto int_ptr = std::get_if<long long>(&brightness)) {
                g_strip.setBrightness(static_cast<uint8_t>(*int_ptr));
              } else {
                g_strip.setBrightness(128);  // 默认值
              }
              g_strip.show();

              g_ws2812b_iot_entity->UpdateState("brightness", std::get<int64_t>(brightness));
            }
          }
        } else if (iot_message_event->function == "Clear") {  // Turn off all lights
          printf("Clear all LEDs\n");

          g_strip.clear();
          g_strip.show();
          for (uint32_t i = 1; i <= kLedNum; ++i) {
            std::string prop_name = "color" + std::to_string(i);
            g_ws2812b_iot_entity->UpdateState(prop_name, "无");
          }
        }
      }
    }
  }

  taskYIELD();
}