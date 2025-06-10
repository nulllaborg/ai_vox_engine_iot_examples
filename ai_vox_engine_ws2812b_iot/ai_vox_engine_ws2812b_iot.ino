#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <cJSON.h>
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

constexpr uint32_t kLedNum = 12;  // Led nums

std::shared_ptr<ai_vox::iot::Entity> g_ws2812b_iot_entity;
auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

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
          "亮度(0-255)",                   // property description
          ai_vox::iot::ValueType::kNumber  // property type
      },
      {
          "LedNums",                       // property name
          "灯的数量",                      // property description
          ai_vox::iot::ValueType::kNumber  // property type
      },
      // add more properties as needed
  });

  for (uint32_t i = 1; i <= kLedNum; ++i) {
    std::string property_name = "color" + std::to_string(i);
    std::string property_describe = std::to_string(i) + "号灯颜色";

    ws2812b_properties.push_back({property_name, property_describe, ai_vox::iot::ValueType::kString});
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
  g_ws2812b_iot_entity->UpdateState("LedNums", kLedNum);

  for (uint32_t i = 1; i <= kLedNum; ++i) {
    std::string property_name = "color" + std::to_string(i);
    g_ws2812b_iot_entity->UpdateState(property_name, R"({"red":0,"green":0,"blue":0})");
  }

  // 5.Register the WS2812B RGB LED ring entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_ws2812b_iot_entity);
}

void InitWs2812b() {
  g_strip.begin();
  g_strip.setBrightness(128);  // Set the default brightness to medium
  g_strip.show();              // Turn off all LEDs during initialization
}

std::string ConvertRGBToJsonString(const int64_t& red, const int64_t& green, const int64_t& blue) {
  cJSON* root = cJSON_CreateObject();
  if (!root) {
    return "{}";
  }

  if (red >= 0 && red <= 255) {
    cJSON_AddNumberToObject(root, "red", static_cast<int>(red));
  } else {
    cJSON_AddNumberToObject(root, "red", 0);
  }

  if (green >= 0 && green <= 255) {
    cJSON_AddNumberToObject(root, "green", static_cast<int>(green));
  } else {
    cJSON_AddNumberToObject(root, "green", 0);
  }

  if (blue >= 0 && blue <= 255) {
    cJSON_AddNumberToObject(root, "blue", static_cast<int>(blue));
  } else {
    cJSON_AddNumberToObject(root, "blue", 0);
  }

  char* json_str = cJSON_PrintUnformatted(root);
  std::string result = json_str ? json_str : "{}";

  if (json_str) {
    cJSON_free(json_str);
  }
  cJSON_Delete(root);

  return result;
}

// Implementation of Region Filling Function - RGB version
void FillRangeLed(
    const uint16_t& start_index, const uint16_t& end_index, const uint8_t& red, const uint8_t& green, const uint8_t& blue) {
  // Ensure that the starting and ending Indexs are within the valid range
  if (start_index >= kLedNum || end_index >= kLedNum || start_index > end_index) {
    return;
  }

  uint32_t color_value = g_strip.Color(red, green, blue);

  // Implementation of Region Filling Function
  for (uint16_t i = start_index; i <= end_index; i++) {
    g_strip.setPixelColor(i, color_value);
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

  InitWs2812b();

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

          std::optional<int64_t> index;
          std::optional<int64_t> red;
          std::optional<int64_t> green;
          std::optional<int64_t> blue;
          bool valid = true;

          if (const auto it = iot_message_event->parameters.find("index"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              index = std::get<int64_t>(it->second);

              // Parameter validation
              if (!index.has_value()) {
                printf("Error: lamp number acquisition failed, please check.\n");
                valid = false;
              } else if (*index < 0 || *index > kLedNum) {
                printf("Error:lamp number  is out of range (0-%lld), got: %lld\n", kLedNum - 1, *index);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);

              // red parameter validation
              if (!red.has_value()) {
                printf("Error: Missing required parameter 'red'.\n");
                valid = false;
              } else if (*red < 0 || *red > 255) {
                printf("Error: 'red' out of range (0-255), got: %lld\n", *red);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);

              // green parameter validation
              if (!green.has_value()) {
                printf("Error: Missing required parameter 'green'.\n");
                valid = false;
              } else if (*green < 0 || *green > 255) {
                printf("Error: 'green' out of range (0-255), got: %lld\n", *green);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);

              // blue parameter validation
              if (!blue.has_value()) {
                printf("Error: Missing required parameter 'blue'.\n");
                valid = false;
              } else if (*blue < 0 || *blue > 255) {
                printf("Error: 'blue' out of range (0-255), got: %lld\n", *blue);
                valid = false;
              }
            }
          }

          if (valid) {
            printf("Set LED %lld to color RGB(%lld, %lld, %lld)\n", *index, *red, *green, *blue);

            std::string property_name = "color" + std::to_string(*index + 1);

            std::string color_str = ConvertRGBToJsonString(*red, *green, *blue);
            g_ws2812b_iot_entity->UpdateState(property_name, color_str);
            g_strip.setPixelColor(index.value_or(0), g_strip.Color(*red, *green, *blue));

            g_strip.show();
          }
        } else if (iot_message_event->function == "SetRangeIndexsColor") {  // Set the color from one light to another
          std::optional<int64_t> start;
          std::optional<int64_t> end;
          std::optional<int64_t> red;
          std::optional<int64_t> green;
          std::optional<int64_t> blue;
          bool valid = true;

          if (const auto it = iot_message_event->parameters.find("start"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              start = std::get<int64_t>(it->second);

              // start arameter validation
              if (!start.has_value()) {
                printf("Error: start lamp number acquisition failed, please check.\n");
                valid = false;
              } else if (*start < 0 || *start > kLedNum) {
                printf("Error: start lamp number is out of range (0-%lld), got: %lld\n", kLedNum - 1, *start);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("end"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              end = std::get<int64_t>(it->second);

              // end parameter validation
              if (!end.has_value()) {
                printf("Error: end lamp number acquisition failed, please check.\n");
                valid = false;
              } else if (*end < 0 || *end > kLedNum) {
                printf("Error: end lamp number is out of range (0-%lld), got: %lld\n", kLedNum - 1, *end);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);
              // red parameter validation
              if (!red.has_value()) {
                printf("Error: Missing required parameter 'red'.\n");
                valid = false;
              } else if (*red < 0 || *red > 255) {
                printf("Error: 'red' out of range (0-255), got: %lld\n", *red);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);
              // green parameter validation
              if (!green.has_value()) {
                printf("Error: Missing required parameter 'green'.\n");
                valid = false;
              } else if (*green < 0 || *green > 255) {
                printf("Error: 'green' out of range (0-255), got: %lld\n", *green);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);
              // blue parameter validation
              if (!blue.has_value()) {
                printf("Error: Missing required parameter 'blue'.\n");
                valid = false;
              } else if (*blue < 0 || *blue > 255) {
                printf("Error: 'blue' out of range (0-255), got: %lld\n", *blue);
                valid = false;
              }
            }
          }

          if (valid) {
            // ensure   start <= end
            if (*start > *end) {
              start.swap(end);
            }

            printf("Set LEDs from %lld to %lld to color RGB(%lld, %lld, %lld)\n", *start, *end, *red, *green, *blue);

            // Send operation instructions to WS2812B
            FillRangeLed(*start, *end, *red, *green, *blue);
            std::string color_str = ConvertRGBToJsonString(*red, *green, *blue);
            for (uint32_t i = *start + 1; i <= *end + 1; ++i) {
              std::string property_name = "color" + std::to_string(i);
              g_ws2812b_iot_entity->UpdateState(property_name, color_str);
            }
            g_strip.show();
          }
        } else if (iot_message_event->function == "SetAllIndexsColor") {  // Unified setting of all lights
          std::optional<int64_t> red;
          std::optional<int64_t> green;
          std::optional<int64_t> blue;
          bool valid = true;

          if (const auto it = iot_message_event->parameters.find("red"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              red = std::get<int64_t>(it->second);
              // red parameter validation
              if (!red.has_value()) {
                printf("Error: Missing required parameter 'red'.\n");
                valid = false;
              } else if (*red < 0 || *red > 255) {
                printf("Error: 'red' out of range (0-255), got: %lld\n", *red);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("green"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              green = std::get<int64_t>(it->second);
              // green parameter validation
              if (!green.has_value()) {
                printf("Error: Missing required parameter 'green'.\n");
                valid = false;
              } else if (*green < 0 || *green > 255) {
                printf("Error: 'green' out of range (0-255), got: %lld\n", *green);
                valid = false;
              }
            }
          }

          if (const auto it = iot_message_event->parameters.find("blue"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              blue = std::get<int64_t>(it->second);
              // blue parameter validation
              if (!blue.has_value()) {
                printf("Error: Missing required parameter 'blue'.\n");
                valid = false;
              } else if (*blue < 0 || *blue > 255) {
                printf("Error: 'blue' out of range (0-255), got: %lld\n", *blue);
                valid = false;
              }
            }
          }

          if (valid) {
            printf("Set all LEDs to color RGB(%lld, %lld, %lld)\n", *red, *green, *blue);

            // Send operation instructions to WS2812B
            uint32_t color = g_strip.Color(*red, *green, *blue);
            g_strip.fill(color);
            std::string color_str = ConvertRGBToJsonString(*red, *green, *blue);
            for (uint32_t i = 1; i <= kLedNum; ++i) {
              std::string property_name = "color" + std::to_string(i);
              g_ws2812b_iot_entity->UpdateState(property_name, color_str);
            }
            g_strip.show();
          }
        } else if (iot_message_event->function == "SetBrightness") {  // Set Brightness
          if (const auto it = iot_message_event->parameters.find("brightness"); it != iot_message_event->parameters.end()) {
            std::optional<int64_t> brightness;
            bool valid = true;

            if (std::get_if<int64_t>(&it->second)) {
              brightness = std::get<int64_t>(it->second);

              // brightness parameter validation
              if (!brightness.has_value()) {
                printf("Error: Missing required parameter 'brightness'.\n");
                valid = false;
              } else if (*brightness < 0 || *brightness > 255) {
                printf("Error: 'brightness' out of range (0-255), got %lld\n", *brightness);
                valid = false;
              }
            }
            if (valid) {
              printf("Set LED brightness: %lld\n", *brightness);

              if (auto int_ptr = *brightness) {
                g_strip.setBrightness(static_cast<uint8_t>(int_ptr));
              } else {
                g_strip.setBrightness(128);
              }
              g_strip.show();

              g_ws2812b_iot_entity->UpdateState("brightness", *brightness);
            }
          }
        } else if (iot_message_event->function == "Clear") {  // Turn off all lights
          printf("Clear all LEDs\n");

          g_strip.clear();
          g_strip.show();
          for (uint32_t i = 1; i <= kLedNum; ++i) {
            std::string property_name = "color" + std::to_string(i);
            g_ws2812b_iot_entity->UpdateState(property_name, R"({"red":0,"green":0,"blue":0})");
          }
        }
      }
    }
  }

  taskYIELD();
}