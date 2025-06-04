#include <WiFi.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include <condition_variable>
#include <mutex>

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

constexpr gpio_num_t kButtonPin = GPIO_NUM_46;  // Button pin
constexpr gpio_num_t kLedPin = GPIO_NUM_43;     // Led pin

// Microphone Pin Definition (SPI Communication)
constexpr gpio_num_t kMicPinBclk = GPIO_NUM_5;
constexpr gpio_num_t kMicPinWs = GPIO_NUM_2;
constexpr gpio_num_t kMicPinDin = GPIO_NUM_4;

// Speaker Pin Definition (I2S Communication)
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

constexpr gpio_num_t kWs2812LedPin = GPIO_NUM_41;  // WS2812B LED light bar data pin

// Display screen parameter configuration
constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

std::shared_ptr<ai_vox::iot::Entity> g_button_led_iot_entity;
auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

int g_temp_blink_count = 0;
bool g_is_parsing_blink = false;
uint32_t g_last_button_time = 0;
uint8_t g_button_press_count = 0;

// Key state enumeration
enum class ButtonAction { kSingleClick, kDoubleClick, kLongPress };

// Key status
enum class ButtonState { IDLEE, PRESSED, RELEASED, CLICKED, DOUBLE_CLICKED, LONG_PRESSED };

struct LedAction {
  bool is_blink = false;  // Is it a flashing action
  int blink_count = 0;    // Flashing frequency (only valid when is-blink is true)
  int interval_ms = 200;  // Flashing interval (milliseconds, default 200ms)
  bool state = false;     // Switch status (only valid when is-blink is false)
};

std::map<ButtonAction, LedAction> g_button_rules = {
    {ButtonAction::kSingleClick, {false, 0, 200, true}},   // Default click to turn on the light
    {ButtonAction::kDoubleClick, {false, 0, 200, false}},  // Default double-click to turn off lights
    {ButtonAction::kLongPress, {true, 3, 200, false}}      // Default long press flashes 3 times
};

ButtonState g_button_state = ButtonState::IDLEE;
unsigned long g_last_press_time = 0;
unsigned long g_last_release_time = 0;
int g_click_count = 0;
bool g_long_press_triggered = false;  // New logo added to distinguish long press release

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

bool g_pause_event_processing = false;
std::mutex g_event_mutex;
std::condition_variable g_event_cv;

void InitIot() {
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();

  // LED
  // 1.Define the properties for the LED entity
  std::vector<ai_vox::iot::Property> led_properties({
      {
          "state",                       // property name
          "LED灯开关状态",               // property description
          ai_vox::iot::ValueType::kBool  // property type
      },
      // add more properties as needed
  });

  // 2.Define the functions for the LED entity
  std::vector<ai_vox::iot::Function> led_functions({
      {"TurnOn",     // function name
       "打开LED灯",  // function description
       {
           // no parameters
       }},
      {"TurnOff",    // function name
       "关闭LED灯",  // function description
       {
           // no parameters
       }},
      // add more functions as needed
  });

  // 3.Create the LED entity
  g_button_led_iot_entity = std::make_shared<ai_vox::iot::Entity>("Led",                      // name
                                                                  "LED灯",                    // description
                                                                  std::move(led_properties),  // properties
                                                                  std::move(led_functions)    // functions
  );

  // 4.Initialize the LED entity with default values
  g_button_led_iot_entity->UpdateState("state", false);

  // 5.Register the LED entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_button_led_iot_entity);
}

void HandleButtonAction(ButtonAction action) {
  LedAction led_action_config = g_button_rules[action];
  if (led_action_config.is_blink) {
    if (led_action_config.blink_count == -1) {  // Infinite flicker (X times per second)

      // Start independent tasks to achieve continuous flashing (requires FreeRTOS tasks)
      xTaskCreatePinnedToCore(
          [](void* param) {
            LedAction* config = static_cast<LedAction*>(param);
            while (digitalRead(kButtonPin) == LOW) {  // Flashing when the button is continuously pressed
              digitalWrite(kLedPin, HIGH);
              delay(config->interval_ms);
              digitalWrite(kLedPin, LOW);
              delay(config->interval_ms);
            }
            vTaskDelete(nullptr);  // Stop the task after releasing the button
          },
          "LongPressBlinkTask",
          2048,
          &led_action_config,
          1,
          nullptr,
          1);
    } else {  // Limited number of flashes
      for (int i = 0; i < led_action_config.blink_count; i++) {
        digitalWrite(kLedPin, HIGH);
        delay(led_action_config.interval_ms);
        digitalWrite(kLedPin, LOW);
        delay(led_action_config.interval_ms);
      }
    }
    printf("LED %s闪烁（%d次/秒，间隔%dms）\n",
           led_action_config.blink_count == -1 ? "持续" : "单次",
           led_action_config.blink_count == -1 ? 1000 / (2 * led_action_config.interval_ms) : led_action_config.blink_count,
           led_action_config.interval_ms);
  } else {
    // Switching
    digitalWrite(kLedPin, led_action_config.state);
    g_button_led_iot_entity->UpdateState("state", led_action_config.state);
    printf("LED %s\n", led_action_config.state ? "开启" : "关闭");
  }
}

// Key detection function
void CheckButtonStatus() {
  static bool last_state = HIGH;  // Initialize to high level (assuming using pull-up input)
  bool current_state = digitalRead(kButtonPin);

  // Detecting changes in status
  if (current_state != last_state) {
    delay(20);                                // eliminate dithering
    current_state = digitalRead(kButtonPin);  // Read and confirm again

    if (current_state != last_state) {
      last_state = current_state;

      if (current_state == LOW) {  // Key Down
        g_last_press_time = millis();
        g_button_state = ButtonState::PRESSED;
        g_long_press_triggered = false;  // Reset long press flag
        printf("按键检测：按下\n");
      } else {  // Press button release
        g_last_release_time = millis();
        g_button_state = ButtonState::RELEASED;

        // Only increase the click count without triggering a long press
        if (!g_long_press_triggered) {
          g_click_count++;
          printf("按键检测：释放，点击计数 = %d\n", g_click_count);
        }
      }
    }
  }

  // Handle different button states
  switch (g_button_state) {
    case ButtonState::PRESSED:
      // Long press detection
      if (millis() - g_last_press_time > 500) {  // Long press threshold 500ms
        HandleButtonAction(ButtonAction::kLongPress);
        printf("按键检测：长按触发\n");
        g_button_state = ButtonState::LONG_PRESSED;  // Use new status to indicate long press in
        g_long_press_triggered = true;
      }
      break;

    case ButtonState::LONG_PRESSED:
      // Long press to hold the status, wait for release
      if (current_state == HIGH) {  // Long press and release
        g_button_state = ButtonState::IDLEE;
        g_click_count = 0;  // Long press to release and reset click count
        printf("按键检测：长按释放\n");
      }
      break;

    case ButtonState::RELEASED:
      // If it is a long press release, it has already been processed in the LONG-PRESSED state
      if (!g_long_press_triggered) {
        // Check if it is the first strike of double clicking
        if (g_click_count == 1) {
          // Set double-click time window (within 300ms)
          if (millis() - g_last_release_time > 300) {
            HandleButtonAction(ButtonAction::kSingleClick);
            printf("按键检测：单击触发\n");
            g_button_state = ButtonState::IDLEE;
            g_click_count = 0;
          }
        }
        // Double click to confirm
        else if (g_click_count == 2) {
          HandleButtonAction(ButtonAction::kDoubleClick);
          printf("按键检测：双击触发\n");
          g_button_state = ButtonState::IDLEE;
          g_click_count = 0;
        }
      }
      break;

    default:
      break;
  }
}

// --------------------- Voice command parsing ---------------------
// Improved voice command parsing function - supports multi command parsing
void ParseVoiceCommand(const std::string& content) {
  // Define delimiter：,、。、；etc
  const std::vector<std::string> delimiters = {"，", "。", "；", ",", ".", ";"};

  // Store segmented sub instructions
  std::vector<std::string> commands;

  // Try dividing the content by delimiter first
  size_t start_pos = 0;
  bool has_more = true;

  while (has_more) {
    size_t min_pos = std::string::npos;
    std::string used_delim;

    // Find the nearest delimiter
    for (const auto& delim : delimiters) {
      size_t pos = content.find(delim, start_pos);
      if (pos != std::string::npos && (pos < min_pos || min_pos == std::string::npos)) {
        min_pos = pos;
        used_delim = delim;
      }
    }

    if (min_pos != std::string::npos) {
      // Extract sub instructions
      commands.push_back(content.substr(start_pos, min_pos - start_pos));
      start_pos = min_pos + used_delim.length();
    } else {
      // No more separators, add remaining parts
      commands.push_back(content.substr(start_pos));
      has_more = false;
    }
  }

  // Process each sub instruction
  for (const auto& cmd : commands) {
    // Skip empty instructions
    if (cmd.empty()) continue;

    // Processing a single instruction
    printf("处理指令: %s\n", cmd.c_str());

    ButtonAction action = ButtonAction::kSingleClick;
    LedAction new_action;
    bool is_button = false;

    // Analyze key actions
    if (cmd.find("按一下") != std::string::npos) {
      action = ButtonAction::kSingleClick;
      is_button = true;
    } else if (cmd.find("按两下") != std::string::npos) {
      action = ButtonAction::kDoubleClick;
      is_button = true;
    } else if (cmd.find("长按") != std::string::npos) {
      action = ButtonAction::kLongPress;
      is_button = true;
    } else {
      printf("未识别的按键动作: %s\n", cmd.c_str());
      continue;  // Skip current instruction
    }

    // Analyze operation type (flashing/on/off)
    if (cmd.find("闪") != std::string::npos) {
      new_action.is_blink = true;

      // Extract the numerical part as the number of flashes
      size_t num_start = cmd.find_first_of("0123456789");
      if (num_start != std::string::npos) {
        size_t num_end = cmd.find_first_not_of("0123456789", num_start);
        if (num_end == std::string::npos) num_end = cmd.length();

        std::string number_str = cmd.substr(num_start, num_end - num_start);

        // Manually verify if the string is a valid number
        bool is_number = true;
        for (char c : number_str) {
          if (!std::isdigit(c)) {
            is_number = false;
            break;
          }
        }

        if (is_number && !number_str.empty()) {
          new_action.blink_count = std::stoi(number_str);

          // Analysis interval (optional)
          size_t interval_pos = cmd.find("间隔");
          if (interval_pos != std::string::npos) {
            // Find the number after 'interval' and 'milliseconds'
            size_t ms_pos = cmd.find("毫秒", interval_pos);
            if (ms_pos != std::string::npos) {
              // Find the number between 'interval' and 'milliseconds'
              size_t int_start = cmd.find_first_of("0123456789", interval_pos);
              if (int_start != std::string::npos && int_start < ms_pos) {
                size_t int_end = cmd.find_first_not_of("0123456789", int_start);
                if (int_end == std::string::npos || int_end > ms_pos) int_end = ms_pos;

                std::string interval_str = cmd.substr(int_start, int_end - int_start);

                // Manually verify if the string is a valid number
                bool is_interval_number = true;
                for (char c : interval_str) {
                  if (!std::isdigit(c)) {
                    is_interval_number = false;
                    break;
                  }
                }

                if (is_interval_number && !interval_str.empty()) {
                  new_action.interval_ms = std::stoi(interval_str);
                }
              }
            }
          }
        } else {
          new_action.blink_count = 3;  // Default 3 times
        }
      } else {
        new_action.blink_count = 3;  // Default 3 times
      }
    } else if (cmd.find("开灯") != std::string::npos) {
      new_action = {false, 0, 200, true};
    } else if (cmd.find("关灯") != std::string::npos) {
      new_action = {false, 0, 200, false};
    } else {
      printf("未识别的操作类型: %s\n", cmd.c_str());
      continue;  // Skip current instruction
    }

    // Analyze the format of 'X times per second'
    if (cmd.find("每秒") != std::string::npos && cmd.find("次") != std::string::npos) {
      new_action.is_blink = true;

      // Locate the positions of 'per second' and 'times'
      size_t per_second_pos = cmd.find("每秒");
      size_t ci_pos = cmd.find("次", per_second_pos);

      if (per_second_pos != std::string::npos && ci_pos != std::string::npos && ci_pos > per_second_pos) {
        // Extract the portion between 'per second' and 'times'
        std::string middle = cmd.substr(per_second_pos + 2, ci_pos - per_second_pos - 2);

        // Find the middle number
        size_t num_start = middle.find_first_of("0123456789");
        if (num_start != std::string::npos) {
          size_t num_end = middle.find_first_not_of("0123456789", num_start);
          if (num_end == std::string::npos) num_end = middle.length();

          std::string number_str = middle.substr(num_start, num_end - num_start);

          // Manually verify if the string is a valid number
          bool is_number = true;
          for (char c : number_str) {
            if (!std::isdigit(c)) {
              is_number = false;
              break;
            }
          }

          if (is_number && !number_str.empty()) {
            int times_per_second = std::stoi(number_str);
            if (times_per_second > 0) {
              new_action.blink_count = -1;                           // Infinite flashing (marked with negative numbers)
              new_action.interval_ms = 1000 / times_per_second / 2;  // Half cycle interval
            }
          }
        }
      }
    }

    // update rule
    g_button_rules[action] = new_action;
    printf("已设置：%s → %s%d次（间隔%dms）\n",
           action == ButtonAction::kSingleClick   ? "单击"
           : action == ButtonAction::kDoubleClick ? "双击"
                                                  : "长按",
           new_action.is_blink ? "闪烁" : (new_action.state ? "开启" : "关闭"),
           new_action.blink_count,
           new_action.interval_ms);
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

  pinMode(kLedPin, OUTPUT);
  pinMode(kButtonPin, INPUT_PULLUP);  // Initialize control button
  digitalWrite(kLedPin, LOW);

  InitDisplay();  // Screen initialization

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

  CheckButtonStatus();

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

          // Pause event handling
          {
            std::lock_guard<std::mutex> lock(g_event_mutex);
            g_pause_event_processing = true;
          }

          ParseVoiceCommand(chat_message_event->content);

          // Recovery event handling
          {
            std::lock_guard<std::mutex> lock(g_event_mutex);
            g_pause_event_processing = false;
          }
          g_event_cv.notify_one();  // Notification event loop continues
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

      if (iot_message_event->name == "Led") {
        if (iot_message_event->function == "TurnOn") {
          printf("turn on led\n");
          digitalWrite(kLedPin, HIGH);
          g_button_led_iot_entity->UpdateState("state", false);  //  Note: Must UpdateState after change the device state
        } else if (iot_message_event->function == "TurnOff") {
          printf("turn off led\n");
          digitalWrite(kLedPin, LOW);
          g_button_led_iot_entity->UpdateState("state", false);  // Note: Must UpdateState after change the device state
        }
      }
    }
  }
  taskYIELD();
}