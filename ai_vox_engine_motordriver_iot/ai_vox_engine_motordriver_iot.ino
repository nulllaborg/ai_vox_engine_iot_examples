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

// Define motor control pins
constexpr gpio_num_t kMotorAIn1 = GPIO_NUM_46;  // IN1
constexpr gpio_num_t kMotorAIn2 = GPIO_NUM_48;  // IN2
constexpr gpio_num_t kMotorBIn1 = GPIO_NUM_47;  // IN3
constexpr gpio_num_t kMotorBIn2 = GPIO_NUM_44;  // IN4

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

// Display screen parameter configuration
constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

constexpr bool kForword = true;   // Indicate forward rotation
constexpr bool kReverse = false;  // Indicate reversal

std::shared_ptr<ai_vox::iot::Entity> g_motor_driver_iot_entity;
auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

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

  g_display = std::make_unique<Display>(panel_io, panel, kDisplayWidth, kDisplayHeight, 0, 0, kDisplayMirrorX, kDisplayMirrorY, kDisplaySwapXY);
  g_display->Start();
}

void InitIot() {
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();

  // MotorDriver IoT Entity Definition

  // 1.Define the properties for the motor driver entity
  std::vector<ai_vox::iot::Property> motor_driver_properties({{
                                                                  "speed_a",                       // property name
                                                                  "电机A的当前速度(0-255)",        // property description
                                                                  ai_vox::iot::ValueType::kNumber  // property type: number, string or bool
                                                              },
                                                              {"direction_a", "电机A的当前方向", ai_vox::iot::ValueType::kBool},
                                                              {"speed_b", "电机B的当前速度(0-255)", ai_vox::iot::ValueType::kNumber},
                                                              {"direction_b", "电机B的当前方向", ai_vox::iot::ValueType::kBool},
                                                              {"status_a", "电机A驱动状态", ai_vox::iot::ValueType::kString},
                                                              {"status_b", "电机B驱动状态", ai_vox::iot::ValueType::kString}});

  // 2.Define the functions for the motor driver entity
  std::vector<ai_vox::iot::Function> motor_driver_functions({
      {"SetMotorA",      // function name
       "设置电机A参数",  // function description
       {{
            "speed",                          // parameter name
            "电机速度(0-255)",                // parameter description
            ai_vox::iot::ValueType::kNumber,  // parameter type
            true                              // parameter required
        },
        {
            "direction",                    // parameter name
            "电机方向(FORWARD/REVERSE)",    // parameter description
            ai_vox::iot::ValueType::kBool,  // parameter type
            true                            // parameter required
        }}},
      {"SetMotorB",
       "设置电机B参数",
       {{"speed", "电机速度(0-255)", ai_vox::iot::ValueType::kNumber, true},
        {"direction", "电机方向(FORWARD/REVERSE)", ai_vox::iot::ValueType::kBool, true}}},
      {"SetBothMotors",
       "同时设置电机A和电机B",
       {{"speed_a", "电机A速度(0-255)", ai_vox::iot::ValueType::kNumber, true},
        {"speed_b", "电机B速度(0-255)", ai_vox::iot::ValueType::kNumber, true},
        {
            "direction_a",                  // parameter name
            "电机A方向(FORWARD/REVERSE)",   // parameter description
            ai_vox::iot::ValueType::kBool,  // parameter type
            true                            // parameter required
        },
        {
            "direction_b",                  // parameter name
            "电机B方向(FORWARD/REVERSE)",   // parameter description
            ai_vox::iot::ValueType::kBool,  // parameter type
            true                            // parameter required
        }}},
  });

  // 3.Create the motor driver entity
  g_motor_driver_iot_entity = std::make_shared<ai_vox::iot::Entity>("MotorDriver",                       // name
                                                                    "电机驱动",                          // description
                                                                    std::move(motor_driver_properties),  // properties
                                                                    std::move(motor_driver_functions)    // functions
  );

  // 4.Initialize the motor driver entity with default values
  g_motor_driver_iot_entity->UpdateState("speed_a", 0);
  g_motor_driver_iot_entity->UpdateState("direction_a", kForword);
  g_motor_driver_iot_entity->UpdateState("speed_b", 0);
  g_motor_driver_iot_entity->UpdateState("direction_b", kForword);
  g_motor_driver_iot_entity->UpdateState("status_a", "stopped");
  g_motor_driver_iot_entity->UpdateState("status_b", "stopped");

  // 5.Register the motor driver entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_motor_driver_iot_entity);
}

void SetMotorDirectionSpeed(const char motor, const bool direction, int speed) {
  speed = constrain(speed, 0, 255);
  int motor_in_1;
  int motor_in_2;

  // Select motor
  if (motor == 'A' || motor == 'a') {
    motor_in_1 = kMotorAIn1;
    motor_in_2 = kMotorAIn2;
  } else if (motor == 'B' || motor == 'b') {
    motor_in_1 = kMotorBIn1;
    motor_in_2 = kMotorBIn2;
  } else {
    printf("Error: detected non motor A and motor B, can only control motor A and motor B. please try again.\n");
    return;  // Invalid motor identification
  }

  // Set direction
  if (direction == kForword) {
    analogWrite(motor_in_1, speed);
    digitalWrite(motor_in_2, LOW);
  } else {
    analogWrite(motor_in_1, speed);
    digitalWrite(motor_in_2, HIGH);
  }
}

void InitMotor() {
  pinMode(kMotorAIn1, OUTPUT);
  pinMode(kMotorAIn2, OUTPUT);
  pinMode(kMotorBIn1, OUTPUT);
  pinMode(kMotorBIn2, OUTPUT);
  SetMotorDirectionSpeed('A', kForword, 0);  // Set the speed to 0 to stop
  SetMotorDirectionSpeed('B', kForword, 0);  // Set the speed to 0 to stop
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

  // Initialize motor drive
  InitMotor();

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

      if (iot_message_event->name == "MotorDriver") {
        if (iot_message_event->function == "SetBothMotors") {  // Simultaneously activate motors A and B
          int64_t speed_a = 0;
          int64_t speed_b = 0;
          bool direction_a = kForword;
          bool direction_b = kForword;

          if (const auto it = iot_message_event->parameters.find("speed_a"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              speed_a = std::get<int64_t>(it->second);

              if (speed_a < 0 || speed_a > 255) {
                printf("Error: motor A speed is out of range (0-255), got: %lld\n", speed_a);
                continue;
              }
            } else {
              printf("Error: motor A speed acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'speed_a' not obtained, please check.\n");
            continue;
          }

          if (const auto it = iot_message_event->parameters.find("speed_b"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              speed_b = std::get<int64_t>(it->second);

              if (speed_b < 0 || speed_b > 255) {
                printf("Error: motor B speed is out of range (0-255), got: %lld\n", speed_b);
                continue;
              }
            } else {
              printf("Error: motor B speed acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'speed_b' not obtained, please check.\n");
            continue;
          }

          if (const auto it = iot_message_event->parameters.find("direction_a"); it != iot_message_event->parameters.end()) {
            if (std::get_if<bool>(&it->second)) {
              direction_a = std::get<bool>(it->second);

            } else {
              printf("Error: motor A direction acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'direction_a' not obtained, please check.\n");
            continue;
          }

          if (const auto it = iot_message_event->parameters.find("direction_b"); it != iot_message_event->parameters.end()) {
            if (std::get_if<bool>(&it->second)) {
              direction_b = std::get<bool>(it->second);

            } else {
              printf("Error: motor B direction acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'direction_b' not obtained, please check.\n");
            continue;
          }

          printf("Starting both motor - MotorA: Speed %lld, Direction %s | MotorB: Speed %lld, Direction %s\n",
                 speed_a,
                 direction_a ? "FORWARD" : "REVERSE",
                 speed_b,
                 direction_b ? "FORWARD" : "REVERSE");

          SetMotorDirectionSpeed('A', direction_a, speed_a);
          SetMotorDirectionSpeed('B', direction_b, speed_b);
          g_motor_driver_iot_entity->UpdateState("speed_a", speed_a);
          g_motor_driver_iot_entity->UpdateState("direction_a", direction_a);
          g_motor_driver_iot_entity->UpdateState("speed_b", speed_b);
          g_motor_driver_iot_entity->UpdateState("direction_b", direction_b);
          g_motor_driver_iot_entity->UpdateState("status_a", speed_a > 0 ? "running" : "stopped");
          g_motor_driver_iot_entity->UpdateState("status_b", speed_b > 0 ? "running" : "stopped");

        } else if (iot_message_event->function == "SetMotorA") {  // Operate motor A
          int64_t speed = 0;
          bool direction = kForword;

          if (const auto it = iot_message_event->parameters.find("speed"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              speed = std::get<int64_t>(it->second);

              if (speed < 0 || speed > 255) {
                printf("Error: motor A speed is out of range (0-255), got: %lld\n", speed);
                continue;
              }
            } else {
              printf("Error: motor A speed acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'speed' not obtained, please check.\n");
            continue;
          }

          if (const auto it = iot_message_event->parameters.find("direction"); it != iot_message_event->parameters.end()) {
            if (std::get_if<bool>(&it->second)) {
              direction = std::get<bool>(it->second);

            } else {
              printf("Error: motor A direction acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'direction' not obtained, please check.\n");
            continue;
          }

          printf("Setting MotorA - Speed: %lld, Direction: %s\n", speed, direction ? "FORWARD" : "REVERSE");
          SetMotorDirectionSpeed('A', direction, speed);
          g_motor_driver_iot_entity->UpdateState("speed_a", speed);
          g_motor_driver_iot_entity->UpdateState("direction_a", direction);
          g_motor_driver_iot_entity->UpdateState("status_a", speed > 0 ? "running" : "stopped");

        } else if (iot_message_event->function == "SetMotorB") {  // Operate Motor B
          int64_t speed = 0;
          bool direction = kForword;

          if (const auto it = iot_message_event->parameters.find("speed"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              speed = std::get<int64_t>(it->second);

              if (speed < 0 || speed > 255) {
                printf("Error: motor B speed is out of range (0-255), got: %lld\n", speed);
                continue;
              }
            } else {
              printf("Error: motor B speed acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'speed' not obtained, please check.\n");
            continue;
          }

          if (const auto it = iot_message_event->parameters.find("direction"); it != iot_message_event->parameters.end()) {
            if (std::get_if<bool>(&it->second)) {
              direction = std::get<bool>(it->second);

            } else {
              printf("Error: motor B direction acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'direction' not obtained, please check.\n");
            continue;
          }

          printf("Setting MotorB - Speed: %lld, Direction: %s\n", speed, direction ? "FORWARD" : "REVERSE");
          SetMotor('B', direction, speed);
          g_motor_driver_iot_entity->UpdateState("speed_b", speed);
          g_motor_driver_iot_entity->UpdateState("direction_b", direction);
          g_motor_driver_iot_entity->UpdateState("status_b", speed > 0 ? "running" : "stopped");
        }
      }
    }
  }

  taskYIELD();
}