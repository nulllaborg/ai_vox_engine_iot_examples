#include <WiFi.h>
#include <Wire.h>
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

#define DHTLIB_OK 0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT -2

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

// Define the pins of US04 ultrasonic waves
constexpr gpio_num_t kUs04PinTrig = GPIO_NUM_44;  // trig pin
constexpr gpio_num_t kUs04PinEcho = GPIO_NUM_46;  // echo pin

// Define temperature and humidity sensor pins
constexpr gpio_num_t kDht11Pin = GPIO_NUM_47;  // echo pin

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

// Display screen parameter configuration
constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

std::shared_ptr<ai_vox::iot::Entity> g_dht11_sensor_iot_entity;
std::shared_ptr<ai_vox::iot::Entity> g_us04_ultrasonic_sensor_iot_entity;

auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();
bool uso4_iot_is_measure = true;
int uso4_iot_measure_count = 0;
int us04_iot_measure_distance = 0;
int dht11_iot_read_humidity;
int dht11_iot_read_temperature;

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

  // UltrasonicSensor IoT Entity Definition

  // 1.Define the properties for the ultrasonic sensor entity
  std::vector<ai_vox::iot::Property> us04_ultrasonic_sensor_iot_properties({{
      "distance",                      // property name
      "当前障碍物距离",                // property description
      ai_vox::iot::ValueType::kNumber  // property type: number, string or bool
  }

  });

  // 2.Define the functions for the ultrasonic sensor entity
  std::vector<ai_vox::iot::Function> us04_ultrasonic_sensor_iot_functions({{"MeasureDistance",  // function name
                                                                            "距离测量",         // function description
                                                                            {}}

  });

  // 3.Create the ultrasonic sensor entity
  g_us04_ultrasonic_sensor_iot_entity =
      std::make_shared<ai_vox::iot::Entity>("UltrasonicSensor",                                // name
                                            "超声波传感器",                                    // description
                                            std::move(us04_ultrasonic_sensor_iot_properties),  // properties
                                            std::move(us04_ultrasonic_sensor_iot_functions)    // functions
      );

  // 4.Initialize the ultrasonic sensor entity with default values
  g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", 0);
  // 5.Register the ultrasonic sensor entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_us04_ultrasonic_sensor_iot_entity);

  // 1.Define the properties for the g_dht11_iot_controller sensor entity
  std::vector<ai_vox::iot::Property> dht11_sensor_properties({{"temperature", "当前温度", ai_vox::iot::ValueType::kNumber},
                                                              {"humidity", "当前湿度", ai_vox::iot::ValueType::kNumber}});

  // 2.Define the functions for the g_dht11_iot_controller sensor entity
  std::vector<ai_vox::iot::Function> dht11_sensor_functions({{"ReadTemperature", "触发一次温度测量", {}},
                                                             {"ReadHumidity", "触发一次湿度测量", {}},
                                                             {"ReadBoth", "同时触发温度和湿度测量", {}}});

  // 3.Create the g_dht11_iot_controller sensor entity
  g_dht11_sensor_iot_entity = std::make_shared<ai_vox::iot::Entity>(
      "DHT11Sensor", "DHT11温湿度传感器", std::move(dht11_sensor_properties), std::move(dht11_sensor_functions));

  // 4.Initialize the g_dht11_iot_controller sensor entity with default values
  g_dht11_sensor_iot_entity->UpdateState("temperature", 0);
  g_dht11_sensor_iot_entity->UpdateState("humidity", 0);

  // 5.Register the g_dht11_iot_controller sensor entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_dht11_sensor_iot_entity);
}

// DHT11 temperature and humidity sensor reading function
int Read_temperature_humidity() {
  // BUFFER TO RECEIVE
  uint8_t bits[5];
  uint8_t count = 7;
  uint8_t index = 0;

  // EMPTY BUFFER
  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(kDht11Pin, OUTPUT);
  digitalWrite(kDht11Pin, LOW);
  delay(18);
  digitalWrite(kDht11Pin, HIGH);
  delayMicroseconds(40);
  pinMode(kDht11Pin, INPUT);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loop_count = 10000;
  while (digitalRead(kDht11Pin) == LOW)
    if (loop_count-- == 0) return DHTLIB_ERROR_TIMEOUT;

  loop_count = 10000;
  while (digitalRead(kDht11Pin) == HIGH)
    if (loop_count-- == 0) return DHTLIB_ERROR_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loop_count = 10000;
    while (digitalRead(kDht11Pin) == LOW)
      if (loop_count-- == 0) return DHTLIB_ERROR_TIMEOUT;

    unsigned long t = micros();

    loop_count = 10000;
    while (digitalRead(kDht11Pin) == HIGH)
      if (loop_count-- == 0) return DHTLIB_ERROR_TIMEOUT;

    if ((micros() - t) > 40) bits[index] |= (1 << count);
    if (count == 0)  // next byte?
    {
      count = 7;  // restart at MSB
      index++;    // next byte!
    } else
      count--;
  }

  // WRITE TO RIGHT VARS
  // as bits[1] and bits[3] are allways zero they are omitted in formulas.
  dht11_iot_read_humidity = bits[0];
  dht11_iot_read_temperature = bits[2];

  uint8_t sum = bits[0] + bits[2];

  if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
  return DHTLIB_OK;
}

// Initialize the US04 ultrasonic module
void InitUs04() {
  pinMode(kUs04PinTrig, OUTPUT);
  pinMode(kUs04PinEcho, INPUT);
}

// US04 measurement function
void MeasureDistance() {
  // Send trigger signal
  digitalWrite(kUs04PinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(kUs04PinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(kUs04PinTrig, LOW);

  // MeasureDistance echo time
  long duration = pulseIn(kUs04PinEcho, HIGH);

  us04_iot_measure_distance = (int)duration * 0.034 / 2;
  // Calculate distance (in centimeters)
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

  InitUs04();

  InitDisplay();  // Screen initialization Screen initialization

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

  MeasureDistance();
  g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", us04_iot_measure_distance);
  printf("2触发单次距离测量，结果：%d 厘米\n", us04_iot_measure_distance);

  Read_temperature_humidity();

  g_dht11_sensor_iot_entity->UpdateState("temperature", dht11_iot_read_temperature);
  g_dht11_sensor_iot_entity->UpdateState("humidity", dht11_iot_read_humidity);
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  Read_temperature_humidity();  // Initialize temperature and humidity pins

  if (uso4_iot_is_measure && uso4_iot_measure_count == 100) {
    uso4_iot_is_measure = false;
    uso4_iot_measure_count = 0;
    MeasureDistance();
    g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", us04_iot_measure_distance);
    printf("2触发单次距离测量，结果：%d 厘米\n", us04_iot_measure_distance);
  }
  uso4_iot_measure_count++;
  uso4_iot_is_measure = true;

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

      if (iot_message_event->name == "UltrasonicSensor") {
        // Trigger a distance measurement once
        if (iot_message_event->function == "MeasureDistance") {
          if (uso4_iot_is_measure) {
            uso4_iot_is_measure = false;
            MeasureDistance();
          }
          uso4_iot_is_measure = true;
          g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", us04_iot_measure_distance);
          printf("触发单次距离测量，结果：%d 厘米\n", us04_iot_measure_distance);
        }

      } else if (iot_message_event->name == "DHT11Sensor") {
        // Trigger a temperature measurement once
        if (iot_message_event->function == "ReadTemperature") {
          // Enter critical zone (disable interrupt)
          portDISABLE_INTERRUPTS();

          Read_temperature_humidity();

          // Exit critical zone (enable interrupt)
          portENABLE_INTERRUPTS();

          if (dht11_iot_read_temperature != 0) {  // Effective value check
            g_dht11_sensor_iot_entity->UpdateState("temperature", dht11_iot_read_temperature);
            printf("温度: %d℃\n", dht11_iot_read_temperature);
          } else {
            printf("读取失败，请重试\n");
          }
        }
        // Trigger a humidity measurement once
        else if (iot_message_event->function == "ReadHumidity") {
          portDISABLE_INTERRUPTS();

          Read_temperature_humidity();
          portENABLE_INTERRUPTS();

          if (dht11_iot_read_humidity != 0) {
            g_dht11_sensor_iot_entity->UpdateState("humidity", dht11_iot_read_humidity);
            printf("读取湿度: %d℃\n", dht11_iot_read_humidity);
          } else {
            printf("读取失败，请重试\n");
          }

        }
        // Simultaneously triggering temperature and humidity measurement
        else if (iot_message_event->function == "ReadBoth") {
          portDISABLE_INTERRUPTS();
          Read_temperature_humidity();

          portENABLE_INTERRUPTS();

          if (dht11_iot_read_humidity != 0) {
            g_dht11_sensor_iot_entity->UpdateState("temperature", dht11_iot_read_temperature);
            g_dht11_sensor_iot_entity->UpdateState("humidity", dht11_iot_read_humidity);
            printf("读取温湿度：温度 %d ℃，湿度 %d %%\n", dht11_iot_read_temperature, dht11_iot_read_humidity);
          } else {
            printf("读取失败，请重试\n");
          }
        }
      }
    }
  }

  taskYIELD();
}