#include <WiFi.h>
#include <Wire.h>
#include <driver/i2c_master.h>
#include <esp_lcd_io_i2c.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_ssd1306.h>

#include "ai_vox_engine.h"
#include "ai_vox_observer.h"
#include "display.h"
#include "i2s_std_audio_input_device.h"
#include "i2s_std_audio_output_device.h"

#define DHTLIB_OK 0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT -2

#ifndef WIFI_SSID
#define WIFI_SSID "ssid"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "password"
#endif

namespace {

// Define the pins of US04 ultrasonic waves
constexpr gpio_num_t kUs04PinTrig = GPIO_NUM_12;  // trig pin
constexpr gpio_num_t kUs04PinEcho = GPIO_NUM_36;  // echo pin

// Define temperature and humidity sensor pins
constexpr gpio_num_t kDht11Pin = GPIO_NUM_14;

// Define the pins of the photosensitive sensor
constexpr gpio_num_t kPhotosensitivePin = GPIO_NUM_39;

constexpr gpio_num_t kMicPinBclk = GPIO_NUM_25;
constexpr gpio_num_t kMicPinWs = GPIO_NUM_26;
constexpr gpio_num_t kMicPinDin = GPIO_NUM_19;

constexpr gpio_num_t kSpeakerPinBclk = GPIO_NUM_33;
constexpr gpio_num_t kSpeakerPinWs = GPIO_NUM_32;
constexpr gpio_num_t kSpeakerPinDout = GPIO_NUM_23;

constexpr gpio_num_t kI2cPinSda = GPIO_NUM_21;
constexpr gpio_num_t kI2cPinScl = GPIO_NUM_22;

constexpr gpio_num_t kTriggerPin = GPIO_NUM_34;

constexpr uint32_t kDisplayWidth = 128;
constexpr uint32_t kDisplayHeight = 64;
constexpr bool kDisplayMirrorX = true;
constexpr bool kDisplayMirrorY = true;

constexpr uint32_t kSensorReadInterval = 2000;

uint32_t us04_iot_measure_distance = 0;
uint32_t dht11_iot_read_humidity;
uint32_t dht11_iot_read_temperature;
uint32_t photosensitive_iot_read_value = 0;
uint32_t g_last_sensor_read_time = 0;

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

std::shared_ptr<ai_vox::iot::Entity> g_dht11_sensor_iot_entity;
std::shared_ptr<ai_vox::iot::Entity> g_us04_ultrasonic_sensor_iot_entity;
std::shared_ptr<ai_vox::iot::Entity> g_photosensitive_sensor_iot_entity;

auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

void InitDisplay() {
  printf("InitDisplay\n");
  i2c_master_bus_handle_t display_i2c_bus;
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = kI2cPinSda,
      .scl_io_num = kI2cPinScl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags =
          {
              .enable_internal_pullup = 1,
              .allow_pd = false,
          },
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr = 0x3C,
      .on_color_trans_done = nullptr,
      .user_ctx = nullptr,
      .control_phase_bytes = 1,
      .dc_bit_offset = 6,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .flags =
          {
              .dc_low_on_data = 0,
              .disable_control_phase = 0,
          },
      .scl_speed_hz = 400 * 1000,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus, &io_config, &panel_io));

  esp_lcd_panel_handle_t panel = nullptr;
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = -1;
  panel_config.bits_per_pixel = 1;

  esp_lcd_panel_ssd1306_config_t ssd1306_config = {
      .height = static_cast<uint8_t>(kDisplayHeight),
  };
  panel_config.vendor_config = &ssd1306_config;

  ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io, &panel_config, &panel));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
  g_display = std::make_unique<Display>(panel_io, panel, kDisplayWidth, kDisplayHeight, kDisplayMirrorX, kDisplayMirrorY);
  g_display->Start();
}

void InitIot() {
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();

  // UltrasonicSensor IoT Entity Definition

  // 1.Define the properties for the ultrasonic sensor entity
  std::vector<ai_vox::iot::Property> us04_ultrasonic_sensor_iot_properties({{
      "distance",                            // property name
      "当前障碍物距离(小于0 表示测量超时)",  // property description
      ai_vox::iot::ValueType::kNumber        // property type: number, string or bool
  }

  });

  // 2.Define the functions for the ultrasonic sensor entity
  std::vector<ai_vox::iot::Function> us04_ultrasonic_sensor_iot_functions({{"MeasureDistance",  // function name
                                                                            "距离测量",         // function description
                                                                            {}}

  });

  // 3.Create the ultrasonic sensor entity
  g_us04_ultrasonic_sensor_iot_entity = std::make_shared<ai_vox::iot::Entity>("UltrasonicSensor",                                // name
                                                                              "超声波传感器",                                    // description
                                                                              std::move(us04_ultrasonic_sensor_iot_properties),  // properties
                                                                              std::move(us04_ultrasonic_sensor_iot_functions)    // functions
  );

  // 4.Initialize the ultrasonic sensor entity with default values
  g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", 0);
  // 5.Register the ultrasonic sensor entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_us04_ultrasonic_sensor_iot_entity);

  // 1.Define the properties for the g_dht11_iot_controller sensor entity
  std::vector<ai_vox::iot::Property> dht11_sensor_properties({{"temperature", "当前温度(等于-22时代表测量超时)", ai_vox::iot::ValueType::kNumber},
                                                              {"humidity", "当前湿度(等于-22时代表测量超时)", ai_vox::iot::ValueType::kNumber}});

  // 2.Define the functions for the g_dht11_iot_controller sensor entity
  std::vector<ai_vox::iot::Function> dht11_sensor_functions({{"ReadBoth", "测量温湿度", {}}});

  // 3.Create the g_dht11_iot_controller sensor entity
  g_dht11_sensor_iot_entity = std::make_shared<ai_vox::iot::Entity>(
      "DHT11Sensor", "DHT11温湿度传感器", std::move(dht11_sensor_properties), std::move(dht11_sensor_functions));

  // 4.Initialize the g_dht11_iot_controller sensor entity with default values
  g_dht11_sensor_iot_entity->UpdateState("temperature", 0);
  g_dht11_sensor_iot_entity->UpdateState("humidity", 0);

  // 5.Register the g_dht11_iot_controller sensor entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_dht11_sensor_iot_entity);

  // 1. Define the properties of photosensitive sensors
  std::vector<ai_vox::iot::Property> photosensitive_sensor_iot_properties(
      {{"illuminance", "当前光敏值", ai_vox::iot::ValueType::kNumber}, {"day_night", "当前昼夜状态(白天/夜晚)", ai_vox::iot::ValueType::kString}});

  // 2. Define the function of photosensitive sensors
  std::vector<ai_vox::iot::Function> photosensitive_sensor_iot_functions({{"ReadIlluminance", "光敏测量", {}}});

  // 3. Create a photosensitive sensor entity
  g_photosensitive_sensor_iot_entity = std::make_shared<ai_vox::iot::Entity>(
      "PhotosensitiveSensor", "光敏传感器", std::move(photosensitive_sensor_iot_properties), std::move(photosensitive_sensor_iot_functions));

  // 4. Initialize default values
  g_photosensitive_sensor_iot_entity->UpdateState("illuminance", 0);     // Default light intensity 0 lux
  g_photosensitive_sensor_iot_entity->UpdateState("day_night", "白天");  // Default daytime state

  ai_vox_engine.RegisterIotEntity(g_photosensitive_sensor_iot_entity);
}

// DHT11 temperature and humidity sensor reading function
void ReadTemperatureHumidity() {
  // BUFFER TO RECEIVE
  uint8_t bits[5] = {0};
  uint8_t count = 7;
  uint8_t index = 0;

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
    if (loop_count-- == 0) {
      dht11_iot_read_humidity = -22;
      dht11_iot_read_temperature = -22;
      printf("Error: DHT11 sensor measure timeout\n");
      return;
    }

  loop_count = 10000;
  while (digitalRead(kDht11Pin) == HIGH)
    if (loop_count-- == 0) {
      dht11_iot_read_humidity = -22;
      dht11_iot_read_temperature = -22;
      printf("Error: DHT11 sensor measure timeout\n");
      return;
    }

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loop_count = 10000;
    while (digitalRead(kDht11Pin) == LOW)
      if (loop_count-- == 0) {
        dht11_iot_read_humidity = -22;
        dht11_iot_read_temperature = -22;
        printf("Error: DHT11 sensor measure timeout\n");

        return;
      }

    unsigned long t = micros();

    loop_count = 10000;
    while (digitalRead(kDht11Pin) == HIGH)
      if (loop_count-- == 0) {
        dht11_iot_read_humidity = -22;
        dht11_iot_read_temperature = -22;
        printf("Error: DHT11 sensor measure timeout\n");
        return;
      }

    if ((micros() - t) > 40) bits[index] |= (1 << count);
    if (count == 0)  // next byte?
    {
      count = 7;  // restart at MSB
      index++;    // next byte!
    } else
      count--;
  }

  dht11_iot_read_humidity = bits[0];
  dht11_iot_read_temperature = bits[2];
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

  const unsigned long timeout = 30000;  // 30毫秒超时（约5米）

  // MeasureDistance echo time
  const long duration = pulseIn(kUs04PinEcho, HIGH, timeout);

  if (duration <= 0) {
    printf("US04 sensor measure timeout\n");

    us04_iot_measure_distance = -1;
    return;
  }

  us04_iot_measure_distance = (int)duration * 0.034 / 2;
  // printf("MeasureDistance:%d\n",(int)duration * 0.034 / 2);
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
  printf("Init\n");

  pinMode(kPhotosensitivePin, INPUT);

  InitUs04();

  InitDisplay();

  g_display->ShowStatus("Wifi connecting...");

  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
    WiFi.useStaticBuffers(true);
  } else {
    WiFi.useStaticBuffers(false);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    printf("Connecting to WiFi, ssid: %s, password: %s\n", WIFI_SSID, WIFI_PASSWORD);
    delay(1000);
  }

  printf("WiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
  g_display->ShowStatus("Wifi connected");

  InitIot();

  auto audio_input_device = std::make_shared<ai_vox::I2sStdAudioInputDevice>(kMicPinBclk, kMicPinWs, kMicPinDin);
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetTrigger(kTriggerPin);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  ai_vox_engine.Start(audio_input_device, g_audio_output_device);
  g_display->ShowStatus("AI Vox starting...");
  printf("AI Vox engine started\n");
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  const uint32_t current_time = millis();
  if (current_time - g_last_sensor_read_time >= kSensorReadInterval) {
    g_last_sensor_read_time = current_time;
    ReadTemperatureHumidity();  // Initialize temperature and humidity pins
    g_dht11_sensor_iot_entity->UpdateState("temperature", dht11_iot_read_temperature);
    g_dht11_sensor_iot_entity->UpdateState("humidity", dht11_iot_read_humidity);

    MeasureDistance();
    g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", us04_iot_measure_distance);

    photosensitive_iot_read_value = analogRead(kPhotosensitivePin);  // Read the photosensitive value
    const std::string light_status = photosensitive_iot_read_value > 500 ? "白天" : "晚上";
    g_photosensitive_sensor_iot_entity->UpdateState("illuminance", photosensitive_iot_read_value);
    g_photosensitive_sensor_iot_entity->UpdateState("day_night", std::move(light_status));
  }

  const auto events = g_observer->PopEvents();
  for (auto& event : events) {
    if (auto activation_event = std::get_if<ai_vox::Observer::ActivationEvent>(&event)) {
      printf("activation code: %s, message: %s\n", activation_event->code.c_str(), activation_event->message.c_str());
      g_display->ShowStatus((std::string("激活设备") + activation_event->code).c_str());
      g_display->SetChatMessage(activation_event->message);
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
          g_display->SetChatMessage("");
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
          break;
        }
        case ai_vox::ChatRole::kUser: {
          printf("role: user, content: %s\n", chat_message_event->content.c_str());
          break;
        }
      }
      g_display->SetChatMessage(chat_message_event->content);
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
          MeasureDistance();
          g_us04_ultrasonic_sensor_iot_entity->UpdateState("distance", us04_iot_measure_distance);
          printf("触发单次距离测量，结果：%d 厘米\n", us04_iot_measure_distance);
        }

      } else if (iot_message_event->name == "DHT11Sensor") {
        if (iot_message_event->function == "ReadBoth") {
          ReadTemperatureHumidity();
          g_dht11_sensor_iot_entity->UpdateState("temperature", dht11_iot_read_temperature);
          g_dht11_sensor_iot_entity->UpdateState("humidity", dht11_iot_read_humidity);
          printf("读取温湿度：温度 %d ℃，湿度 %d %%\n", dht11_iot_read_temperature, dht11_iot_read_humidity);
        }
      } else if (iot_message_event->name == "PhotosensitiveSensor") {
        // Trigger a distance measurement once
        if (iot_message_event->function == "ReadIlluminance") {
          photosensitive_iot_read_value = analogRead(kPhotosensitivePin);  // Read the photosensitive value
          const std::string light_status = photosensitive_iot_read_value > 500 ? "白天" : "晚上";
          g_photosensitive_sensor_iot_entity->UpdateState("illuminance", photosensitive_iot_read_value);
          g_photosensitive_sensor_iot_entity->UpdateState("day_night", std::move(light_status));

          printf("触发光敏值测量，结果：%d,判断是白天/晚上：%s \n", photosensitive_iot_read_value, light_status.c_str());
        }
      }
    }
  }

  taskYIELD();
}