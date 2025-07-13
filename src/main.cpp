#include "pins_config.h"
#include "LovyanGFX_Driver.h"
#include <Arduino.h>
#include <lvgl.h>
#include "ui.h" // Ensure ui.h is in include folder
#include <TCA9534.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

#define ESPNOW_DEBUG(fmt, ...) Serial.printf("[ESPNOW] " fmt, ##__VA_ARGS__)

TCA9534 ioex;
LGFX gfx;

// LVGL Mutex for thread safety
SemaphoreHandle_t lvgl_mutex = NULL;

// Define color and opacity constants for easier maintenance
const lv_color_t COLOR_NORMAL = lv_color_hex(0x00FF00);   // Green for normal state
const lv_color_t COLOR_WARNING = lv_color_hex(0xFF0000); // Red for warning/active state
const lv_opa_t OPA_NORMAL = LV_OPA_10;                   // Almost transparent for normal
const lv_opa_t OPA_WARNING = LV_OPA_COVER;               // Fully opaque for warning

// Declare UI objects from ui_screen1.c
extern lv_obj_t *ui_rpmslider;
extern lv_obj_t *ui_RPM; // Label for RPM display
extern lv_obj_t *ui_Speed; // Label for speed display
extern lv_obj_t *ui_ParkingBrake; // Parking brake indicator
extern lv_obj_t *ui_TurnSignal; // Turn signal indicator
extern lv_obj_t *ui_CheckEngine; // Check engine light indicator
extern lv_obj_t *ui_DSC; // DSC indicator
extern lv_obj_t *ui_Watertemp; // Water temp indicator
lv_obj_t *ui_Hz; // Label for CANBus Update Rate display
static uint8_t *buf = NULL;
static uint8_t *buf1 = NULL;

/* ESP-NOW data structure */
typedef struct {
  uint16_t rpm;
  uint8_t coolantTemp;
  uint8_t angleFgrPedal;
  uint8_t driverDemand;
  uint8_t checkEngineLight : 1;
  uint8_t engineWarningLight : 1;
  uint8_t boostFailureLight : 1;
  uint8_t overheating : 1;
  uint8_t manifoldPressure;
  uint8_t fuelTankLevel;
  uint8_t switchFillingStatus : 1;
  uint8_t handbrakeSwitch : 1;
  uint8_t turnSignalIndicator : 2;
  uint8_t odbFault : 1;
  uint8_t manualGearSelected : 4;
  uint8_t requestAsc : 1;
  uint8_t requestMsr : 1;
  uint8_t ascLampStatus : 1;
  uint8_t vehicleSpeed : 5;
} CanData;

CanData receivedData;
volatile bool dataReceived = false;
volatile uint32_t lastPacketTime = 0; // Track packet timing
static uint32_t lastDataUpdateTime = 0; // Track last data update for timeout
#define DATA_TIMEOUT_MS 1000 // Timeout to set indicators to disabled

volatile uint32_t espnow_packet_count = 0;

// Track previous states to avoid redundant updates
static uint16_t prev_rpm = 0;
static uint8_t prev_speed = 0;
static bool prev_parkingBrake = false;
static bool prev_turnSignal = false;
static bool prev_checkEngine = false;
static bool prev_dsc = false;
static bool prev_watertemp = false;
static uint32_t prev_hz = 0;

// LVGL task handle
TaskHandle_t lvglTask = NULL;

// Display refresh
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  if (gfx.getStartCount() > 0) {
    gfx.endWrite();
  }
  // Width and height calculation
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  // Push raw pixels (cast to RGB565)
  gfx.pushImage(area->x1, area->y1, w, h, (lgfx::rgb565_t *)px_map);
  lv_display_flush_ready(disp);
}

// Touch input (commented out)
// void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
//   data->state = LV_INDEV_STATE_REL;
//   bool touched = gfx.getTouch(&touch_x, &touch_y);
//   if (touched) {
//     data->state = LV_INDEV_STATE_PR;
//     data->point.x = touch_x;
//     data->point.y = touch_y;
//     Serial.printf("Touch detected at: (%d, %d)\n", touch_x, touch_y);
//   }
// }

/* ESP-NOW callback function with throttled debugging */
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
    static uint32_t last_log_time = 0;
    const uint32_t log_interval = 1000; // Log every 1000 ms

    if (len != sizeof(CanData)) {
        ESPNOW_DEBUG("ESP-NOW: Error: Invalid data length, expected %d, got %d\n", sizeof(CanData), len);
        return;
    }

    uint32_t current_time = millis();
    // Only process if enough time has passed (to enforce ~20 Hz)
    if (current_time - lastPacketTime >= 40) { // ~25 Hz to allow jitter
        memcpy(&receivedData, incomingData, sizeof(CanData));
        __atomic_fetch_add(&espnow_packet_count, 1, __ATOMIC_SEQ_CST); // Atomic increment for safety
        lastPacketTime = current_time;
        lastDataUpdateTime = current_time;
        dataReceived = true;

        if (current_time - last_log_time >= log_interval) {
            ESPNOW_DEBUG("ESP-NOW: Received RPM: %u, Coolant Temp: %u, Vehicle Speed: %u\n",
                         receivedData.rpm, receivedData.coolantTemp, receivedData.vehicleSpeed);
            ESPNOW_DEBUG("ESP-NOW: Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], 
                         recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
            last_log_time = current_time;
        }
    }
}

// LVGL task function
void lvgl_task(void *pvParameter) {
  static uint32_t previous_millis = 0;
  while (1) {
    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) == pdTRUE) {
      uint32_t current_millis = millis();
      lv_tick_inc(current_millis - previous_millis);
      previous_millis = current_millis;
      lv_timer_handler();
      xSemaphoreGive(lvgl_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Starting WiFi initialization...");
  Serial.println("Setting WiFi mode to WIFI_STA...");
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi mode set to WIFI_STA.");
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());
  delay(100); // Short delay to allow initialization
  Serial.print("Local MAC Address from WiFi.macAddress(): ");
  Serial.println(WiFi.macAddress());

  uint8_t mac[6];
  esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
  if (err == ESP_OK) {
    Serial.printf("MAC Address from esp_wifi_get_mac: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.printf("Failed to get MAC from esp_wifi_get_mac, error: %d\n", err);
  }

  // Set WiFi channel to ensure compatibility with sender
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  Serial.println("WiFi channel set to 1");

  Serial.println("Initializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized successfully.");
  esp_now_register_recv_cb(OnDataRecv);

  // Add broadcast peer for ESP-NOW
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
  } else {
    Serial.println("Broadcast peer added successfully");
  }

  // Log configuration
  Serial.printf("Free PSRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  Serial.printf("Display resolution: %d x %d\n", LCD_H_RES, LCD_V_RES);

  // Initialize I2C for TCA9534
  Wire.begin(15, 16);
  delay(50);
  ioex.attach(Wire);
  ioex.setDeviceAddress(0x18);
  ioex.config(1, TCA9534::Config::OUT);
  ioex.config(2, TCA9534::Config::OUT);
  ioex.output(1, TCA9534::Level::H);
  delay(20);
  ioex.output(2, TCA9534::Level::H);
  delay(100);
  pinMode(1, INPUT);

  // Initialize display
  Serial.println("Initializing display...");
  gfx.init();
  delay(100);
  gfx.startWrite();

  // Test display with colors
  Serial.println("Testing display with color patterns...");
  gfx.fillScreen(TFT_RED);
  Serial.println("Screen should be RED");
  delay(1000);
  gfx.fillScreen(TFT_GREEN);
  Serial.println("Screen should be GREEN");
  delay(1000);
  gfx.fillScreen(TFT_BLUE);
  Serial.println("Screen should be BLUE");
  delay(1000);
  gfx.fillScreen(TFT_BLACK);
  Serial.println("Screen should be BLACK");

  // Initialize LVGL
  Serial.println("Initializing LVGL...");
  lv_init();

  // Create LVGL mutex
  lvgl_mutex = xSemaphoreCreateMutex();
  if (lvgl_mutex == NULL) {
    Serial.println("Failed to create LVGL mutex!");
    while (1);
  }

// Print initial free memory
Serial.println("Free memory before allocation:");
Serial.printf("Free DMA: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
Serial.printf("Free SPIRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
Serial.printf("Free Internal: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

// Set buffer size for 150 lines (partial buffering), in bytes for RGB565 (2 bytes/pixel)
size_t buffer_size_bytes = 800 * 48 * 2;  // Assuming LV_COLOR_DEPTH=16 (RGB565)
Serial.printf("Attempting to allocate DMA buffer of size: %u bytes\n", buffer_size_bytes);

// Allocate primary buffer in DMA-capable memory
buf = (uint8_t *)heap_caps_malloc(buffer_size_bytes, MALLOC_CAP_DMA);
if (!buf) {
  Serial.println("Buffer allocation failed for buf! Halting.");
  while (1);
}
Serial.println("Buffer buf allocated successfully in DMA memory");

// Attempt secondary buffer for double buffering
buf1 = (uint8_t *)heap_caps_malloc(buffer_size_bytes, MALLOC_CAP_DMA);
if (!buf1) {
  Serial.println("Buffer allocation failed for buf1! Falling back to single buffer.");
  buf1 = NULL;
} else {
  Serial.println("Buffer buf1 allocated successfully in DMA memory");
}

// Print free memory after allocation
Serial.println("Free memory after allocation:");
Serial.printf("Free DMA: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
Serial.printf("Free SPIRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
Serial.printf("Free Internal: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

// Initialize display driver
lv_display_t *disp = lv_display_create(800, 480);
if (disp == NULL) {
  Serial.println("Failed to create display!");
  while (1);
}

// Set color format (assuming RGB565 based on your LovyanGFX usage)
lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

// Set buffers (size in bytes; use PARTIAL mode for smaller buffers)
lv_display_set_buffers(disp, buf, buf1, buffer_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

// Set flush callback
lv_display_set_flush_cb(disp, my_disp_flush);

  // Touch driver (commented out)
  // static lv_indev_drv_t indev_drv;
  // lv_indev_drv_init(&indev_drv);
  // indev_drv.type = LV_INDEV_TYPE_POINTER;
  // indev_drv.read_cb = my_touchpad_read;
  // lv_indev_drv_register(&indev_drv);

  // Initialize UI with mutex
  if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) == pdTRUE) {
    Serial.println("Initializing UI...");
    ui_init();
    // Verify ui_rpmslider, ui_RPM, ui_Speed, and indicators
    if (ui_rpmslider == NULL) {
        Serial.println("ERROR: ui_rpmslider is NULL, check ui_init()");
        while (1);
    }
    if (ui_RPM == NULL) {
        Serial.println("ERROR: ui_RPM is NULL, check ui_init()");
        while (1);
    }
    if (ui_Speed == NULL) {
        Serial.println("ERROR: ui_Speed is NULL, check ui_init()");
        while (1);
    }
    if (ui_ParkingBrake == NULL) {
        Serial.println("ERROR: ui_ParkingBrake is NULL, check ui_init()");
        while (1);
    }
    if (ui_TurnSignal == NULL) {
        Serial.println("ERROR: ui_TurnSignal is NULL, check ui_init()");
        while (1);
    }
    if (ui_CheckEngine == NULL) {
        Serial.println("ERROR: ui_CheckEngine is NULL, check ui_init()");
        while (1);
    }
    if (ui_DSC == NULL) {
        Serial.println("ERROR: ui_DSC is NULL, check ui_init()");
        while (1);
    }
    if (ui_Watertemp == NULL) {
        Serial.println("ERROR: ui_Watertemp is NULL, check ui_init()");
        while (1);
    }
    Serial.println("ui_rpmslider, ui_RPM, ui_Speed, ui_ParkingBrake, ui_TurnSignal, ui_CheckEngine, ui_DSC, ui_Watertemp initialized successfully");
    // Ensure widgets are not hidden
    lv_obj_clear_flag(ui_rpmslider, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_RPM, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Speed, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_ParkingBrake, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_TurnSignal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_CheckEngine, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_DSC, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Watertemp, LV_OBJ_FLAG_HIDDEN);
    // Set arc range to 0-11000
    lv_arc_set_range(ui_rpmslider, 0, 11000);
    lv_arc_set_value(ui_rpmslider, 0);
    lv_label_set_text(ui_RPM, "0");
    lv_label_set_text(ui_Speed, "0");
    // Initialize indicators to off state (no background color, almost transparent image)
    lv_obj_set_style_bg_opa(ui_ParkingBrake, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_ParkingBrake, OPA_NORMAL, 0);
    lv_obj_set_style_bg_opa(ui_TurnSignal, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_TurnSignal, OPA_NORMAL, 0);
    lv_obj_set_style_bg_opa(ui_CheckEngine, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_CheckEngine, OPA_NORMAL, 0);
    lv_obj_set_style_bg_opa(ui_DSC, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_DSC, OPA_NORMAL, 0);
    lv_obj_set_style_bg_opa(ui_Watertemp, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_Watertemp, OPA_NORMAL, 0);
    // Create Hz label in bottom left corner
    extern lv_obj_t *ui_Screen1;
    ui_Hz = lv_label_create(ui_Screen1);
    lv_obj_set_pos(ui_Hz, 10, 480 - 30);
    lv_label_set_text(ui_Hz, "CANBUS 0 Hz");
    lv_obj_set_style_text_color(ui_Hz, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Hz, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(ui_Hz, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(ui_ParkingBrake);
    lv_obj_invalidate(ui_TurnSignal);
    lv_obj_invalidate(ui_CheckEngine);
    lv_obj_invalidate(ui_DSC);
    lv_obj_invalidate(ui_Watertemp);
    lv_obj_invalidate(ui_rpmslider);
    lv_obj_invalidate(ui_RPM);
    lv_obj_invalidate(ui_Speed);
    lv_obj_invalidate(ui_Hz);
    lv_refr_now(NULL);
    Serial.println("Initial RPM set to 0, Speed set to 0 km/h, indicators set to off state (no background, almost transparent)");
    if (ui_Screen1) {
      lv_scr_load(ui_Screen1);
      Serial.println("Screen1 explicitly loaded");
    } else {
      Serial.println("Error: ui_Screen1 is NULL!");
    }
    xSemaphoreGive(lvgl_mutex);
  }

    lastDataUpdateTime = millis();
    Serial.println("Setup done");

  // Create LVGL task on core 0
  xTaskCreatePinnedToCore(lvgl_task, "LVGL", 8192, NULL, 2, &lvglTask, 0); // Increased stack size

  Serial.println("Setup done");
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastHzCalcTime = 0;
  static uint32_t prev_packet_count = 0;

  if (dataReceived || (now - lastDataUpdateTime > DATA_TIMEOUT_MS)) {
    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY) == pdTRUE) {
      if (dataReceived) {
        dataReceived = false;
       // Serial.printf("Updating UI with RPM: %u, Speed: %u\n", receivedData.rpm, receivedData.vehicleSpeed); // Debug print

        // Update RPM if changed
        if (prev_rpm != receivedData.rpm) {
          lv_arc_set_value(ui_rpmslider, receivedData.rpm);
          char rpm_text[16];
          snprintf(rpm_text, sizeof(rpm_text), "%u", receivedData.rpm);
          lv_label_set_text(ui_RPM, rpm_text);
          lv_obj_invalidate(ui_rpmslider);
          lv_obj_invalidate(ui_RPM);
          prev_rpm = receivedData.rpm;
        }

        // Update speed if changed
        if (prev_speed != receivedData.vehicleSpeed) {
          prev_speed = receivedData.vehicleSpeed;
          lv_label_set_text_fmt(ui_Speed, "%d", receivedData.vehicleSpeed);
          lv_obj_invalidate(ui_Speed);
        }

        // Update parking brake if changed
        bool current_parking = receivedData.handbrakeSwitch;
        if (prev_parkingBrake != current_parking) {
          prev_parkingBrake = current_parking;
          if (current_parking) {
            lv_obj_set_style_bg_color(ui_ParkingBrake, COLOR_WARNING, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_ParkingBrake, OPA_WARNING, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_ParkingBrake, OPA_WARNING, 0);
          } else {
            lv_obj_set_style_bg_opa(ui_ParkingBrake, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_ParkingBrake, OPA_NORMAL, 0);
          }
          lv_obj_invalidate(ui_ParkingBrake);
        }

        // Update turn signal if changed (show if any indicator active)
        bool current_turn = (receivedData.turnSignalIndicator != 0);
        if (prev_turnSignal != current_turn) {
          prev_turnSignal = current_turn;
          if (current_turn) {
            lv_obj_set_style_bg_color(ui_TurnSignal, COLOR_WARNING, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_TurnSignal, OPA_WARNING, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_TurnSignal, OPA_WARNING, 0);
          } else {
            lv_obj_set_style_bg_opa(ui_TurnSignal, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_TurnSignal, OPA_NORMAL, 0);
          }
          lv_obj_invalidate(ui_TurnSignal);
        }

        // Update check engine if changed
        bool current_check = receivedData.checkEngineLight;
        if (prev_checkEngine != current_check) {
          prev_checkEngine = current_check;
          if (current_check) {
            lv_obj_set_style_bg_color(ui_CheckEngine, COLOR_WARNING, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_CheckEngine, OPA_WARNING, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_CheckEngine, OPA_WARNING, 0);
          } else {
            lv_obj_set_style_bg_opa(ui_CheckEngine, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_CheckEngine, OPA_NORMAL, 0);
          }
          lv_obj_invalidate(ui_CheckEngine);
        }

        // Update DSC if changed
        bool current_dsc = receivedData.ascLampStatus;
        if (prev_dsc != current_dsc) {
          prev_dsc = current_dsc;
          if (current_dsc) {
            lv_obj_set_style_bg_color(ui_DSC, COLOR_WARNING, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_DSC, OPA_WARNING, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_DSC, OPA_WARNING, 0);
          } else {
            lv_obj_set_style_bg_opa(ui_DSC, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_DSC, OPA_NORMAL, 0);
          }
          lv_obj_invalidate(ui_DSC);
        }

        // Update Watertemp if changed
        bool current_watertemp = receivedData.overheating;
        if (prev_watertemp != current_watertemp) {
          prev_watertemp = current_watertemp;
          if (current_watertemp) {
            lv_obj_set_style_bg_color(ui_Watertemp, COLOR_WARNING, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_Watertemp, OPA_WARNING, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_Watertemp, OPA_WARNING, 0);
          } else {
            lv_obj_set_style_bg_opa(ui_Watertemp, LV_OPA_0, LV_PART_MAIN);
            lv_obj_set_style_img_opa(ui_Watertemp, OPA_NORMAL, 0);
          }
          lv_obj_invalidate(ui_Watertemp);
        }
      } else {
        // Timeout: Reset UI to defaults
        Serial.println("Data timeout: Resetting UI to defaults");
        if (prev_rpm != 0) {
          lv_arc_set_value(ui_rpmslider, 0);
          lv_label_set_text(ui_RPM, "0");
          lv_obj_invalidate(ui_rpmslider);
          lv_obj_invalidate(ui_RPM);
          prev_rpm = 0;
        }
        if (prev_speed != 0) {
          lv_label_set_text(ui_Speed, "0");
          lv_obj_invalidate(ui_Speed);
          prev_speed = 0;
        }
        if (prev_parkingBrake) {
          lv_obj_set_style_bg_opa(ui_ParkingBrake, LV_OPA_0, LV_PART_MAIN);
          lv_obj_set_style_img_opa(ui_ParkingBrake, OPA_NORMAL, 0);
          lv_obj_invalidate(ui_ParkingBrake);
          prev_parkingBrake = false;
        }
        if (prev_turnSignal) {
          lv_obj_set_style_bg_opa(ui_TurnSignal, LV_OPA_0, LV_PART_MAIN);
          lv_obj_set_style_img_opa(ui_TurnSignal, OPA_NORMAL, 0);
          lv_obj_invalidate(ui_TurnSignal);
          prev_turnSignal = false;
        }
        if (prev_checkEngine) {
          lv_obj_set_style_bg_opa(ui_CheckEngine, LV_OPA_0, LV_PART_MAIN);
          lv_obj_set_style_img_opa(ui_CheckEngine, OPA_NORMAL, 0);
          lv_obj_invalidate(ui_CheckEngine);
          prev_checkEngine = false;
        }
        if (prev_dsc) {
          lv_obj_set_style_bg_opa(ui_DSC, LV_OPA_0, LV_PART_MAIN);
          lv_obj_set_style_img_opa(ui_DSC, OPA_NORMAL, 0);
          lv_obj_invalidate(ui_DSC);
          prev_dsc = false;
        }
        if (prev_watertemp) {
          lv_obj_set_style_bg_opa(ui_Watertemp, LV_OPA_0, LV_PART_MAIN);
          lv_obj_set_style_img_opa(ui_Watertemp, OPA_NORMAL, 0);
          lv_obj_invalidate(ui_Watertemp);
          prev_watertemp = false;
        }
        if (prev_hz != 0) {
          lv_label_set_text(ui_Hz, "CANBUS 0 Hz");
          lv_obj_invalidate(ui_Hz);
          prev_hz = 0;
        }
      }

      // Calculate and update the CAN bus update rate (Hz)
      if (now - lastHzCalcTime >= 1000) {
        uint32_t hz = espnow_packet_count - prev_packet_count;
        if (prev_hz != hz) {
          lv_label_set_text_fmt(ui_Hz, "CANBUS %u Hz", hz);
          lv_obj_invalidate(ui_Hz);
          prev_hz = hz;
        }
        prev_packet_count = espnow_packet_count;
        lastHzCalcTime = now;
      }

      xSemaphoreGive(lvgl_mutex);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(5)); // Keep main loop light
}