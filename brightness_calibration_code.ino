#include "esp_camera.h"

bool hasPSRAM = false;

const int TEST_ROW = 240;
const int VGA_WIDTH = 640;

sensor_t* s;

void setup() {
  Serial.begin(115200);

  camera_config_t config;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.pin_xclk = 0;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_d7 = 35;
  config.pin_d6 = 34;
  config.pin_d5 = 39;
  config.pin_d4 = 36;
  config.pin_d3 = 21;
  config.pin_d2 = 19;
  config.pin_d1 = 18;
  config.pin_d0 = 5;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_pclk = 22;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_VGA;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    hasPSRAM = true;
  }
  else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
  }
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err == ESP_OK) {
    Serial.println("Initialization complete.");
    s = esp_camera_sensor_get();
  }
  else {
    Serial.println("Error occurred during initialization.");
  }
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb != NULL) {
    uint8_t* imageArr = fb->buf;
    size_t arrLen = fb->len;

    int startIndex = VGA_WIDTH * TEST_ROW;
    int endIndex = startIndex + VGA_WIDTH;
    for (int i = startIndex; i < endIndex; i += 20) { // Increment by 20 for faster processing 
      Serial.print("Pixel brightness: ");
      Serial.println(imageArr[i]); 
    }
    esp_camera_fb_return(fb);
  }
  delay(1000);
}