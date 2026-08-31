#include "esp_camera.h"
#include <ESP32Servo.h>

bool hasPSRAM = false;
Servo balanceServo;

const byte SERVO_PIN = 14;

const int BRIGHTNESS_THRESHOLD = 50; // TODO: Establish what the ESP32 Camera perceives as the brightness of the black line against the white wall.
const int LOOP_DELAY = 130;
const int VGA_FRAMESIZE = 307200;
const int VGA_WIDTH = 640;

unsigned long lastLoop;

sensor_t* s;

void setup() {
  Serial.begin(115200);

  balanceServo.attach(SERVO_PIN);
  balanceServo.write(90); // Ensuring that the servoMotor starts at its center (90° given that its range is 0° to 180°).

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
  if (millis() - lastLoop >= LOOP_DELAY) {
    lastLoop = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb != NULL) {
      uint8_t* imageArr = fb->buf;

      int totalDarkPixels = 0;
      int x_col = 0;
      int y_row = -1; // In the first iteration of the loop, 0 % VGA_WIDTH evaluates to 0, so the row would change from 0 to 1. This would cause the first row to be skipped; to solve this issue, we start y_row at -1 because -1 + 1 = 0.
      unsigned long total_x = 0;
      unsigned long total_y = 0;
      float totXSquared = 0;
      float totYSquared = 0;
      float totXY = 0;
      for (int i = 0; i < VGA_FRAMESIZE; i++) {
        if (i % VGA_WIDTH == 0) { // Using the % operator ensures that the row changes whenever we reach the 'width' of each row.
            y_row += 1;
        }
        x_col = i % VGA_WIDTH;
        if (imageArr[i] < BRIGHTNESS_THRESHOLD) { // This narrows the pixels to those that are deemed dark by the loop above, skipping over the 'non-dark' pixels.
          totalDarkPixels += 1;
          total_x += x_col; // Finding the total sum of the x values to determine the centroid
          total_y += y_row; // Finding the total sum of the y values to determine the centroid

          totXSquared += pow(x_col, 2.0);
          totYSquared += pow(y_row, 2.0);
          totXY += (x_col * y_row);
        }
      }

      if (totalDarkPixels == 0) {
        esp_camera_fb_return(fb);
        return;
      }
      else {
        float mean_x = 0;
        float mean_y = 0;
        mean_x = float(total_x) / totalDarkPixels;
        mean_y = float(total_y) / totalDarkPixels;

        float demeanedTotXSquared = totXSquared - (totalDarkPixels * pow(mean_x, 2.0));
        float demeanedTotYSquared = totYSquared - (totalDarkPixels * pow(mean_y, 2.0));
        float demeanedTotXY = totXY - (totalDarkPixels * mean_x * mean_y);

        float tiltAngle = (atan2((2 * demeanedTotXY), (demeanedTotXSquared - demeanedTotYSquared)) / 2) * (180 / PI);
        float servoAngle = 90 - tiltAngle;
        Serial.print("Title angle: ");
        Serial.println(tiltAngle);
        balanceServo.write(servoAngle);

        int timeDiff = millis() - lastLoop;
        Serial.print("Time difference: ");
        Serial.println(timeDiff);

        esp_camera_fb_return(fb);
      }
    }
  }
}
