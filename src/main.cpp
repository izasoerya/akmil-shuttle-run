#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <AlashUltrasonic.h>

#define EEPROM_SIZE 1
#define CAMERA_MODEL_AI_THINKER
#include "camera_lib/camera_pins.h"

const byte oneWirePin = 12; // TODO: CHANGE PIN LATER
const byte sdaPin = 4;
const byte sclPin = 16;

LiquidCrystal_I2C lcd(0x27, 20, 4);
AlashUltrasonic sensorOneWire(oneWirePin, ONEWIRE_MODE);
int pictureNumber = 0;
float distanceCm = 1000.0;

TaskHandle_t ultraSonicHandle = NULL;

camera_config_t cameraInit()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  return config;
}

void ultraSonicTask(void *parameter)
{
  for (;;)
  {
    distanceCm = sensorOneWire.getDistance();
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  
  camera_config_t config = cameraInit(); 
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);
  s->set_awb_gain(s, 2);
  
  if (!SD_MMC.begin("/sdcard", true, false))
  {
    Serial.println("SD Card Mount Failed");
    return;
  }
  sensorOneWire.begin();
  digitalWrite(oneWirePin, HIGH); // Pull up I2C data line

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }

  EEPROM.begin(EEPROM_SIZE);

  xTaskCreatePinnedToCore(
      ultraSonicTask,    // Task function
      "UltraSonicTask",  // Task name
      1024,               // Stack size (bytes)
      NULL,              // Parameters
      1,                 // Priority
      &ultraSonicHandle, // Task handle
      1                  // Core 1
  );
}

void loop()
{
  Serial.printf("Distance: %.2f cm\n", distanceCm);
  if (distanceCm < 20)
  {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      return;
    }

    String path = "/picture" + String(pictureNumber) + ".jpg";
    fs::FS &fs = SD_MMC;
    Serial.printf("Saving: %s\n", path.c_str());

    File file = fs.open(path.c_str(), FILE_WRITE);
    if (!file)
    {
      Serial.println("Failed to open file in writing mode");
    }
    else
    {
      file.write(fb->buf, fb->len);
      file.close();
      Serial.printf("Saved file to path: %s\n", path.c_str());
      EEPROM.write(0, pictureNumber);
      EEPROM.commit();
    }
    esp_camera_fb_return(fb);

    pictureNumber++; // Increment for next image
    if (pictureNumber > 9999)
      pictureNumber = 0; // Optional: wrap after 9999
  }
}