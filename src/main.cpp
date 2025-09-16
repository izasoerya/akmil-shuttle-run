#include <WiFi.h>
#include <time.h>

// WiFi credentials (fill in your SSID and password below)
const char *ssid = "Subhanallah";
const char *password = "muhammadnabiyullah";

// WiFi connection logic
void connectToWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40)
  { // ~20 seconds timeout
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nWiFi connection failed!");
  }
}

// NTP
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // GMT+7
const int daylightOffset_sec = 0;

char timeString[16] = "--:--:--";
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
#include <Wire.h>

#define EEPROM_SIZE 1
#define CAMERA_MODEL_AI_THINKER
#include "camera_lib/camera_pins.h"

const byte oneWirePin = 12;
const byte sdaPin = 16;
const byte sclPin = 13;

LiquidCrystal_I2C lcd(0x27, 20, 4);
AlashUltrasonic sensorOneWire(oneWirePin, ONEWIRE_MODE);
int pictureNumber = 0;
volatile float distanceCm = 1000.0;

void showLcdContent(const char *timeStr, float distance);

void ultrasonicTask(void *pvParameters)
{
  while (1)
  {
    float d = sensorOneWire.getDistance();
    if (d > 20)
      distanceCm = d;
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms interval
  }
}

// RTOS: Main application task
void mainTask(void *pvParameters)
{
  while (1)
  {
    // Get current time
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
    else
    {
      strncpy(timeString, "--:--:--", sizeof(timeString));
    }

    // Show on LCD
    showLcdContent(timeString, distanceCm);

    Serial.printf("Time: %s | Distance: %.2f cm\n", timeString, distanceCm);
    if (distanceCm < 40)
    {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Camera capture failed");
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
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
    vTaskDelay(pdMS_TO_TICKS(1000)); // Main task loop delay (update LCD every second)
  }
}

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
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.fb_count = 1;

  return config;
}

// Function to show time and distance on the LCD
void showLcdContent(const char *timeStr, float distance)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(timeStr);
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance, 1);
  lcd.print(" cm   ");
}

void setup()
{
  // connectToWiFi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

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

  Wire.begin(sdaPin, sclPin, 100000); // Start I2C
  lcd.init();
  lcd.backlight();
  showLcdContent("--:--:--", distanceCm);

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }

  EEPROM.begin(EEPROM_SIZE);

  // RTOS: Start ultrasonic reading task
  xTaskCreatePinnedToCore(
      ultrasonicTask,   // Task function
      "UltrasonicTask", // Name
      2048,             // Stack size
      NULL,             // Parameters
      1,                // Priority
      NULL,             // Task handle
      1                 // Run on core 1
  );

  // RTOS: Start main application task
  xTaskCreatePinnedToCore(
      mainTask,   // Task function
      "MainTask", // Name
      4096,       // Stack size
      NULL,       // Parameters
      1,          // Priority
      NULL,       // Task handle
      1           // Run on core 1
  );
}

void loop()
{
  // Empty. Main logic runs in mainTask (FreeRTOS)
}