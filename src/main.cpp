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
#include <WiFi.h>
#include <time.h>

#define EEPROM_SIZE 1
#define CAMERA_MODEL_AI_THINKER
#include "camera_lib/camera_pins.h"
#include <esp_task_wdt.h>

const char *ssid = "Subhanallah";
const char *password = "muhammadnabiyullah";
const byte oneWirePin = 12;
const byte sdaPin = 16;
const byte sclPin = 13;
const byte builtinLedPin = 33;
const byte builtinFlashPin = 4;
const byte addressLCD = 0x27;
const byte columnLCD = 20;
const byte rowLCD = 4;
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // GMT+7
const byte daylightOffset_sec = 0;
const byte thresholdShutter = 40;

char timeString[16] = "--:--:--";
LiquidCrystal_I2C lcd(addressLCD, columnLCD, rowLCD);
AlashUltrasonic sensorOneWire(oneWirePin, ONEWIRE_MODE);

void connectToWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40)
  {
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

void fbTask(float distanceCm)
{
  static byte pictureNumber = 0;
  if (distanceCm < thresholdShutter)
  {
    digitalWrite(builtinFlashPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(builtinFlashPin, LOW);

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
    if (pictureNumber > 254)
      pictureNumber = 0; // Optional: wrap after 9999
  }
}

void flashLedTask(void *pvParameters)
{
  while (1)
  {
    digitalWrite(builtinLedPin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500)); // ON for 100 ms
    digitalWrite(builtinLedPin, LOW);
    vTaskDelay(pdMS_TO_TICKS(500)); // OFF for 5 seconds
  }
}

void shutterCaptureTask(void *pvParameters)
{
  unsigned long lastPrint = millis();
  while (1)
  {
    float d = sensorOneWire.getDistance();
    if (d > 20)
      fbTask(d);

    if (millis() - lastPrint >= 1000)
    {
      showLcdContent(timeString, d);
      Serial.printf("Time: %s | Distance: %.2f cm\n", timeString, d);
      lastPrint = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void syncNTPTask(void *pvParameters)
{
  while (1)
  {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
      snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    else
      strncpy(timeString, "--:--:--", sizeof(timeString));
    esp_task_wdt_reset();            // Feed the watchdog
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
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

void setup()
{
  delay(1000);
  pinMode(builtinLedPin, OUTPUT);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("\nStarting setup...");

  // Inisialisasi I2C dan LCD
  Wire.begin(sdaPin, sclPin, 100000);
  lcd.init();
  lcd.backlight();
  lcd.print("System Booting...");

  Serial.print("Initializing SD_MMC Card (1-bit mode)...");
  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.println(" SD Card Mount Failed. Retrying...");
    delay(1000);
    if (!SD_MMC.begin("/sdcard", true))
    {
      Serial.println(" SD Card Mount Failed again. Restarting...");
      lcd.init();
      lcd.backlight();
      lcd.print("SD Card Fail!");
      delay(2000);
      ESP.restart();
    }
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }
  Serial.println(" SD Card Initialized.");

  // Inisialisasi Kamera
  camera_config_t config = cameraInit();
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    lcd.clear();
    lcd.print("Camera Fail!");
    delay(2000);
    ESP.restart();
  }

  // Atur setelan kamera
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);
  s->set_awb_gain(s, 2);
  pinMode(4, OUTPUT);

  // Inisialisasi EEPROM dan Sensor
  EEPROM.begin(EEPROM_SIZE);
  sensorOneWire.begin();
  digitalWrite(oneWirePin, HIGH);

  showLcdContent("--:--:--", 0);

  // Task untuk membaca sensor ultrasonic di Core 1
  xTaskCreatePinnedToCore(
      shutterCaptureTask, "shutterCaptureTask", 24 * 2048, NULL, 1, NULL, 1);

  // Task untuk LED flash di Core 1
  xTaskCreatePinnedToCore(
      flashLedTask, "FlashLedTask", 1024, NULL, 2, NULL, 0);

  // Task untuk LED flash di Core 0
  // xTaskCreatePinnedToCore(
  //     syncNTPTask, "NTPSyncTask", 4096, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }