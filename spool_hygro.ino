#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <driver/rtc_io.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/rtc.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include "secrets.h"

#define SDA GPIO_NUM_12
#define SCL GPIO_NUM_13
#define BME_I2C_ADDR 0x76
#define IMU_I2C_ADDR 0x6B
#define SCREEN_ADDR 0x3C

#define IMU_INTERRUPT_PIN GPIO_NUM_6

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  3600          


LSM6DS3Core imu(I2C_MODE, IMU_I2C_ADDR);
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiMulti WiFiMulti;
NetworkClient netClient;
PubSubClient client(netClient);
esp_sleep_wakeup_cause_t wakeup_reason;
float t, rH, Vbat;


void show_display() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  display.cp437(true);
  display.setTextSize(2);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(HYGRO_ID);
  display.print(t, 1);
  display.print(" ");
  display.write(248);
  display.println("C");
  display.print(rH, 1);
  display.println(" %rH");
  display.print(Vbat, 2);
  display.println(" V");
  display.display();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  pinMode(GPIO_NUM_48, OUTPUT);
  digitalWrite(GPIO_NUM_48, LOW);
  gpio_hold_en(GPIO_NUM_48);
  gpio_deep_sleep_hold_en();
  WiFiMulti.addAP(WIFI_SSID, WIFI_PASS);

  print_wakeup_reason();
  //delay(1000); // time to get serial running

  Wire.begin(SDA, SCL);

  if (!bme.begin(BME_I2C_ADDR, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }

  t = bme.readTemperature();
  rH = bme.readHumidity();
  analogReadResolution(12);
  Vbat = analogReadMilliVolts(GPIO_NUM_3) * 0.0022;

  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(rH);
  Serial.println(" %");

  Serial.print("Vbat = ");
  Serial.print(Vbat);
  Serial.println(" V");

  Serial.print("Vbat(mv) = ");
  Serial.print(analogReadMilliVolts(GPIO_NUM_3));
  Serial.println(" V");

  Serial.print("Vbat(raw) = ");
  Serial.print(analogRead(GPIO_NUM_3));
  Serial.println(" V");

  setupIMU();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    show_display();
  }

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(MQTT_HOST, MQTT_PORT);

  Serial.print("Attempting MQTT connection...");
  if (client.connect(HYGRO_ID)) {
    Serial.println("connected");
    String msg = String("[{\"id\": \"") + String(HYGRO_ID) + "\", \"t\": " + String(t) + ", \"rH\": " + String(rH) + ", \"Vbat\": " + String(Vbat) + "}]";
    Serial.println("attempting to publish");
    if (client.publish(MQTT_TOPIC, msg.c_str())) {
      Serial.print("message sent: ");
      Serial.println(msg.c_str());
    }
    else {
      Serial.print("publish failed, rc=");
      Serial.print(client.state());
    }
  } else {
    Serial.print("connect failed, rc=");
    Serial.print(client.state());
  }
  delay(1000);
  
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("next wakup in " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.flush();
  esp_deep_sleep_start();
}

void setupIMU() {

  if (imu.beginCore() != 0) {
    Serial.println("imu160: Error");
  } else {
    Serial.println("imu160: OK");
  }

  uint8_t errorAccumulator = 0;
  uint8_t dataToWrite = 0;

  //Setup the accelerometer******************************
  dataToWrite = 0;  //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorAccumulator += imu.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  // Enable tap detection on X, Y, Z axis, but do not latch output
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E);

  // Set tap threshold
  // Write 0Ch into TAP_THS_6D
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x03);

  // Set Duration, Quiet and Shock time windows
  // Write 7Fh into INT_DUR2
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  // Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
  // Write 80h into WAKE_UP_THS
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  // Single tap interrupt driven to INT1 pin -- enable latch
  // Write 08h into MD1_CFG
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);

  if (errorAccumulator) {
    Serial.println("Problem configuring the device.");
  } else {
    Serial.println("Device O.K.");
  }
  esp_sleep_enable_ext0_wakeup(IMU_INTERRUPT_PIN, HIGH);
}


void print_wakeup_reason() {
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
