#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BMI160Gen.h>
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
#define IMU_I2C_ADDR 0x69
#define SCREEN_ADDR 0x3C

#define IMU_INTERRUPT_PIN GPIO_NUM_6

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  3600          


Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WiFiMulti WiFiMulti;
NetworkClient netClient;
PubSubClient client(netClient);
esp_sleep_wakeup_cause_t wakeup_reason;
float t, rH, Vbat, p;


void bmi160_intr(void)
{
  Serial.println("BMI160 interrupt: TAP!");
}

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

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  bme.takeForcedMeasurement();

  t = bme.readTemperature();
  rH = bme.readHumidity();
  p = bme.readPressure() / 100.0;
  analogReadResolution(12);
  Vbat = analogReadMilliVolts(GPIO_NUM_3) * 0.0022;

  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(rH);
  Serial.println(" %");

  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" hPa");

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

  if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) {
    show_display();
  }

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  for (int i=0; WiFiMulti.run() != WL_CONNECTED && i < 20; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(MQTT_HOST, MQTT_PORT);

  Serial.print("Attempting MQTT connection...");
  if (client.connect(HYGRO_ID)) {
    Serial.println("connected");
    String msg = String("{\"id\": \"") + String(HYGRO_ID) + "\", \"t\": " + String(t) + ", \"rH\": " + String(rH) + ", \"p\": " + String(p)  + ", \"Vbat\": " + String(Vbat) + "}";
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

  BMI160.begin(BMI160GenClass::I2C_MODE, Wire, IMU_I2C_ADDR, IMU_INTERRUPT_PIN);
  BMI160.attachInterrupt(bmi160_intr);
  BMI160.setIntTapEnabled(true);
  esp_sleep_enable_ext0_wakeup(IMU_INTERRUPT_PIN, LOW);
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
