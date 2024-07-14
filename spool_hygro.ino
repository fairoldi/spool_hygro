#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <driver/rtc_io.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"

#define SDA GPIO_NUM_12
#define SCL GPIO_NUM_13
#define BME_I2C_ADDR 0x76
#define IMU_I2C_ADDR 0x6B
#define IMU_INTERRUPT_PIN GPIO_NUM_6

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


LSM6DS3Core imu(I2C_MODE, IMU_I2C_ADDR );
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);


  print_wakeup_reason();
  //delay(1000); // time to get serial running

  Wire.begin(SDA, SCL);
  
  if (!bme.begin(BME_I2C_ADDR, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }

  float t = bme.readTemperature();
  float rH = bme.readHumidity();

  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(rH);
  Serial.println(" %");

  setupIMU();
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
   
  display.cp437(true); 
  display.setTextSize(2);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);   

  display.print(t, 1); display.print(" "); display.write(248); display.println("C");
  display.print(rH, 1); display.println(" %rH");
  display.display();

  delay(2000);
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  esp_deep_sleep_start();
}

void setupIMU() {
  //Call .beginCore() to configure the IMU
	if( imu.beginCore() != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite = 0;  //Temporary variable

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

	// //Now, write the patched together data
	errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	errorAccumulator += imu.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

	// Enable tap detection on X, Y, Z axis, but do not latch output

	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E );
	
	// Set tap threshold
	// Write 0Ch into TAP_THS_6D
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x03 );

	// Set Duration, Quiet and Shock time windows
	// Write 7Fh into INT_DUR2
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F );
	
	// Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
	// Write 80h into WAKE_UP_THS
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80 );
	
	// Single tap interrupt driven to INT1 pin -- enable latch
	// Write 08h into MD1_CFG
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x48 );

	if( errorAccumulator )
	{
		Serial.println("Problem configuring the device.");
	}
	else
	{
		Serial.println("Device O.K.");
	}	


  esp_sleep_enable_ext0_wakeup(IMU_INTERRUPT_PIN, HIGH);
}


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}



