# ESP32-S3 based spool hygrometer

## Features
- reports data every hour via MQTT
- on-demand measurment by tapping
- OLED display for conveniently show on-demand measurements
- made with off-the-shelf, easy to source components
- powered by a single 3.7v lithium battery

## Bill of material
- 150 tie points breadboard or proto board
- BMI160 breakout module
- BME280 breakout module
- SSD1306 OLED display module
- esp32-s3 supermini module

## Wiring
- Wire battery wires directly to the battery pads of the esp32-s3 supermini
- connect pin 12 and 13 respectiveky to the SDA and SCL pins of all other modules
- connect pin 6 to the INT1 pin of the BMI160 module
- do not connect the unused pins of the BME280 and BMI160 modules

Buils pictures, CAD and 3D print files are available at https://makerworld.com/en/models/549316

## Getting started
1. Open with arduino IDE
2. Install dependencies:
  - Adafruit SSD1306 (also install its dependencies when prompted)
  - Adafruit BME280 Library (also install its dependencies when prompted)
  - EmotiBit BMI160
  - PubSubClient
3. Create a `secrets.h` file, starting from the example in `secrets_example.h`
4. Connect your esp32-s3 supermini board to your PC
5. press and hold the "boot" button, then tap the "reset" button and release "boot". This will put the board in programming mode.
6. compile and upload the spool_hygro.ino sketch
