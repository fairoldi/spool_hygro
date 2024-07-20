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

