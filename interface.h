#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
#define OLED_MOSI  A2
#define OLED_CLK   A1
#define OLED_DC    A3
#define OLED_CS    A5
#define OLED_RESET A4

#define KEY_P   A7
#define KEY_M   A6
#define KEY_OK  A0

void interface_init(void);
uint16_t interface_input_mode(void);
uint16_t interface_input_angle(void);
uint16_t interface_input_length(void);
uint16_t interface_input_radius(void);

#endif