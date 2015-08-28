#include "interface.h"
#include "imu.h"

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void interface_init(void) {

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  pinMode(KEY_P,  INPUT_PULLUP);
  pinMode(KEY_M,  INPUT_PULLUP);
  pinMode(KEY_OK, INPUT_PULLUP);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Wind\nPendulum\n\n");
  display.setTextSize(1);
  display.print("2015-08 AHU");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(500);
}

uint16_t interface_input_mode(void) {
  uint16_t mode = 1;
  display.display();
  for (;;) {
    if (digitalRead(KEY_P) == 0) {
      mode += 1;
    }
    if (digitalRead(KEY_M) == 0) {
      mode -= 1;
    }
    if (mode > 5 ) {
      mode = 1;
    }
    if (mode == 0 ) {
      mode = 5;
    }
    if (digitalRead(KEY_OK) == 0) {
      display.clearDisplay();
      display.setTextSize(3);
      display.setCursor(35, 20);
      display.print("OK!");
      display.display();
      delay(1200);
      return mode;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Select mode:");
    display.setCursor(0, 32);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(mode);
    display.display();
    delay(300);
  }
}

uint16_t interface_input_angle(void) {
  uint16_t angle = 0;
  display.display();
  for (;;) {
    if (digitalRead(KEY_P) == 0) {
      angle += 10;
    }
    if (digitalRead(KEY_M) == 0) {
      angle -= 10;
    }
    if (angle > 180 ) {
      angle = 0;
    }
    if (digitalRead(KEY_OK) == 0) {
      display.clearDisplay();
      display.setTextSize(3);
      display.setCursor(35, 20);
      display.print("OK!");
      display.display();
      delay(1200);
      return angle;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Input angle:");
    display.setCursor(0, 32);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(angle);
    display.display();
    delay(300);
  }
}

uint16_t interface_input_length(void) {
  uint16_t length = 30;
  display.display();
  for (;;) {
    if (digitalRead(KEY_P) == 0) {
      length += 1;
    }
    if (digitalRead(KEY_M) == 0) {
      length -= 1;
    }
    if (length < 30 ) {
      length = 60;
    } else if (length > 60 ) {
      length = 30;
    }
    if (digitalRead(KEY_OK) == 0) {
      display.clearDisplay();
      display.setTextSize(3);
      display.setCursor(35, 20);
      display.print("OK!");
      display.display();
      delay(1200);
      return length;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Input length:");
    display.setCursor(0, 32);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(length);
    display.display();
    delay(300);
  }
}

uint16_t interface_input_radius(void) {
  uint16_t radius = 15;
  display.display();
  for (;;) {
    if (digitalRead(KEY_P) == 0) {
      radius += 1;
    }
    if (digitalRead(KEY_M) == 0) {
      radius -= 1;
    }
    if (radius < 15 ) {
      radius = 35;
    } else if (radius > 35 ) {
      radius = 15;
    }
    if (digitalRead(KEY_OK) == 0) {
      display.clearDisplay();
      display.setTextSize(3);
      display.setCursor(35, 20);
      display.print("OK!");
      display.display();
      delay(1200);
      return radius;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Input radius:");
    display.setCursor(0, 32);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(radius);
    display.display();
    delay(300);
  }
}