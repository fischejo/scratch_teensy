#ifndef PANEL_H
#define PANEL_H

#include "state.h"

#include <SmartButton.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_ADDRESS 0x3C 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define GPIO_BUTTON 3
#define GPIO_LED 2

using namespace smartbutton;

class Panel {
private:
    Adafruit_SSD1306 display;
    SmartButton button;

public:
    Panel();
    bool begin(SmartButton::EventCallback eventCallback);
    void update(control_state_t state, float battery_level);
};


#endif