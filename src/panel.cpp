#include "panel.h"

Panel::Panel()
: 
    display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1),
    button(GPIO_BUTTON, SmartButton::InputType::NORMAL_HIGH) 
{}


bool Panel::begin(SmartButton::EventCallback eventCallback) {
    // button & led
    pinMode(GPIO_LED, OUTPUT);
    digitalWrite(GPIO_LED, LOW);
    pinMode(GPIO_BUTTON, INPUT_PULLUP);
    button.begin(eventCallback);

    // init display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        return false;
    } 
    display.clearDisplay(); 
    return true;
 }


void Panel::update(control_state_t state, float battery_level) {

    String msg;
    switch (state)
    {
    case DISCONNECTED:
        msg = String("NO CONN.");
        analogWrite(GPIO_LED, 0);
        break;
    case IDLE:
        msg = String("IDLE");
        analogWrite(GPIO_LED, 50);
        break;
    case LOW_BATTERY:
        msg = String("LOW BAT");
        analogWrite(GPIO_LED, 50);
        break;
    case RUNNING:
        msg = String("RUN");
        analogWrite(GPIO_LED, 255);
        break;
    case BLOCKED:
        msg = String("BLOCKED");
        break;
    case ERROR:
        msg = String("ERROR");
        break;
    case INIT:
        msg = String("INIT");
    default:
        msg = String(state);
        break;
    }
    String msg_bat = String(battery_level*100, 0) + String("%");

    display.clearDisplay(); 
    display.setTextColor(SSD1306_WHITE); 
    display.setTextSize(2);    
    display.setCursor((SCREEN_WIDTH-msg.length()*12)/2,SCREEN_HEIGHT/2-20);    
    display.print(msg);    
    display.setCursor((SCREEN_WIDTH-msg_bat.length()*12)/2,SCREEN_HEIGHT/2+4);
    display.print(msg_bat);
    display.display();

    SmartButton::service();
}
