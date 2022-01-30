#include "display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "battery.h"
#include "battery_header.h"
#include "wifi_header.h"

#define SCREEN_ADDRESS 0x3C 

#define BATTERY_MIN 36.0f
#define BATTERY_MAX 42.0f

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define BATTERY_X (SCREEN_WIDTH-battery_width)/2
#define BATTERY_Y (SCREEN_HEIGHT-battery_height)/2


#define PROGRESS_WIDTH 100
#define PROGRESS_HEIGHT 40
#define PROGRESS_X (SCREEN_WIDTH-PROGRESS_WIDTH)/2
#define PROGRESS_Y (SCREEN_HEIGHT-PROGRESS_HEIGHT)/2

extern float battery_level;
extern control_state_t state;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);


void display_draw_battery_screen(float percent)
{
    static int last_inner_w;
    int inner_w = ((float) BATTERY_INNER_W)*percent;
    if(last_inner_w != inner_w) {
        display.clearDisplay();
        display.drawXBitmap(BATTERY_X,BATTERY_Y, battery_bits,battery_width,battery_height,SSD1306_WHITE);
	    display.fillRect(BATTERY_X + BATTERY_INNER_X, BATTERY_Y + BATTERY_INNER_Y,
                         inner_w,BATTERY_INNER_H,SSD1306_WHITE);
        display.display();
        last_inner_w = inner_w;
    }
}

void display_draw_low_battery_screen()
{
    display.clearDisplay();
    display.drawXBitmap(BATTERY_X,BATTERY_Y, battery_bits,battery_width,battery_height,SSD1306_WHITE);
    display.setCursor(BATTERY_X + BATTERY_INNER_X + ((BATTERY_INNER_W-12*3)/2),
                      BATTERY_Y + BATTERY_INNER_Y + ((BATTERY_INNER_H-16)/2));
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.print("Low");    
}

/*
void draw_header()
{
    int inner_w = ((float) BATTERY_HEADER_INNER_WIDTH)*battery_percent;
    display.drawXBitmap(0,0, BATTERY_HEADER_bits,BATTERY_HEADER_WIDTH,BATTERY_HEADER_HEIGHT,SSD1306_WHITE);
	display.fillRect(BATTERY_HEADER_INNER_X,BATTERY_HEADER_INNER_Y,inner_w,
                    BATTERY_HEADER_INNER_HEIGHT,SSD1306_WHITE);    

    if(wifi_connected) {
        display.drawXBitmap(SCREEN_WIDTH-WIFI_width,0, WIFI_bits,WIFI_width,WIFI_height,SSD1306_WHITE);
    }
}
*/

void display_draw_progress_screen(float progress)
{
    display.clearDisplay();
    int width = ((float) PROGRESS_WIDTH-4)*progress;
    
    display.drawRoundRect(PROGRESS_X, PROGRESS_Y, PROGRESS_WIDTH, PROGRESS_HEIGHT, 4, SSD1306_WHITE);
    display.fillRoundRect(PROGRESS_X+2, PROGRESS_Y+2, width, PROGRESS_HEIGHT-4, 2, SSD1306_WHITE);
    display.display();
}

void display_draw_error_screen(control_error_t error) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextColor(SSD1306_WHITE); 
    display.setTextSize(2);
    display.print("Error");
}


void display_show_state_screen() {

    String msg;
    switch (state)
    {
    case DISCONNECTED:
        msg = String("NO CONN.");
        break;
    case IDLE:
        msg = String("IDLE");
        break;
    case LOW_BATTERY:
        msg = String("LOW BAT");
        break;
    case RUNNING:
        msg = String("RUN");
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
}


bool display_init() {
    // init display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        return false;
    }
    display.clearDisplay();
    return true;
}
