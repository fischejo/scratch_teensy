#ifndef CONTROL_H
#define CONTROL_H
#include "state.h"
bool display_init();

//void display_draw_battery_screen();
//void display_draw_low_battery_screen();
//void display_draw_progress_screen(float progress);
//void display_draw_error_screen(control_error_t error);
void display_show_state_screen();

#endif