#ifndef ODRIVE_H
#define ODRIVE_H


struct odrive_state_t {
    float voltage;
    float estimated_left_velocity;
    float estimated_right_velocity;
    bool motor_error;
    bool encoder_error;
};

bool odrive_init();
    
void odrive_set_velocity(float left, float right);

void odrive_start();

void odrive_stop();

void odrive_update(odrive_state_t *info);

#endif