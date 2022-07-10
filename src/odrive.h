#ifndef ODRIVE_H
#define ODRIVE_H


class Odrive {

public:
    
    float voltage;
    float estimated_left_velocity;
    float estimated_right_velocity;
    float estimated_left_position;
    float estimated_right_position;

    bool motor_error;
    bool encoder_error;
    bool axis_error;

    Odrive() {};

    bool begin();

    void setVelocity(float left, float right);
    void start();
    void stop();
    void update();
    void reboot();
};


#endif