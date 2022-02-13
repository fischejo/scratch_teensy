#ifndef ODRIVE_H
#define ODRIVE_H


class Odrive {

public:
    
    float voltage;
    float estimated_left_velocity;
    float estimated_right_velocity;
    bool motor_error;
    bool encoder_error;

    Odrive() {};

    bool begin();

    void setVelocity(float left, float right);
    void start();
    void stop();
    void update();
};


#endif