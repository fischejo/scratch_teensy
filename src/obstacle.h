#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Arduino.h>
#include "TCA9548A.h"
#include <VL53L0X.h>
#include <sensor_msgs/msg/range.h>

static const int CHANNEL_COUNT = 1;
static const int SENSOR_COUNT = CHANNEL_COUNT*2;


class Obstacle {
private:    
    TCA9548A i2c_muc; 
    VL53L0X laser_sensor;
    
    sensor_msgs__msg__Range sensors[SENSOR_COUNT];
    sensor_msgs__msg__Range__Sequence sequence;

public:
    Obstacle();
    bool begin();
    void update();

    sensor_msgs__msg__Range__Sequence *getSequence();
};


#endif