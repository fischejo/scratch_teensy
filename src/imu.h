#ifndef IMU_H
#define IMU_H

#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>
#include "time.h"

#include <Wire.h>


#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


class Imu {
private:
    sensor_msgs__msg__Imu msg;
    Adafruit_BNO055 bno;

    float accel_cov = 0.6;
    float gyro_cov = 0.003;
    float pose_cov = 0.002;

public:

    Imu();
    bool begin();
    imu::Vector<3> getEuler();
    sensor_msgs__msg__Imu *getMsg();
};

#endif