#ifndef STATE_H
#define STATE_H
#include <teensy_message/msg/teensy_status.h>

enum control_state_t {
    STATE_UNKNOWN = teensy_message__msg__TeensyStatus__STATE_UNKNOWN,
    STATE_INIT = teensy_message__msg__TeensyStatus__STATE_INIT,
    STATE_DISCONNECTED = teensy_message__msg__TeensyStatus__STATE_DISCONNECTED, // keine Verbindung zu ROS
    STATE_LOW_BATTERY = teensy_message__msg__TeensyStatus__STATE_LOW_BATTERY, // zu niedriger Batteriestand    
    STATE_IDLE = teensy_message__msg__TeensyStatus__STATE_IDLE, // Nicht aktiv, aber eine Verbindung zu ROS, Roboter sitzt
    STATE_MEASURE = teensy_message__msg__TeensyStatus__STATE_MEASURE,
    STATE_RUNNING = teensy_message__msg__TeensyStatus__STATE_RUNNING, // aktiv & Verbindung zu ROS, Roboter ist aufgestanden
    STATE_ERROR = teensy_message__msg__TeensyStatus__STATE_ERROR, // Ein Fehler ist aufgetreten
};

enum control_error_t {
    ERROR_UNKNOWN = teensy_message__msg__TeensyStatus__ERROR_UNKNOWN,
    IMU_INIT = teensy_message__msg__TeensyStatus__ERROR_IMU_INIT,
    ODRIVE_INIT = teensy_message__msg__TeensyStatus__ERROR_ODRIVE_INIT,
    ROS_INIT = teensy_message__msg__TeensyStatus__ERROR_ROS_INIT,
    ROS_EXECUTOR = teensy_message__msg__TeensyStatus__ERROR_ROS_EXECUTOR,
    OBSTACLE_INIT = teensy_message__msg__TeensyStatus__ERROR_OBSTACLE_INIT,
    ODRIVE_ENCODER_ERROR = teensy_message__msg__TeensyStatus__ERROR_ODRIVE_ENCODER,
    ODRIVE_MOTOR_ERROR = teensy_message__msg__TeensyStatus__ERROR_ODRIVE_MOTOR,
    ODRIVE_AXIS_ERROR = teensy_message__msg__TeensyStatus__ERROR_ODRIVE_AXIS,
    IMU_NO_RESPONSE = teensy_message__msg__TeensyStatus__ERROR_IMU_RESPONSE,
    FALL_OVER = teensy_message__msg__TeensyStatus__ERROR_FALL_OVER,
};

#endif
