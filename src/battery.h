#ifndef BATTERY_H
#define BATTERY_H


#include <sensor_msgs/msg/battery_state.h>
#include <micro_ros_utilities/string_utilities.h>

#define MIN_VOLTAGE 36
#define MAX_VOLTAGE 42
#define LOW_VOLTAGE 37

class Battery {
private:
    int voltage; // TODO switch to float
    unsigned long prev_low_voltage_time;
    bool low_voltage_detected = false;
    float level;
    bool low_battery = false;

    sensor_msgs__msg__BatteryState msg;

    float calculate_level(int voltage) {
        // right now, just a direct mapping
        return (float) (voltage - MIN_VOLTAGE) / (float) (MAX_VOLTAGE - MIN_VOLTAGE);
    }

public:
    Battery() {};

    void setVoltage(int voltage) {
        this->voltage = voltage;

        if(voltage > 30 ) {
            if(voltage <= LOW_VOLTAGE) {
                if(!low_voltage_detected) {
                    prev_low_voltage_time = millis();
                    low_voltage_detected = true;
                }
                if(millis() - prev_low_voltage_time >= 1000) {
                    low_battery = true;
                }
            } else {
                low_voltage_detected = false;
            }
        }
    };

    bool isCritical() { 
        return low_battery; 
    };

    float getLevel() {
        return level;
    };

    sensor_msgs__msg__BatteryState *getMsg() {
        msg.voltage = voltage;
        msg.present = true;

        return &msg;
    };
};

#endif