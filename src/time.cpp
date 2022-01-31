#include "time.h"
#include <micro_ros_arduino.h>

static unsigned long long time_offset = 0;

bool syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    if(!rmw_uros_sync_session(10)) {
        return false;
    }
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
    return true;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}