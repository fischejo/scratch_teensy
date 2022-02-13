#include <micro_ros_utilities/string_utilities.h>
#include "obstacle.h"



#define Log Serial2
#define Wire Wire1


Obstacle::Obstacle() {

}



bool Obstacle::begin() {
    i2c_muc.begin(Wire);             // Wire instance is passed to the library

    sensor_msgs__msg__Range *msg;
    for(int channel = 0; channel < CHANNEL_COUNT; channel++) {
        i2c_muc.openChannel(channel);
        laser_sensor.setBus(&Wire);
        laser_sensor.setTimeout(100);
        laser_sensor.setMeasurementTimingBudget(20000);
        if(!laser_sensor.init()) {
            return false;
        }
        laser_sensor.startContinuous();
        i2c_muc.closeChannel(channel);

        // laser 
        msg = &sensors[channel*2+0];
        msg->header.frame_id = micro_ros_string_utilities_set(msg->header.frame_id, "infrared_link");
        msg->radiation_type = 1;
        msg->field_of_view = 0.43; // 25 degree
        msg->min_range = 0.0;
        msg->max_range = 2.0;

        // sonar
        msg = &sensors[channel*2+1];
        msg->header.frame_id = micro_ros_string_utilities_set(msg->header.frame_id, "ultrasound_link");
        msg->radiation_type = 0;
        msg->field_of_view = 0.52; // 30 degree
        msg->min_range = 0.0;
        msg->max_range = 2.0;
    }
    sequence.data = sensors;
    sequence.size = SENSOR_COUNT;

    return true;
}


void Obstacle::update() {
    uint32_t data;
    float sonar_distance;
    float laser_distance;

    for(int channel = 0; channel < CHANNEL_COUNT; channel++) {    
        i2c_muc.openChannel(channel);
        
        // get laser
        sensors[channel*2+0].range = (float) laser_sensor.readRangeContinuousMillimeters() / 1000;

        // get sonar 
        Wire.requestFrom(0x57,3); 
        data  = Wire.read();data <<= 8;
        data |= Wire.read();data <<= 8;
        data |= Wire.read();
        sensors[channel*2+1].range = float(data) / 1000 / 1000;

        // send sonar
        Wire.beginTransmission(0x57);
        Wire.write(1);          //1 = cmd to start meansurement
        Wire.endTransmission();
        // normally wait 120ms, but this function has a 100ms duty cycle

        
        
                    
        if (laser_sensor.timeoutOccurred()) { 
            Log.println("Laser TIMEOUT"); 
        }
        
        i2c_muc.closeChannel(channel);
    }    
}

sensor_msgs__msg__Range__Sequence *Obstacle::getSequence() {
    return &sequence;
}