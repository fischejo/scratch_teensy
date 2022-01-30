#include <Arduino.h>
#include "TCA9548A.h"
#include <VL53L0X.h>

TCA9548A I2CMux; 
VL53L0X sensor;
#define Log Serial2

void detection_init() {
  
    I2CMux.begin(Wire1);             // Wire instance is passed to the library

    I2CMux.openChannel(0);
    sensor.setBus(&Wire);
    sensor.setTimeout(100);
    sensor.setMeasurementTimingBudget(20000);
    if(!sensor.init()) {
        while(1) {
            Log.println("VL53L0X init failed");
        }
    }
    sensor.startContinuous();
    I2CMux.closeChannel(0);
    
    
}

void detection_call() 
{
    I2CMux.openChannel(0);
    Log.print("Detection: ");
    Log.println(sensor.readRangeContinuousMillimeters());
    if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
    I2CMux.closeChannel(0);
}