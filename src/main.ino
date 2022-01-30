
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <SmartButton.h>

#include <PIDController.h>

#include "display.h"
#include "odrive.h"
#include "state.h"

#define GPIO_COMPUTER 23
#define GPIO_BUTTON 3
#define GPIO_LED 2

#define SPIN_TIME RCL_MS_TO_NS(100)

#define Log Serial2

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Log.println("RCCHECK failed");}}

#define MIN_VOLTAGE 36
#define MAX_VOLTAGE 42
#define LOW_VOLTAGE 37

#define RUNNING_ANGLE 25

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t motion_timer;
bool ros_initialized = false;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
sensors_event_t imu_vector;
odrive_state_t odrive_state;

using namespace smartbutton;
SmartButton button(3, SmartButton::InputType::NORMAL_HIGH);

control_state_t state = DISCONNECTED;
control_error_t error;
float battery_level;
position_state_t position_state = UNKNOWN;
double position = 0;
bool low_voltage_detected = false;
PIDController pid_cog;
PIDController pid_balance;

void motion_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

   // Log.println("whuhu");
   // bno.getEvent(&imu_vector);
    //Log.println(imu_vector.orientation.z); 

    position += odrive_state.estimated_left_velocity*last_call_time/1000;

    double cog_angle = pid_cog.compute(position);

    double velocity = pid_balance.compute(cog_angle);
    //odrive_set_velocity(velocity, velocity);
    

 

}

void ros_init() {

    // allocator
    allocator = rcl_get_default_allocator();

    // support
    if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        goto err;

    // create executor
    if(rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
        goto err;
    
    // init motion
    if(rclc_timer_init_default(&motion_timer, &support, SPIN_TIME, motion_timer_callback) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &motion_timer) != RCL_RET_OK)
        goto err;

    return;
err:
    set_error(ROS_INIT);
}

void ros_deinit() {
    rcl_timer_fini(&motion_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
}


void button_clicked(SmartButton *button, SmartButton::Event event, int clickCounter)
{
    if (event == SmartButton::Event::CLICK) {   // Click event handler
        Log.println("clicked");
        switch(state) {
            case IDLE:
                set_state(RUNNING);
                break;
            case RUNNING:
                set_state(IDLE);
                break;
            default:
                break;
        }
    }
}



void request_odrive_status() {
    static unsigned long prev_low_voltage_time;
    
    // get additional information from the Odrive
    odrive_update(&odrive_state);  
    
    if(odrive_state.motor_error) {
        set_error(ODRIVE_MOTOR_ERROR);
        return;
    }

    if(odrive_state.encoder_error) {
        set_error(ODRIVE_ENCODER_ERROR);
        return;
    }

    if(odrive_state.voltage > 0 ) {
        if(odrive_state.voltage <= LOW_VOLTAGE) {
            if(!low_voltage_detected) {
                prev_low_voltage_time = millis(); 
                low_voltage_detected = true;
            }
            if(millis() - prev_low_voltage_time >= 1000) {
                set_state(LOW_BATTERY);
            }
        } else {
            low_voltage_detected = false;
        }
        battery_level = (float) (odrive_state.voltage - MIN_VOLTAGE) / (float) (MAX_VOLTAGE - MIN_VOLTAGE);
    }
}

void setup() {
    Log.begin(9600);

    // init ros communication
    set_microros_transports();

    set_state(INIT);

    // init button
    pinMode(GPIO_LED, OUTPUT);
    digitalWrite(GPIO_LED, LOW);
    pinMode(GPIO_BUTTON, INPUT_PULLUP);
    button.begin(button_clicked);

    // init display
    //if(!display_init()) {
    //     Log.println("Failed to init display");
    //}

    // power computer on
    pinMode(GPIO_COMPUTER, OUTPUT);
    digitalWrite(GPIO_COMPUTER, HIGH);
    
    // init imu
    if (!bno.begin()) {
        set_error(IMU_INIT);
    }

    // init odrive
    if(!odrive_init()) {
        set_error(ODRIVE_INIT);
    } 

    // init PIDs
    pid_cog.begin();
    pid_cog.setpoint(0);
    pid_cog.tune(0,0,0);
    pid_cog.limit(-5, 5); 

    pid_balance.begin();
    pid_balance.setpoint(0);
    pid_balance.tune(0.2,0,20.0);
    pid_balance.limit(-5, 5); 
    
    set_state(DISCONNECTED);
}

void set_state(control_state_t new_state) {
    if(state != new_state && state != ERROR) {
        switch(new_state) {
            case INIT:       
                Log.println("State: INIT");
                break;

            case DISCONNECTED:
                Log.println("State: DISCONNECTED");
                odrive_set_velocity(0,0);
                odrive_stop();
                analogWrite(GPIO_LED, 0);
                break;
            case IDLE:
                Log.println("State: IDLE");
                //digitalWrite(GPIO_LED, LOW);
                analogWrite(GPIO_LED, 50);
                odrive_set_velocity(0,0);
                odrive_stop();
                break;
            case RUNNING:
                Log.println("State: RUNNING");
                analogWrite(GPIO_LED, 255);
                odrive_set_velocity(0,0);
                odrive_start();
                break;
            case LOW_BATTERY:
                Log.println("State: LOW BATTERY");
                odrive_set_velocity(0,0);
                analogWrite(GPIO_LED, 50);
                odrive_stop();
                digitalWrite(GPIO_COMPUTER, LOW);
                break;
            case ERROR:
                Log.println("State: ERROR");
                odrive_set_velocity(0,0);
                odrive_stop();
                break;
            default:
                break;
        }
        state = new_state;
    }
}

void set_error(control_error_t new_error) {
    set_state(ERROR);
    if(error != new_error) {
        error = new_error;
        switch(error) {
            case IMU_INIT:
                Log.println("Error: IMU_INIT");
                break;
            case ODRIVE_INIT:
                Log.println("Error: ODRIVE_INIT");
                break;
            case ODRIVE_ENCODER_ERROR:
                Log.println("Error: ODRIVE_ENCODER");
                break;
            case ODRIVE_MOTOR_ERROR:
                Log.println("Error: ODRIVE_MOTOR");
                break;
            case ROS_INIT:
                Log.println("Error: ROS_INIT");
                break;
            case ROS_EXECUTOR:
                Log.println("Error: ROS_EXECUTOR");
                break;
            default:
                Log.println(error);
                break;
        }
    }
}

void loop() {
    static unsigned long prev_connect_test_time; // check if the agent got disconnected at 10Hz

    if (millis() - prev_connect_test_time >= 10) {
        prev_connect_test_time = millis(); 
        
        request_odrive_status();
        //display_show_state_screen();

        // service the power button
        SmartButton::service();

        // reconnect if agent got disconnected or haven't at all
        rmw_ret_t ping = rmw_uros_ping_agent(10,2);
        if (RMW_RET_OK == ping) { 
            if (state == DISCONNECTED) {
                ros_init();
                set_state(IDLE);
            }
        } else if (state != DISCONNECTED) {
            set_state(DISCONNECTED);
            ros_deinit();
        }
    }
    delay(10);
    if (state != DISCONNECTED ) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}
