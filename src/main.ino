#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include <PIDController.h>

#include "battery.h"
#include "panel.h"
#include "odrive.h"
#include "state.h"
#include "imu.h"
#include "time.h"


#define GPIO_COMPUTER 23

#define SPIN_TIME RCL_MS_TO_NS(10)
#define PUBLISH_TIME RCL_MS_TO_NS(100)

#define Log Serial2

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Log.println("RCCHECK failed");}}

#define RUNNING_ANGLE 25


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t motion_timer;
rcl_timer_t publish_timer;
rcl_node_t node;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t battery_publisher;
rcl_subscription_t twist_subscriber;

//rcl_publisher_t state_publisher;
//rcl_subscription_t config_subscriber;

nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;


Imu _imu;
Panel panel;
Battery battery;

odrive_state_t odrive_state;

control_state_t state = DISCONNECTED;
control_error_t error;

position_state_t position_state = UNKNOWN;
double position = 0;

PIDController pid_cog;
PIDController pid_balance;
unsigned long prev_twist_msg_time = 0;

void motion_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    //imu.getEuler();
    

    if(((millis() - prev_twist_msg_time) >= 200))  {
        // TODO no twist messages from PC...deal with it
    }

    //Log.println(imu_vector.orientation.z); 
    
    //twist_msg.linear.x = 0.0;
    //twist_msg.linear.y = 0.0;
    //twist_msg.angular.z = 0.0;    

    position += odrive_state.estimated_left_velocity*last_call_time/1000;

    double cog_angle = pid_cog.compute(position);

    double velocity = pid_balance.compute(cog_angle);
    //odrive_set_velocity(velocity, velocity);
}


void twist_callback(const void * msgin) 
{
    prev_twist_msg_time = millis();
}


void publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // odom
    //odom_msg.header.stamp.sec = time_stamp.tv_sec;
    //odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // TODO check if sending is successful
    rcl_publish(&imu_publisher, _imu.getMsg(), NULL);
    rcl_publish(&battery_publisher, battery.getMsg(), NULL);
    //rcl_publish(&odom_publisher, &odom_msg, NULL);
}


void request_odrive_status() {
    
    
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

    battery.setVoltage(odrive_state.voltage);
    if(battery.isCritical()) {
        set_state(LOW_BATTERY);
    }
}

void ros_init() {

    // allocator
    allocator = rcl_get_default_allocator();

    // support
    if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        goto err;

    // create node
    if(rclc_node_init_default(&node, "teensy_node", "", &support) != RCL_RET_OK) 
        goto err;

    // create odometry publisher
    if( rclc_publisher_init_best_effort(&odom_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered" ) != RCL_RET_OK) 
        goto err;

    // create imu publisher
    if(rclc_publisher_init_best_effort(&imu_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data") != RCL_RET_OK)
        goto err;

    // create battery publisher
    if(rclc_publisher_init_default(&battery_publisher, &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery") != RCL_RET_OK)
        goto err;

    // create twist command subscriber
    if(rclc_subscription_init_best_effort(&twist_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK)
        goto err;

    // create config subscriber
    //if(rclc_subscription_init_best_effort(&config_subscriber, &node,
    //        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK)
    //    goto err;

    // init motion
    if(rclc_timer_init_default(&motion_timer, &support, SPIN_TIME, motion_timer_callback) != RCL_RET_OK)
        goto err;

    if(rclc_timer_init_default(&publish_timer, &support, PUBLISH_TIME, publish_timer_callback) != RCL_RET_OK)
        goto err;        

    // create executor
    if(rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK)
        goto err;
    
    if(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, 
            ON_NEW_DATA) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &motion_timer) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &publish_timer) != RCL_RET_OK)
        goto err;

    if(!syncTime())
        goto err;

    set_state(IDLE);
    return;
err:
    set_error(ROS_INIT);
}

void ros_deinit() {
    set_state(DISCONNECTED);
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



void setup() {
    Log.begin(9600);
    set_state(INIT);

    // init ros communication
    set_microros_transports();

    // init panel
    panel.begin(button_clicked);

    // power computer on
    pinMode(GPIO_COMPUTER, OUTPUT);
    digitalWrite(GPIO_COMPUTER, HIGH);
    
    // init imu
    if (!_imu.begin()) {
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
                break;
            case IDLE:
                Log.println("State: IDLE");
                odrive_set_velocity(0,0);
                odrive_stop();
                break;
            case RUNNING:
                Log.println("State: RUNNING");
                odrive_set_velocity(0,0);
                odrive_start();
                break;
            case LOW_BATTERY:
                Log.println("State: LOW BATTERY");
                odrive_set_velocity(0,0);
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
        
        panel.update(state, battery.getLevel());
        request_odrive_status();

        // reconnect if agent got disconnected or haven't at all
        rmw_ret_t ping = rmw_uros_ping_agent(10,2);
        if (RMW_RET_OK == ping) { 
            if (state == DISCONNECTED) {
                ros_init();
            }
        } else if (state != DISCONNECTED) {            
            ros_deinit();
        }
    }

    if (state != DISCONNECTED ) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}
