/**
 * @file main.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-04
 * 
 * @copyright Copyright (c) 2022
 * 
 * # Steuerung
 * @todo check of daten korrekt ankommen
 * @todo imu einrichten
 * @todo räder steuern via X-Box Controller
 * @todo rosout Topic verwenden
 * @todo robot_state (error states, state machine) custom message
 * 
 * # Obsticale Detection
 * @todo Prototype löten
 * @todo Prototype implementieren
 * @todo 3D Model erstellen
 * @todo Alle Sensoren anrbingen
 * @todo Logik implementieren
 */


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <PIDController.h>

#include "obstacle.h"
#include "battery.h"
#include "panel.h"
#include "odrive.h"
#include "state.h"
#include "imu.h"
#include "time.h"
#include "kinematics.h"
#include "odometry.h"
#include "time.h"

#define GPIO_COMPUTER 23

#define SPIN_TIME RCL_MS_TO_NS(10)
#define PUBLISH_TIME RCL_MS_TO_NS(50)
#define OBSTACLE_TIME RCL_MS_TO_NS(150)


#define Log Serial2

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Log.println("RCCHECK failed");}}

#define RUNNING_ANGLE 25


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t motion_timer;
rcl_timer_t publish_timer;
rcl_timer_t obstacle_timer;
rcl_node_t node;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t obstacle_publisher[SENSOR_COUNT];
rcl_subscription_t twist_subscriber;

//rcl_publisher_t state_publisher;
//rcl_subscription_t config_subscriber;

geometry_msgs__msg__Twist twist_msg;


Imu imu_sensor;
Panel panel;
Battery battery;
Odrive odrive;
Kinematics kinematics;
Odometry odometry;
Obstacle obstacle;

control_state_t state = DISCONNECTED;
control_error_t error;

position_state_t position_state = UNKNOWN;
double position = 0;

PIDController pid_cog;
PIDController pid_balance;

unsigned long prev_twist_msg_time = 0;
unsigned long prev_odom_update = 0;
static unsigned long long time_offset = 0;

void motion_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

    if(((millis() - prev_twist_msg_time) >= 200))  {
        // TODO no twist messages from PC...deal with it
    //    Log.println("PC didn't sent twist message for more than 200ms");
    }

     // update kinematics
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );
    
    odrive.setVelocity(req_rpm.motor1/60, req_rpm.motor2/60);

    //imu::Vector<3> vec = imu.getEuler();
    //Log.println(vec.x);
    //Log.println(imu_vector.orientation.z);     
    //twist_msg.linear.x = 0.0;
    //twist_msg.linear.y = 0.0;
    //twist_msg.angular.z = 0.0;    
    //position += odrive.estimated_left_velocity*last_call_time/1000;
    //double cog_angle = pid_cog.compute(position);
    //double velocity = pid_balance.compute(cog_angle);
    

    // update odometry
    Kinematics::velocities current_vel = kinematics.getVelocities(
        odrive.estimated_left_velocity, 
        odrive.estimated_right_velocity
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}


void twist_callback(const void * msgin) 
{
    prev_twist_msg_time = millis();
}



void obstacle_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    obstacle.update();
}


void publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    struct timespec time_stamp = getTime();        
    nav_msgs__msg__Odometry *odom_msg = odometry.getMsg();
    sensor_msgs__msg__Imu *imu_msg = imu_sensor.getMsg();
    sensor_msgs__msg__BatteryState *battery_msg = battery.getMsg();
    sensor_msgs__msg__Range__Sequence *obstacle_seq = obstacle.getSequence();

    odom_msg->header.stamp.sec = time_stamp.tv_sec;
    odom_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&odom_publisher, odom_msg, NULL);

    imu_msg->header.stamp.sec = time_stamp.tv_sec;
    imu_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&imu_publisher, imu_msg, NULL);
    
    battery_msg->header.stamp.sec = time_stamp.tv_sec;
    battery_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&battery_publisher, battery_msg, NULL);
    
    sensor_msgs__msg__Range msg;
    for(int i = 0; i < SENSOR_COUNT; i++) {
        msg = obstacle_seq->data[i];
        msg.header.stamp.sec = time_stamp.tv_sec;
        msg.header.stamp.nanosec = time_stamp.tv_nsec;
        rcl_publish(&obstacle_publisher[i], &msg, NULL);
    }
}


void request_odrive_status() {
    // get additional information from the Odrive
    odrive.update();
    
    if(odrive.motor_error) {
        set_error(ODRIVE_MOTOR_ERROR);
        return;
    }

    if(odrive.encoder_error) {
        set_error(ODRIVE_ENCODER_ERROR);
        return;
    }
    battery.setVoltage(odrive.voltage);
   //Log.println(odrive.voltage);
    if(battery.isCritical()) {
        Log.println(odrive.voltage);
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
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom" ) != RCL_RET_OK) 
        goto err;

    // create imu publisher
    if(rclc_publisher_init_best_effort(&imu_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK)
        goto err;

    // create battery publisher
    if(rclc_publisher_init_default(&battery_publisher, &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery") != RCL_RET_OK)
        goto err;

    // create obstacle publisher
    for(int i = 0; i < SENSOR_COUNT; i++) {
        String topic = String("obstacle_") + i;
        
        if(rclc_publisher_init_default(&obstacle_publisher[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic.c_str()) != RCL_RET_OK) {
            Log.print("error sensor: ");
            Log.println(topic.c_str());
            goto err;
        }
    }

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

    if(rclc_timer_init_default(&obstacle_timer, &support, OBSTACLE_TIME, obstacle_timer_callback) != RCL_RET_OK)
        goto err;         

    // create executor
    if(rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK)
        goto err;
    
    if(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, 
            &twist_callback, ON_NEW_DATA) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &motion_timer) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &publish_timer) != RCL_RET_OK)
        goto err;

    if(rclc_executor_add_timer(&executor, &obstacle_timer) != RCL_RET_OK)
        goto err;        

    if(!syncTime()) {
        Log.println("Time syncronisation failed");
        //goto err;
    }
      //  goto err;

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
    if (!imu_sensor.begin()) {
        set_error(IMU_INIT);
    }

    // init odrive
    if(!odrive.begin()) {
        set_error(ODRIVE_INIT);
    } 

    if(!obstacle.begin()) {
   //     set_error(OBSTACLE_INIT);
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
                odrive.setVelocity(0,0);
                odrive.stop();
                break;
            case IDLE:
                Log.println("State: IDLE");
                odrive.setVelocity(0,0);
                odrive.stop();
                break;
            case RUNNING:
                Log.println("State: RUNNING");
                odrive.setVelocity(0,0);
                odrive.start();
                break;
            case LOW_BATTERY:
                Log.println("State: LOW BATTERY");
                odrive.setVelocity(0,0);
                odrive.stop();
                digitalWrite(GPIO_COMPUTER, LOW);
                break;
            case ERROR:
                Log.println("State: ERROR");
                odrive.setVelocity(0,0);
                odrive.stop();
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



bool syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    if(rmw_uros_sync_session(1000 != RCL_RET_OK)) {
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