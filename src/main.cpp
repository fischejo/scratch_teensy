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


#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <geometry_msgs/msg/twist.h>

#include "battery.h"
#include "panel.h"
#include "odrive.h"
#include "state.h"
#include "imu.h"
#include "kinematics.h"
#include "odometry.h"
#include "time.h"
#include "balance.h"
#include <math.h>
#include <teensy_message/msg/teensy_status.h>

#define GPIO_COMPUTER 23

#define SPIN_TIME RCL_MS_TO_NS(10)
#define PUBLISH_TIME RCL_MS_TO_NS(100)


#define Log Serial2


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
rcl_publisher_t teensy_publisher;
rcl_subscription_t twist_subscriber;
rclc_parameter_server_t param_server;
geometry_msgs__msg__Twist twist_msg;


Imu imu_sensor;
Panel panel;
Battery battery;
Odrive odrive;
Kinematics kinematics;
Odometry odometry;

teensy_message__msg__TeensyStatus teensy_msg;


double p = 0.9;
double i = 0.005;
double d = 150.0;
double setpoint = -3.9449734687805176;
PIDParameter param_balance(p,i,d, 5.0, setpoint);
Balance balance(param_balance);


control_state_t state = STATE_DISCONNECTED;
control_error_t error;

unsigned long prev_twist_msg_time = 0;
unsigned long prev_odom_update = 0;
static unsigned long long time_offset = 0;
double last_motion_call;

double measure_time = 0; // s
bool enable_measure = false;

const rclc_parameter_options_t param_options = {
    .notify_changed_over_dds = true,
    .max_params = 10
};


void set_state(control_state_t new_state, bool overwrite = false);
void set_error(control_error_t new_error);
void on_parameter_changed(Parameter *param);
bool sync_time();
struct timespec get_time();


void motion_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    float left = 0;
    float right = 0;
    
    last_motion_call = last_call_time/1000/1000;


    // update kinematics
    Kinematics::rpm req_rpm = kinematics.getRPM(
            twist_msg.linear.x, 
            twist_msg.linear.y, 
            twist_msg.angular.z
    ); 
    
    if(((millis() - prev_twist_msg_time) < 200))  {
        left = req_rpm.motor1/60/10;
        right = req_rpm.motor2/60/10;
    }
    
    //absolute_position += (odrive.estimated_left_velocity + odrive.estimated_right_velocity)/2*10*last_motion_call;
    //absolute_position = (odrive.estimated_left_position + odrive.estimated_right_position)/2*10;


    // calculate velocity
    balance.update(imu_sensor.getEuler());
    
    if (balance.is_fall_over()) {
        set_error(FALL_OVER);
    } else {
        left += balance.velocity;
        right += balance.velocity;
    }
    
    switch(state) {
        case STATE_MEASURE:
            measure_time += last_motion_call;
            if(measure_time > 10*1000) {
                param_balance.setpoint = imu_sensor.getEuler().y()*100;
                set_state(STATE_RUNNING);
            }
            break;
        case STATE_RUNNING:
            odrive.setVelocity(left, right);
            break;
        default:
            break;        
    }

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


void publish_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    struct timespec time_stamp = get_time();        
    nav_msgs__msg__Odometry *odom_msg = odometry.getMsg();
    sensor_msgs__msg__Imu *imu_msg = imu_sensor.getMsg();
    sensor_msgs__msg__BatteryState *battery_msg = battery.getMsg();

    odom_msg->header.stamp.sec = time_stamp.tv_sec;
    odom_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&odom_publisher, odom_msg, NULL);

    imu_msg->header.stamp.sec = time_stamp.tv_sec;
    imu_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&imu_publisher, imu_msg, NULL);
    
    battery_msg->header.stamp.sec = time_stamp.tv_sec;
    battery_msg->header.stamp.nanosec = time_stamp.tv_nsec;    
    rcl_publish(&battery_publisher, battery_msg, NULL);


    teensy_msg.angle = balance.angle;
    teensy_msg.spin = last_motion_call;
    teensy_msg.velocity = balance.velocity;
    teensy_msg.state = (uint8_t) state;
    teensy_msg.error = (uint8_t) error;

    rcl_publish(&teensy_publisher, &teensy_msg, NULL);
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

    if(odrive.axis_error) {
        set_error(ODRIVE_AXIS_ERROR);
        return;
    }

    
    battery.setVoltage(odrive.voltage);
    if(battery.isCritical()) {
        Log.print("voltage: ");
        Log.println(odrive.voltage);
        set_state(STATE_LOW_BATTERY);
    }
}

void ros_init() {

    // allocator
    allocator = rcl_get_default_allocator();

    // support
    if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        Log.println("rclc_support_init failed");
        goto err;
    }

    // create node
    if(rclc_node_init_default(&node, "teensy", "", &support) != RCL_RET_OK) {
        Log.println("rclc_node_init_default failed");
        goto err;
    }


  // Initialize parameter server with configured options
    if(rclc_parameter_server_init_with_option(&param_server, &node, &param_options) != RCL_RET_OK) {
        Log.println("parameter_init_with_option failed");
        goto err;
    }

    // create odometry publisher
    if( rclc_publisher_init_best_effort(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom" ) != RCL_RET_OK) {
        Log.println("rclc_publisher_init_best_effort odom failed");
        goto err;
    }

    // create imu publisher
    if(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK) {
        Log.println("rclc_publisher_init_best_effort imu failed");
        goto err;
    }

    // create battery publisher
    if(rclc_publisher_init_best_effort(&battery_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery") != RCL_RET_OK) {
        Log.println("rclc_publisher_init_default battery failed");
        goto err;
    }

    // create teensy_message publisher
    if( rclc_publisher_init_best_effort(&teensy_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(teensy_message, msg, TeensyStatus), "teensy" ) != RCL_RET_OK) {
        Log.println("rclc_publisher_init_best_effort teensy failed");
        goto err;
    }

    // create twist command subscriber
    if(rclc_subscription_init_best_effort(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK)
        goto err;

    // init motion
    if(rclc_timer_init_default(&motion_timer, &support, SPIN_TIME, motion_timer_callback) != RCL_RET_OK)
        goto err;

    if(rclc_timer_init_default(&publish_timer, &support, PUBLISH_TIME, publish_timer_callback) != RCL_RET_OK)
        goto err;      

    // create executor
    if(rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER+5, &allocator) != RCL_RET_OK) {
        Log.println("rclc_executo_init failed");
        goto err;
    }

    if(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA) != RCL_RET_OK)
        goto err;


    if(rclc_executor_add_timer(&executor, &motion_timer) != RCL_RET_OK)
        goto err;
    
    if(rclc_executor_add_timer(&executor, &publish_timer) != RCL_RET_OK)
       goto err;


    // Initialize parameter server with configured options
    if(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed) != RCL_RET_OK) {
        Log.println("rclc_executor_add_parameter_server failed");
        goto err;
    }
    rclc_add_parameter(&param_server, "p", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "i", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "d", RCLC_PARAMETER_DOUBLE);
    //rclc_add_parameter(&param_server, "limit", RCLC_PARAMETER_DOUBLE);
    rclc_add_parameter(&param_server, "setpoint", RCLC_PARAMETER_DOUBLE);



    if(!sync_time()) {
        Log.println("Time sycronisation failed");
        goto err;
    }
    
    set_state(STATE_IDLE);
    return;
err:
    set_error(ROS_INIT);
}

void ros_deinit() {
    set_state(STATE_DISCONNECTED);
    rcl_timer_fini(&motion_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rclc_parameter_server_fini(&param_server, &node);
}

void on_parameter_changed(Parameter *param)
{
    PIDParameter *_param = &param_balance;

    if (strcmp(param->name.data, "p") == 0 && param->value.type == RCLC_PARAMETER_DOUBLE) {
		_param->p = param->value.double_value;
	} else if (strcmp(param->name.data, "i") == 0 && param->value.type == RCLC_PARAMETER_DOUBLE) {
		_param->i = param->value.double_value;
	} else if (strcmp(param->name.data, "d") == 0 && param->value.type == RCLC_PARAMETER_DOUBLE) {
		_param->d = param->value.double_value;
	} else if (strcmp(param->name.data, "limit") == 0 && param->value.type == RCLC_PARAMETER_DOUBLE) {
		_param->limit = param->value.double_value;
	} else if (strcmp(param->name.data, "setpoint") == 0 && param->value.type == RCLC_PARAMETER_DOUBLE) {
        _param->setpoint = param->value.double_value;
	}

    balance.tune(param_balance);

    Log.print("Parameter ");
    Log.print(param->name.data);
    Log.print(" set to ");
    Log.println(param->value.double_value);
}



void fix_error()
{
    switch(error) {
        case ODRIVE_MOTOR_ERROR:
        case ODRIVE_AXIS_ERROR:
        case ODRIVE_ENCODER_ERROR:
            Log.println("fix error: odrive");
            odrive.reboot();
            set_state(STATE_IDLE, true);
            break;
        case FALL_OVER:
            Log.println("fix error: fall_over");
            set_state(STATE_IDLE, true);
            break;
        case IMU_NO_RESPONSE:
            set_state(STATE_IDLE, true);
            break;
        default:
            Log.println("cannot fix error");
            break;
    }
}


void button_clicked(SmartButton *button, SmartButton::Event event, int clickCounter)
{
    if (event == SmartButton::Event::CLICK) {   // Click event handler
        Log.println("Button clicked");
        switch(state) {
            case STATE_IDLE:
                set_state(enable_measure ? STATE_MEASURE : STATE_RUNNING);
                break;
            case STATE_RUNNING:
                set_state(STATE_IDLE);
                break;
            default:
                break;
        }
    } else if (event == SmartButton::Event::LONG_HOLD) {   // Click event handler
        Log.println("Button long clicked");
        switch(state) {
            case STATE_ERROR:
                fix_error();
                break;
            default:
                break;
        }
    }
}


void setup() {
    Log.begin(9600);
    Log.println('setup');
    set_state(STATE_INIT);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

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

   // if(!obstacle.begin()) {
   //     set_error(OBSTACLE_INIT);
    //}
    
    set_state(STATE_DISCONNECTED);
}

void set_state(control_state_t new_state, bool overwrite) {
    if(state != new_state && (overwrite || state != STATE_ERROR)) {
        switch(new_state) {
            case STATE_INIT:       
                Log.println("State: INIT");
                break;

            case STATE_DISCONNECTED:
                Log.println("State: STATE_DISCONNECTED");
                odrive.setVelocity(0,0);
                odrive.stop();
                break;
            case STATE_IDLE:
                Log.println("State: IDLE");
                odrive.setVelocity(0,0);
                odrive.stop();
                break;
            case STATE_MEASURE:            
                Log.println("State: STATE_MEASURE");
                measure_time = 0;
                odrive.setVelocity(0,0);                
                odrive.start();
                break;                
            case STATE_RUNNING:
                Log.println("State: STATE_RUNNING");
                balance.reset_position();
                odrive.setVelocity(0,0);                
                odrive.start();
                break;
            case STATE_LOW_BATTERY:
                Log.println("State: LOW BATTERY");
                odrive.setVelocity(0,0);
                odrive.stop();
                digitalWrite(GPIO_COMPUTER, LOW);
                break;
            case STATE_ERROR:
                Log.println("State: STATE_ERROR");
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
    set_state(STATE_ERROR);
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
            case ODRIVE_AXIS_ERROR:
                Log.println("Error: ODRIVE_AXIS");
                break;
            case ROS_INIT:
                Log.println("Error: ROS_INIT");
                break;
            case ROS_EXECUTOR:
                Log.println("Error: ROS_EXECUTOR");
                break;
            case IMU_NO_RESPONSE:
                Log.println("Error: IMO_NO_RESPONSE");
                break;
            case FALL_OVER:
                Log.println("Error: FALL_OVER");
                break;
            default:
                Log.print("Error unknown: ");
                Log.println(error);
                break;
        }
    }
}


void loop() {
    static unsigned long prev_connect_test_time; // check if the agent got disconnected at 10Hz

    if (millis() - prev_connect_test_time >= 100) {
        prev_connect_test_time = millis(); 
       
        panel.update(state, battery.getLevel());
        request_odrive_status();
       
        // reconnect if agent got disconnected or haven't at all
        rmw_ret_t ping = rmw_uros_ping_agent(5,2);
        if (RMW_RET_OK == ping) { 
            if (state == STATE_DISCONNECTED) {
                ros_init();
            }
        } else if (state != STATE_DISCONNECTED) {            
            ros_deinit();
        }
    }
    
    if (state != STATE_DISCONNECTED ) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}


bool sync_time()
{
    for(int attempt = 0; attempt < 12; attempt++) {
        // get the current time from the agent
        unsigned long now = millis();
        if(rmw_uros_sync_session(1000) == RCL_RET_OK) {
            unsigned long long ros_time_ms = rmw_uros_epoch_millis();
            // now we can find the difference between ROS time and uC time
            time_offset = ros_time_ms - now;
            return true;
        }
        Log.println("Time syncronisation timeout...retry in 5s");
        delay(5*1000); // retry in 5s
    }
    return false;
}

struct timespec get_time()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}