float32 spin
float32 angle
float32 velocity

uint8 STATE_UNKNOWN = 0
uint8 STATE_INIT = 1
uint8 STATE_DISCONNECTED = 2
uint8 STATE_LOW_BATTERY = 3
uint8 STATE_IDLE = 4
uint8 STATE_MEASURE = 5
uint8 STATE_RUNNING = 6
uint8 STATE_ERROR = 7

uint8 state

uint8 ERROR_UNKNOWN = 0
uint8 ERROR_IMU_INIT = 1
uint8 ERROR_ODRIVE_INIT = 2
uint8 ERROR_ROS_INIT = 3
uint8 ERROR_ROS_EXECUTOR = 4
uint8 ERROR_OBSTACLE_INIT = 5
uint8 ERROR_ODRIVE_ENCODER = 6
uint8 ERROR_ODRIVE_MOTOR = 7
uint8 ERROR_ODRIVE_AXIS = 8
uint8 ERROR_IMU_RESPONSE = 9
uint8 ERROR_FALL_OVER = 10

uint8 error