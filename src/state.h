#ifndef STATE_H
#define STATE_H

enum control_state_t {
    STATE_UNKNOWN,
    INIT,
    DISCONNECTED, // keine Verbindung zu ROS
    LOW_BATTERY, // zu niedriger Batteriestand    
    IDLE, // Nicht aktiv, aber eine Verbindung zu ROS, Roboter sitzt
    RUNNING, // aktiv & Verbindung zu ROS, Roboter ist aufgestanden
    BLOCKED, // aktiv aber etwas verhindert die freie Bewegung
    ERROR, // Ein Fehler ist aufgetreten
};

enum control_error_t {
    IMU_INIT,
    ODRIVE_INIT,
    ROS_INIT,
    ROS_EXECUTOR,
    OBSTACLE_INIT,
    ODRIVE_ENCODER_ERROR,
    ODRIVE_MOTOR_ERROR,
    POSITION_BACK, // auf die Rückseite gefallen
    POSITION_FRONT, // auf die Vorderseite gefallen
    STUCK // räder drehen sich, aber keine IMU Bewegung
};

enum position_state_t {
    UNKNOWN = 0,
    SITTING_FRONT,
    SITTING_REAR,
    ERECTED,
    DROPPED_FRONT,
    DROPPED_REAR,
};

#endif
// -> DISCONNECTED -> IDLE -> RUNNING
//                         <- 
//                 <---------
//                                  