#include "imu.h"

Imu::Imu(): bno(55, 0x28, &Wire2) {
    msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "imu_link");

    msg.angular_velocity_covariance[0] = gyro_cov;
        msg.angular_velocity_covariance[4] = gyro_cov;
        msg.angular_velocity_covariance[8] = gyro_cov;
        msg.orientation_covariance[0] = pose_cov;
        msg.orientation_covariance[4] = pose_cov;
        msg.orientation_covariance[8] = pose_cov;
        msg.angular_velocity_covariance[0] = gyro_cov;
        msg.angular_velocity_covariance[4] = gyro_cov;
        msg.angular_velocity_covariance[8] = gyro_cov;
};

bool Imu::begin() {
    return bno.begin();
};

imu::Vector<3> Imu::getEuler() {
    return bno.getVector(Adafruit_BNO055::VECTOR_EULER);
};

sensor_msgs__msg__Imu *Imu::getMsg() {
        imu::Quaternion quat = bno.getQuat();

        msg.orientation.w = quat.w();
        msg.orientation.x = quat.x();
        msg.orientation.y = quat.y();
        msg.orientation.z = quat.z();

        imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        msg.linear_acceleration.x = acceleration.x();
        msg.linear_acceleration.y = acceleration.y();
        msg.linear_acceleration.z = acceleration.z();

        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        msg.angular_velocity.x = gyro.x();
        msg.angular_velocity.y = gyro.y();
        msg.angular_velocity.z = gyro.z();


        if(msg.angular_velocity.x > -0.01 && msg.angular_velocity.x < 0.01 )
            msg.angular_velocity.x = 0; 
         
        if(msg.angular_velocity.y > -0.01 && msg.angular_velocity.y < 0.01 )
            msg.angular_velocity.y = 0;

        if(msg.angular_velocity.z > -0.01 && msg.angular_velocity.z < 0.01 )
            msg.angular_velocity.z = 0;

        return &msg;
};