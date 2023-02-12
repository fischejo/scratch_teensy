#ifndef BALANCE_H
#define BALANCE_H

#include "imu.h"
#include <PIDController.h>

class PIDParameter {
public:    
    double p;
    double i;
    double d;
    double setpoint;
    double limit;

    PIDParameter(double _p, double _i, double _d, double _limit, double _setpoint) : p(_p), i(_i), d(_d), limit(_limit), setpoint(_setpoint) {}
};

class Balance {
public:
    PIDController pid_balance;
    
    PIDParameter &param;

    double angle;
    double velocity;
    Balance(PIDParameter &param);

    void tune(PIDParameter &param_balance);
    void reset_position();
    double update(imu::Vector<3> euler);
    bool is_fall_over();

};


#endif