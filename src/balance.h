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
    PIDController pid_cog;
    PIDController pid_balance;
    
    PIDParameter &param;
    PIDParameter &param_cog;

    double angle;
    double velocity;
    Balance(PIDParameter &param, PIDParameter &cog);

    double position;

    void tune(PIDParameter &param, PIDParameter &cog);

    double update(imu::Vector<3> euler, double position);

};


#endif