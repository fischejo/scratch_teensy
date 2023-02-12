#include "balance.h"


#define RUNNING_ANGLE 25

#define FALL_OVER_ANGLE 80

#define Log Serial2


Balance::Balance(PIDParameter &_param) : param(_param) {
    pid_balance.begin();
	tune(_param);
}


void Balance::tune(PIDParameter &_balance) {
	param = _balance;
	
	pid_balance.setpoint(param.setpoint);
	pid_balance.tune(param.p, param.i, param.d);
}


void Balance::reset_position() {
	pid_balance.errSum = 0;
}

double Balance::update(imu::Vector<3> euler) {
	angle = euler.y()*100;
    
	if( angle < RUNNING_ANGLE && angle > -RUNNING_ANGLE) {
		velocity = pid_balance.compute(angle);
		velocity = fmaxf(-param.limit, fminf(velocity, param.limit));
	} else {
		velocity = 0.0;
	}
	
	return velocity;
    
}

bool Balance::is_fall_over() {
	return angle > FALL_OVER_ANGLE || angle < -FALL_OVER_ANGLE;
}