#include "balance.h"


#define RUNNING_ANGLE 25

#define Log Serial2


typedef struct {
	float kp;
	float kd;
	float ki;
	float setpoint;
	int frequency;
	float error_sum;
	float previous_error;
	float max_output;
	float min_output;
} pid_config_t;


void pid_init(pid_config_t *pid)
{
	pid->error_sum = 0;
	pid->previous_error = 0;
}


float pid_output(pid_config_t *pid, float process_variable)
{
	float error = pid->setpoint - process_variable;
	pid->error_sum += error *pid->frequency;
	float error_d = (error-pid->previous_error)/pid->frequency;
	//error_sum = fmaxf(-500, fminf(error_sum, 500));

	//calculate output from P, I and D values
	float output = pid->kp*error + pid->ki*pid->error_sum + pid->kd*error_d;
	pid->previous_error = error;

	// min, max 
	output = fmaxf(pid->min_output, fminf(output, pid->max_output));
	
	return output;
}


pid_config_t pid_balance_old;


Balance::Balance(PIDParameter &_param, PIDParameter &_cog) : param(_param), param_cog(_cog) {
  // init PIDs
    pid_cog.begin();
    pid_cog.setpoint(0);
    pid_cog.tune(0.1,0,0.1);
    pid_cog.limit(-5, 5); 

    pid_balance.begin();
	tune(_param, _cog);
}



void Balance::tune(PIDParameter &_balance, PIDParameter &_cog) {
	param = _balance;
	param_cog = _cog;
	
	pid_balance.setpoint(param.setpoint);
	//pid_balance.limit(-param.limit, param.limit);
	pid_balance.tune(param.p, param.i, param.d);




	pid_cog.tune(param_cog.p, param_cog.i, param_cog.d);
}


double Balance::update(imu::Vector<3> euler, double absolute) {
    position = absolute;
    //position += odrive.estimated_left_velocity*last_call_time/1000;

    //param.setpoint = -pid_cog.compute(position);
    
	//pid_balance.setpoint(param.setpoint);
    
    
	angle = euler.y()*100;
    velocity = pid_balance.compute(angle);
	velocity = fmaxf(-param.limit, fminf(velocity, param.limit));
	Log.print("errSum: ");
	Log.println(pid_balance.errSum);

    return velocity;
}

