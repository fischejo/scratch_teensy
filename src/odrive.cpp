#include "odrive.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ODriveCanbusTranslator.h"


class UserImplementedClass : public ODriveCanbusTranslator<CAN_message_t>
{
public:
    CONSTRUCTORS(UserImplementedClass)

    CAN_message_t pack(uint32_t id, uint8_t len, const uint8_t *buf, bool rtr)
    {
        CAN_message_t msg;
        msg.id = id;
        msg.len = len;
        std::memcpy(msg.buf, buf, len);
        msg.flags.remote = rtr;
        return msg;
    }
};

uint32_t node_ids[] = {0, 1};
UserImplementedClass odrive(node_ids, 2);
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

void odrive_can_sniff(const CAN_message_t &msg)
{
    odrive.filter(msg.id, msg.len, msg.buf);
}



bool odrive_init() {
    Can0.begin();
    Can0.setBaudRate(250000);
    Can0.setMaxMB(16);  
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(odrive_can_sniff);
    Can0.mailboxStatus();    

    Can0.write(odrive(0).RebootOdrive());
    return true; // TODO ensure that odrive works
}

void odrive_start() {
    Can0.write(odrive(0).SetAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL));
    Can0.write(odrive(1).SetAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL));

}

void odrive_stop() {
    Can0.write(odrive(0).SetAxisRequestedState(AXIS_STATE_IDLE));
    Can0.write(odrive(1).SetAxisRequestedState(AXIS_STATE_IDLE));

}

void odrive_set_velocity(float left, float right) {
    Can0.write(odrive(0).SetInputVel(left));
    Can0.write(odrive(1).SetInputVel(-right));
}

void odrive_update(odrive_state_t *state) {
    // odrive
    Can0.write(odrive(0).GetVbusVoltage());
    Can0.events();

    Can0.write(odrive(0).GetEncoderEstimates());
    Can0.write(odrive(1).GetEncoderEstimates());
    Can0.events();

    Can0.write(odrive(0).GetMotorError());
    Can0.write(odrive(1).GetMotorError());
    Can0.events();

    Can0.write(odrive(0).GetEncoderError());
    Can0.write(odrive(1).GetEncoderError());    
    
    Can0.events();

    state->voltage = odrive(0).GetVbusVoltage.vbus;
    state->estimated_left_velocity = odrive(0).GetEncoderEstimates.vel;
    state->estimated_right_velocity = -odrive(1).GetEncoderEstimates.vel;
    state->motor_error = (odrive(0).GetMotorError.error == MOTOR_ERROR_NONE &&
                          odrive(1).GetMotorError.error == MOTOR_ERROR_NONE) ? false : true;
    state->encoder_error = (odrive(0).GetEncoderError.error == ENCODER_ERROR_NONE &&
                            odrive(1).GetEncoderError.error == ENCODER_ERROR_NONE) ? false : true;
}
