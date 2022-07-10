#include "odrive.h"


#include <Arduino.h>
#include "ODriveCanbusTranslator.h"
#include <FlexCAN_T4.h>

#define Log Serial2


class UserImplementedClass : public ODriveCanbusTranslator<CAN_message_t> {
    public:
        CONSTRUCTORS(UserImplementedClass)

    CAN_message_t pack(uint32_t id, uint8_t len, const uint8_t *buf, bool rtr) {
        CAN_message_t msg;
        msg.id = id;
        msg.len = len;
        std::memcpy(msg.buf, buf, len);
        msg.flags.remote = rtr;
        return msg;
    }
};



static uint32_t node_ids[2] = {0,1};
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can;
static UserImplementedClass odrive(node_ids, 2);

void receive(const CAN_message_t &msg)
{
    odrive.filter(msg.id, msg.len, msg.buf);
}

bool Odrive::begin() {
    can.begin();
    can.setBaudRate(250000);
    can.setMaxMB(16);  
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.onReceive(receive);
    can.mailboxStatus();    
    
    reboot();
    return true; // TODO ensure that odrive works
}

void Odrive::reboot() {
    // or can.write(odrive(0).ClearErrors());
    can.write(odrive(0).RebootOdrive());
    delay(1);
}

void Odrive::start() {
    can.write(odrive(0).SetAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL));
    delay(1);
    can.write(odrive(0).SetControllerModes(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH));
    delay(1);
    can.write(odrive(1).SetAxisRequestedState(AXIS_STATE_CLOSED_LOOP_CONTROL));
    delay(1);
    can.write(odrive(1).SetControllerModes(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH));
    delay(1);
}


void Odrive::stop() {
    can.write(odrive(0).SetAxisRequestedState(AXIS_STATE_IDLE));
    can.write(odrive(1).SetAxisRequestedState(AXIS_STATE_IDLE));
    delay(1);
}   


void Odrive::setVelocity(float left, float right) {
    //Log.println(odrive(0).Heartbeat.state == AXIS_STATE_CLOSED_LOOP_CONTROL);
    can.write(odrive(0).SetInputVel(left));
    can.write(odrive(1).SetInputVel(-right));
    delay(1);
}


void Odrive::update() {
    // odrive
    can.write(odrive(0).GetVbusVoltage());
    can.events();

    can.write(odrive(0).GetEncoderEstimates());
    can.write(odrive(1).GetEncoderEstimates());
    can.events();

    
    can.write(odrive(0).GetMotorError());
    can.write(odrive(1).GetMotorError());
    can.events();

    can.write(odrive(0).GetEncoderError());
    can.write(odrive(1).GetEncoderError());    
    
    can.events();

    this->voltage = odrive(0).GetVbusVoltage.vbus;
    this->estimated_left_velocity = odrive(0).GetEncoderEstimates.vel;
    this->estimated_right_velocity = -odrive(1).GetEncoderEstimates.vel;
    this->estimated_left_position = odrive(0).GetEncoderEstimates.pos;
    this->estimated_right_position = -odrive(1).GetEncoderEstimates.pos;


    this->axis_error = odrive(0).Heartbeat.error != AXIS_ERROR_NONE ||
                       odrive(1).Heartbeat.error != AXIS_ERROR_NONE;
    this->motor_error = odrive(0).GetMotorError.error != MOTOR_ERROR_NONE ||
                        odrive(1).GetMotorError.error != MOTOR_ERROR_NONE;
    this->encoder_error = odrive(0).GetEncoderError.error != ENCODER_ERROR_NONE ||
                          odrive(1).GetEncoderError.error != ENCODER_ERROR_NONE;
}
