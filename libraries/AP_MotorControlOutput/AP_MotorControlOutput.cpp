#include "AP_MotorControlOutput.h"

#if AP_MOTOR_CONTROL_OUTPUT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorControlOutput::var_info[] = {
    // @Param: RATE
    // @DisplayName: Motor control output rate
    // @Description: Serial publish rate for normalized roll, pitch, yaw, throttle and motor state sent to an external mixer
    // @Range: 1 400
    // @Units: Hz
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RATE", 1, AP_MotorControlOutput, _rate_hz, AP_MotorControlOutput::DEFAULT_RATE_HZ),

    AP_GROUPEND
};

AP_MotorControlOutput::AP_MotorControlOutput() :
    _uart(nullptr),
    _last_send_us(0),
    _sequence(0),
    _pilot_passthrough_active(false),
    _pilot_roll(0.0f),
    _pilot_pitch(0.0f),
    _pilot_yaw(0.0f),
    _pilot_throttle(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MotorControlOutput::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MotorControlOutput, 0);
    if (_uart != nullptr) {
        _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

bool AP_MotorControlOutput::should_send(const uint32_t now_us)
{
    const uint16_t rate_hz = constrain_int16(_rate_hz.get(), 1, MAX_RATE_HZ);
    if (rate_hz >= AP::scheduler().get_loop_rate_hz()) {
        return true;
    }

    const uint32_t period_us = 1000000UL / rate_hz;
    if (now_us - _last_send_us < period_us) {
        return false;
    }

    _last_send_us = now_us;
    return true;
}

uint16_t AP_MotorControlOutput::make_flags(const AP_Motors &motors, const bool output_enabled) const
{
    uint16_t flags = 0;

    if (motors.armed()) {
        flags |= FLAG_MOTORS_ARMED;
    }
    if (hal.util->get_soft_armed()) {
        flags |= FLAG_SOFT_ARMED;
    }
    if (motors.get_interlock()) {
        flags |= FLAG_INTERLOCK;
    }
    if (SRV_Channels::get_emergency_stop()) {
        flags |= FLAG_EMERGENCY_STOP;
    }
    if (motors.get_spoolup_block()) {
        flags |= FLAG_SPOOLUP_BLOCK;
    }
    if (motors.initialised_ok()) {
        flags |= FLAG_MOTORS_INITIALIZED;
    }
    if (output_enabled) {
        flags |= FLAG_OUTPUT_ENABLED;
    }
    if (motors.limit.roll) {
        flags |= FLAG_LIMIT_ROLL;
    }
    if (motors.limit.pitch) {
        flags |= FLAG_LIMIT_PITCH;
    }
    if (motors.limit.yaw) {
        flags |= FLAG_LIMIT_YAW;
    }
    if (motors.limit.throttle_lower) {
        flags |= FLAG_LIMIT_THROTTLE_LOWER;
    }
    if (motors.limit.throttle_upper) {
        flags |= FLAG_LIMIT_THROTTLE_UPPER;
    }
    if (motors.get_thrust_boost()) {
        flags |= FLAG_THRUST_BOOST;
    }
    if (_pilot_passthrough_active) {
        flags |= FLAG_PILOT_PASSTHROUGH;
    }

    return flags;
}

void AP_MotorControlOutput::set_pilot_passthrough(const float roll,
                                                  const float pitch,
                                                  const float yaw,
                                                  const float throttle)
{
    _pilot_roll = constrain_float(roll, -1.0f, 1.0f);
    _pilot_pitch = constrain_float(pitch, -1.0f, 1.0f);
    _pilot_yaw = constrain_float(yaw, -1.0f, 1.0f);
    _pilot_throttle = constrain_float(throttle, 0.0f, 1.0f);
    _pilot_passthrough_active = true;
}

void AP_MotorControlOutput::clear_pilot_passthrough()
{
    _pilot_passthrough_active = false;
    _pilot_roll = 0.0f;
    _pilot_pitch = 0.0f;
    _pilot_yaw = 0.0f;
    _pilot_throttle = 0.0f;
}

void AP_MotorControlOutput::fill_packet(const AP_Motors &motors, Packet &packet)
{
    packet = {};
    packet.magic = PACKET_MAGIC;
    packet.version = PACKET_VERSION;
    packet.length = sizeof(packet);
    packet.message_id = MESSAGE_ID_CONTROL;
    packet.sequence = _sequence++;
    packet.timestamp_usec = AP_HAL::micros64();
    packet.spool_state = uint8_t(motors.get_spool_state());
    packet.desired_spool_state = uint8_t(motors.get_desired_spool_state());

    const float roll_feedback = motors.get_roll();
    const float pitch_feedback = motors.get_pitch();
    const float yaw_feedback = motors.get_yaw();
    const float roll_feedforward = motors.get_roll_ff();
    const float pitch_feedforward = motors.get_pitch_ff();
    const float yaw_feedforward = motors.get_yaw_ff();
    const bool output_enabled = motors.armed() &&
                                motors.get_interlock() &&
                                !SRV_Channels::get_emergency_stop();

    // Primary commands are zero unless the external controller may drive actuators.
    if (output_enabled) {
        if (_pilot_passthrough_active) {
            packet.roll = _pilot_roll;
            packet.pitch = _pilot_pitch;
            packet.yaw = _pilot_yaw;
            packet.throttle = _pilot_throttle;
        } else {
            packet.roll = roll_feedback + roll_feedforward;
            packet.pitch = pitch_feedback + pitch_feedforward;
            packet.yaw = yaw_feedback + yaw_feedforward;
            packet.throttle = motors.get_throttle();
        }
    }

    packet.flags = make_flags(motors, output_enabled);
    packet.roll_feedback = roll_feedback;
    packet.pitch_feedback = pitch_feedback;
    packet.yaw_feedback = yaw_feedback;
    packet.roll_feedforward = roll_feedforward;
    packet.pitch_feedforward = pitch_feedforward;
    packet.yaw_feedforward = yaw_feedforward;
    packet.throttle_filtered = motors.get_throttle();
    packet.throttle_out = motors.get_throttle_out();
    packet.forward = motors.get_forward();
    packet.lateral = motors.get_lateral();
    packet.dt = motors.get_dt();
    packet.crc = crc_xmodem(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc));
}

void AP_MotorControlOutput::update()
{
    if (_uart == nullptr) {
        return;
    }

    AP_Motors *motors = AP_Motors::get_singleton();
    if (motors == nullptr || !motors->initialised_ok()) {
        return;
    }

    const uint32_t now_us = AP_HAL::micros();
    if (!should_send(now_us)) {
        return;
    }

    Packet packet {};
    fill_packet(*motors, packet);

    if (_uart->txspace() < sizeof(packet)) {
        return;
    }

    _uart->write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

#endif // AP_MOTOR_CONTROL_OUTPUT_ENABLED
