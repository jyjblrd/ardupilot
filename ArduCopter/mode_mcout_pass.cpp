#include "Copter.h"

#if AP_MOTOR_CONTROL_OUTPUT_ENABLED

// Initialise the MCOUT override before the first mode update.
bool ModeMcoutPass::init(bool ignore_checks)
{
    copter.motor_control_output.set_pilot_passthrough(0.0f, 0.0f, 0.0f, 0.0f);
    attitude_control->reset_target_and_rate(true);
    attitude_control->reset_rate_controller_I_terms();
    return true;
}

// Pass calibrated pilot positions to MCOUT without attitude control.
void ModeMcoutPass::run()
{
    const bool rc_valid = !copter.failsafe.radio && rc().has_ever_seen_rc_input();
    const float roll = rc_valid ? channel_roll->norm_input_dz() : 0.0f;
    const float pitch = rc_valid ? channel_pitch->norm_input_dz() : 0.0f;
    const float yaw = rc_valid ? channel_yaw->norm_input_dz() : 0.0f;
    const float throttle = rc_valid ? constrain_float(channel_throttle->get_control_in() * 0.001f, 0.0f, 1.0f) : 0.0f;

    copter.motor_control_output.set_pilot_passthrough(roll, pitch, yaw, throttle);

    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Mark takeoff before the land detector sees unrestricted throttle output.
    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED &&
        !motors->limit.throttle_lower) {
        set_land_complete(false);
    }

    // Keep the normal motor state aligned with the exported command.
    motors->set_roll(roll);
    motors->set_roll_ff(0.0f);
    motors->set_pitch(pitch);
    motors->set_pitch_ff(0.0f);
    motors->set_yaw(yaw);
    motors->set_yaw_ff(0.0f);
    motors->set_forward(0.0f);
    motors->set_lateral(0.0f);
    attitude_control->set_throttle_out(throttle, false, 0.0f);
}

void ModeMcoutPass::exit()
{
    copter.motor_control_output.clear_pilot_passthrough();
    motors->set_roll(0.0f);
    motors->set_roll_ff(0.0f);
    motors->set_pitch(0.0f);
    motors->set_pitch_ff(0.0f);
    motors->set_yaw(0.0f);
    motors->set_yaw_ff(0.0f);
    motors->set_forward(0.0f);
    motors->set_lateral(0.0f);
    attitude_control->set_throttle_out(0.0f, false, 0.0f);
}

#endif // AP_MOTOR_CONTROL_OUTPUT_ENABLED
