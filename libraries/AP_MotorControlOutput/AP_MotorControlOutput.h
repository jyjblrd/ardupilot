#pragma once

#include "AP_MotorControlOutput_config.h"

#if AP_MOTOR_CONTROL_OUTPUT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_Motors;

class AP_MotorControlOutput {
public:
    AP_MotorControlOutput();

    CLASS_NO_COPY(AP_MotorControlOutput);

    static const struct AP_Param::GroupInfo var_info[];

    enum Flags : uint16_t {
        FLAG_MOTORS_ARMED         = 1U << 0,
        FLAG_SOFT_ARMED           = 1U << 1,
        FLAG_INTERLOCK            = 1U << 2,
        FLAG_EMERGENCY_STOP       = 1U << 3,
        FLAG_SPOOLUP_BLOCK        = 1U << 4,
        FLAG_MOTORS_INITIALIZED   = 1U << 5,
        FLAG_OUTPUT_ENABLED       = 1U << 6,
        FLAG_LIMIT_ROLL           = 1U << 7,
        FLAG_LIMIT_PITCH          = 1U << 8,
        FLAG_LIMIT_YAW            = 1U << 9,
        FLAG_LIMIT_THROTTLE_LOWER = 1U << 10,
        FLAG_LIMIT_THROTTLE_UPPER = 1U << 11,
        FLAG_THRUST_BOOST         = 1U << 12,
    };

    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,
        GROUND_IDLE = 1,
        SPOOLING_UP = 2,
        THROTTLE_UNLIMITED = 3,
        SPOOLING_DOWN = 4,
    };

    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,
        GROUND_IDLE = 1,
        THROTTLE_UNLIMITED = 2,
    };

    static constexpr uint16_t PACKET_MAGIC = 0xC04D;
    static constexpr uint8_t PACKET_VERSION = 1;
    static constexpr uint16_t MESSAGE_ID_CONTROL = 1;
    static constexpr uint16_t DEFAULT_RATE_HZ = 400;
    static constexpr uint16_t MAX_RATE_HZ = 400;

    struct PACKED Packet {
        uint16_t magic;
        uint8_t version;
        uint8_t length;
        uint16_t message_id;
        uint32_t sequence;
        uint64_t timestamp_usec;
        uint16_t flags;
        uint8_t spool_state;
        uint8_t desired_spool_state;
        float roll;
        float pitch;
        float yaw;
        float throttle;
        float roll_feedback;
        float pitch_feedback;
        float yaw_feedback;
        float roll_feedforward;
        float pitch_feedforward;
        float yaw_feedforward;
        float throttle_filtered;
        float throttle_out;
        float forward;
        float lateral;
        float dt;
        uint16_t crc;
    };

    static_assert(sizeof(Packet) == 84, "AP_MotorControlOutput packet size changed");

    // Find the configured serial port and prepare output state.
    void init();

    // Publish the latest AP_Motors control state when the rate limiter allows.
    void update();

private:
    bool should_send(uint32_t now_us);
    void fill_packet(const AP_Motors &motors, Packet &packet);
    uint16_t make_flags(const AP_Motors &motors, bool output_enabled) const;

    AP_Int16 _rate_hz;
    AP_HAL::UARTDriver *_uart;
    uint32_t _last_send_us;
    uint32_t _sequence;
};

#endif // AP_MOTOR_CONTROL_OUTPUT_ENABLED
