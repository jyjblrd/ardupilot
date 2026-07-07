#include <array>
#include <cinttypes>
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "mcout";

constexpr uart_port_t UART_PORT = UART_NUM_1;
constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_16;
constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_18;
constexpr int UART_BAUD = 921600;
constexpr size_t UART_RX_BUFFER_SIZE = 4096;
constexpr size_t READ_CHUNK = 256;

constexpr uint8_t MAGIC0 = 0x4D;
constexpr uint8_t MAGIC1 = 0xC0;
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t PACKET_LENGTH = 84;
constexpr uint16_t MESSAGE_ID_CONTROL = 1;
constexpr uint32_t PACKET_TIMEOUT_US = 50000;
constexpr uint32_t CONTROL_PERIOD_US = 2500;
constexpr TickType_t PRINT_PERIOD_TICKS = pdMS_TO_TICKS(1000);

constexpr uint16_t FLAG_MOTORS_ARMED = 1U << 0;
constexpr uint16_t FLAG_SOFT_ARMED = 1U << 1;
constexpr uint16_t FLAG_INTERLOCK = 1U << 2;
constexpr uint16_t FLAG_EMERGENCY_STOP = 1U << 3;
constexpr uint16_t FLAG_OUTPUT_ENABLED = 1U << 6;

struct __attribute__((packed)) ControlOutputPacket {
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

static_assert(sizeof(ControlOutputPacket) == PACKET_LENGTH, "MCOUT packet size mismatch");

struct LinkStats {
    uint32_t received;
    uint32_t received_interval;
    uint32_t missed_total;
    uint32_t missed_interval;
    uint32_t sequence_delta_interval;
    uint32_t last_sequence;
    bool have_sequence;
};

struct SharedState {
    ControlOutputPacket latest {};
    int64_t latest_rx_us;
    LinkStats stats {};
    portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
};

SharedState g_state;

constexpr uint16_t crc_xmodem_update(uint16_t crc, const uint8_t byte)
{
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

class PacketReader {
public:
    bool feed(const uint8_t byte, ControlOutputPacket &packet)
    {
        if (_index == 0) {
            if (byte != MAGIC0) {
                return false;
            }
            _frame[0] = byte;
            _crc = crc_xmodem_update(0, byte);
            _index = 1;
            return false;
        }

        if (_index == 1 && byte != MAGIC1) {
            reset();
            if (byte == MAGIC0) {
                _frame[0] = byte;
                _crc = crc_xmodem_update(0, byte);
                _index = 1;
            }
            return false;
        }

        _frame[_index] = byte;
        if (_index < offsetof(ControlOutputPacket, crc)) {
            _crc = crc_xmodem_update(_crc, byte);
        }

        if (_index == offsetof(ControlOutputPacket, version) && byte != PACKET_VERSION) {
            reset();
            return false;
        }
        if (_index == offsetof(ControlOutputPacket, length) && byte != PACKET_LENGTH) {
            reset();
            return false;
        }

        _index++;
        if (_index < _frame.size()) {
            return false;
        }

        uint16_t expected_crc;
        std::memcpy(&expected_crc, &_frame[offsetof(ControlOutputPacket, crc)], sizeof(expected_crc));
        bool valid = _crc == expected_crc;
        if (valid) {
            std::memcpy(&packet, _frame.data(), sizeof(packet));
            valid = packet.message_id == MESSAGE_ID_CONTROL;
        }
        reset();
        return valid;
    }

private:
    void reset()
    {
        _index = 0;
        _crc = 0;
    }

    std::array<uint8_t, PACKET_LENGTH> _frame {};
    size_t _index = 0;
    uint16_t _crc = 0;
};

const char *spool_name(const uint8_t spool_state)
{
    switch (spool_state) {
    case 0:
        return "STOP";
    case 1:
        return "IDLE";
    case 2:
        return "UP";
    case 3:
        return "RUN";
    case 4:
        return "DOWN";
    default:
        return "?";
    }
}

void append_flag(char *buffer, const size_t buffer_len, const char *flag)
{
    const size_t used = std::strlen(buffer);
    if (used >= buffer_len - 1) {
        return;
    }
    std::snprintf(buffer + used, buffer_len - used, "%s%s", used == 0 ? "" : " ", flag);
}

void flags_to_string(const uint16_t flags, char *buffer, const size_t buffer_len)
{
    buffer[0] = '\0';
    if (flags & FLAG_MOTORS_ARMED) {
        append_flag(buffer, buffer_len, "M_ARM");
    }
    if (flags & FLAG_SOFT_ARMED) {
        append_flag(buffer, buffer_len, "S_ARM");
    }
    if (flags & FLAG_INTERLOCK) {
        append_flag(buffer, buffer_len, "ILK");
    }
    if (flags & FLAG_EMERGENCY_STOP) {
        append_flag(buffer, buffer_len, "ESTOP");
    }
    if (flags & (1U << 4)) {
        append_flag(buffer, buffer_len, "SPL_BLK");
    }
    if (flags & (1U << 5)) {
        append_flag(buffer, buffer_len, "M_INIT");
    }
    if (flags & FLAG_OUTPUT_ENABLED) {
        append_flag(buffer, buffer_len, "OUT");
    }
    if (flags & (1U << 7)) {
        append_flag(buffer, buffer_len, "LIM_R");
    }
    if (flags & (1U << 8)) {
        append_flag(buffer, buffer_len, "LIM_P");
    }
    if (flags & (1U << 9)) {
        append_flag(buffer, buffer_len, "LIM_Y");
    }
    if (flags & (1U << 10)) {
        append_flag(buffer, buffer_len, "LIM_TL");
    }
    if (flags & (1U << 11)) {
        append_flag(buffer, buffer_len, "LIM_TH");
    }
    if (flags & (1U << 12)) {
        append_flag(buffer, buffer_len, "BOOST");
    }
    if (buffer[0] == '\0') {
        std::snprintf(buffer, buffer_len, "none");
    }
}

bool output_allowed(const ControlOutputPacket &packet, const int64_t now_us, const int64_t rx_us)
{
    const uint16_t required = FLAG_MOTORS_ARMED | FLAG_SOFT_ARMED | FLAG_INTERLOCK | FLAG_OUTPUT_ENABLED;
    return (now_us - rx_us) <= PACKET_TIMEOUT_US &&
           (packet.flags & required) == required &&
           (packet.flags & FLAG_EMERGENCY_STOP) == 0;
}

void set_outputs_safe()
{
    // Add actuator shutdown here before enabling real outputs.
}

void apply_mixer_outputs(const ControlOutputPacket &packet)
{
    // Add custom underactuated mixing here after bench validation.
    (void)packet;
}

void update_link_stats(const ControlOutputPacket &packet)
{
    portENTER_CRITICAL(&g_state.lock);
    LinkStats &stats = g_state.stats;
    stats.received++;
    stats.received_interval++;
    stats.sequence_delta_interval++;

    if (stats.have_sequence) {
        const uint32_t expected = stats.last_sequence + 1;
        if (packet.sequence != expected) {
            const uint32_t missed = packet.sequence - expected;
            if (missed < 100000U) {
                stats.missed_total += missed;
                stats.missed_interval += missed;
                stats.sequence_delta_interval += missed;
            }
        }
    }

    stats.last_sequence = packet.sequence;
    stats.have_sequence = true;
    g_state.latest = packet;
    g_state.latest_rx_us = esp_timer_get_time();
    portEXIT_CRITICAL(&g_state.lock);
}

void uart_rx_task(void *)
{
    PacketReader reader;
    std::array<uint8_t, READ_CHUNK> read_buf {};
    ControlOutputPacket packet {};

    while (true) {
        const int read_count = uart_read_bytes(UART_PORT, read_buf.data(), read_buf.size(), pdMS_TO_TICKS(20));
        for (int i = 0; i < read_count; i++) {
            if (reader.feed(read_buf[i], packet)) {
                update_link_stats(packet);
            }
        }
    }
}

void control_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        ControlOutputPacket packet {};
        int64_t rx_us = 0;
        const int64_t now_us = esp_timer_get_time();

        portENTER_CRITICAL(&g_state.lock);
        packet = g_state.latest;
        rx_us = g_state.latest_rx_us;
        portEXIT_CRITICAL(&g_state.lock);

        if (output_allowed(packet, now_us, rx_us)) {
            apply_mixer_outputs(packet);
        } else {
            set_outputs_safe();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_US / 1000));
    }
}

void monitor_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake, PRINT_PERIOD_TICKS);

        ControlOutputPacket packet {};
        int64_t rx_us = 0;
        LinkStats interval {};

        portENTER_CRITICAL(&g_state.lock);
        packet = g_state.latest;
        rx_us = g_state.latest_rx_us;
        interval = g_state.stats;
        g_state.stats.received_interval = 0;
        g_state.stats.missed_interval = 0;
        g_state.stats.sequence_delta_interval = 0;
        portEXIT_CRITICAL(&g_state.lock);

        if (!interval.have_sequence) {
            ESP_LOGI(TAG, "waiting for valid MCOUT packets");
            continue;
        }

        const float period_s = static_cast<float>(PRINT_PERIOD_TICKS) / static_cast<float>(configTICK_RATE_HZ);
        const float rx_hz = interval.received_interval / period_s;
        const float seq_hz = interval.sequence_delta_interval / period_s;
        const bool live = output_allowed(packet, esp_timer_get_time(), rx_us);
        char flags_buffer[96];
        flags_to_string(packet.flags, flags_buffer, sizeof(flags_buffer));

        ESP_LOGI(TAG,
                 "#%-8" PRIu32 " %-6s rx=%5.1fHz seq=%5.1fHz miss=%-4" PRIu32
                 " fc_dt=%5.2fms spool=%s/%s flags=%s",
                 packet.sequence,
                 live ? "OUTPUT" : "SAFE",
                 rx_hz,
                 seq_hz,
                 interval.missed_interval,
                 packet.dt * 1000.0f,
                 spool_name(packet.spool_state),
                 spool_name(packet.desired_spool_state),
                 flags_buffer);

        ESP_LOGI(TAG,
                 "cmd   roll=%+.3f pitch=%+.3f yaw=%+.3f thr=%.3f fwd=%+.3f lat=%+.3f",
                 packet.roll,
                 packet.pitch,
                 packet.yaw,
                 packet.throttle,
                 packet.forward,
                 packet.lateral);

        ESP_LOGI(TAG,
                 "terms fb=(%+.3f,%+.3f,%+.3f) ff=(%+.3f,%+.3f,%+.3f) thr_filt=%.3f thr_out=%.3f",
                 packet.roll_feedback,
                 packet.pitch_feedback,
                 packet.yaw_feedback,
                 packet.roll_feedforward,
                 packet.pitch_feedforward,
                 packet.yaw_feedforward,
                 packet.throttle_filtered,
                 packet.throttle_out);
    }
}

void init_uart()
{
    uart_config_t uart_config {};
    uart_config.baud_rate = UART_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUFFER_SIZE, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

} // namespace

extern "C" void app_main(void)
{
    init_uart();
    ESP_LOGI(TAG, "Listening for ArduPilot MCOUT on UART%d RX GPIO%d at %d baud", UART_PORT, UART_RX_PIN, UART_BAUD);

    xTaskCreatePinnedToCore(uart_rx_task, "mcout_uart", 4096, nullptr, 12, nullptr, 0);
    xTaskCreatePinnedToCore(control_task, "mcout_control", 4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(monitor_task, "mcout_monitor", 4096, nullptr, 4, nullptr, 0);
}
