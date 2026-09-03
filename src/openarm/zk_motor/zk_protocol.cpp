// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#include <openarm/zk_motor/zk_protocol.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openarm::zk_motor {
namespace {

constexpr double POSITION_MIN = -12.5;
constexpr double POSITION_MAX = 12.5;
constexpr double VELOCITY_MIN = -65.0;
constexpr double VELOCITY_MAX = 65.0;
constexpr double KP_MIN = 0.0;
constexpr double KP_MAX = 500.0;
constexpr double KD_MIN = 0.0;
constexpr double KD_MAX = 5.0;
constexpr double TORQUE_MIN = -100.0;
constexpr double TORQUE_MAX = 100.0;
constexpr double SINGLE_VELOCITY_MIN = -18.0;
constexpr double SINGLE_VELOCITY_MAX = 18.0;
constexpr double SINGLE_TORQUE_MIN = -60.0;
constexpr double SINGLE_TORQUE_MAX = 60.0;
constexpr double SINGLE_FEEDBACK_VELOCITY_MIN = -54.0;
constexpr double SINGLE_FEEDBACK_VELOCITY_MAX = 54.0;
constexpr double SINGLE_FEEDBACK_TORQUE_MIN = -45.0;
constexpr double SINGLE_FEEDBACK_TORQUE_MAX = 45.0;

void put_u16_le(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value & 0xFFU);
    output[1] = static_cast<uint8_t>(value >> 8U);
}

uint16_t get_u16_le(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(input[1]) << 8U);
}

void validate_motor_id(uint8_t motor_id) {
    if (motor_id == 0 || motor_id > MAX_MOTORS) {
        throw std::invalid_argument("ZK multi-joint motor ID must be from 1 to 6");
    }
}

uint64_t float_to_uint(double value, double minimum, double maximum, unsigned bits) {
    const double clamped = std::clamp(value, minimum, maximum);
    const uint64_t maximum_raw = (uint64_t{1} << bits) - 1;
    return static_cast<uint64_t>((clamped - minimum) * static_cast<double>(maximum_raw) /
                                 (maximum - minimum));
}

uint64_t get_u64_be(const uint8_t* input) {
    uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        value = (value << 8U) | input[i];
    }
    return value;
}

}  // namespace

uint16_t float_to_u16(double value, double minimum, double maximum) {
    const double clamped = std::clamp(value, minimum, maximum);
    return static_cast<uint16_t>(
        std::lround((clamped - minimum) * 65535.0 / (maximum - minimum)));
}

double u16_to_float(uint16_t value, double minimum, double maximum) {
    return static_cast<double>(value) * (maximum - minimum) / 65535.0 + minimum;
}

JointSlot pack_mit_command(const damiao_motor::MITParam& command) {
    const std::array<uint16_t, 5> values = {
        float_to_u16(command.q, POSITION_MIN, POSITION_MAX),
        float_to_u16(command.dq, VELOCITY_MIN, VELOCITY_MAX),
        float_to_u16(command.kp, KP_MIN, KP_MAX),
        float_to_u16(command.kd, KD_MIN, KD_MAX),
        float_to_u16(command.tau, TORQUE_MIN, TORQUE_MAX),
    };
    JointSlot slot{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        put_u16_le(slot.data() + i * 2, values[i]);
    }
    return slot;
}

SingleMITFrame pack_single_mit_command(const damiao_motor::MITParam& command) {
    const uint64_t value =
        (float_to_uint(command.kp, KP_MIN, KP_MAX, 12) << 49U) |
        (float_to_uint(command.kd, KD_MIN, KD_MAX, 9) << 40U) |
        (float_to_uint(command.q, POSITION_MIN, POSITION_MAX, 16) << 24U) |
        (float_to_uint(command.dq, SINGLE_VELOCITY_MIN, SINGLE_VELOCITY_MAX, 12) << 12U) |
        float_to_uint(command.tau, SINGLE_TORQUE_MIN, SINGLE_TORQUE_MAX, 12);
    SingleMITFrame frame{};
    for (std::size_t i = 0; i < frame.size(); ++i) {
        frame[frame.size() - 1 - i] = static_cast<uint8_t>(value >> (i * 8U));
    }
    return frame;
}

SingleEnableFrame pack_single_enable_command(bool enabled) {
    return {0x8A, static_cast<uint8_t>(enabled ? 1 : 0)};
}

SingleZeroFrame pack_single_zero_command() { return {0x82}; }

JointSlot pack_special_command(uint8_t command) {
    JointSlot slot{};
    slot.fill(0xFF);
    slot.back() = command;
    return slot;
}

HostFrame pack_control_frame(const std::vector<uint8_t>& motor_ids,
                             const std::vector<damiao_motor::MITParam>& commands,
                             uint16_t message_id, uint8_t rolling_counter) {
    if (motor_ids.size() != commands.size()) {
        throw std::invalid_argument("ZK motor IDs and MIT commands must have the same size");
    }
    HostFrame frame{};
    put_u16_le(frame.data(), message_id);
    const JointSlot disabled = pack_special_command(DISABLE_COMMAND);
    for (std::size_t i = 0; i < MAX_MOTORS; ++i) {
        std::copy(disabled.begin(), disabled.end(), frame.begin() + 2 + i * 10);
    }
    for (std::size_t i = 0; i < motor_ids.size(); ++i) {
        validate_motor_id(motor_ids[i]);
        const JointSlot slot = pack_mit_command(commands[i]);
        std::copy(slot.begin(), slot.end(), frame.begin() + 2 + (motor_ids[i] - 1) * 10);
    }
    frame[62] = rolling_counter & 0x0F;
    // ponytail: ZK V1.4 omits CRC-8 parameters; keep the proven zero value until supplied.
    frame[63] = 0;
    return frame;
}

HostFrame pack_special_frame(const std::vector<uint8_t>& motor_ids, uint8_t command,
                             uint16_t message_id, uint8_t rolling_counter) {
    HostFrame frame{};
    put_u16_le(frame.data(), message_id);
    const JointSlot disabled = pack_special_command(DISABLE_COMMAND);
    for (std::size_t i = 0; i < MAX_MOTORS; ++i) {
        std::copy(disabled.begin(), disabled.end(), frame.begin() + 2 + i * 10);
    }
    const JointSlot selected = pack_special_command(command);
    for (const uint8_t motor_id : motor_ids) {
        validate_motor_id(motor_id);
        std::copy(selected.begin(), selected.end(), frame.begin() + 2 + (motor_id - 1) * 10);
    }
    frame[62] = rolling_counter & 0x0F;
    frame[63] = 0;
    return frame;
}

Feedback decode_feedback(const uint8_t* data, std::size_t size, uint8_t motor_id) {
    validate_motor_id(motor_id);
    if (data == nullptr || (size != 12 && size != 16)) {
        throw std::invalid_argument("ZK feedback must contain 12 or 16 bytes");
    }
    const int motor_temperature = static_cast<int>(data[8]) - 40;
    Feedback result{
        get_u16_le(data),
        motor_id,
        u16_to_float(get_u16_le(data + 2), POSITION_MIN, POSITION_MAX),
        u16_to_float(get_u16_le(data + 4), VELOCITY_MIN, VELOCITY_MAX),
        u16_to_float(get_u16_le(data + 6), TORQUE_MIN, TORQUE_MAX),
        motor_temperature,
        size == 16 ? static_cast<int>(data[9]) - 40 : motor_temperature,
        data[10],
        size == 16 ? static_cast<uint32_t>(data[11]) |
                         (static_cast<uint32_t>(data[12]) << 8U) |
                         (static_cast<uint32_t>(data[13]) << 16U)
                   : static_cast<uint32_t>(data[9]),
        size == 16 ? static_cast<uint8_t>(data[14] & 0x0F) : static_cast<uint8_t>(0),
    };
    return result;
}

Feedback decode_single_feedback(const uint8_t* data, std::size_t size, uint8_t motor_id) {
    if (motor_id == 0 || data == nullptr || size != 8) {
        throw std::invalid_argument("ZK single-joint feedback must contain 8 bytes");
    }
    const uint64_t value = get_u64_be(data);
    const uint8_t message_type = static_cast<uint8_t>(value >> 61U);
    if (message_type != 1) {
        throw std::invalid_argument("ZK single-joint feedback must be type 1");
    }
    const uint8_t error = static_cast<uint8_t>((value >> 56U) & 0x1FU);
    return Feedback{
        0,
        motor_id,
        u16_to_float(static_cast<uint16_t>((value >> 40U) & 0xFFFFU), POSITION_MIN,
                     POSITION_MAX),
        static_cast<double>((value >> 28U) & 0xFFFU) *
                (SINGLE_FEEDBACK_VELOCITY_MAX - SINGLE_FEEDBACK_VELOCITY_MIN) / 4095.0 +
            SINGLE_FEEDBACK_VELOCITY_MIN,
        static_cast<double>((value >> 16U) & 0xFFFU) *
                (SINGLE_FEEDBACK_TORQUE_MAX - SINGLE_FEEDBACK_TORQUE_MIN) / 4095.0 +
            SINGLE_FEEDBACK_TORQUE_MIN,
        static_cast<int>(std::lround((static_cast<int>((value >> 8U) & 0xFFU) - 50) / 2.0)),
        static_cast<int>(std::lround((static_cast<int>(value & 0xFFU) - 50) / 2.0)),
        0,
        static_cast<uint32_t>(error) << 1U,
        0,
    };
}

}  // namespace openarm::zk_motor
