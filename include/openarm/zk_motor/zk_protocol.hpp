// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <openarm/damiao_motor/dm_motor_control.hpp>

namespace openarm::zk_motor {

inline constexpr std::size_t MAX_MOTORS = 6;
inline constexpr uint8_t SINGLE_MOTOR_ID = 7;
inline constexpr uint8_t GRIPPER_MOTOR_ID = 8;
inline constexpr std::size_t MAX_CONFIGURED_MOTORS = 8;
inline constexpr std::size_t HOST_FRAME_SIZE = 64;
inline constexpr uint16_t DEFAULT_MESSAGE_ID = 0x0001;
inline constexpr uint8_t ENABLE_COMMAND = 0xFC;
inline constexpr uint8_t DISABLE_COMMAND = 0xFD;
inline constexpr uint8_t SET_ZERO_COMMAND = 0xEF;

using JointSlot = std::array<uint8_t, 10>;
using HostFrame = std::array<uint8_t, HOST_FRAME_SIZE>;
using SingleMITFrame = std::array<uint8_t, 8>;
using SingleEnableFrame = std::array<uint8_t, 2>;
using SingleZeroFrame = std::array<uint8_t, 1>;

struct Feedback {
    uint16_t message_id;
    uint8_t motor_id;
    double position;
    double velocity;
    double torque;
    int motor_temperature;
    int mos_temperature;
    uint8_t voltage;
    uint32_t status;
    uint8_t rolling_counter;
};

uint16_t float_to_u16(double value, double minimum, double maximum);
double u16_to_float(uint16_t value, double minimum, double maximum);
JointSlot pack_mit_command(const damiao_motor::MITParam& command);
SingleMITFrame pack_single_mit_command(const damiao_motor::MITParam& command);
SingleEnableFrame pack_single_enable_command(bool enabled);
SingleZeroFrame pack_single_zero_command();
JointSlot pack_special_command(uint8_t command);
HostFrame pack_control_frame(const std::vector<uint8_t>& motor_ids,
                             const std::vector<damiao_motor::MITParam>& commands,
                             uint16_t message_id = DEFAULT_MESSAGE_ID,
                             uint8_t rolling_counter = 0);
HostFrame pack_special_frame(const std::vector<uint8_t>& motor_ids, uint8_t command,
                             uint16_t message_id = DEFAULT_MESSAGE_ID,
                             uint8_t rolling_counter = 0);
Feedback decode_feedback(const uint8_t* data, std::size_t size, uint8_t motor_id);
Feedback decode_single_feedback(const uint8_t* data, std::size_t size, uint8_t motor_id);

}  // namespace openarm::zk_motor
