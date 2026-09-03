// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#include <openarm/zk_motor/zk_protocol.hpp>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

int main() {
    using namespace openarm;

    const damiao_motor::MITParam zero{0, 0, 0, 0, 0};
    const auto slot = zk_motor::pack_mit_command(zero);
    assert(slot[0] == 0x00 && slot[1] == 0x80);
    assert(slot[2] == 0x00 && slot[3] == 0x80);
    assert(slot[4] == 0x00 && slot[5] == 0x00);
    assert(slot[6] == 0x00 && slot[7] == 0x00);
    assert(slot[8] == 0x00 && slot[9] == 0x80);

    const auto enable = zk_motor::pack_special_frame({1, 6}, zk_motor::ENABLE_COMMAND, 1, 7);
    assert(enable[0] == 1 && enable[1] == 0 && enable[62] == 7 && enable[63] == 0);
    assert(enable[11] == zk_motor::ENABLE_COMMAND);
    assert(enable[21] == zk_motor::DISABLE_COMMAND);
    assert(enable[61] == zk_motor::ENABLE_COMMAND);

    const auto control = zk_motor::pack_control_frame({2}, {zero});
    assert(control[11] == zk_motor::DISABLE_COMMAND);
    assert(control[12] == 0x00 && control[13] == 0x80);

    const damiao_motor::MITParam single_command{0, 1, 0, 2, 0};
    const auto single = zk_motor::pack_single_mit_command(single_command);
    const uint8_t expected_single[8] = {0x00, 0x00, 0x66, 0x7F, 0xFF, 0x8E, 0x37, 0xFF};
    for (std::size_t i = 0; i < single.size(); ++i) assert(single[i] == expected_single[i]);
    const auto gripper = zk_motor::pack_single_mit_command(single_command);
    assert(gripper == single);
    assert(zk_motor::pack_single_enable_command(true) ==
           (zk_motor::SingleEnableFrame{0x8A, 0x01}));
    assert(zk_motor::pack_single_enable_command(false) ==
           (zk_motor::SingleEnableFrame{0x8A, 0x00}));
    assert(zk_motor::pack_single_zero_command() == (zk_motor::SingleZeroFrame{0x82}));

    uint8_t feedback[16]{};
    feedback[0] = 1;
    feedback[2] = feedback[4] = feedback[6] = 0x00;
    feedback[3] = feedback[5] = feedback[7] = 0x80;
    feedback[8] = 65;
    feedback[9] = 70;
    feedback[10] = 48;
    feedback[11] = 0x09;
    feedback[14] = 0x1B;
    const auto state = zk_motor::decode_feedback(feedback, sizeof(feedback), 2);
    assert(state.message_id == 1 && state.motor_id == 2);
    assert(std::abs(state.position) < 0.001);
    assert(std::abs(state.velocity) < 0.002);
    assert(std::abs(state.torque) < 0.004);
    assert(state.motor_temperature == 25 && state.mos_temperature == 30);
    assert(state.voltage == 48 && state.status == 0x09 && state.rolling_counter == 0x0B);

    const uint64_t single_feedback_value =
        (uint64_t{1} << 61U) | (uint64_t{32768} << 40U) | (uint64_t{2048} << 28U) |
        (uint64_t{2048} << 16U) | (uint64_t{100} << 8U) | uint64_t{90};
    uint8_t single_feedback[8]{};
    for (std::size_t i = 0; i < 8; ++i) {
        single_feedback[7 - i] = static_cast<uint8_t>(single_feedback_value >> (i * 8U));
    }
    const auto single_state = zk_motor::decode_single_feedback(single_feedback, 8, 7);
    assert(single_state.motor_id == 7 && std::abs(single_state.position) < 0.001);
    assert(std::abs(single_state.velocity) < 0.02 && std::abs(single_state.torque) < 0.02);
    assert(single_state.motor_temperature == 25 && single_state.mos_temperature == 20);
    single_feedback[0] |= 0x01;
    assert(zk_motor::decode_single_feedback(single_feedback, 8, 7).status == 0x02);
    assert(zk_motor::decode_single_feedback(single_feedback, 8, 8).motor_id == 8);

    bool rejected = false;
    try {
        zk_motor::decode_feedback(feedback, 15, 2);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}
