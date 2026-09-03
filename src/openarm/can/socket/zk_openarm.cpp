// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#include <openarm/can/socket/zk_openarm.hpp>

#include <linux/can.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <unordered_set>

namespace openarm::can::socket {
ZKOpenArm::ZKOpenArm(const std::string& can_interface, const std::vector<uint8_t>& motor_ids,
                     uint16_t message_id)
    : can_interface_(can_interface), motor_ids_(motor_ids), message_id_(message_id) {
    if (motor_ids_.empty() || motor_ids_.size() > zk_motor::MAX_CONFIGURED_MOTORS) {
        throw std::invalid_argument("ZKOpenArm requires between 1 and 8 motors");
    }
    std::unordered_set<uint8_t> unique_ids;
    for (const uint8_t motor_id : motor_ids_) {
        if (motor_id == 0 || motor_id > zk_motor::GRIPPER_MOTOR_ID ||
            !unique_ids.insert(motor_id).second) {
            throw std::invalid_argument("ZKOpenArm motor IDs must be unique values from 1 to 8");
        }
        if (motor_id <= zk_motor::MAX_MOTORS) multi_motor_ids_.push_back(motor_id);
        motors_.emplace_back(motor_id);
    }
    last_commands_.resize(motor_ids_.size(), damiao_motor::MITParam{0, 0, 0, 0, 0});
    can_socket_ = std::make_unique<canbus::CANSocket>(can_interface_, true);
}

bool ZKOpenArm::has_gripper() const {
    return std::find(motor_ids_.begin(), motor_ids_.end(), zk_motor::GRIPPER_MOTOR_ID) !=
           motor_ids_.end();
}

void ZKOpenArm::send_single_frame(uint8_t motor_id, const uint8_t* data, std::size_t size) {
    canfd_frame frame{};
    frame.can_id = motor_id;
    frame.len = static_cast<__u8>(size);
    frame.flags = CANFD_BRS;
    std::memcpy(frame.data, data, size);
    if (!can_socket_->write_canfd_frame(frame)) {
        throw canbus::CANSocketException("Failed to write ZK single-joint CAN FD frame on " +
                                         can_interface_);
    }
}

void ZKOpenArm::send_controls() {
    std::vector<damiao_motor::MITParam> multi_commands;
    multi_commands.reserve(multi_motor_ids_.size());
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_ids_[i] <= zk_motor::MAX_MOTORS) {
            multi_commands.push_back(last_commands_[i]);
        }
    }
    if (!multi_motor_ids_.empty()) {
        send_frame(zk_motor::pack_control_frame(multi_motor_ids_, multi_commands, message_id_));
    }
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_ids_[i] <= zk_motor::MAX_MOTORS) continue;
        const auto frame = zk_motor::pack_single_mit_command(last_commands_[i]);
        send_single_frame(motor_ids_[i], frame.data(), frame.size());
    }
}

void ZKOpenArm::send_frame(zk_motor::HostFrame frame) {
    frame[62] = rolling_counter_;
    frame[63] = 0;

    canfd_frame can_frame{};
    can_frame.can_id = 0;
    can_frame.len = static_cast<__u8>(frame.size());
    can_frame.flags = CANFD_BRS;
    std::memcpy(can_frame.data, frame.data(), frame.size());
    if (!can_socket_->write_canfd_frame(can_frame)) {
        throw canbus::CANSocketException("Failed to write ZK CAN FD frame on " + can_interface_);
    }
    rolling_counter_ = static_cast<uint8_t>((rolling_counter_ + 1) & 0x0F);
}

void ZKOpenArm::send_special(uint8_t command) {
    if (!multi_motor_ids_.empty()) {
        send_frame(zk_motor::pack_special_frame(multi_motor_ids_, command, message_id_));
    }
}

void ZKOpenArm::enable_all() {
    send_special(zk_motor::ENABLE_COMMAND);
    for (const uint8_t motor_id : motor_ids_) {
        if (motor_id <= zk_motor::MAX_MOTORS) continue;
        const auto frame = zk_motor::pack_single_enable_command(true);
        send_single_frame(motor_id, frame.data(), frame.size());
    }
}

void ZKOpenArm::disable_all() {
    std::fill(last_commands_.begin(), last_commands_.end(),
              damiao_motor::MITParam{0, 0, 0, 0, 0});
    send_special(zk_motor::DISABLE_COMMAND);
    for (const uint8_t motor_id : motor_ids_) {
        if (motor_id <= zk_motor::MAX_MOTORS) continue;
        const auto frame = zk_motor::pack_single_enable_command(false);
        send_single_frame(motor_id, frame.data(), frame.size());
    }
}

void ZKOpenArm::set_zero_all() {
    disable_all();
    recv_all(5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    send_special(zk_motor::SET_ZERO_COMMAND);
    for (const uint8_t motor_id : motor_ids_) {
        if (motor_id <= zk_motor::MAX_MOTORS) continue;
        const auto frame = zk_motor::pack_single_zero_command();
        send_single_frame(motor_id, frame.data(), frame.size());
    }
    recv_all(5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    enable_all();
    recv_all(5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    refresh_all();
    recv_all(5000);
}

void ZKOpenArm::refresh_all() {
    std::fill(last_commands_.begin(), last_commands_.end(),
              damiao_motor::MITParam{0, 0, 0, 0, 0});
    send_controls();
}

void ZKOpenArm::mit_control_one(int index, const damiao_motor::MITParam& command) {
    if (index < 0 || static_cast<std::size_t>(index) >= last_commands_.size()) {
        throw std::out_of_range("ZK motor index is out of range");
    }
    last_commands_[static_cast<std::size_t>(index)] = command;
    send_controls();
}

void ZKOpenArm::mit_control_all(const std::vector<damiao_motor::MITParam>& commands) {
    if (commands.size() != motor_ids_.size()) {
        throw std::invalid_argument("ZK MIT command count must match configured motor count");
    }
    last_commands_ = commands;
    send_controls();
}

void ZKOpenArm::recv_all(int first_timeout_us) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::microseconds(std::max(first_timeout_us, 0));
    std::array<bool, zk_motor::MAX_CONFIGURED_MOTORS + 1> received{};
    std::size_t received_count = 0;
    last_received_count_ = 0;

    do {
        const auto now = std::chrono::steady_clock::now();
        const int remaining_us = first_timeout_us <= 0
                                     ? 0
                                     : static_cast<int>(
                                           std::chrono::duration_cast<std::chrono::microseconds>(
                                               deadline - now)
                                               .count());
        if (remaining_us < 0 || !can_socket_->is_data_available(remaining_us)) {
            break;
        }

        canfd_frame frame{};
        if (!can_socket_->read_canfd_frame(frame)) {
            break;
        }
        const uint8_t motor_id = static_cast<uint8_t>(frame.can_id & CAN_SFF_MASK);
        const auto id_it = std::find(motor_ids_.begin(), motor_ids_.end(), motor_id);
        if (id_it == motor_ids_.end()) {
            continue;
        }
        try {
            const bool single = motor_id > zk_motor::MAX_MOTORS;
            const zk_motor::Feedback feedback = single
                                                    ? zk_motor::decode_single_feedback(
                                                          frame.data, frame.len, motor_id)
                                                    : zk_motor::decode_feedback(
                                                          frame.data, frame.len, motor_id);
            if (!single && feedback.message_id != message_id_) continue;
            motors_[static_cast<std::size_t>(id_it - motor_ids_.begin())].update(feedback);
            if (!received[motor_id]) {
                received[motor_id] = true;
                ++received_count;
            }
        } catch (const std::invalid_argument&) {
            continue;
        }
    } while (received_count < motor_ids_.size() &&
             std::chrono::steady_clock::now() <= deadline);
    last_received_count_ = received_count;
}

}  // namespace openarm::can::socket
