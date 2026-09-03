// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <openarm/canbus/can_socket.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <openarm/zk_motor/zk_motor.hpp>

namespace openarm::can::socket {

class ZKOpenArm {
public:
    explicit ZKOpenArm(const std::string& can_interface,
                       const std::vector<uint8_t>& motor_ids = {1, 2, 3, 4, 5, 6, 7},
                       uint16_t message_id = zk_motor::DEFAULT_MESSAGE_ID);

    const std::string& can_interface() const { return can_interface_; }
    bool can_fd_enabled() const { return true; }
    ZKOpenArm& get_arm() { return *this; }
    const std::vector<uint8_t>& get_motor_ids() const { return motor_ids_; }
    std::vector<zk_motor::Motor> get_motors() const { return motors_; }
    std::size_t last_received_count() const { return last_received_count_; }
    bool has_gripper() const;

    void enable_all();
    void disable_all();
    void set_zero_all();
    void refresh_all();
    void recv_all(int first_timeout_us = 1000);
    void mit_control_one(int index, const damiao_motor::MITParam& command);
    void mit_control_all(const std::vector<damiao_motor::MITParam>& commands);
    void set_callback_mode_all(damiao_motor::CallbackMode) {}

private:
    void send_frame(zk_motor::HostFrame frame);
    void send_single_frame(uint8_t motor_id, const uint8_t* data, std::size_t size);
    void send_controls();
    void send_special(uint8_t command);

    std::string can_interface_;
    std::vector<uint8_t> motor_ids_;
    std::vector<uint8_t> multi_motor_ids_;
    uint16_t message_id_;
    uint8_t rolling_counter_ = 0;
    std::unique_ptr<canbus::CANSocket> can_socket_;
    std::vector<zk_motor::Motor> motors_;
    std::vector<damiao_motor::MITParam> last_commands_;
    std::size_t last_received_count_ = 0;
};

}  // namespace openarm::can::socket
