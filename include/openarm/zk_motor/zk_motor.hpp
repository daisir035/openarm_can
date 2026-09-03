// Copyright 2026 daisir035
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include "zk_protocol.hpp"

namespace openarm::zk_motor {

class Motor {
public:
    explicit Motor(uint8_t motor_id) : motor_id_(motor_id) {}

    uint8_t get_motor_id() const { return motor_id_; }
    double get_position() const { return position_; }
    double get_velocity() const { return velocity_; }
    double get_torque() const { return torque_; }
    int get_state_tmos() const { return mos_temperature_; }
    int get_state_trotor() const { return motor_temperature_; }
    uint8_t get_voltage() const { return voltage_; }
    uint32_t get_status() const { return status_; }
    bool is_enabled() const { return (status_ & 1U) != 0; }
    bool has_fault() const { return (status_ & ~1U) != 0; }

    void update(const Feedback& feedback) {
        position_ = feedback.position;
        velocity_ = feedback.velocity;
        torque_ = feedback.torque;
        motor_temperature_ = feedback.motor_temperature;
        mos_temperature_ = feedback.mos_temperature;
        voltage_ = feedback.voltage;
        status_ = feedback.status;
    }

private:
    uint8_t motor_id_;
    double position_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    int motor_temperature_ = 0;
    int mos_temperature_ = 0;
    uint8_t voltage_ = 0;
    uint32_t status_ = 0;
};

}  // namespace openarm::zk_motor
