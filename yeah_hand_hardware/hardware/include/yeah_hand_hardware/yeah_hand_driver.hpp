// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Author: Vittorio Lumare
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "yeah_hand_hardware/btserial.h"

namespace yeah_hand
{
class YeahHandSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(YeahHandSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool decodePacket(const std::string &line, const std::string &expected_cmd, std::vector<int> &data);
  void setFactorState(const std::vector<double> &c_factor);
  void setTemperatureState(const std::vector<double> &temperature);
  void setVoltageState(const std::vector<double> &voltage);
  void setAmperageState(const std::vector<double> &amperage);
  void setLoadState(const std::vector<double> &load);

private:
  const size_t expected_state_intefaces_count = 5;
  std::vector<double> joint_position_states_;
  std::vector<double> joint_temperature_states_;
  std::vector<double> joint_amperage_states_;
  std::vector<double> joint_voltage_states_;
  std::vector<double> joint_load_states_;

  std::vector<double> joint_position_commands_;

private:
  BtSerial *bt_;
};

}  // namespace yeah_hand
