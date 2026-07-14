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

#include "yeah_hand_hardware/yeah_hand_driver.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yeah_hand
{
hardware_interface::CallbackReturn YeahHandSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i<info_.joints.size(); i++){

    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  joint_position_commands_.assign(info_.joints.size(), 0.0);
  joint_position_states_.assign(info_.joints.size(), 0.0);
  joint_velocity_states_.assign(info_.joints.size(), 0.0);

  // BT
  bt_ = new BtSerial("/dev/rfcomm0", B115200);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YeahHandSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YeahHandSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{ 
  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YeahHandSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type YeahHandSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // For now, we’ll mirror commanded positions into state, so /joint_states reflects the commands.
  // Later, you’ll replace this with real sensor readings from Yeah Hand.

  const double index       = get_command("index_servo_joint/position");
  const double middle      = get_command("middle_servo_joint/position");
  const double ring_little = get_command("ring_little_servo_joint/position");
  const double thumb       = get_command("thumb_servo_joint/position");
  const double thumb_rot   = get_command("thumb_rotation_servo_joint/position");

  // Update state interfaces
  set_state("index_servo_joint/position", index);
  set_state("middle_servo_joint/position", middle);
  set_state("ring_little_servo_joint/position", ring_little);
  set_state("thumb_servo_joint/position", thumb);
  set_state("thumb_rotation_servo_joint/position", thumb_rot);

  // get factor values from yeah hand via BT
//  std::string line;
//  if (!bt_->readLine(line)) {
//    RCLCPP_ERROR(rclcpp::get_logger("YeahHandSystem"), "BT read failed");
//    return hardware_interface::return_type::ERROR;
//  }

//  auto cmd = line.substr(0,4);
//  if(cmd == "ROS "){
//    // get index factor
//    size_t value_start = 4;
//    size_t value_end = line.find_first_of(' ', value_start + 1);
//    const int index_factor = std::stoi(cmd.substr(value_start, value_end));
//    // get middle factor
//    value_start = value_end + 1;
//    value_end = line.find_first_of(' ', value_start + 1);
//    const int middle_factor = std::stoi(cmd.substr(value_start, value_end));
//    // get ring_little factor
//    value_start = value_end + 1;
//    value_end = line.find_first_of(' ', value_start + 1);
//    const int ring_little_factor = std::stoi(cmd.substr(value_start, value_end));
//    // get thumb factor
//    value_start = value_end + 1;
//    value_end = line.find_first_of(' ', value_start + 1);
//    const int thumb_factor = std::stoi(cmd.substr(value_start, value_end));
//    // get thumb_rotation factor
//    value_start = value_end + 1;
//    value_end = line.find_first_of(' ', value_start + 1);
//    const int thumb_rotation_factor = std::stoi(cmd.substr(value_start, value_end));

//    std::cout<< index_factor
//             << middle_factor
//             << ring_little_factor
//             << thumb_factor
//             << thumb_rotation_factor << std::endl;

//  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type YeahHandSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  if (!bt_->isOpen()) {
     RCLCPP_ERROR(rclcpp::get_logger("YeahHandSystem"), "BT port not open");
     return hardware_interface::return_type::ERROR;
  }

  const double index       = get_command("index_servo_joint/position");
  const double middle      = get_command("middle_servo_joint/position");
  const double ring_little = get_command("ring_little_servo_joint/position");
  const double thumb       = get_command("thumb_servo_joint/position");
  const double thumb_rot   = get_command("thumb_rotation_servo_joint/position");

  // Convert to [0,100] factor
  const int index_factor = static_cast<int>((index / 6.28) * 100);
  const int middle_factor = static_cast<int>((middle / 6.28) * 100);
  const int ring_little_factor = static_cast<int>((ring_little / 6.28) * 100);
  const int thumb_factor = static_cast<int>((thumb / 6.28) * 100);
  const int thumb_rot_factor = static_cast<int>((thumb_rot / 6.28) * 100);

  std::ostringstream ss;
  ss << "ROS "
     << index_factor << " "
     << middle_factor << " "
     << ring_little_factor << " "
     << thumb_factor << " "
     << thumb_rot_factor << "\r\n";

  std::cout << ss.str() << std::endl;

  if (!bt_->writeString(ss.str())) {
    RCLCPP_ERROR(rclcpp::get_logger("YeahHandSystem"), "BT write failed");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


}  // namespace yeah_hand

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  yeah_hand::YeahHandSystem, hardware_interface::SystemInterface)
