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

    if (joint.state_interfaces.size() != expected_state_intefaces_count)
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
  joint_load_states_.assign(info_.joints.size(), 0.0);
  joint_amperage_states_.assign(info_.joints.size(), 0.0);
  joint_voltage_states_.assign(info_.joints.size(), 0.0);
  joint_temperature_states_.assign(info_.joints.size(), 0.0);


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
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, get_state(name));
  }


  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn YeahHandSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool YeahHandSystem::decodePacket(const std::string &line, const std::string &expected_cmd, std::vector<int> &data){

    const size_t cmd_start = 0;
    const size_t cmd_end = expected_cmd.size();
    auto cmd = line.substr(cmd_start, cmd_end);
    size_t value_start = 0;
    size_t value_end = 0;
    if(cmd == expected_cmd){
      try{
        for(size_t i = 0 ; i < data.size(); i++){
          if(i == 0){
            // get first value
            value_start = cmd_end;
            value_end = line.find_first_of(' ', value_start);
            data.at(i) = std::stoi(line.substr(value_start, value_end));
          }else{
            // get i-th value
            value_start = value_end + 1;
            value_end = line.find_first_of(' ', value_start + 1);
            data.at(i) = std::stoi(line.substr(value_start, value_end));
          }
        }
      }
      catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(),
                     "BT parse failed (stoi): '%s' — %s",
                     line.c_str(), e.what());
        return false;
      }
    } else {
      return false;
    }

  return true;
}

inline void logPacket(const std::string &comment, std::vector<int> data){
  std::cout << comment << " ";
  for (size_t i = 0; i < data.size();i++){
    std::cout << data.at(i) << " ";
  }
  std::cout << std::endl;
}

inline void convertFactor(const std::vector<int> & factor, std::vector<double> & c_factor){
  c_factor.resize(factor.size());
  c_factor.at(0) = (static_cast<double>(factor.at(0)) / 100.0) * 6.28;
  c_factor.at(1) = (static_cast<double>(factor.at(1)) / 100.0) * 6.28;
  c_factor.at(2) = (static_cast<double>(factor.at(2)) / 100.0) * 6.28;
  c_factor.at(3) = (static_cast<double>(factor.at(3)) / 100.0) * 6.28;
  c_factor.at(4) = (static_cast<double>(factor.at(4)) / 100.0) * 1.57; // thumb rotation
}

inline void convertTemperature(const std::vector<int> & temp, std::vector<double> & c_temp){
  c_temp.resize(temp.size());
  c_temp.at(0) = static_cast<double>(temp.at(0));
  c_temp.at(1) = static_cast<double>(temp.at(1));
  c_temp.at(2) = static_cast<double>(temp.at(2));
  c_temp.at(3) = static_cast<double>(temp.at(3));
  c_temp.at(4) = static_cast<double>(temp.at(4));
}

inline void convertVoltage(const std::vector<int> & volt, std::vector<double> & c_volt){
  c_volt.resize(volt.size());
  c_volt.at(0) = static_cast<double>(volt.at(0))/10.0;
  c_volt.at(1) = static_cast<double>(volt.at(1))/10.0;
  c_volt.at(2) = static_cast<double>(volt.at(2))/10.0;
  c_volt.at(3) = static_cast<double>(volt.at(3))/10.0;
  c_volt.at(4) = static_cast<double>(volt.at(4))/10.0;
}

inline void convertAmperage(const std::vector<int> & amp, std::vector<double> & c_amp){
  // TODO CORRECT FROM TECHNICAL FILE
  c_amp.resize(amp.size());
  c_amp.at(0) = static_cast<double>(amp.at(0))/10.0;
  c_amp.at(1) = static_cast<double>(amp.at(1))/10.0;
  c_amp.at(2) = static_cast<double>(amp.at(2))/10.0;
  c_amp.at(3) = static_cast<double>(amp.at(3))/10.0;
  c_amp.at(4) = static_cast<double>(amp.at(4))/10.0;
}

inline void convertLoad(const std::vector<int> & load, std::vector<double> & c_load){
  // TODO CORRECT FROM TECHNICAL FILE
  c_load.resize(load.size());
  c_load.at(0) = static_cast<double>(load.at(0))/10.0;
  c_load.at(1) = static_cast<double>(load.at(1))/10.0;
  c_load.at(2) = static_cast<double>(load.at(2))/10.0;
  c_load.at(3) = static_cast<double>(load.at(3))/10.0;
  c_load.at(4) = static_cast<double>(load.at(4))/10.0;
}

void YeahHandSystem::setFactorState(const std::vector<double> &converted_factor){
  set_state("index_servo_joint/position", converted_factor.at(0));
  set_state("middle_servo_joint/position", converted_factor.at(1));
  set_state("ring_little_servo_joint/position", converted_factor.at(2));
  set_state("thumb_servo_joint/position", converted_factor.at(3));
  set_state("thumb_rotation_servo_joint/position", converted_factor.at(4));
}

void YeahHandSystem::setTemperatureState(const std::vector<double> &temperature){
  set_state("index_servo_joint/temperature", temperature.at(0));
  set_state("middle_servo_joint/temperature", temperature.at(1));
  set_state("ring_little_servo_joint/temperature", temperature.at(2));
  set_state("thumb_servo_joint/temperature", temperature.at(3));
  set_state("thumb_rotation_servo_joint/temperature", temperature.at(4));
}

void YeahHandSystem::setVoltageState(const std::vector<double> &voltage){
  set_state("index_servo_joint/voltage", voltage.at(0));
  set_state("middle_servo_joint/voltage", voltage.at(1));
  set_state("ring_little_servo_joint/voltage", voltage.at(2));
  set_state("thumb_servo_joint/voltage", voltage.at(3));
  set_state("thumb_rotation_servo_joint/voltage", voltage.at(4));
}

void YeahHandSystem::setAmperageState(const std::vector<double> &amperage){
  set_state("index_servo_joint/amperage", amperage.at(0));
  set_state("middle_servo_joint/amperage", amperage.at(1));
  set_state("ring_little_servo_joint/amperage", amperage.at(2));
  set_state("thumb_servo_joint/amperage", amperage.at(3));
  set_state("thumb_rotation_servo_joint/amperage", amperage.at(4));
}

void YeahHandSystem::setLoadState(const std::vector<double> &load){
  set_state("index_servo_joint/load", load.at(0));
  set_state("middle_servo_joint/load", load.at(1));
  set_state("ring_little_servo_joint/load", load.at(2));
  set_state("thumb_servo_joint/load", load.at(3));
  set_state("thumb_rotation_servo_joint/load", load.at(4));
}

hardware_interface::return_type YeahHandSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Mocked Driver
  // For now, we’ll mirror commanded positions into state, so /joint_states reflects the commands.
  // Later, you’ll replace this with real sensor readings from Yeah Hand.

  //  const double index       = get_command("index_servo_joint/position");
  //  const double middle      = get_command("middle_servo_joint/position");
  //  const double ring_little = get_command("ring_little_servo_joint/position");
  //  const double thumb       = get_command("thumb_servo_joint/position");
  //  const double thumb_rot   = get_command("thumb_rotation_servo_joint/position");

  // Update state interfaces
  //  set_state("index_servo_joint/position", index);
  //  set_state("middle_servo_joint/position", middle);
  //  set_state("ring_little_servo_joint/position", ring_little);
  //  set_state("thumb_servo_joint/position", thumb);
  //  set_state("thumb_rotation_servo_joint/position", thumb_rot);

  // get factor values from yeah hand via BT
  std::string line;
  bool success_read = bt_->readLine(line);
  std::cout << "success_read:" << success_read << std::endl;

  if (!success_read) {
    return hardware_interface::return_type::OK;
  }

  std::cout<< "LINE:"<< line << std::endl;  

  std::vector<int> factor(info_.joints.size());
  std::vector<int> temperature(info_.joints.size());
  std::vector<int> voltage(info_.joints.size());
  std::vector<int> amperage(info_.joints.size());
  std::vector<int> load(info_.joints.size());

  if(decodePacket(line, "ROSPOS ", factor)){
    logPacket("READ factor", factor);
    std::vector<double> c_factor;
    convertFactor(factor, c_factor);
    setFactorState(c_factor);
  } else
  if(decodePacket(line, "ROSTEMP ", temperature)){
    logPacket("READ temperature", temperature);
    std::vector<double> c_temperature;
    convertTemperature(temperature, c_temperature);
    setTemperatureState(c_temperature);
  }else
  if(decodePacket(line, "ROSVOLT ", voltage)){
    logPacket("READ voltage", voltage);
    std::vector<double> c_voltage;
    convertVoltage(voltage, c_voltage);
    setVoltageState(c_voltage);
  }
  else
    if(decodePacket(line, "ROSAMPR ", amperage)){
      logPacket("READ amperage", amperage);
      std::vector<double> c_amperage;
      convertAmperage(amperage, c_amperage);
      setAmperageState(c_amperage);
    }
  else
    if(decodePacket(line, "ROSLOAD ", load)){
      logPacket("READ load", load);
      std::vector<double> c_load;
      convertLoad(load, c_load);
      setLoadState(c_load);
    }
  else {
    return hardware_interface::return_type::OK;
  }



    //TODO convrsion and setters for temp cur volt torq

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
  const int thumb_rot_factor = static_cast<int>((thumb_rot / 1.57) * 100);

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
