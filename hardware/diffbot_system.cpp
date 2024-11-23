// Copyright 2021 ros2_control Development Team
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

#include "snowplow_motor_controller/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace snowplow_motor_controller
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  //cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.loop_rate = 120.0;
  //cfg_.device = info_.hardware_parameters["device"].c_str();
  cfg_.device = "/dev/ttyACM0";
  //cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  //cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  //cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.baud_rate = 115200;
  cfg_.timeout = 1000;
  cfg_.enc_counts_per_rev = 1024;
  cfg_.openLoop = true;

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{

  //Sets up interfaces for position/velocity of each wheel, whenever they get updated, the rest of ros2 control system becomes aware

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel, whenever the rest of ros2 control system sets the velocity, it changes value of command members

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring ...please wait...");
  if (comms_.isConnected()) {
    comms_.disconnect();
  }

  comms_.connect(cfg_.device, cfg_.timeout);
  //comms_.connect();

  if (comms_.isConnected()) {
    //comms_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Succesfully connected to serial port!");

  
    comms_.configure(cfg_.openLoop);


  } else {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Unable to connect to serial port!");
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  //comms_.connect(cfg_.device, cfg_.timeout);
  //comms_.connect();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  //comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  //Reads encoder values, calculates number of seconds since last time we did it, stores previous position of wheel, 
  //calculates new position, sets velocity to be difference between new position and old position divided by time



  //comms_.readEncoders();
  // 
  wheel_l_.enc = comms_.readEncoderCh1() / 28.7;
  wheel_r_.enc = comms_.readEncoderCh2() / 28.7;

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calcEncAngle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calcEncAngle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type snowplow_motor_controller ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "wheel_l_.cmd: %f", wheel_l_.cmd);


   int motor_l_counters_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
   int motor_r_counters_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  //int motor_l_rpm = (wheel_l_.cmd * 60) / (2 * M_PI * wheel_l_.rads_per_count);
  //int motor_r_rpm = (wheel_r_.cmd * 60) / (2 * M_PI * wheel_r_.rads_per_count);

  RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Motor1 Speed: %d, Motor2 Speed: %d",
      motor_l_counters_per_loop,
      motor_r_counters_per_loop
  );

  // if (motor_l_counters_per_loop > 500) {
  //   motor_l_counters_per_loop = 500;
  // }

  // if (motor_r_counters_per_loop > 500) {
  //   motor_r_counters_per_loop = 500;
  // }

  // comms_.setMotor1Speed(60);
  // comms_.setMotor2Speed(60);
  if (cfg_.openLoop) {
    comms_.setMotorSpeeds(cfg_.openLoop, motor_l_counters_per_loop, motor_r_counters_per_loop);
  } else {
    comms_.setMotorSpeeds(cfg_.openLoop, 60, 60);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace snowplow_motor_contrcfg_.opeoller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  snowplow_motor_controller::DiffBotSystemHardware, hardware_interface::SystemInterface)
