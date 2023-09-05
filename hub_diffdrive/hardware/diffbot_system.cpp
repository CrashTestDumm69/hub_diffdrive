// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef _WIN64
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "hub_diffdrive/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hub_diffdrive
{
	hardware_interface::CallbackReturn HubDiffDriveHardware::on_init(
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
		cfg_.device = info_.hardware_parameters["device"];
		cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
		cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

		wheel_l_.setup(cfg_.left_wheel_name);
		wheel_r_.setup(cfg_.right_wheel_name);
		for (const hardware_interface::ComponentInfo & joint : info_.joints)
		{
			// DiffBotSystem has exactly two states and one command interface on each joint
			if (joint.command_interfaces.size() != 1)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("HubDiffDriveHardware"),
					"Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
					joint.command_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("HubDiffDriveHardware"),
					"Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
					joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}

			if (joint.state_interfaces.size() != 2)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("HubDiffDriveHardware"),
					"Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
					joint.state_interfaces.size());
				return hardware_interface::CallbackReturn::ERROR;
			}

			if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("HubDiffDriveHardware"),
					"Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
					joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
				return hardware_interface::CallbackReturn::ERROR;
			}

			if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
			{
				RCLCPP_FATAL(
					rclcpp::get_logger("HubDiffDriveHardware"),
					"Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
					joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
				return hardware_interface::CallbackReturn::ERROR;
			}
		}
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> HubDiffDriveHardware::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;

		state_interfaces.emplace_back(hardware_interface::StateInterface(
			wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

		state_interfaces.emplace_back(hardware_interface::StateInterface(
			wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
		state_interfaces.emplace_back(hardware_interface::StateInterface(
			wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> HubDiffDriveHardware::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;

			command_interfaces.emplace_back(hardware_interface::CommandInterface(
				wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

			command_interfaces.emplace_back(hardware_interface::CommandInterface(
				wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

		return command_interfaces;
	}

	hardware_interface::CallbackReturn HubDiffDriveHardware::on_configure(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Configuring ...please wait...");

		comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Successfully configured!");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn HubDiffDriveHardware::on_cleanup(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Cleaning up ...please wait...");

		comms_.disconnect();

		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Successfully cleaned up!");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn HubDiffDriveHardware::on_activate(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Activating ...please wait...");
		
		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Successfully activated!");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn HubDiffDriveHardware::on_deactivate(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{
		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Deactivating ...please wait...");

		RCLCPP_INFO(rclcpp::get_logger("HubDiffDriveHardware"), "Successfully deactivated!");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type HubDiffDriveHardware::read(
		const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
	{
		/*int u_l = wheel_l_.pos;
		int u_r = wheel_r_.pos;*/

		comms_.read_motor(wheel_l_.vel, wheel_r_.vel, wheel_l_.pos, wheel_r_.pos);

		/*int v_l = wheel_l_.pos;
		int v_r = wheel_r_.pos;

		std::cout << u_l << " " << v_l << " " << wheel_l_.vel << std::endl;
		
		if(wheel_l_.vel > 0)
			if(v_l < u_l)
				wheel_l_.rev++;

		if(wheel_r_.vel > 0)
			if(v_r < u_r)
				wheel_r_.rev++;

		if(wheel_l_.vel < 0)
			if(v_l > u_l)
				wheel_l_.rev--;

		if(wheel_r_.vel < 0)
			if(v_r > u_r)
				wheel_r_.rev--;

		if(wheel_l_.vel != 0)
			wheel_l_.pos = wheel_l_.rev * 360 + wheel_l_.pos;	
		wheel_l_.pos *= 0.0175;
		if(wheel_r_.vel != 0)
			wheel_r_.pos = wheel_r_.rev * 360 + wheel_r_.pos;
		wheel_r_.pos *= 0.0175;
		std::cout << wheel_l_.rev << " " << wheel_r_.rev << std::endl;*/
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type hub_diffdrive ::HubDiffDriveHardware::write(
		const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
	{
		int motor_l_rpm = (wheel_l_.cmd * 60 * 7) / 44;
		int motor_r_rpm = (wheel_r_.cmd * 60 * 7) / 44;
		double v_l, v_r;
		comms_.set_motor_values(motor_l_rpm, motor_r_rpm, v_l, v_r);
		wheel_l_.pos += (v_l * (period.seconds() / 60)) * 6.28319;
		wheel_r_.pos += (v_r * (period.seconds() / 60)) * 6.28319;
		return hardware_interface::return_type::OK;
	}

}// namespace hub_diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
	hub_diffdrive::HubDiffDriveHardware, hardware_interface::SystemInterface)