/*********************************************************************
 *
 *  Inferfaces for doosan robot controllor
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Doosan Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef DSR_HARDWARE2__DR_HW_INTERFACE2_H
#define DSR_HARDWARE2__DR_HW_INTERFACE2_H

#include <algorithm>
#include <array>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// #include "../../../common2/include/DRFLEx.h"
// TODO #include "../../../common2/include/dsr_serial.h"

#include "../../common2/include/DRFLEx.h"

namespace dsr_hardware2 {

class HARDWARE_INTERFACE_PUBLIC DRHWInterface
        : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;

    std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces(
    ) override;

    return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
            override;

    ~DRHWInterface();

protected:
    /// The size of this vector is (standard_interfaces_.size() x nr_joints)
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_efforts_command_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> ft_states_;
    std::vector<double> ft_command_;


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_torque;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_act_joint_torque;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_raw_joint_torque;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_ext_joint_torque;
    float torque_state[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
            {"position", {}},
            {"velocity", {}},
            {"effort",   {}}
    };

    enum ControlMode {
        UNKNOWN = 0,
        POSITION,
        VELOCITY,
        TORQUE
    } control_mode_;

    pinocchio::Model robot_mdl_;
    pinocchio::Data  robot_data_;
    DRAFramework::CDRFLEx drfl;
};


}  // namespace dsr_hardware2

#endif  // end
